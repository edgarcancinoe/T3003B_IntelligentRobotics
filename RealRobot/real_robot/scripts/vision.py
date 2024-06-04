#!/usr/bin/env python3

#Node for Identify Target
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform, Polygon, Point, QuaternionStamped, PointStamped
from std_msgs.msg import Header, Bool
import base64
from real_robot_util.util import get_vision_params

class ArucoDetector:
    def __init__(self, aruco_size, camera_topic, camera_matrix, distortion_coeffs, inertial_frame_id,
                 camera_frame_id, object_frame_id, 
                 target_id, target_detection_topic, 
                 ibvs_activate_topic,
                 goal_publisher_topic,
                 done_vision_topic,
                 stream_video=True, verbose=True, 
                 target_samples_required: int = 20):
        
        #self.image_sub = rospy.Subscriber(camera_topic, CompressedImage, self._image_processing)
        self.reset_sub = rospy.Subscriber('/reset_vision', Bool, self._reset)
        self.reached_goal_sub = rospy.Subscriber(goal_publisher_topic, Bool, self._reset)
        self.corner_pub = rospy.Publisher(target_detection_topic, Polygon, queue_size=10)
        #self.send_image = rospy.Publisher('/img_comp', CompressedImage, queue_size = 10)
        #self.active_ibvs_controller_pub = rospy.Publisher(ibvs_activate_topic, Bool, queue_size=10)
        self.done_vision_pub = rospy.Publisher(done_vision_topic, Bool, queue_size=10)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()

        self.stream_video = stream_video
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        self.aruco_size = aruco_size
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.inertial_frame_id = inertial_frame_id
        self.camera_frame_id = camera_frame_id
        self.object_frame_id = object_frame_id

        self.target_id = target_id
        
        self.on_sight = []
        self.knowledge = {}

        self.verbose = verbose
        
        # Target knowledge buffer
        self.target_positions = []
        self.target_samples_required = target_samples_required
        rospy.sleep(2)

        # Initialize video capture
        gst_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            rospy.logerr("Failed to open video capture")
            raise RuntimeError("Failed to open video capture")
        rospy.Timer(rospy.Duration(3), self._verbose)
        self.camaraTimer = rospy.Timer(rospy.Duration(1/30), self._image_processing)
       
    #def _ibvs_callback(self, msg):
    #    if  msg.data:
    #        self.camaraTimer.shutdown
    #        self.done_vision_pub.publish(True)


    def _reset(self, _):
        self.knowledge = {}
        self.on_sight = []
        self.target_positions = []

    def _verbose(self, _):
        if self.verbose:
            for marker in self.on_sight:
                tvec_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, 
                                                             PointStamped(header=Header(stamp=rospy.Time(0), frame_id=self.camera_frame_id),
                                                                                                    point=Point(*self.knowledge.get(marker)[0])))
                rospy.loginfo(f"\nMarkers {marker} detected at:\n{tvec_inertial.point}\n")
            self.on_sight = []

    def _image_processing(self, _):
        if not self.cap.isOpened():
            rospy.logerr("Video capture is not open, trying to reopen.")
            self.cap.open()

        ret, cv = self.cap.read()
        result, jpeg_image = cv2.imencode('.jpg', cv)
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = rospy.Time.now()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = jpeg_image.tobytes()
        #self.send_image.publish(compressed_image_msg)
        if not ret:
            rospy.logerr("Failed to capture image")
            return
        
        # Display the frame
        if self.stream_video:
            cv2.imshow('frame', cv)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
        
        # Detect aruco markers
        gray_image = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters,
                                              cameraMatrix=self.camera_matrix, distCoeff=self.distortion_coeffs)

        if ids is not None:
            ids = ids.flatten() 
        else:
            self.corner_pub.publish(Polygon()) # Send empty 
            return
        
        # Objects detected
        for i in range(len(ids)):
            # Only send information regarding the target marker
            if ids[i] != self.target_id:
                continue

            # Estimate pose
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.aruco_size, self.camera_matrix,
                                                            self.distortion_coeffs)
            rvec = rvec.flatten()
            tvec = tvec.flatten()

            # Draw the marker axis and border
            if self.stream_video:
                cv2.aruco.drawAxis(cv, self.camera_matrix, self.distortion_coeffs, rvec, tvec, self.aruco_size)
                cv2.aruco.drawDetectedMarkers(cv, corners)

            if ids[i] not in self.on_sight:
                self.on_sight.append(ids[i])
                self.knowledge[ids[i]] = (tvec, rvec)

            # Broadcast the static transform of target marker (median of n samples found)
            if len(self.target_positions) < self.target_samples_required:
                self.target_positions.append((tvec, rvec, corners[i][0]))
                continue

            elif len(self.target_positions) == self.target_samples_required:
                self.target_positions.append((tvec, rvec, corners[i][0])) # Ensure that the last sample is included so that transform is not sent again
                median_tvec = np.median([pos[0] for pos in self.target_positions], axis=0)
                median_rvec = np.median([pos[1] for pos in self.target_positions], axis=0)
                median_quat = tft.quaternion_from_euler(median_rvec[0], median_rvec[1], median_rvec[2])

                # Get target's pose in inertial frame
                tvec_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, 
                                                                PointStamped(header=Header(stamp=rospy.Time(0), frame_id=self.camera_frame_id),
                                                                                                        point=Point(*median_tvec)))
                quat_inertial = self.tf_listener.transformQuaternion(self.inertial_frame_id, 
                                                                QuaternionStamped(header=Header(stamp=rospy.Time(0), frame_id=self.camera_frame_id),
                                                                                                    quaternion=Quaternion(*median_quat)))

                rospy.loginfo(f'Found at: {tvec_inertial.point}')
                self.tf_broadcaster.sendTransform(TransformStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id=self.inertial_frame_id),
                    child_frame_id=self.object_frame_id,
                    transform=Transform(
                        translation=Vector3(tvec_inertial.point.x, tvec_inertial.point.y, tvec_inertial.point.z),
                        rotation=quat_inertial.quaternion
                    )
                ))
            
            # Broadcast target's corners to detection topic (median of last n samples found)
            self.corner_pub.publish(Polygon(
                points=[Point(x=corner[0], y=corner[1], z=tvec[2]) for corner in corners[i][0]]
            ))

if __name__ == '__main__':
    rospy.init_node('aruco_detector_server', anonymous=True)
    params = get_vision_params()
    
    inertial_frame_id = rospy.get_param('/inertial_frame')
    camera_frame_id = rospy.get_param('/camera_frame')
    object_frame_id = rospy.get_param('/object_frame')
    camera_topic = rospy.get_param('/camera_topic')
    target_id = rospy.get_param('/target_id')
    target_detection_topic = rospy.get_param('/target_detection_topic')
    ibvs_activate_topic = rospy.get_param('/ibvs_activate_topic')
    goal_publisher_topic = rospy.get_param('/unlock_topic', 'unlock')
    done_vision_topic = rospy.get_param('/done_vision_topic')
    camera_matrix = np.array(rospy.get_param('/camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('/distortion_coeffs'))
    aruco_size = rospy.get_param('/aruco_size')

    #Init Class
    ad = ArucoDetector(aruco_size= aruco_size,
                    camera_topic=camera_topic, 
                    camera_matrix=camera_matrix, 
                    distortion_coeffs=distortion_coeffs, 
                    inertial_frame_id=inertial_frame_id,
                    camera_frame_id=camera_frame_id, 
                    object_frame_id=object_frame_id, 
                    target_id=target_id,
                    target_detection_topic=target_detection_topic, 
                    ibvs_activate_topic=ibvs_activate_topic,
                    goal_publisher_topic=goal_publisher_topic,
                    done_vision_topic=done_vision_topic,
                    stream_video=False)
    
    try:
        rospy.logwarn('Aruco detector service running')
        rospy.spin()
        ad.cap.release()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")

    cv2.destroyAllWindows()