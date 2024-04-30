#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform, Polygon, Point
from std_msgs.msg import Header

class ArucoDetector:
    def __init__(self, camera_topic, camera_matrix, distortion_coeffs, 
                 camera_frame_id, object_frame_id, 
                 target_id, visual_controller_topic, 
                 stream_video=True, verbose=True):
        
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        self.corner_pub = rospy.Publisher(visual_controller_topic, Polygon, queue_size=10)

        self.bridge = CvBridge()

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        self.stream_video = stream_video
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.camera_frame_id = camera_frame_id
        self.object_frame_id = object_frame_id

        self.target_id = target_id
        
        self.on_sight = []
        self.knowledge = {}

        self.verbose = verbose
        
        rospy.Timer(rospy.Duration(1.5), self._verbose)

    def _verbose(self, _):
        if self.verbose:
            for marker in self.on_sight:
                rospy.loginfo(f"Markers on sight: {marker} detected at {self.knowledge.get(marker)[0]}")
            self.on_sight = []

    def image_callback(self, frame):
        # Convert the image message to cv image
        cv = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        
        # Detect aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY), self.aruco_dict, parameters=self.parameters,
                                                              cameraMatrix=self.camera_matrix, distCoeff = self.distortion_coeffs)
        
        ids = ids.flatten() if ids is not None else []

        # Objects detected
        for i in range(0, len(ids)):

            # Estimate pose
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.072, self.camera_matrix,
                                                                    self.distortion_coeffs)
            rvec = rvec.flatten()
            tvec = tvec.flatten()

            # Draw the marker axis and border
            if self.stream_video:
                cv2.aruco.drawAxis(cv, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 0.05)
                cv2.aruco.drawDetectedMarkers(cv, corners)

            # Adjust frame for tf
            tvec[1] = -tvec[1]
            tvec[0] = -tvec[0]

            quat = tft.quaternion_from_euler(rvec[1], rvec[2], rvec[0])

            # Stick to the ground with custom orientation (DEBUG)
            # tvec[1] = 0.02 - 0.09
            # quat = tft.quaternion_from_euler(np.pi, np.pi, 0)

            if ids[i] not in self.on_sight:
                self.on_sight.append(ids[i])
                self.knowledge[ids[i]] = (tvec, quat)
            
            # Only send information regarding the target marker
            if ids[i] != self.target_id:
                continue

            # Broadcast the static transform
            self.tf_broadcaster.sendTransform(TransformStamped(
                header=Header(stamp=rospy.Time.now(), frame_id=self.camera_frame_id),
                child_frame_id=self.object_frame_id + str(ids[i]),
                transform=Transform(
                    translation=Vector3(tvec[0], tvec[1], tvec[2]),
                    rotation=Quaternion(quat[1], quat[2], quat[3], quat[0]) 
                )
            ))

            # Broadcast corners to controller topic
            self.corner_pub.publish(Polygon(
                points=[Point(x=corner[0], y=corner[1]) for corner in corners[i][0]]
            ))


        if self.stream_video:
            cv2.imshow('Detector', cv)
            cv2.waitKey(3)

if __name__ == '__main__':

    rospy.init_node('aruco_detector', anonymous=True)

    camera_matrix = np.array(rospy.get_param('/camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('/distortion_coeffs'))
    camera_frame_id = rospy.get_param('/camera_frame')
    object_frame_id = rospy.get_param('/object_frame')
    camera_topic = rospy.get_param('/camera_topic')
    target_id = rospy.get_param('/target_id')
    visual_controller_topic = rospy.get_param('/visual_controller_topic')

    ad = ArucoDetector(camera_topic, 
                       camera_matrix, 
                       distortion_coeffs, 
                       camera_frame_id, 
                       object_frame_id, 
                       target_id,
                       visual_controller_topic, 
                       stream_video=True)
    try:
        rospy.logwarn('Aruco detector running')
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()