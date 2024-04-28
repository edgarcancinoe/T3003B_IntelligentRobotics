#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform
from std_msgs.msg import Header

class ArucoDetector:
    def __init__(self, camera_topic, camera_matrix, distortion_coeffs, camera_frame_id, object_frame_id, stream_video=True):
        rospy.init_node('aruco_detector', anonymous=True)
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        self.bridge = CvBridge()

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.stream_video = stream_video
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.camera_frame_id = camera_frame_id
        self.object_frame_id = object_frame_id

    def image_callback(self, frame):
        # Convert the image message to a cv image
        cv = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        # Convert to grayscale
        gray = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
        
        # Detect the markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters,
                                                              cameraMatrix=camera_matrix, distCoeff = distortion_coeffs)

        # Objects detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.072, self.camera_matrix,
                                                                        self.distortion_coeffs)
                rvec = rvec.flatten()
                tvec = tvec.flatten()

                # Draw the marker axis and border
                if self.stream_video:
                    cv2.aruco.drawAxis(cv, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 0.05)
                    cv2.aruco.drawDetectedMarkers(cv, corners)

                tvec[1] = -tvec[1]
                tvec[0] = -tvec[0]
                quat = tft.quaternion_from_euler(rvec[1], rvec[2], rvec[0])
                
                # Stick to the ground with custom orientation (DEBUG)
                # tvec[1] = 0.02 - 0.09
                # quat = tft.quaternion_from_euler(np.pi, np.pi, 0)
                
                rospy.loginfo(f"Marker {ids[i]} detected at {tvec}")
                
                # Broadcast the static transform
                odom_transform = TransformStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id=self.camera_frame_id),
                    child_frame_id=self.object_frame_id,
                    transform=Transform(
                        translation=Vector3(tvec[0], tvec[1], tvec[2]),
                        rotation=Quaternion(quat[1], quat[2], quat[3], quat[0]) 
                    )
                )

                # Broadcast the static transform
                self.tf_broadcaster.sendTransform(odom_transform)


                
                
        if self.stream_video:
            cv2.imshow('Detector', cv)
            cv2.waitKey(3)

if __name__ == '__main__':

    camera_matrix = np.array([[615.6994619, 0, 310.65921052],
                          [0, 614.09468385, 221.27468646],
                          [0, 0, 1]])

    distortion_coeffs = np.array([1.03432284e-02, 6.15742130e-02, 5.88511152e-03, -2.73265332e-04, -1.26407881])
    
    camera_frame_id = 'camera_link'
    object_frame_id = 'object'
    camera_topic = '/usb_cam/image_raw'
    ad = ArucoDetector(camera_topic, camera_matrix, distortion_coeffs, camera_frame_id, object_frame_id, stream_video=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()