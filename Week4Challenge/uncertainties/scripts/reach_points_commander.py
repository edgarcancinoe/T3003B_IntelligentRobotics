#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import Bool, Float32
import tf.transformations as tft
if __name__ == '__main__':

    rospy.init_node('reach_points_commander')
    
    rate = rospy.Rate(50)

    pose_controller_topic = rospy.get_param('/pose_controller_topic')
    pose_controller_activate_topic = rospy.get_param('/pose_control_activate_topic')
    pose_controller_publisher = rospy.Publisher(pose_controller_topic, Pose, queue_size=10)
    pose_controller_activate_publisher = rospy.Publisher(pose_controller_activate_topic, Bool, queue_size=10)
    
    orientation_controller_topic = rospy.get_param('/orientation_controller_topic')
    orientation_controller_activate_topic = rospy.get_param('/orientation_control_activate_topic')
    orientation_controller_publisher = rospy.Publisher(orientation_controller_topic, Float32, queue_size=10)
    orientation_controller_activate_publisher = rospy.Publisher(orientation_controller_activate_topic, Bool, queue_size=10)

    unlock_topic = rospy.get_param('/unlock_topic')
    unlocked = True
    def unlock_callback(msg):
        global unlocked
        unlocked = msg.data

    unlock_suscriber = rospy.Subscriber(unlock_topic, Bool, unlock_callback)


    waypoints = [[[0.5, 0.0, 0.0], 0.0],
                 [[0.5, 0.5, 0.0], np.pi/2],
                 [[0.0, 0.5, 0.0], np.pi],
                 [[0.0, 0.0, 0.0], 3*np.pi/2]]
    
    try:
        rospy.sleep(3)
        while not rospy.is_shutdown():
            print('Attempting to perform waypoints trajectory')
            for point, yaw in waypoints:
                print(point, yaw)
                unlocked = False
                orientation_controller_activate_publisher.publish(Bool(True))
                orientation_controller_publisher.publish(Float32(yaw)) # In range [0, 2*pi]
                
                while not unlocked:
                    if rospy.is_shutdown():
                        break
                    pass
                if rospy.is_shutdown():
                    break
                orientation_controller_activate_publisher.publish(Bool(False))

                rospy.sleep(1)
                unlocked = False
                pose_controller_activate_publisher.publish(Bool(True))
                quat = tft.quaternion_from_euler(0.0, 0.0, yaw)
                pose = Pose(position=Point(*point), orientation=Quaternion(*quat))

                pose_controller_publisher.publish(pose)

                while not unlocked:
                    if rospy.is_shutdown():
                        break
                    pass
                if rospy.is_shutdown():
                    break

                rospy.sleep(1)
                pose_controller_activate_publisher.publish(Bool(False))

    except rospy.ROSInterruptException:
        pass
