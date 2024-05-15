#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Bool, Float32

if __name__ == '__main__':
    rospy.init_node('cmd_generator')
    rate = rospy.Rate(50)
    pub = rospy.Publisher(rospy.get_param('/commands_topic'), Twist, queue_size=10)
    print('cmd generator running')

    orientation_controller_publisher = rospy.Publisher(rospy.get_param('/orientation_controller_topic'), Float32, queue_size=10)
    orientation_controller_activate_publisher = rospy.Publisher(rospy.get_param('/orientation_control_activate_topic'), Bool, queue_size=10)

    rospy.sleep(2)
    orientation_controller_publisher.publish(Float32(np.pi))
    rospy.sleep(3)
    radius = .5
    t = 0
    v_0 = 0.2
    try:
        while not rospy.is_shutdown():
            

            v = v_0 * np.abs(np.sin(t))
            v = v_0
            omega = v / radius         
            vel_msg = Twist()
            
            vel_msg.linear.x = v
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = omega
            
            pub.publish(vel_msg)
            rate.sleep()
            t = t + 1/100
    except rospy.ROSInterruptException:
        pass
