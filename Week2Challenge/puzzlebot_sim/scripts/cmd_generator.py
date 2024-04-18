#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    rospy.init_node('cmd_generator')
    rate = rospy.Rate(rospy.get_param('~node_rate', 100))
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    print('cmd generator running')
    
    radius = .6
    t = 0
    v_0 = 0.5 
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
