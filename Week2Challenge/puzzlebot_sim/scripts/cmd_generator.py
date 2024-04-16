#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist

def circle_motion(radius, angular_speed):
    # Assuming radius is the radius of the circle and angular_speed is the desired angular speed
    # Calculating linear velocities for the wheels using the formula v = ω * r
    left_velocity = angular_speed * (radius - 0.5)  # Adjusted for left wheel
    right_velocity = angular_speed * (radius + 0.5)  # Adjusted for right wheel
    return left_velocity, right_velocity

if __name__ == '__main__':
    rospy.init_node('cmd_generator')
    rate = rospy.Rate(rospy.get_param('~node_rate', 100))
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    print('cmd generator running')
    
    radius = 1.0  # Adjust the radius of the circle as needed
    angular_speed = 1.0  # Adjust the angular speed as needed
    
    try:
        while not rospy.is_shutdown():
            left_vel, right_vel = circle_motion(radius, angular_speed)
            
            twist_msg = Twist()
            twist_msg.linear.x = left_vel
            twist_msg.linear.y = right_vel
            twist_msg.angular.z = 0.0
            
            pub.publish(twist_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
