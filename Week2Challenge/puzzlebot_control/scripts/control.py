#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry

class Puzzlebot_controller():
    def __init__(self, s_d, kp_V, kp_w, d_tolerance=0.01, deg_tolerance = 1.5):

        # Class attributes
        self.kp_l = kp_V
        self.kp_w = kp_w
        self.s_d : Pose = s_d

        # Initial state
        self.s : Pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), 
                             orientation= Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        self.w_error_tolerance : float = (deg_tolerance*(1/360)) * (2 * np.pi) 
        self.l_error_tolerance : float = d_tolerance
        
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        
    def odometry_callback(self, msg):
        # rospy.loginfo(f'x= {msg.pose.pose.position.x}, y= {msg.pose.pose.position.y}')
        # rospy.loginfo(f'theta ={msg.pose.pose.orientation.z}')
        self.s = msg.pose.pose        
    
    def _compute_errors(self):
        e_x = self.s_d.position.x - self.s.position.x
        e_y = self.s_d.position.y - self.s.position.y
        e_l = np.sqrt((e_x * e_x) + (e_y * e_y))
        w_d = np.arctan2(e_y, e_x)
        e_w = w_d - self.s.orientation.z

        return e_l, e_w
    
    def compute_output(self):
        e_l, e_w, = self._compute_errors()
        twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        if abs(e_w) > self.w_error_tolerance:
            twist_msg.angular.z = self.kp_w * e_w
        elif abs(e_l) > self.l_error_tolerance:
            twist_msg.linear.x = self.kp_l * e_l

        self.cmd_vel_publisher.publish(twist_msg)
    


freq = 100
if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("control")

    # Control Parameter
    kp_V = 0.4
    kp_w = 1.5

    goal_position : Pose = Pose(position = Point(x = 7.0, y = 3.0, z = 0.0),
                       orientation = Quaternion(x = 0.0, y = 0.0, z = np.pi/2.0, w = 1.0))

    puzzlebot_controller = Puzzlebot_controller(goal_position, kp_V, kp_w)

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", freq))

    # Setup the Subscribers
    odom_sub = rospy.Subscriber("/odom", Odometry, puzzlebot_controller.odometry_callback)
    
    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            puzzlebot_controller.compute_output()
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node