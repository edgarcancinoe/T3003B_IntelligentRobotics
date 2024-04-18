#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry

class Puzzlebot_controller():
    def __init__(self, s_d, kp_V, kp_w, d_tolerance=0.0025, deg_tolerance = .25):

        # Class attributes
        self.kp_l = kp_V
        self.kp_w = kp_w
        self.s_d : Point = s_d

        # Initial state
        self.s : Pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), 
                             orientation= Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        self.w_error_tolerance : float = (deg_tolerance*(1/360)) * (2 * np.pi) 
        self.l_error_tolerance : float = d_tolerance
        
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    def _compute_errors(self):
        e_x = self.s_d.x - self.s.position.x
        e_y = self.s_d.y - self.s.position.y
        e_l = np.sqrt((e_x * e_x) + (e_y * e_y))
        w_d = np.arctan2(e_y, e_x)
        e_w = w_d - self.s.orientation.z

        return e_l, e_w
    
    def odometry_callback(self, msg):
        self.s = msg.pose.pose        
    
    def set_goal(self, goal):
        self.s_d = goal

    def compute_output(self):
        e_l, e_w, = self._compute_errors()
        twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        if abs(e_w) > self.w_error_tolerance:
            twist_msg.angular.z = self.kp_w * e_w
        elif abs(e_l) > self.l_error_tolerance:
            twist_msg.linear.x = self.kp_l * e_l
        else:
            return True
        self.cmd_vel_publisher.publish(twist_msg)
        return False
    


freq = 100
goals = [Point(x = 1.0, y = 0.0, z = 0.0),
         Point(x = 1.0, y = 1.0, z = 0.0),
         Point(x = 0.0, y = 1.0, z = 0.0),
         Point(x = 0.0, y = 0.0, z = 0.0)]
i = 0
if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("control")

    # Control Parameter
    kp_V = 0.45
    kp_w = 0.3

    goal_position : Point = goals[i]

    puzzlebot_controller = Puzzlebot_controller(goal_position, kp_V, kp_w)

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", freq))

    # Setup the Subscribers
    odom_sub = rospy.Subscriber("/odom", Odometry, puzzlebot_controller.odometry_callback)
    
    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            reached_goal = puzzlebot_controller.compute_output()

            if reached_goal:
                i += 1
                i = i % 4
                puzzlebot_controller.set_goal(goals[i])

            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node