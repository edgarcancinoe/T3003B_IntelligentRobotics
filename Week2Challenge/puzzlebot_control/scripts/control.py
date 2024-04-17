#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry

class Puzzlebot_control():
    def __init__(self,freq, s_d, Kp_l, Kp_w, grad=2.0,e_l=0.0, e_w=0.0):
        #Atributes
        self.Kp_l = Kp_l
        self.Kp_w = Kp_w
        self.s_d : Pose = s_d

        self.dt = freq

        self.s : Pose = Pose(position=Point(x=0.0,y=0.0,z=0.0),orientation= Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
        self.tol_e : Float32 = (grad*(1/360)) * (2 * np.pi) 

        self.twist_msg : Twist = Twist()

        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x= 0.0
        self.twist_msg.angular.y= 0.0
        self.twist_msg.angular.z= 0.0
        

        #Publishers
        self.vel_cmd_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        
        
    def odometry_callback(self, msg):
        rospy.loginfo(f'x= {msg.pose.pose.position.x}, y= {msg.pose.pose.position.y}')
        rospy.loginfo(f'theta ={msg.pose.pose.orientation.z}')
        self.s.position = msg.pose.pose.position
        self.s.orientation = msg.pose.pose.orientation
    
    def error(self):
        e_x = self.s_d.position.x - self.s.position.x
        e_y = self.s_d.position.y - self.s.position.y

        self.e_l = np.sqrt((e_x * e_x) + (e_y) * (e_y))
        self.e_w = np.arctan2(e_x,e_y)

    def calculate_vel(self):
        self.error()
        if(self.e_w > self.tol_e):
            self.twist_msg.angular.z = self.Kp_w * self.e_w
        else:
            self.twist_msg.linear.x = self.Kp_l * self.e_l
        self.vel_cmd_publisher.publish(self.twist_msg)
    


freq = 100
if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("control")

    #Control Parameter
    Kp_l = 1
    Kp_w = 1.5
    s_d : Pose = Pose()
    #Initial Position
    set_position : Point = Point()
    set_position.x = 5.0
    set_position.y = 3.0
    set_position.z = 0.0 #Always is 0

    #Initial orientention
    set_orientation : Quaternion = Quaternion()
    set_orientation.x = 0.0
    set_orientation.y = 0.0
    set_orientation.z = np.pi
    set_orientation.w = 1.0

    s_d.position = set_position
    s_d.orientation = set_orientation

    control_puzzlebot = Puzzlebot_control(freq, s_d, Kp_l, Kp_w)

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",freq))

    # Setup the Subscribers
    odom_sub = rospy.Subscriber("/odom", Odometry, control_puzzlebot.odometry_callback)
    
    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        while not rospy.is_shutdown():
            control_puzzlebot.calculate_vel()

            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node