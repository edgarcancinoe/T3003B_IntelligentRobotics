#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion

class PuzzlebotKinematicModel():
    def __init__(self, x = 0.0, y = 0.0, theta = 0.0, r = .05, l = 0.10) -> None:
        # Identifier
        self.frame_id = 'puzzlebot'

        # Frequency
        self.prev_time = rospy.Time.now()

        # Attributes
        self.r = r
        self.l = l

        # Initialize PoseStamped object to keep track of pose
        self.s: Pose = Pose()

        # Initial position Point object
        initial_position: Point = Point()
        initial_position.x = x
        initial_position.y = y
        initial_position.z = 0.0 # Z is non-mutable

        # Initial orientation Quaternion object (we are not using quaternion but x,y,z,1 euler rotation)
        initial_orientation: Quaternion = Quaternion()
        initial_orientation.x = 0.0
        initial_orientation.y = 0.0
        initial_orientation.z = theta # Theta
        initial_orientation.w = 1.0

        # Current state
        self.s.position = initial_position
        self.s.orientation = initial_orientation

        # Linear velocities (robot frame)
        self.v = 0.0
        self.v_l = 0.0
        self.v_r = 0.0
        
        # Angular velocities (robot frame)
        self.w_l = 0.0
        self.w_r = 0.0

        # Publishers
        self.pose_publisher = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        self.w_l_publisher = rospy.Publisher('/wl', Float32, queue_size=10)
        self.w_r_publisher = rospy.Publisher('/wr', Float32, queue_size=10)

    def cmd_vel_callback(self, msg):
        # Option 1: Controlling v_l & v_r
        self.v_l = msg.linear.x
        self.v_r = msg.linear.y
        # Option 2: Controlling w_l & w_r
        self.w_l = msg.angular.x
        self.w_r = msg.angular.y

    def step(self):

        # Compute dt based on elapsed time
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time


        # Option 1: Controlling v_l & v_r
        v = (self.v_r + self.v_l) / 2.0
        theta_dot = (self.v_r - self.v_l) / self.l
        self.w_l = self.r * self.v_l
        self.w_r = self.r * self.v_r   

        # Option 2: Controlling w_l & w_r
        # v = self.r * (self.w_r + self.w_l) / 2.0
        # theta_dot = self.r * (self.w_r - self.w_l) / self.l

        x_dot = v * np.cos(self.s.orientation.z)
        y_dot = v * np.sin(self.s.orientation.z)
        
        # Euler numeric integration
        self.s.position.x = self.s.position.x + x_dot * dt
        self.s.position.y = self.s.position.y + y_dot * dt
        self.s.orientation.z = self._wrap_to_Pi(self.s.orientation.z + theta_dot * dt)
        
        # Publish current state
        self.pose_publisher.publish(self._stamp(self.s))
        self.w_l_publisher.publish(self.w_l)
        self.w_r_publisher.publish(self.w_r)
        self.v_l = 0.0
        self.v_r = 0.0
        self.w_l = 0.0
        self.w_r = 0.0

    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
        
    def _stamp(self, pose):
        # Stamp the current pose with current time
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = rospy.Time.now()
        # Create the PoseStamped object with current time and pose
        poseStamped = PoseStamped()
        poseStamped.header = header
        poseStamped.pose = pose

        return poseStamped

freq = 100
if __name__ == '__main__':
    rospy.init_node('puzzlebot_kinematic_model')
    loop_rate = rospy.Rate(rospy.get_param('~node_rate', freq))

    model = PuzzlebotKinematicModel()

    rospy.Subscriber('cmd_vel', Twist, model.cmd_vel_callback)

    print('Node running')

    try:
        while not rospy.is_shutdown():
            model.step()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass