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

        # Initialize PoseStamped object to keep track of pose (position Z is non-mutable)
        self.s: Pose = Pose(position = Point(x = x, y = y, z = 0.0), orientation = Quaternion(x = 0.0, y = 0.0, z = theta, w = 1.0))

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
        w = (self.v_r - self.v_l) / self.l
        self.w_l = self.r * self.v_l
        self.w_r = self.r * self.v_r   

        # Option 2: Controlling w_l & w_r
        # v = self.r * (self.w_r + self.w_l) / 2.0
        # theta_dot = self.r * (self.w_r - self.w_l) / self.l

        # We capture the system's current state s = [x y theta]
        s = np.array([self.s.position.x, self.s.position.y, self.s.orientation.z])
        # Compute RK4 updates
        k1 = self._state_gradient(s, v, w)
        k2 = self._state_gradient(s + dt*k1/2.0, v, w)
        k3 = self._state_gradient(s + dt*k2/2.0, v, w)
        k4 = self._state_gradient(s + dt*k3, v, w)

        # Update position and orientation using RK4
        delta_pose = dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        s = s + delta_pose
        
        # Update s
        self.s = Pose(position = Point(x = s[0], y = s[1], z = 0.0), orientation = Quaternion(x = 0.0, y = 0.0, z = s[2], w = 1.0))

        # Publish current state
        self.pose_publisher.publish(self._stamp(self.s))
        self.w_l_publisher.publish(self.w_l)
        self.w_r_publisher.publish(self.w_r)

        # Reset velocities
        self.v_l = 0.0
        self.v_r = 0.0
        self.w_l = 0.0
        self.w_r = 0.0

    def _state_gradient(self, state, v, w):
        theta = state[2] # Get theta out of state np.array
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = w
        return np.array([x_dot, y_dot, theta_dot])

    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
        
    def _stamp(self, pose):
        # Stamp the current pose with current time
        header = Header(frame_id = self.frame_id, stamp = rospy.Time.now())
        # Create the PoseStamped object with current time and pose
        return PoseStamped(header = header, pose = pose)

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