#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from puzzlebot_util.util import get_global_params

class PuzzlebotKinematicModel():
    """
        This class listens to a command topic receiving [V w] instructions to move the 
        differential robot at a velocity V and angular velocity w in the robot frame.


        The necessary angular speeds for left and right wheels are computed in accordance
        with the kinematic model, and the new robot pose (numerically integrated using the RK4 method)
        and instantaneous wheel angular speeds are outputed to the corresponding topics.

    """

    def __init__(self, frame_id: str, x: float, y:float, theta:float, 
                 r:float, l:float, pose_topic: str, wl_topic: str, wr_topic: str) -> None:
        
        # Identifier
        self.frame_id = frame_id

        # Frequency
        self.prev_time = rospy.Time.now()

        # Attributes
        self.r = r
        self.l = l

        # Initialize PoseStamped object to keep track of pose (position Z is non-mutable)
        self.s: Pose = Pose(position = Point(x = x, y = y, z = 0.0), orientation = Quaternion(x = 0.0, y = 0.0, z = theta, w = 1.0))

        # Desired velocities (inertial frame)
        self.V = 0.0
        self.w = 0.0
        
        # Control to wheel velocities matrix
        self.u2w_mat_inv = np.linalg.inv(np.array([[self.r/2.0, self.r/2.0], [self.r/(2.0 * self.l), -self.r/(2.0 * self.l)]]))

        # Publishers
        self.pose_publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
        self.w_l_publisher = rospy.Publisher(wl_topic, Float32, queue_size=10)
        self.w_r_publisher = rospy.Publisher(wr_topic, Float32, queue_size=10)

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
    
    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        return dt
    
    def _state_gradient(self, theta, wr, wl):
        """ Jacobian matrix"""
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        J = np.array([[self.r * cos_theta / 2.0, self.r * cos_theta / 2.0], 
                      [self.r * sin_theta / 2.0, self.r * sin_theta / 2.0], 
                      [self.r / (2.0 * self.l), -self.r / (2.0 * self.l)]])
        
        return np.dot(J, np.array([wr, wl]))
    
    def cmd_vel_callback(self, msg):
        # Controlling V & w
        self.V = msg.linear.x
        self.w = msg.angular.z
    
    def _rk_integration(self, dt, state, wr, wl):
        # Compute RK4 updates
        k1 = self._state_gradient(state[2], wr, wl)
        k2 = self._state_gradient(state[2] + dt*k1[2]/2.0, wr, wl)
        k3 = self._state_gradient(state[2] + dt*k2[2]/2.0, wr, wl)
        k4 = self._state_gradient(state[2] + dt*k3[2], wr, wl)

        # Update position and orientation using RK4
        return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        
    def step(self):
        # Compute dt based on elapsed time
        dt = self._get_dt()

        wr, wl = np.dot(self.u2w_mat_inv, np.array([self.V, self.w]))

        # We capture the system's current state s = [x y theta]
        curr_s = np.array([self.s.position.x, self.s.position.y, self.s.orientation.z])
        
        curr_s = curr_s + self._rk_integration(dt, curr_s, wr, wl)
        
        # Update state
        self.s = Pose(position = Point(x = curr_s[0], y = curr_s[1], z = 0.0), orientation = Quaternion(x = 0.0, y = 0.0, z = self._wrap_to_Pi(curr_s[2]), w = 1.0))

        # Publish current state
        self.pose_publisher.publish(self._stamp(self.s))
        self.w_l_publisher.publish(wl)
        self.w_r_publisher.publish(wr)

        # Reset velocities
        self.V = 0.0
        self.w = 0.0

if __name__ == '__main__':
    rospy.init_node('puzzlebot_kinematic_model')

    # Get ROS parameters

    # Global
    params = get_global_params
    # Local
    s_0 = rospy.get_param('~starting_state', {'x': 0.0, 'y': 0.0, 'theta': 0.0})

    # Initialize class
    model = PuzzlebotKinematicModel(frame_id=params['world_frame_name'], 
                                    x=s_0['x'], 
                                    y=s_0['y'], 
                                    theta=s_0['theta'], 
                                    r=params['radius'], l=params['track'],
                                    pose_topic=params['pose_topic'],
                                    wl_topic=params['wl_topic'],
                                    wr_topic=params['wr_topic'])

    # Susbcribe to commands and use model class' method
    rospy.Subscriber(params['commands_topic'], Twist, model.cmd_vel_callback)

    rospy.loginfo('Kinematic model running')
    try:
        loop_rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            model.step()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass