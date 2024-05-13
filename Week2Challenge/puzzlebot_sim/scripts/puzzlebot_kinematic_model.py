#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from puzzlebot_util.util import *
import tf.transformations as tft
class PuzzlebotKinematicModel():
    """
        This class listens to a command topic receiving [V w] instructions to move the 
        differential robot at a velocity V and angular velocity w in the robot frame.


        The necessary angular speeds for left and right wheels are computed in accordance
        with the kinematic model, and the new robot pose (numerically integrated using the RK4 method)
        and instantaneous wheel angular speeds are outputed to the corresponding topics.

    """

    def __init__(self, inertial_frame: str, 
                 x: float, y:float, theta:float, 
                 r:float, l:float, damping_factor: float,
                 pose_topic: str, wl_topic: str, wr_topic: str, commands_topic: str,
                 sim_rate: float) -> None:
        
        # Identifier
        self.frame_id = inertial_frame

        # Attributes
        self.r = r
        self.l = l

        self.damping_factor = damping_factor
        # Initialize PoseStamped object to keep track of pose (position Z is non-mutable)
        self.s: Pose = Pose(position = Point(x = x, y = y, z = 0.0), orientation = Quaternion(*tft.quaternion_from_euler(0.0, 0.0, theta)))

        # Desired velocities (inertial frame)
        self.V = 0.0
        self.w = 0.0
        
        # Control to wheel velocities matrix
        self.u2w_mat_inv = np.linalg.inv(np.array([[self.r/2.0, self.r/2.0], 
                                                   [self.r/(self.l), -self.r/(self.l)]]))
        
        # Subscriber to commands_topic
        rospy.logwarn('Subscribing to ' + commands_topic + ' topic for commands (kinematic model)')
        rospy.Subscriber(commands_topic, Twist, self.cmd_vel_callback)
    
        # Publishers
        rospy.logwarn('Publishing to ' + pose_topic + ' topic for estimated pose (kinematic model)')
        self.pose_publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
        self.w_l_publisher = rospy.Publisher(wl_topic, Float32, queue_size=10)
        self.w_r_publisher = rospy.Publisher(wr_topic, Float32, queue_size=10)
        
        # Timer to mantain simulation dt constant
        self.listening = False
        self.last_timestamp = rospy.Time.now()
        rospy.Timer(rospy.Duration(1.0/sim_rate), self.simulate)

    def _stamp(self, pose):
        # Stamp the current pose with current time
        header = Header(frame_id = self.frame_id, stamp = rospy.Time.now())
        # Create the PoseStamped object with current time and pose
        return PoseStamped(header = header, pose = pose)
    
    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_timestamp).to_sec()
        self.last_timestamp = current_time
        return dt
    
    def _state_gradient(self, theta, wr, wl):
        """ Jacobian matrix"""

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        J = np.array([[self.r * cos_theta / 2.0, self.r * cos_theta / 2.0], 
                      [self.r * sin_theta / 2.0, self.r * sin_theta / 2.0], 
                      [self.r / (self.l), -self.r / (self.l)]])
        
        return np.dot(J, np.array([wr, wl]))
    
    def cmd_vel_callback(self, msg):
        # Controlling V & w
        if self.listening:
            self.V = msg.linear.x
            self.w = msg.angular.z
            # self.listening = False
    
    def _rk_integration(self, state, wr, wl):
        # Compute dt based on elapsed time
        dt = self._get_dt()

        # Compute RK4 updates
        k1 = self._state_gradient(state[2], wr, wl)
        k2 = self._state_gradient(state[2] + dt*k1[2]/2.0, wr, wl)
        k3 = self._state_gradient(state[2] + dt*k2[2]/2.0, wr, wl)
        k4 = self._state_gradient(state[2] + dt*k3[2], wr, wl)

        # Update position and orientation using RK4
        return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        
    def simulate(self, _):

        wr, wl = np.dot(self.u2w_mat_inv, np.array([self.V, self.w]))

        # We capture the system's current state s = [x y theta]
        curr_s = np.array([self.s.position.x, self.s.position.y, tft.euler_from_quaternion([self.s.orientation.x, self.s.orientation.y, self.s.orientation.z, self.s.orientation.w])[2]])
        
        delta = self._rk_integration(curr_s, wr, wl)
        curr_s = curr_s + delta
        # print('Current orientation: ', curr_s[2], 'Delta orientation: ', delta[2])

        # Update state
        self.s = Pose(position = Point(x = curr_s[0], y = curr_s[1], z = 0.0), 
                      orientation = Quaternion(*tft.quaternion_from_euler(0.0, 0.0, curr_s[2])))

        # Publish current state
        self.pose_publisher.publish(self._stamp(self.s))
        self.w_l_publisher.publish(wl)
        self.w_r_publisher.publish(wr)


        self.listening = True

if __name__ == '__main__':
    rospy.init_node('puzzlebot_kinematic_model')

    # Get ROS parameters
    params = get_global_params()

    # Initialize class
    model = PuzzlebotKinematicModel(inertial_frame=params['inertial_frame_name'], 
                                    x=params['starting_state']['x'], 
                                    y=params['starting_state']['y'], 
                                    theta=params['starting_state']['theta'], 
                                    r=params['wheel_radius'], l=params['track_length'],
                                    damping_factor=params['damping'],
                                    pose_topic=params['pose_topic'],
                                    wl_topic=params['wl_topic'],
                                    wr_topic=params['wr_topic'],
                                    commands_topic=params['commands_topic'],
                                    sim_rate = params['kmodel_rate'])
    try:
        rospy.loginfo('Kinematic model node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass