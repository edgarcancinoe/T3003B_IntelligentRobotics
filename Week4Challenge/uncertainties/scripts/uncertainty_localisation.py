#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np
from puzzlebot_util.util import *
import tf.transformations as tft

### Node purpose: Listen to /wl and /wr topics and output estimated robot states
class Locater():
    def __init__(self, odometry_topic, inertial_frame_name, robot_frame_name, 
                 wheel_radius, track_length, starting_state, odom_rate, k_l, k_r, wl, wr):
                
        self.frame_id = inertial_frame_name
        self.child_frame_id = robot_frame_name
        
        # Robot parameters
        self.r = wheel_radius
        self.l = track_length

        # Initial state variables at inertial state
        self.sx = starting_state['x']
        self.sy = starting_state['y']
        self.stheta = starting_state['theta']

        # Uncertainty parameters
        self.k_l = k_l
        self.k_r = k_r

        # Initial covariance matrix
        self.sigma = np.ndarray(shape=(3,3))
        self.sigma.fill(0.0)
        
        # Velocities (robot frame)
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0

        self.decodematrix = np.array([[self.r / 2.0, self.r / 2.0], 
                                      [self.r / (self.l), - self.r / (self.l)]])

        # Publisher
        rospy.logwarn('Publishing to ' + odometry_topic + ' topic for estimated pose (DR odometry)')
        self.odom_publisher = rospy.Publisher(odometry_topic, Odometry, queue_size=10)

        # wl wr Subscriber
        rospy.logwarn('Subscribing to /wl and /wr topics for wheel velocities')
        wlsub = message_filters.Subscriber(wl, Float32)
        wrsub = message_filters.Subscriber(wr, Float32)

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([wlsub, wrsub], queue_size=10, slop=1.0/odom_rate, allow_headerless=True)
        ts.registerCallback(self._w_callback)
        
        # Control simulation / integration rate
        self.listening = False
        rospy.Timer(rospy.Duration(1.0/odom_rate), self.step)
        self.last_timestamp = rospy.Time.now()

    def _publishOdom(self):
        self.odom_publisher.publish(Odometry(
            header = Header(frame_id = self.frame_id, stamp = rospy.Time.now()),
            child_frame_id = self.child_frame_id,
            # Pose in inertial frame (world_frame)
            pose = PoseWithCovariance(
                pose = Pose(
                    position = Point(x = self.sx, y = self.sy, z = 0.0),
                    orientation = Quaternion(*tft.quaternion_from_euler(0.0, 0.0, self.stheta))
                ),
                covariance = np.array([self.sigma[0,0], self.sigma[0,1], 0.0, 0.0, 0.0, self.sigma[0,2],
                                       self.sigma[1,0], self.sigma[1,1], 0.0, 0.0, 0.0, self.sigma[1,2],
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       self.sigma[2,0], self.sigma[2,1], 0.0, 0.0, 0.0, self.sigma[2,2]])
            ),
            # Twist in child frame (puzzlebot)
            twist = TwistWithCovariance(
                twist = Twist(
                    linear = Vector3(x = self.v, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = self.w)
                ),
                covariance = None
            )
        ))
       
    def _w_callback(self, wl, wr):
        if self.listening:
            self.v, self.w = np.dot(self.decodematrix, np.array([wr.data, wl.data]).T).flatten()
            self.wl = wl.data
            self.wr = wr.data
            self.listening = False

    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_timestamp).to_sec()
        self.last_timestamp = current_time
        return dt
    
    def _system_gradient(self, theta, v, w):
        return np.array([v * np.cos(theta), 
                         v * np.sin(theta), 
                         w])
    
    def _rk4_delta(self, theta, v, w, dt):
        k1 = self._system_gradient(theta, v, w)
        k2 = self._system_gradient(theta + dt*k1[2]/2.0, v, w)
        k3 = self._system_gradient(theta + dt*k2[2]/2.0, v, w)
        k4 = self._system_gradient(theta + dt*k3[2], v, w)
        return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    
    def _jacobian(self, theta, v, dt):
        """
            Get the Jacobian matrix of the robot motion model
            H_k = [1, 0, -v * sin(theta) * dt]
                  [0, 1, v * cos(theta) * dt]
                  [0, 0, 1]
        """
        return np.array([[1, 0, -v * np.sin(theta) * dt],
                         [0, 1, v * np.cos(theta) * dt],
                         [0, 0, 1]])
    
    def _h(self, theta, v, w, dt):
        """
            Get predicted state at time k using the robot motion model
            h(s_{k-1}, u_k) = s_{k-1} + [v * cos(theta), v * sin(theta), w] * dt
        """
        # X is the state vector X = [x, y, theta]
        deltaX = self._rk4_delta(theta, v, w, dt)
        return np.array([self.sx + deltaX[0], self.sy + deltaX[1], self.stheta + deltaX[2]])
    
    def _noise_propagation_matrix(self, dt):
        """
            Nabla matrix used to propagate the wheel speed noise in the system
        """
        return (1/2.0) * self.r * dt * np.array([[np.cos(self.stheta), np.cos(self.stheta)],
                                                [np.sin(self.stheta), np.sin(self.stheta)],
                                                [2/self.l, -2/self.l]])
    
    def _get_pose_covariance_matrix(self, dt):
        """
            Get the process noise covariance matrix
            Q_k = nabla_k * sigma_delta_k * nabla_k^T
        """
        nabla_k = self._noise_propagation_matrix(dt)
        sigma_delta_k = np.array([[self.k_l * np.abs(self.wr), 0],
                                 [0, self.k_r * np.abs(self.wl)]])
        return nabla_k @ sigma_delta_k @ nabla_k.T
    
    def step(self, _):
        # Get dt
        dt = self._get_dt()
        # Get estimated position of the robot
        miu_k = self._h(self.stheta, self.v, self.w, dt)
        # Calculate the linearised model to be used in the uncertainty propagation
        H_k = self._jacobian(self.stheta, self.v, dt)
        # Compute Q_k
        Q_k = self._get_pose_covariance_matrix(dt)
        # Propagate the uncertainty
        self.sigma = H_k @ self.sigma @ H_k.T + Q_k

        # Update the state variables
        self.sx = miu_k[0]
        self.sy = miu_k[1]
        self.stheta = miu_k[2]

        # Publish odometry message (pose and uncertainty)
        self._publishOdom()
        
        self.listening = True


if __name__ == '__main__':
    rospy.init_node('uncertainty_localisation')

    # Get Global ROS parameters
    params = get_global_params()

    locater = Locater(odometry_topic=params['odometry_topic'], 
                      inertial_frame_name=params['inertial_frame_name'],
                      robot_frame_name=params['robot_frame_name'], 
                      wheel_radius=params['wheel_radius'],
                      track_length=params['track_length'], 
                      starting_state=params['starting_state'],
                      odom_rate=params['odom_rate'],
                      k_l=rospy.get_param('~k_l', 0.0),
                      k_r=rospy.get_param('~k_r', 0.0),
                      wl=rospy.get_param('~wl_topic'),
                      wr=rospy.get_param('~wr_topic'))

    try:
        rospy.loginfo('Localisation node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass