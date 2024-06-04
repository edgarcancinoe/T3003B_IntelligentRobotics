#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, Twist, PoseWithCovariance, TwistWithCovariance, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from real_robot_util.util import get_kalman_localisation_params
import tf.transformations as tft
import yaml

### Node purpose: Listen to /wl and /wr topics and output estimated robot states
class Locater():
    def __init__(self, map_topic, map_frame, robot_frame_name, 
                 wheel_radius, track_length, starting_state, map_path, odom_rate, k_l, k_r, wl, wr):
                         
        self.map_frame = map_frame

        self.child_frame_id = robot_frame_name
        
        # Robot parameters
        self.r = wheel_radius
        self.l = track_length

        # Initial state variables at inertial state
        self.mu = [starting_state['x'], starting_state['y'], starting_state['theta']]

        # Uncertainty parameters
        self.k_l = k_l
        self.k_r = k_r

        # Initial covariance matrix
        self.sigma = np.zeros((3, 3))

        # Velocities (robot frame)
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0

        self.decodematrix = np.array([[self.r / 2.0, self.r / 2.0], 
                                      [self.r / (self.l), - self.r / (self.l)]])
        
        # Kalman filter parameters
        self.landmarks = self.load_map(map_path)

        # Publisher
        rospy.logwarn('Publishing to ' + map_topic + ' topic for estimated pose (DR odometry)')
        self.map_publisher = rospy.Publisher(map_topic, Odometry, queue_size=10)

        # wl wr Subscriber
        rospy.logwarn('Subscribing to /wl and /wr topics for wheel velocities')
        wlsub = message_filters.Subscriber(wl, Float32)
        wrsub = message_filters.Subscriber(wr, Float32)
        
        # Landmark detection managment
        self.landmark_suscriber = rospy.Subscriber('/detected_landmarks', PoseArray, self._landmark_detection_callback)
        self.detected_landmarks = {}
        
        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([wlsub, wrsub], queue_size=10, slop=1.0/odom_rate, allow_headerless=True)
        ts.registerCallback(self._w_callback)
        
        # Control simulation / integration rate
        self.listening = False
        rospy.Timer(rospy.Duration(1.0/odom_rate), self.step)
        self.last_timestamp = rospy.Time.now()

    def _landmark_detection_callback(self, detecection_array):
        self.detected_landmarks = {}
        for i, detection in enumerate(detecection_array.poses):
            # Find which landmark is the closest to the detection
            min_distance = float('inf')
            closest_landmark = None
            for j, landmark in enumerate(self.landmarks):
                distance = np.linalg.norm([detection.position.x - landmark[0], detection.position.y - landmark[1]])
                if distance < min_distance:
                    min_distance = distance
                    closest_landmark = j
            self.detected_landmarks[closest_landmark] = detection
    
    def _publishMap(self):
        self.map_publisher.publish(Odometry(
            header = Header(frame_id = self.map_frame, stamp = rospy.Time.now()),
            child_frame_id = self.child_frame_id,
            # Pose in inertial frame (world_frame)
            pose = PoseWithCovariance(
                pose = Pose(
                    position = Point(x = self.mu[0], y = self.mu[1], z = 0.0),
                    orientation = Quaternion(*tft.quaternion_from_euler(0.0, 0.0, self.mu[2]))
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
        return np.array([v * np.cos(theta), v * np.sin(theta), w])
    
    def _rk4_delta(self, theta, v, w, dt):
        k1 = self._system_gradient(theta, v, w)
        k2 = self._system_gradient(theta + dt*k1[2]/2.0, v, w)
        k3 = self._system_gradient(theta + dt*k2[2]/2.0, v, w)
        k4 = self._system_gradient(theta + dt*k3[2], v, w)
        return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    
    def _jacobianH(self, theta, v, dt):
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
        return self.mu + deltaX

    def _noise_propagation_matrix(self, dt):
        """
            Nabla matrix used to propagate the wheel speed noise in the system
        """
        return (1/2.0) * self.r * dt * np.array([[np.cos(self.mu[2]), np.cos(self.mu[2])],
                                                [np.sin(self.mu[2]), np.sin(self.mu[2])],
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
    
    def _g(self, dx, dy, theta):
        """
        Observation model. [rho, alpha] is the measurement vector that consists of landmark angle and bearing in robot frame.
        g(x, y, theta, landmark) = [rho, alpha].T
        returns [g1, g2, ... , gN] = [sqrt(rho1), alpha1, sqrt(rho2), alpha2, ... , sqrt(rhoN), alphaN]
        """
        rho = np.sqrt(dx**2 + dy**2)
        alpha = np.arctan2(dy, dx) - theta
        return np.vstack((rho, alpha)).T.flatten()
    
    def _jacobianG(self, dx, dy, observation_model):
        """
        Get the Jacobian matrix of the observation model
        G_i_k = [-dx/rho, -dy/rho, 0]
                [dy/rho^2, -dx/rho^2, -1]

        Inputs:
        - dx
        - dy
        - observation model vector [sqrt_rho_1, alpha_1, ... , sqrt_rho_n, alpha_n]
        """
        rho = observation_model[::2]  # Extract rho values from the observation model
        Gk = np.zeros((2 * len(dx), 3))
        
        Gk[::2, 0] = -dx / rho
        Gk[::2, 1] = -dy / rho
        Gk[::2, 2] = 0
        
        Gk[1::2, 0] = dy / (rho**2)
        Gk[1::2, 1] = -dx / (rho**2)
        Gk[1::2, 2] = -1
        
        return Gk

    def step(self, _):
        # Define useful matrices based on detected landmarks
        landmarks_on_sight = list(self.detected_landmarks.keys())
        landmarks_on_sight_map_positions = self.landmarks[landmarks_on_sight]
        landmarks_on_sight_measured_positions = np.array([[landmark.position.x, landmark.position.y] for landmark in self.detected_landmarks.values()])
        
        # Get dt
        dt = self._get_dt()

        # No landmark detection, usual odometry

        # Get estimated position of the robot
        mu_hat = self._h(self.mu[2], self.v, self.w, dt)
        # Calculate the linearised model to be used in the uncertainty propagation
        H_k = self._jacobianH(self.mu[2], self.v, dt)
        # Compute Q_k
        Q_k = self._get_pose_covariance_matrix(dt)
        # Propagate the uncertainty
        sigma_hat = H_k @ self.sigma @ H_k.T + Q_k
    
        if len(landmarks_on_sight_measured_positions) > 0:
            z_k = np.array([np.linalg.norm(landmarks_on_sight_measured_positions - np.array([self.mu[0], self.mu[1]]), axis=1),
                            np.arctan2(landmarks_on_sight_measured_positions[:,1] - self.mu[1], landmarks_on_sight_measured_positions[:,0] - self.mu[0]) - self.mu[2]]).T.flatten()
            
            # Robot observation model
            dx = landmarks_on_sight_map_positions[:,0] - mu_hat[0]
            dy = landmarks_on_sight_map_positions[:,1] - mu_hat[1]
            z_hat_k = self._g(dx.T, dy.T, mu_hat[2])

            if z_k.shape != z_hat_k.shape:
                rospy.logerr('Skipping because z_k and z_hat_k have different shapes.')
                return

            # Compute the Jacobian matrix of the observation model G_k
            G_k = self._jacobianG(dx, dy, z_hat_k) # (8x3)
            # Zk: Uncertainty propagation
            # (8x3)(3x3) (3x8) = (3,8)
            Z_k = G_k @ sigma_hat @ G_k.T + np.eye(2*len(landmarks_on_sight)) * .25 
            # K_k: Kalman Gain
            K_k = sigma_hat @ G_k.T @ np.linalg.pinv(Z_k) # (3,8)
            
            # Update the state variables
            # (3x1) = (3x1) + (3,8)(8,1)
            self.mu = mu_hat + K_k @ (z_k - z_hat_k)
            self.sigma = (np.eye(3) - K_k @ G_k) @ sigma_hat  
            # print(np.linalg.norm(self.sigma))
        else:
            self.mu = mu_hat
            self.sigma = sigma_hat

        # Publish odometry message (pose and uncertainty)
        self._publishMap()

        self.listening = True

    def load_map(self, map_path):
        """
            Load a yaml file containing the landmarks' x and y positions.
            Return a list of landmark tuples (x, y)  
        """
        with open(map_path, 'r') as file:
            landmarks = yaml.load(file, Loader=yaml.FullLoader)
        landmark_positions = []
        for name, data in landmarks.items():
            if name != 'names':
                x = data['x']
                y = data['y']
                landmark_positions.append((x, y))
        return np.array(landmark_positions)
    
if __name__ == '__main__':
    rospy.init_node('kalman_localisation')

    # Get Global ROS parameters
    params = get_kalman_localisation_params()

    locater = Locater(map_topic = params['odometry_topic'], 
                        map_frame = params['inertial_frame'],
                        robot_frame_name = params['robot_frame'],
                        wheel_radius = params['wheel_radius'],
                        track_length = params['track_length'],
                        starting_state = params['starting_state'],
                        map_path = params['map_path'],
                        odom_rate = params['control_rate'],
                        k_l = params['k_l'],
                        k_r = params['k_r'],
                        wl = params['wl_topic'],
                        wr = params['wr_topic'])

    try:
        rospy.loginfo('Localisation node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass