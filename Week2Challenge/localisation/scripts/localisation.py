#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np
from puzzlebot_util.util import *

### Node purpose: Listen to /wl and /wr topics and output estimated robot states
class Locater():
    def __init__(self, odometry_topic, inertial_frame_name, robot_frame_name, 
                 wheel_radius, track_length, starting_state, odom_rate):
                
        self.frame_id = inertial_frame_name
        self.child_frame_id = robot_frame_name
        
        # Robot parameters
        self.r = wheel_radius
        self.l = track_length

        # Initial state variables at inertial state
        self.sx = starting_state['x']
        self.sy = starting_state['y']
        self.stheta = starting_state['theta']

        # Velocities (robot frame)
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0
        
        self.decodematrix = np.array([[self.r / 2.0, self.r / 2.0], 
                                      [self.r / (2*self.l), - self.r / (2*self.l)]])

        # Publisher
        self.odom_publisher = rospy.Publisher('/' + odometry_topic, Odometry, queue_size=10)

        # wl wr Suscriber
        wlsub = message_filters.Subscriber(params['wl_topic'], Float32)
        wrsub = message_filters.Subscriber(params['wr_topic'], Float32)
        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([wlsub, wrsub], queue_size=10, slop=1.0/odom_rate, allow_headerless=True)
        ts.registerCallback(self._w_callback)
        
        # Control simulation / integration rate
        self.listening = False
        rospy.Timer(rospy.Duration(1.0/odom_rate), self.step)
        self.last_timestamp = rospy.Time.now()

    def _system_gradient(self, theta, dd, dtheta):
        return np.array([dd * np.cos(theta), dd * np.sin(theta), dtheta])
    
    def _rk4_delta(self, dt, theta, dd, dtheta):
        k1 = self._system_gradient(theta, dd, dtheta)
        k2 = self._system_gradient(theta + dt*k1[2]/2.0, dd, dtheta)
        k3 = self._system_gradient(theta + dt*k2[2]/2.0, dd, dtheta)
        k4 = self._system_gradient(theta + dt*k3[2], dd, dtheta)
        return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    
    def _publishOdom(self):
        self.odom_publisher.publish(Odometry(
            header = Header(frame_id = self.frame_id, stamp = rospy.Time.now()),
            child_frame_id = self.child_frame_id,
            # Pose in inertial frame (world_frame)
            pose = PoseWithCovariance(
                pose = Pose(
                    position = Point(x = self.sx, y = self.sy, z = 0.0),
                    orientation = Quaternion(x = 0.0, y = 0.0, z = wrap_to_Pi(self.stheta), w = 1.0)
                ),
                covariance = None
            ),
            # Twist in child frame (puzzlebot)
            twist = TwistWithCovariance(
                twist = Twist(
                    linear = Vector3(x = self.r * self.wl, y = self.r * self.wr, z = 0.0),
                    angular = Vector3(x = self.wl, y = self.wr, z = 0.0)
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
    

    def step(self, _):
        
        dt = self._get_dt()

        # Integration
        delta = self._rk4_delta(dt, self.stheta, self.v, self.w)
        self.sx += delta[0]
        self.sy += delta[1]
        self.stheta += delta[2]

        # Publish new state data
        self._publishOdom()
        
        self.listening = True


if __name__ == '__main__':
    rospy.init_node('localisation')

    # Get Global ROS parameters
    params = get_global_params()

    locater = Locater(odometry_topic=params['odometry_topic'], inertial_frame_name=params['inertial_frame_name'],
                      robot_frame_name=params['robot_frame_name'], wheel_radius=params['wheel_radius'],
                      track_length=params['track_length'], odom_rate=params['odom_rate'], starting_state=params['starting_state'])

    try:
        rospy.loginfo('Localisation node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass