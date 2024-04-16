#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np

### Node purpose: Listen to /wl and /wr topics and output estimated robot states

class Locater():
    def __init__(self):
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.prev_time = rospy.Time.now()
        self.frame_id = 'world_frame'

        # Robot parameters
        self.r = .05
        self.l = 0.19

        # State variables at initial state
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0
        self.sx = 0.0
        self.sy = 0.0
        self.stheta = 0.0

        self.decodematrix = np.array([[self.r / 2.0, self.r / 2.0], 
                                      [self.r / (2*self.l), - self.r / (2*self.l)]])
    
    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        return dt
    
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
            child_frame_id = 'puzzlebot',
            # Pose in inertial frame (world_frame)
            pose = PoseWithCovariance(
                pose = Pose(
                    position = Point(x = self.sx, y = self.sy, z = 0.0),
                    orientation = Quaternion(x = 0.0, y = 0.0, z = self._wrap_to_Pi(self.stheta), w = 1.0)
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

    def w_callback(self, wl, wr):
        self.v, self.w = np.dot(self.decodematrix, np.array([wr.data, wl.data]).T).flatten()
        self.wl = wl.data
        self.wr = wr.data

    def step(self):

        dt = self._get_dt()

        delta = self._rk4_delta(dt, self.stheta, self.v, self.w)

        self.sx += delta[0]
        self.sy += delta[1]
        self.stheta += delta[2]

        self._publishOdom()
        self.v = 0.0
        self.w = 0.0

freq = 100
slop = 0.1
if __name__ == '__main__':
    rospy.init_node('localisation')
    loop_rate = rospy.Rate(rospy.get_param('~node_rate', freq))

    locater = Locater()

    wlsub = message_filters.Subscriber('wl', Float32)
    wrsub = message_filters.Subscriber('wr', Float32)

    # Synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([wlsub, wrsub], queue_size=10, slop=slop, allow_headerless=True)
    ts.registerCallback(locater.w_callback)

    try:
        while not rospy.is_shutdown():
            locater.step()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass