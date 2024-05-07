#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, Polygon, PointStamped, Point, Vector3, Pose
from nav_msgs.msg import Odometry
import tf
import time

class Navigator:
    def __init__(self,  navigator_rate: float, 
                        inertial_frame_id, 
                        object_frame_id, 
                        target_detection_topic: str, 
                        ibvs_activate_topic: str,
                        pose_controller_activate_topic: str,
                        pose_controller_topic: str,
                        cmd_vel_topic: str,
                        odom_topic: str,
                        unlock_topic: str):
        
        # Frames
        self.inertial_frame_id = inertial_frame_id
        self.object_frame_id = object_frame_id

        # Initialize the control publishers
        self.ibvs_commander = rospy.Publisher(ibvs_activate_topic, Bool, queue_size=10)
        self.pose_controller_activate_publisher = rospy.Publisher(pose_controller_activate_topic, Bool, queue_size=10)
        self.pose_controller_publisher = rospy.Publisher(pose_controller_topic, Point, queue_size=10)
        self.reset_vision_pub = rospy.Publisher('/reset_vision', Bool, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # State
        self.state = 'NO_TARGET_DETECTED'
        self.busy = False
        self.s : Pose = None
        self.target_lost = True

        # Class variables
        alignment_offset = 0.6
        self.alignment_position = [0.0, 0.0, alignment_offset]
        self.alignment_position_orientate = [0.0, 0.0, alignment_offset - 0.02]

        # Initialize the subscribers
        rospy.Subscriber(target_detection_topic, Polygon, self._target_detection_callback)
        rospy.Subscriber(unlock_topic, Bool, self._unlock_callback)
        rospy.Subscriber(odom_topic, Odometry, self._odom_callback)
        # Set timer
        rospy.Timer(rospy.Duration(1.0/navigator_rate), self.run)

    def _target_detection_callback(self, _):
        if self.state == 'NO_TARGET_DETECTED':
            self.state = 'TARGET_DETECTED'
            self.busy = False
        if self.state == 'ORIENTATED TOWARDS TARGET':
            self.target_lost = False

    def _unlock_callback(self, _):
        if self.state == 'TARGET_DETECTED':
            self.state = 'ALIGNED TO TARGET'
        elif self.state == 'ALIGNED TO TARGET':
            self.state = 'ORIENTATED TOWARDS TARGET'
        elif self.state == 'ORIENTATED TOWARDS TARGET':
            self.state = 'TARGET_REACHED'
        self.busy = False

    def _odom_callback(self, msg):
        self.s = msg.pose.pose

    def _reset(self):
        self.state = 'NO_TARGET_DETECTED'
        self.reset_vision_pub.publish(Bool(True))
        self.busy = False
        self.target_lost = True
    
    def _search_for_target(self):
        # Spin and search for the target
        self.target_lost = True

        # Spin
        time = rospy.Time.now()
        w = 2.0*3.14159/10.0
        t = 2.0
        while (rospy.Time.now() - time).to_sec() < t:
            if not self.target_lost:
                self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
                return True
            self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=w)))
            rospy.sleep(0.02)
        
        time = rospy.Time.now()
        while (rospy.Time.now() - time).to_sec() < 2*t:
            if not self.target_lost:
                self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
                return True
            self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=-w)))
            rospy.sleep(0.02)

        # Not found, return to the initial position and stop
        time = rospy.Time.now()
        while (rospy.Time.now() - time).to_sec() < t:
            if not self.target_lost:
                self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
                return True
            self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=w)))
            rospy.sleep(0.02)
        self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))

        return False
    
    def run(self, _):
        if self.busy:
            return
        
        if self.state == 'NO_TARGET_DETECTED':
            # Navigation: Searching for the target 
            self.busy = True

        elif self.state == 'TARGET_DETECTED':
            # Compute the desired point and send the command to position-based-controller
            self.busy = True
            self.pose_controller_activate_publisher.publish(Bool(True))
            # Compute the desired point in the camera frame
            point_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, 
                                                             PointStamped(header=Header(stamp=rospy.Time(0), frame_id=self.object_frame_id),
                                                                                                    point=Point(*self.alignment_position)))
            
            rospy.logwarn(f'\nSending command to pose controller: \n >>> Reach alignment point:\n{point_inertial.point}\n')
            rospy.logwarn(point_inertial.point)
            self.pose_controller_publisher.publish(point_inertial.point)
            

        elif self.state == 'ALIGNED TO TARGET':
            self.busy = True
            # Orientate towards the target
            point_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, PointStamped(header=Header(stamp=rospy.Time(0), frame_id=self.object_frame_id),
                                                                point=Point(*self.alignment_position_orientate)))
            rospy.logwarn(f'\nSending command to pose controller: \n >>> Reach correct orientation point:\n{point_inertial.point}\n')
            self.pose_controller_publisher.publish(point_inertial.point)
        

        elif self.state == 'ORIENTATED TOWARDS TARGET':
            # Send command to the visual servoing controller
            self.busy = True
            rospy.logwarn(f'\nLooking for target visual...\n')

            # Turn pose controller off
            self.pose_controller_activate_publisher.publish(Bool(False))

            # Ensure that the target is on sight
            found = self._search_for_target()
            
            if found:
                # Send command to the visual servoing controller
                rospy.logwarn(f'\nTarget found. Activating IBVS controller...\n')
                self.ibvs_commander.publish(Bool(True))
            else:
                rospy.logwarn(f'\nTarget lost.\n')
                self.state = 'NO_TARGET_DETECTED'
                self.busy = False

        elif self.state == 'TARGET_REACHED':
            # Grab box
            self.busy = True
            rospy.logwarn('Target reached. Grabbing box..')
            rospy.sleep(5.0)
            self._reset()
        else:
            pass
        
                                         
if __name__ == '__main__':
    rospy.init_node('navigation_node', anonymous=True)

    navigator_rate = rospy.get_param('/navigator_rate')
    inertial_frame_id = rospy.get_param('/inertial_frame')
    object_frame_id = rospy.get_param('/object_frame')
    target_detection_topic = rospy.get_param('/target_detection_topic')
    ibvs_activate_topic = rospy.get_param('/ibvs_activate_topic')
    pose_controller_topic = rospy.get_param('/pose_controller_topic')
    pose_controller_activate_topic = rospy.get_param('/pose_control_activate_topic')
    unlock_topic = rospy.get_param('/unlock_topic')
    cmd_vel_topic = rospy.get_param('/commands_topic')
    odometry_topic = rospy.get_param('/odometry_topic')

    navigator = Navigator(navigator_rate, 
                            inertial_frame_id,
                            object_frame_id,
                            target_detection_topic, 
                            ibvs_activate_topic,
                            pose_controller_activate_topic,
                            pose_controller_topic,
                            cmd_vel_topic,
                            odometry_topic,
                            unlock_topic)

    # Spin the node
    rospy.spin()


