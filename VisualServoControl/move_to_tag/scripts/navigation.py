#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Twist, Polygon, PointStamped, Quaternion, Point, Vector3, Pose
from nav_msgs.msg import Odometry
import numpy as np
import tf
import tf.transformations as tft
from puzzlebot_util.util import *

class Navigator:
    def __init__(self,  navigator_rate: float, 
                        inertial_frame_id, 
                        object_frame_id, 
                        target_detection_topic: str, 
                        ibvs_activate_topic: str,
                        pose_controller_activate_topic: str,
                        pose_controller_topic: str,
                        orientation_controller_activate_topic,
                        orientation_controller_topic,
                        cmd_vel_topic: str,
                        odom_topic: str,
                        unlock_topic: str):
        
        # Frames
        self.inertial_frame_id = inertial_frame_id
        self.object_frame_id = object_frame_id

        # Initialize the control publishers
        self.ibvs_commander = rospy.Publisher(ibvs_activate_topic, Bool, queue_size=10)
        self.pose_controller_activate_publisher = rospy.Publisher(pose_controller_activate_topic, Bool, queue_size=10)
        self.pose_controller_publisher = rospy.Publisher(pose_controller_topic, Pose, queue_size=10)
        self.orientation_controller_publisher = rospy.Publisher(orientation_controller_topic, Float32, queue_size=10)
        self.orientation_controller_activate_publisher = rospy.Publisher(orientation_controller_activate_topic, Bool, queue_size=10)
        self.reset_vision_pub = rospy.Publisher('/reset_vision', Bool, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # State
        self.state = 'NO_TARGET_DETECTED'
        self.busy = False
        self.s : Pose = None
        self.target_found = False
        self.search_exhausted = False

        # Class variables
        alignment_offset = 0.6
        self.alignment_point_target_frame = Point(*[0.0, 0.0, alignment_offset])

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
        if self.state == 'ALIGNED TO TARGET':
            self.target_found = True
        if self.state == 'SEARCHING':
            self.target_found = True


    def _unlock_callback(self, _):
        if self.state == 'TARGET_DETECTED':
            self.state = 'ALIGNED TO TARGET'
        elif self.state == 'ALIGNED TO TARGET':
            self.state = 'TARGET_REACHED'
        elif self.state == 'SEARCHING':
            self.search_exhausted = True
            rospy.loginfo('Orientation search exhausted')
        self.busy = False

    def _odom_callback(self, msg):
        self.s = msg.pose.pose

    def _reset(self):
        self.state = 'NO_TARGET_DETECTED'
        self.reset_vision_pub.publish(Bool(True))
        self.busy = False
        self.target_found = False
        self.search_exhausted = False
    
    def _quat2yaw(self, quat):
        euler = tft.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        yaw = euler[2]
        return yaw
    
    def _search_for_target(self):
        # Spin and search for the target
        self.target_found = False
        self.state = "SEARCHING"

        self.orientation_controller_activate_publisher.publish(Bool(True))
        current_yaw = self._quat2yaw(self.s.orientation) # [-pi pi] ??

        current_yaw = (current_yaw + 2*np.pi) % (2*np.pi) # Ensure range [0, 2*pi]

        search_radius = 2*np.pi * 0.99

        # Search to left side
        self.search_exhausted = False
        target_orientation = (current_yaw + search_radius) % (2*np.pi)
        self.orientation_controller_publisher.publish(Float32(target_orientation)) # In range [0, 2*pi]
        
        # Wait to controller to finish or target be detected
        rospy.logwarn('Searching for visual target...')
        while not self.target_found and not self.search_exhausted:
            rospy.sleep(0.05)

        self.orientation_controller_activate_publisher.publish(Bool(False))
        return self.target_found
    
    
    def run(self, _):

        if self.busy:
            return

        if self.state == 'NO_TARGET_DETECTED':
            # Navigation: Searching for the target 
            self.busy = True

        elif self.state == 'TARGET_DETECTED':
            # Compute the desired point and send the command to pose controller
            self.busy = True

            # Compute the desired point and orientation in the inertial frame
            header = Header(stamp=rospy.Time(0), frame_id=self.object_frame_id)
            
            target_frame_position = PointStamped(header=header, point=Point(*[0.0, 0.0, 0.0]))
            target_frame_alignment_position = PointStamped(header=header, point=self.alignment_point_target_frame)

            target_point_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, target_frame_position)
            aligned_point_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, target_frame_alignment_position)

            dy = target_point_inertial.point.y - aligned_point_inertial.point.y
            dx = target_point_inertial.point.x - aligned_point_inertial.point.x

            target_z_rotation = np.arctan2(dy, dx)
            target_rotation_quat = tft.quaternion_from_euler(0.0, 0.0, target_z_rotation)
            target_pose = Pose(position=aligned_point_inertial.point, orientation=Quaternion(*target_rotation_quat))
            rospy.logwarn(f'\nSending command to pose controller: \n >>> Reach alignment point:\n{target_point_inertial.point}\nZ axis orientation: {target_z_rotation}\n')
            self.pose_controller_activate_publisher.publish(Bool(True))
            self.pose_controller_publisher.publish(target_pose)
            
        elif self.state == 'ALIGNED TO TARGET':
            # Send command to the visual servoing controller
            self.busy = True
            rospy.logwarn(f'\nLooking for target visual...\n')

            # Deactivate the pose controller
            self.pose_controller_activate_publisher.publish(Bool(False))
            rospy.sleep(4)
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
    orientation_controller_topic = rospy.get_param('/orientation_controller_topic')
    orientation_controller_activate_topic = rospy.get_param('/orientation_control_activate_topic')
    unlock_topic = rospy.get_param('/unlock_topic')
    cmd_vel_topic = rospy.get_param('/commands_topic')
    odometry_topic = rospy.get_param('/sim_odom_topic')

    navigator = Navigator(navigator_rate, 
                            inertial_frame_id,
                            object_frame_id,
                            target_detection_topic, 
                            ibvs_activate_topic,
                            pose_controller_activate_topic,
                            pose_controller_topic,
                            orientation_controller_activate_topic,
                            orientation_controller_topic,
                            cmd_vel_topic,
                            odometry_topic,
                            unlock_topic)

    # Spin the node
    rospy.spin()


