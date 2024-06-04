#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Twist, Polygon, PointStamped, Quaternion, Point, Vector3, Pose
from nav_msgs.msg import Odometry
import numpy as np
import tf
import tf.transformations as tft
from real_robot_util.util import get_global_params
#Service
from real_robot.srv import GripperService, BugService
from real_robot.srv import OrientationService


class Navigator:
    def __init__(self,  navigator_rate: float, 
                        inertial_frame_id, 
                        object_frame_id, 
                        target_detection_topic: str, 
                        ibvs_activate_topic: str,
                        bug_activate_topic: str,
                        #pose_controller_activate_topic: str,
                        #pose_controller_topic: str,
                        #orientation_controller_activate_topic,
                        #orientation_controller_topic,
                        cmd_vel_topic: str,
                        odom_topic: str,
                        ibvs_result_topic: str):
        
        # Frames
        self.inertial_frame_id = inertial_frame_id
        self.object_frame_id = object_frame_id

        #Open Gripper

        # Initialize the control publishers
        self.ibvs_commander = rospy.Publisher(ibvs_activate_topic, Bool, queue_size=10)
        self.call_gripper_service(True)
        self.bug_commander = rospy.Publisher(bug_activate_topic, Bool, queue_size=10)

        #self.pose_controller_activate_publisher = rospy.Publisher(pose_controller_activate_topic, Bool, queue_size=10)
        #self.pose_controller_publisher = rospy.Publisher(pose_controller_topic, Pose, queue_size=10)
        #self.orientation_controller_publisher = rospy.Publisher(orientation_controller_topic, Float32, queue_size=10)
        #self.orientation_controller_activate_publisher = rospy.Publisher(orientation_controller_activate_topic, Bool, queue_size=10)
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
        alignment_offset = 0.25
        self.alignment_point_target_frame = Point(*[0.0, 0.0, alignment_offset])

        # Initialize the subscribers
        rospy.Subscriber(target_detection_topic, Polygon, self._target_detection_callback)
        rospy.Subscriber(ibvs_result_topic, Bool, self._ibvs_result_callback)
        rospy.Subscriber(odom_topic, Odometry, self._odom_callback)
        # Set timer
        rospy.Timer(rospy.Duration(1.0/navigator_rate), self.run)

    def _target_detection_callback(self, corners):
        if len(corners.points) == 0:
            return
        if self.state == 'NO_TARGET_DETECTED' or self.state == 'TARGET_LOST':
            self.state = 'TARGET_DETECTED'
            self.busy = False
            self.target_found = True
            


    def _ibvs_result_callback(self, msg):
        if self.state == 'TARGET_DETECTED':
            if msg.data:
                self.state = 'TARGET_REACHED'
                self.ibvs_commander.publish(Bool(False))
                self.busy = False
            else:
                self.state = 'TARGET_LOST'
                self.ibvs_commander.publish(Bool(False))
                self.target_found = False
                self.busy = False

    def _odom_callback(self, msg):
        self.s = msg.pose.pose

    def _reset(self):
        self.state = 'NO_TARGET_DETECTED'
        self.reset_vision_pub.publish(Bool(True))
        self.busy = False
        self.target_found = False
        self.search_exhausted = False
    
    def call_gripper_service(self, gripper_state):
        rospy.wait_for_service('set_gripper_angle')
        try:
            set_gripper_angle = rospy.ServiceProxy('set_gripper_angle', GripperService)
            response = set_gripper_angle(gripper_state)
            rospy.loginfo(f"Gripper angle set to {response.angle}")
            return response.angle
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def call_bug_service(self, point, bug):
        rospy.wait_for_service('active_bug')
        try:
            print('LLAMAR A GRIPPER')
            active_bug = rospy.ServiceProxy('active_bug', BugService)
            response = active_bug(point,bug)
            rospy.loginfo(f"Bug Finish: {response.finish}")
            return response.finish
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def call_orientation_service(self, goal):
        rospy.wait_for_service('orientation_controller')
        try:
            orientation_controller = rospy.ServiceProxy('orientation_controller', OrientationService)
            response = orientation_controller(Float32(goal))
            rospy.loginfo(f"Orientation Controler Finish {response.finish}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        if not response.finish:
            raise Exception('Orientation controller failed')
        else:
            rospy.loginfo('Orientation controller finished')

    def _quat2yaw(self, quat):
        euler = tft.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        yaw = euler[2]
        return yaw
    
    def _search_for_target(self):
        # Spin and search for the target
        self.target_found = False

        current_yaw = (self._quat2yaw(self.s.orientation) + 2*np.pi) % (2*np.pi) # Ensure range [0, 2*pi]

        # Search to left side
        target_orientations = [(current_yaw + np.radians(8)) % (2*np.pi), (current_yaw - np.radians(8)) % (2*np.pi)]
        idx = 0
        rospy.logwarn('Searching for visual target...')
        while not self.target_found:
            rospy.sleep(0.05)
        
            self.call_orientation_service(target_orientations[idx])

            idx = (idx + 1) % 2

        return self.target_found
    
    
    def run(self, _):

        if self.busy:
            return

        if self.state == 'NO_TARGET_DETECTED':
            # Navigation: Searching for the target 
            self.busy = True
            self.call_gripper_service(True)
            rospy.logwarn('Robot initialized. Waiting for target detection.')

        elif self.state == 'TARGET_DETECTED':
            # Compute the desired point and send the command to pose controller
            rospy.logwarn("Target found. Activating IBVS controller.")
            self.busy = True
            self.ibvs_commander.publish(Bool(True))

        elif self.state == 'TARGET_REACHED':
            rospy.logwarn('Target reached. Closing gripper.')
            self.busy = True
            self.ibvs_commander.publish(Bool(False))
            # Move for 4 seconds
            vel = Twist(linear=Vector3(0.1, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            self.cmd_vel_publisher.publish(vel)
            rospy.sleep(1)
            vel = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            self.cmd_vel_publisher.publish(vel)
            rospy.sleep(1)
            # Close Gripper
            self.call_gripper_service(False)
            rospy.sleep(2)
            point = Point(2.40, 1.4, 0.0)
            rospy.logwarn(f'Gripper closed. Going to point {point}...')
            rospy.sleep(2)
            # Letter B (2.88,1.62)
            self.call_bug_service(point,2)
            self.call_orientation_service(0)
            point = Point(2.50, 1.62, 0.0)
            self.call_bug_service(point,2)
            rospy.sleep(1)
            self.call_gripper_service(True)
            
            vel = Twist(linear=Vector3(-0.1, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            self.cmd_vel_publisher.publish(vel)
            rospy.sleep(1)
            vel = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            self.cmd_vel_publisher.publish(vel)
            rospy.sleep(1)
            point = Point(0.01, 0.01, 0.0)
            self.call_bug_service(point,2)
            rospy.sleep(1)
            self.call_orientation_service(0)
            # self.bug_commander.publish(Bool(True))

        elif self.state == 'TARGET_LOST':
            rospy.logwarn('Target lost. Searching for target...')
            self._search_for_target()
            rospy.logwarn('Target has ben found again. ' + str(self.target_found))
            # self.state = "TARGET_DETECTED"
            self.busy = False


            # Compute the desired point and orientation in the inertial frame
            # header = Header(stamp=rospy.Time(0), frame_id=self.object_frame_id)
            
            # target_frame_position = PointStamped(header=header, point=Point(*[0.0, 0.0, 0.0]))
            # target_frame_alignment_position = PointStamped(header=header, point=self.alignment_point_target_frame)

            # target_point_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, target_frame_position)
            # aligned_point_inertial = self.tf_listener.transformPoint(self.inertial_frame_id, target_frame_alignment_position)

            # dy = target_point_inertial.point.y - aligned_point_inertial.point.y
            # dx = target_point_inertial.point.x - aligned_point_inertial.point.x

            # target_z_rotation = np.arctan2(dy, dx)
            # target_rotation_quat = tft.quaternion_from_euler(0.0, 0.0, target_z_rotation)
            # target_pose = Pose(position=aligned_point_inertial.point, orientation=Quaternion(*target_rotation_quat))
            # rospy.logwarn(f'\nSending command to pose controller: \n >>> Reach alignment point:\n{target_point_inertial.point}\nZ axis orientation: {target_z_rotation}\n')
            # self.pose_controller_activate_publisher.publish(Bool(True))
            # self.pose_controller_publisher.publish(target_pose)
            
        # elif self.state == 'ALIGNED TO TARGET':
        #     # Send command to the visual servoing controller
        #     self.busy = True
        #     rospy.logwarn(f'\nLooking for target visual...\n')

        #     # Deactivate the pose controller
        #     #self.pose_controller_activate_publisher.publish(Bool(False))
        #     rospy.sleep(4)
        #     # Ensure that the target is on sight
        #     found = self._search_for_target()
            
        #     if found:
        #         # Send command to the visual servoing controller
        #         rospy.logwarn(f'\nTarget found. Activating IBVS controller...\n')
        #         self.ibvs_commander.publish(Bool(True))
        #     else:
        #         rospy.logwarn(f'\nTarget lost.\n')
        #         self.state = 'NO_TARGET_DETECTED'
        #         self.busy = False

        # elif self.state == 'TARGET_REACHED':
        #     # Grab box
        #     self.busy = True
        #     rospy.logwarn('Target reached. Grabbing box..')
        #     rospy.sleep(5.0)
        #     self._reset()
        else:
            pass
        
                                         
if __name__ == '__main__':
    rospy.init_node('navigation_node', anonymous=True)
    params = get_global_params()

    navigator_rate = params['navigation_rate']
    inertial_frame_id = params['inertial_frame']
    object_frame_id = params['object_frame']
    target_detection_topic = params['aruco_detection_topic']
    ibvs_activate_topic = params['ibvs_activate_topic']
    bug_activate_topic = params['bug_activate_topic']
    ibvs_result_topic = params['ibvs_done_topic']
    cmd_vel_topic = params['commands_topic']
    odometry_topic = params['odometry_topic']

    #pose_controller_topic = rospy.get_param('/pose_controller_topic')
    #pose_controller_activate_topic = rospy.get_param('/pose_control_activate_topic')
    #orientation_controller_topic = rospy.get_param('/orientation_controller_topic')
    #orientation_controller_activate_topic = rospy.get_param('/orientation_control_activate_topic')
    

    navigator = Navigator(navigator_rate, 
                            inertial_frame_id,
                            object_frame_id,
                            target_detection_topic, 
                            ibvs_activate_topic,
                            bug_activate_topic,
                            #pose_controller_activate_topic,
                            #pose_controller_topic,
                            #orientation_controller_activate_topic,
                            #orientation_controller_topic,
                            cmd_vel_topic,
                            odometry_topic,
                            ibvs_result_topic)
    # Spin the node
    rospy.spin()