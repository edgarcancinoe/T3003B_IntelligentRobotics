#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from real_robot.srv import GripperService, GripperServiceResponse


def handle_gripper(req):
    angle = 90.0 if req.data else -55.0
    servo_pub.publish(angle)
    return GripperServiceResponse(angle)

if __name__ == "__main__":
    rospy.init_node('gripper_server')
    servo_pub = rospy.Publisher('/servo', Float32, queue_size=10)
    service = rospy.Service('set_gripper_angle', GripperService, handle_gripper)
    rospy.loginfo("Gripper angle service is ready.")
    rospy.spin()