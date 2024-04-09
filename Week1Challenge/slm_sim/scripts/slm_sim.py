#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class slm_sim():
    def __init__(self, freq) -> None:
        self.k: float = 0.12
        self.m: float = 0.75
        self.l: float = 0.36
        self.g: float = 9.81

        self.tau: float = 0.0
        self.x1: float = 0.0
        self.x2: float = 0.0

        self.a: float = self.l/2
        self.J: float = 4/3*self.m*self.a

        self.dt: float = 1 / freq

    # Callback function
    def tau_callback(self, msg):
        self.tau = msg.data
        rospy.loginfo(rospy.get_caller_id() + "Heard Tau = %s", msg.data)
        
    # Wrap to pi function
    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
        
    def angle_step(self):
        x2_dot = 1 / self.J * (self.tau - self.m*self.g*self.a*np.cos(self.x1) - self.k*self.x2)

         # Euler
        self.x2 = self.x2 + x2_dot * self.dt
        self.x1 = self.x1 + self.x2 * self.dt

        self.x1 = self._wrap_to_Pi(self.x1)

        # Define HEader msg
        header = Header()
        header.frame_id = 'q'
        header.stamp = rospy.Time.now()

        # Define JointState msg
        msg = JointState()    
        msg.header = header
        msg.position = [self.x1]
        msg.velocity = [self.x2]
        msg.name = ["joint2"]
        msg.effort = [0.0]

        return msg


freq = 100
if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", freq))

    # Setup Runner
    slm_runner = slm_sim(freq=freq)

    # Setup the Subscribers
    rospy.Subscriber('/tau', Float32, slm_runner.tau_callback)

    # Setup de publishers
    joints_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)


    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            joints_publisher.publish(slm_runner.angle_step())
            # Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node