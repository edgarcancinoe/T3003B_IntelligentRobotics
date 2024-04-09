#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32

def random_value():
    value1 = random.uniform(-8, -4)
    value2 = random.uniform(4, 8)
    return random.choice([value1, value2])

def publish_random_value():
    rospy.init_node('random_value_publisher', anonymous=True)
    pub = rospy.Publisher('/tau', Float32, queue_size=10)

    while not rospy.is_shutdown():
        # Generate a random value
        value = random_value()

        # Publish the random value
        pub.publish(value)
        rospy.sleep(0.1)
        pub.publish(0.0)

        rospy.sleep(15)

if __name__ == '__main__':
    try:
        publish_random_value()
    except rospy.ROSInterruptException:
        pass
