#!/usr/bin/env python

import rospy
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand

def publisher():

    pub_steer = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size = 10)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size = 10)
    rospy.init_node('publisher', anonymous = True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
	'''
        rospy.set_param('msg_steer', 1.0)
	msg_steer = rospy.get_param("msg_steer")
        rospy.loginfo(msg_steer)
        pub_steer.publish(msg_steer)

        rospy.set_param('msg_speed' , 0.3)
	msg_speed = rospy.get_param("msg_speed")
        rospy.loginfo(msg_speed)
        pub_speed.publish(msg_speed)

        rate.sleep()
	'''
	msg_steer = NormalizedSteeringCommand()
	msg_steer.value = 1.0
        rospy.loginfo(msg_steer)
        pub_steer.publish(msg_steer)
	msg_speed = SpeedCommand()
	msg_speed.value = 0.3
        rospy.loginfo(msg_speed)
        pub_speed.publish(msg_speed)



if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
