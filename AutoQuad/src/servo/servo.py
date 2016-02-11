#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from std_msgs.msg import String, Float64, Empty, Int32
import traceback
import pigpio
import time


##################
# Initialization #
##################

pi = pigpio.pi()


#############
# Functions #
#############
def servo_feedback(data):
	global servo_state
	servo_state = data.data

#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global servo_state
	servo_state = 0
	
	# Initialize node
	rospy.init_node('servo', anonymous=True)
	rate = rospy.Rate(5) # Hz
	
	# Initialize topics to publish
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/servo_command', Int32, servo_feedback)
	
	while not rospy.is_shutdown():
		try:		
			# actuate servo
			if(servo_state == 0):
				#open
				pi.set_servo_pulsewidth(12,1030)
			elif(servo_state == 1): 
				#closed
				pi.set_servo_pulsewidth(12,1800)
					
			# Publish to topics



		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in servo.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


