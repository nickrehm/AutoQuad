#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from std_msgs.msg import String, Float64, Empty, Int32
import math
import traceback
import serial
import time


##################
# Initialization #
##################

# configure serial connection through GPIO pins
ser = serial.Serial(
	port = '/dev/ttyAMA1',
	baudrate = 115200,
	timeout=0.1)


#############
# Functions #
#############
def get_data():
	global distance
	ser.reset_input_buffer()
	recv = ser.read(9)
	if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
		low = int(recv[2].encode('hex'), 16)
		high = int(recv[3].encode('hex'), 16)
		distance = (low + high * 256)/100.0

#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global distance
	distance = 0.0
	
	# Initialize node
	rospy.init_node('tfmini', anonymous=True)
	rate = rospy.Rate(100) # Hz
	
	# Initialize topics to publish
	pub_alt = rospy.Publisher('/tfmini/altitude', Float64, queue_size=1)

	
	# Subscribe to topics with reference to callback functions

	
	while not rospy.is_shutdown():
		try:		
			# receive data 
			get_data()
					
			# Publish to topics
			pub_alt.publish(distance)


		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in tfmini.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


