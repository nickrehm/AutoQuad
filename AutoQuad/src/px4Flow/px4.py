#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Empty, Int32
import math
import traceback
import time
from px4Flow import PX4FLOW


##################
# Initialization #
##################

px4flow = PX4FLOW()


#############
# Functions #
#############

def printData():
	print"{:5.2f}".format(x_vel),
	print"{:5.2f}".format(y_vel),
	print"{:5.2f}".format(altitude),
	print"{:5.2f}".format(quality)
	print('')


#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global data, x_vel, y_xel, altitude, quality
	
	# Initialize node
	rospy.init_node('px4', anonymous=True)
	rate = rospy.Rate(500) # Hz
	
	# Initialize topics to publish
	pub_px4_x_vel = rospy.Publisher('/px4_data/x_vel', Float64, queue_size=1)
	pub_px4_y_vel = rospy.Publisher('/px4_data/y_vel', Float64, queue_size=1)
	pub_px4_alt = rospy.Publisher('/px4_data/altitude', Float64, queue_size=1)
	pub_px4_quality = rospy.Publisher('/px4_data/quality', Int32, queue_size=1)

	
	# Subscribe to topics with reference to callback functions
	
	
	while not rospy.is_shutdown():
		try:		
			# Get px4 data
			data = px4flow.read()
			x_vel = data[0]/1000.0
			y_vel = data[1]/1000.0
			altitude = data[2]/1000.0
			quality = data[3]
			#printData()
			
			# Publish to topics
			pub_px4_x_vel.publish(x_vel)
			pub_px4_y_vel.publish(y_vel)
			pub_px4_alt.publish(altitude)
			pub_px4_quality.publish(quality)

		except Exception:
			#traceback.print_exc()
			#rospy.loginfo('Some error ocurred in px4.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


