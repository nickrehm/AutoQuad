#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from std_msgs.msg import String, Float64, Empty, Int32
import math
import traceback
import time


##################
# Initialization #
##################


#############
# Functions #
#############
def tag_feedback(data):
	global Px1, Py1, Pz1
	global q1_1, q2_1, q3_1, q0_1
	global target_detected_1
	
	if(len(data.detections)==0):
		indent = 1
		target_detected_1 = 0
	else:	
		for i in range(0,len(data.detections)):
			if(data.detections[i].id[0] == 10): # first tag id, etc...
				Px1 = data.detections[i].pose.pose.pose.position.x
				Py1 = data.detections[i].pose.pose.pose.position.y
				Pz1 = data.detections[i].pose.pose.pose.position.z
				q1_1 = data.detections[i].pose.pose.pose.orientation.x 
				q2_1 = data.detections[i].pose.pose.pose.orientation.y 
				q3_1 = data.detections[i].pose.pose.pose.orientation.z
				q0_1 = data.detections[i].pose.pose.pose.orientation.w 
				target_detected_1 = 1
			else:
				target_detected_1 = 0


def publishPose():
	p = PoseStamped()
	p.header.frame_id = "map"
	p.header.stamp = rospy.Time.now()
	p.pose.position.x = Px
	p.pose.position.y = Py
	p.pose.position.z = Pz
	p.pose.orientation.x = q1
	p.pose.orientation.y = q2
	p.pose.orientation.z = q3
	p.pose.orientation.w = q0
	return p
	
#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global Px1, Py1, Pz1
	global q1_1, q2_1, q3_1, q0_1
	global Px, Py, Pz, q1, q2, q3, q0
	global target_detected, target_detected_1
	target_detected = target_detected_1 = 0
	Px1 = Py1 = Pz1 = 0.0
	q1_1 = q2_1 = q3_1 = 0.0
	q0_1 = 1.0
	Px = Py = Pz = 0.0
	q1 = q2 = q3 = q0 = 0.0
	
	
	# Initialize node
	rospy.init_node('target_planner', anonymous=True)
	rate = rospy.Rate(30) # Hz
	
	# Initialize topics to publish
	pub_pose = rospy.Publisher('/target/pose', PoseStamped, queue_size=1)
	pub_target_detected = rospy.Publisher('/target/target_detected', Int32, queue_size=1)
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_feedback)
	
	while not rospy.is_shutdown():
		try:		
			# decide what target to be tracking here.....fixed on tag 1 for now
			Px = Px1
			Py = Py1
			Pz = Pz1
			q1 = q1_1
			q2 = q2_1
			q3 = q3_1
			q0 = q0_1
			target_detected = target_detected_1

			# Publish to topics
			p = publishPose() #format data into posestamped struct
			pub_pose.publish(p)
			pub_target_detected.publish(target_detected)


		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in target_planner.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


