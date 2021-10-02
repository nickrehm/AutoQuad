#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from std_msgs.msg import String, Float64, Empty, Int32
from nav_msgs.msg import Odometry
import math
import traceback
import time
import numpy as np


##################
# Initialization #
##################


#############
# Functions #
#############

def odom_pose_feedback(data):
	global Xpos, Ypos, alt
	Xpos = data.pose.pose.position.x
	Ypos = data.pose.pose.position.y
	alt = data.pose.pose.position.z
	
	Xpos = Xpos*1.33;
	Ypos = Ypos*1.33;
	


def mission_planner_feedback(data):
	global mission_planner
	mission_planner = data.data
	
def tag_feedback(data):
	global x_obs, y_obs
	global target_detected_0, target_detected_1, target_detected_2
	global detection_distance
	global flag_new_obstacle
	global XPos, YPos
	global mission_planner
	
	if(len(data.detections)==0):
		indent = 1
		#print("No tags detected...")
	else:	
		#print("Tags Detected!")
		for i in range(0,len(data.detections)):
			if(data.detections[i].id[0] == 0 and target_detected_0 == 0 and flag_new_obstacle == 0 and mission_planner == 0): # first tag id
				Px = data.detections[i].pose.pose.pose.position.x
				Py = data.detections[i].pose.pose.pose.position.y
				Pz = data.detections[i].pose.pose.pose.position.z
				q1 = data.detections[i].pose.pose.pose.orientation.x 
				q2 = data.detections[i].pose.pose.pose.orientation.y 
				q3 = data.detections[i].pose.pose.pose.orientation.z
				q0 = data.detections[i].pose.pose.pose.orientation.w
				dist2camera = math.sqrt(Px*Px + Pz*Pz) #only care about z,x camera axes (x,y axes in inertial frame)
				print("tag with id '0' detected at distance: ", dist2camera)
				if (detection_distance > dist2camera): #within detection radius
					target_detected_0 = 1 #we've detected this target, dont detect it again
					#transform from camera frame to inertial frame
					x_obs = Xpos + Pz + r_offset #position in inertial frame
					y_obs = Ypos - Px
					print(x_obs,y_obs)
					flag_new_obstacle = 1 #used to tell in main loop that we've got a new obstacle to deal with
			if(data.detections[i].id[0] == 1 and target_detected_1 == 0 and flag_new_obstacle == 0 and mission_planner == 0): # second tag id
				Px = data.detections[i].pose.pose.pose.position.x
				Py = data.detections[i].pose.pose.pose.position.y
				Pz = data.detections[i].pose.pose.pose.position.z
				q1 = data.detections[i].pose.pose.pose.orientation.x 
				q2 = data.detections[i].pose.pose.pose.orientation.y 
				q3 = data.detections[i].pose.pose.pose.orientation.z
				q0 = data.detections[i].pose.pose.pose.orientation.w
				dist2camera = math.sqrt(Px*Px + Pz*Pz) #only care about z,x camera axes (x,y axes in inertial frame)
				print("tag with id '1' detected at distance: ", dist2camera)
				if (detection_distance > dist2camera): #within detection radius
					target_detected_1 = 1 #we've detected this target, dont detect it again
					#transform from camera frame to inertial frame
					x_obs = Xpos + Pz + r_offset #position in inertial frame
					y_obs = Ypos - Px
					print(x_obs,y_obs)
					flag_new_obstacle = 1 #used to tell in main loop that we've got a new obstacle to deal with
			if(data.detections[i].id[0] == 2 and target_detected_2 == 0 and flag_new_obstacle == 0 and mission_planner == 0): #third tag id
				Px = data.detections[i].pose.pose.pose.position.x
				Py = data.detections[i].pose.pose.pose.position.y
				Pz = data.detections[i].pose.pose.pose.position.z
				q1 = data.detections[i].pose.pose.pose.orientation.x 
				q2 = data.detections[i].pose.pose.pose.orientation.y 
				q3 = data.detections[i].pose.pose.pose.orientation.z
				q0 = data.detections[i].pose.pose.pose.orientation.w
				dist2camera = math.sqrt(Px*Px + Pz*Pz) #only care about z,x camera axes (x,y axes in inertial frame)
				print("tag with id '2' detected at distance: ", dist2camera)
				if (detection_distance > dist2camera): #within detection radius
					target_detected_2 = 1 #we've detected this target, dont detect it again
					#transform from camera frame to inertial frame
					x_obs = Xpos + Pz + r_offset #position in inertial frame
					y_obs = Ypos - Px
					print(x_obs,y_obs)
					flag_new_obstacle = 1 #used to tell in main loop that we've got a new obstacle to deal with
					
			else:
				indent = 1


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
	global target_detected_0, target_detected_1, target_detected_2
	global dt
	global x_des_pub, y_des_pub, NodeID_pub
	global mission_planner
	global Xpos, Ypos, alt
	global x_obs, y_obs, r_obs
	global detection_distance
	global flag_new_obstacle
	global r_offset
	flag_new_obstacle = 0 
	Xpos = Ypos = alt = 0.0
	mission_planner = 1 #1 is stopped, 0 is allowed to traverse
	x_des_pub = y_des_pub = 0.0
	target_detected_0 = target_detected_1 = target_detected_2 = 0
	Px1 = Py1 = Pz1 = 0.0
	q1_1 = q2_1 = q3_1 = 0.0
	q0_1 = 1.0
	Px = Py = Pz = 0.0
	q1 = q2 = q3 = q0 = 0.0
	
	LogPos_arr = np.array([])
	LogXPos_arr = np.array([])
	LogYPos_arr = np.array([])
	
	#important shit to change
	time_elapsed = 0.0
	time_interval = 0.4 #time between traversal from one pos to the next along path
	traverse_index = 0
	time_elapsed_log = 0.0 #for position data logging
	time_interval_log = 0.2
	
	detection_distance = 2.0 #minimum distance to april tag to detect it, meters
	r_obs = 1.0 #obstacle radius, meters
	r_offset = 0.15 #offset from front of obstacle to center (x-direction)
	
	x_des_pub = 0.0 #change to starting coordinates!
	y_des_pub = 2.0 #change to starting coordinates!
	NodeID_pub = 0 #doesnt matter
	
	
	# Initialize node
	rospy.init_node('planner', anonymous=True)
	rate = rospy.Rate(30) # Hz
	dt = 1/30.0
	
	
	
	# Initialize topics to publish
	pub_x_des = rospy.Publisher('/control/Xpos_des', Float64, queue_size=1)
	pub_y_des = rospy.Publisher('/control/Ypos_des', Float64, queue_size=1)
	pub_NodeID = rospy.Publisher('/control/NodeID', Int32, queue_size=1)
	pub_mission_planner = rospy.Publisher('/mission_planner', Int32, queue_size=1)
	pub_x_obs = rospy.Publisher('/obstacle/x_obs', Float64, queue_size=1)
	pub_y_obs = rospy.Publisher('/obstacle/y_obs', Float64, queue_size=1)
	pub_r_obs = rospy.Publisher('/obstacle/r_obs', Float64, queue_size=1)
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_feedback)
	rospy.Subscriber('/mission_planner', Int32, mission_planner_feedback)
	rospy.Subscriber('/camera/odom/sample', Odometry, odom_pose_feedback)
	
	
	# use these arrays as stand-in until we get read in from text files...
	global x_des_arr ,y_des_arr, NodeID_arr
	#x_des_arr = np.array([0.0, 1.0, 1.0, 0.0, 0.0]) #test square
	#y_des_arr = np.array([0.0, 0.0, 1.0, 1.0, 0.0])
	
	#test-circle aka testicle
	#x_des_arr=[]
	#y_des_arr=[]
	#with open("CircleJerk.txt", "r") as filestream:
	#	for line in filestream:
	#		currentline = line.split(",")
	#		x_des_arr = np.append(x_des_arr, float(currentline[1]))
	#		y_des_arr = np.append(y_des_arr, float(currentline[2]))		
	#print(x_des_arr)
	
	#MISSION PLANNER STATES:
	#0 = allow traversal along solution path
	#1 = stop traversal along solution path
	#2 = initial solution is available from D*Lite
	#3 = new solution (1st replan) is available from D*Lite
	#4 = new solution (2nd replan) is available from D*Lite
	#5 = new solution (3rd replan) is available from D*Lite
	#6 = brand new obstacle detected and (x,y,r) published 
	
	pub_x_des.publish(x_des_pub) #publishes starting coordinates for the controller
	pub_y_des.publish(y_des_pub)
	pub_NodeID.publish(NodeID_pub)
	
	while not rospy.is_shutdown():
		try:
			#print(Xpos,Ypos)
			
			#check if new solution is available...if it is, update x_des_arr & y_des_array from text file and set traverse_index = 0, set mission_planner topic to 0 to allow traversal
			if (mission_planner == 2): #initial solution is available from D*Lite node
				print("D*Lite returned initial solution path. Reading in...")
				x_des_arr=[]
				y_des_arr=[]
				NodeID_arr=[]
				with open("output_path1.txt", "r") as filestream:
					for line in filestream:
						currentline = line.split(",")
						NodeID_arr = np.append(NodeID_arr, float(currentline[0]))
						x_des_arr = np.append(x_des_arr, float(currentline[1]))
						y_des_arr = np.append(y_des_arr, float(currentline[2]))
				traverse_index = 0
				mission_planner = 0 #allow traversal over solution path
				pub_mission_planner.publish(mission_planner)
				
				
			if (mission_planner == 3): #1st replanned solution is available from D*Lite node
				print("D*Lite returned 1st replanned solution path. Reading in...")
				x_des_arr=[]
				y_des_arr=[]
				NodeID_arr=[]
				with open("output_path2.txt", "r") as filestream:
					for line in filestream:
						currentline = line.split(",")
						NodeID_arr = np.append(NodeID_arr, float(currentline[0]))
						x_des_arr = np.append(x_des_arr, float(currentline[1]))
						y_des_arr = np.append(y_des_arr, float(currentline[2]))
				traverse_index = 0
				mission_planner = 0 #allow traversal over solution path
				pub_mission_planner.publish(mission_planner)
				
			if (mission_planner == 4): #2nd replanned solution is available from D*Lite node
				print("D*Lite returned 2nd replanned solution path. Reading in...")
				x_des_arr=[]
				y_des_arr=[]
				NodeID_arr=[]
				with open("output_path3.txt", "r") as filestream:
					for line in filestream:
						currentline = line.split(",")
						NodeID_arr = np.append(NodeID_arr, float(currentline[0]))
						x_des_arr = np.append(x_des_arr, float(currentline[1]))
						y_des_arr = np.append(y_des_arr, float(currentline[2]))
				traverse_index = 0
				mission_planner = 0 #allow traversal over solution path
				pub_mission_planner.publish(mission_planner)
				
			if (mission_planner == 5): #3rd replanned solution is available from D*Lite node
				print("D*Lite returned 3rd replanned solution path. Reading in...")
				x_des_arr=[]
				y_des_arr=[]
				NodeID_arr=[]
				with open("output_path4.txt", "r") as filestream:
					for line in filestream:
						currentline = line.split(",")
						NodeID_arr = np.append(NodeID_arr, float(currentline[0]))
						x_des_arr = np.append(x_des_arr, float(currentline[1]))
						y_des_arr = np.append(y_des_arr, float(currentline[2]))
				traverse_index = 0
				mission_planner = 0 #allow traversal over solution path
				pub_mission_planner.publish(mission_planner)
				
			
			#check if obstacle detected, set mission_planner to 6 and publish x,y,r coordinates in inertial frame
			if (flag_new_obstacle == 1): 
				print("New obstacle detected, publishing obstacle coordinates and updating mission_planner=6")
				
				pub_x_obs.publish(x_obs)
				pub_y_obs.publish(y_obs)
				pub_r_obs.publish(r_obs)
				flag_new_obstacle = 0
				mission_planner = 6 #new obstacle detected and coordinates published -> D* Lite node
				pub_mission_planner.publish(mission_planner)
			
			
			#traverse along solution until exhausted
			if (time_elapsed >= time_interval and mission_planner == 0 and traverse_index < len(x_des_arr)): #incriment along coord arrays in intervals of time_interval
				time_elapsed = 0.0
				
				#print(x_des_arr[traverse_index])
				NodeID_pub = NodeID_arr[traverse_index] - 1; #converts to zero based indexing
				x_des_pub = x_des_arr[traverse_index]
				y_des_pub = y_des_arr[traverse_index]
				print("Traversing to Coordinates:",x_des_pub,y_des_pub)
				
				traverse_index = traverse_index + 1
			
			
			#log odometry data to text file
			if (time_elapsed_log >= time_interval_log): #log data every time_interval_log seconds
				time_elapsed_log = 0.0
				#Xpos, Ypos
				LogXPos_arr = np.append(LogXPos_arr, [Xpos])
				LogYPos_arr = np.append(LogYPos_arr, [Ypos])
				LogPos_arr = np.stack((LogXPos_arr,LogYPos_arr), axis=-1)
				np.savetxt("measured_path.txt", LogPos_arr)
				
				
			#publish desired coordinates 
			pub_NodeID.publish(NodeID_pub)
			pub_x_des.publish(x_des_pub)
			pub_y_des.publish(y_des_pub)
			
			
			
			
			
			
			
			time_elapsed = time_elapsed + dt
			time_elapsed_log = time_elapsed_log + dt

			
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


