#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Empty, Int32
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, sqrt, asin, acos, atan
import traceback
import time


##################
# Initialization #
##################




#############
# Functions #
#############

def pose_feedback(data):
	global q0, q1, q2, q3
	global roll, pitch, yaw
	q0 = data.pose.orientation.w
	q1 = data.pose.orientation.x
	q2 = data.pose.orientation.y
	q3 = data.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

def accel_feedback(data):
	global Ax_IMU, Ay_IMU, Az_IMU
	Ax_IMU = data.accel.linear.x
	Ay_IMU = data.accel.linear.y
	Az_IMU = data.accel.linear.z
	
def twist_feedback(data):
	global Gx, Gy, Gz
	Gx = data.twist.angular.x
	Gy = data.twist.angular.y
	Gz = data.twist.angular.z

def x_vel_feedback(data):
	global Vx_px4, dt, Px
	Vx_px4 = data.data
	# integrate velocity for position estimate
	Px = Px + Vx_px4*dt
	
		
def y_vel_feedback(data):
	global Vy_px4, dt, Py
	Vy_px4 = data.data
	# integrate velocity for position estimate 
	Py = Py - Vy_px4*dt
	
def altitude_feedback(data):
	global altitude_px4
	altitude_px4 = data.data
	
def quality_feedback(data):
	global quality_px4
	quality_px4 = data.data
	
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
	
	
def quaternion_to_euler(w, x, y, z):
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll = atan2(t0, t1)
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch = asin(t2)
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw = atan2(t3, t4)
	return [roll, pitch, yaw]

def fuseData():
	global q0, q1, q2, q3
	global roll, pitch, yaw
	global Ax_IMU, Ay_IMU, Az_IMU, Vx_IMU, Vy_IMU
	global Gx, Gy, Gz
	global Vx_px4, Vy_px4, altitude_px4, quality_px4
	global Vx_px4_LP, Vx_px4_LP_prev, Vy_px4_LP, Vy_px4_LP_prev
	global Vx, Vx_prev, Vy, Vy_prev
	global Px, Py, Pz, Pz_prev, altitude_px4_prev
	global Pz_prev_re
	global alt_counter
	global dt
	
	# use orientation to get linear body accelerations 
	#~ Ax = Ax_IMU + sin(pitch)
	#~ Ay = Ay_IMU - sin(roll)
	#~ Az = Az_IMU + abs(Ax_IMU*sin(roll)) + abs(Ay_IMU*sin(pitch)) - 1.0
	
	# LP filter px4 velocity measurements
	B_px4 = 0.38 #0.03
	Vx_px4_LP = (1.0 - B_px4)*Vx_px4_LP_prev + B_px4*Vx_px4
	Vy_px4_LP = (1.0 - B_px4)*Vy_px4_LP_prev + B_px4*Vy_px4
	Vx_px4_LP_prev = Vx_px4_LP
	Vy_px4_LP_prev = Vy_px4_LP
	Vx_px4_LP = Vx_px4_LP*1.0
	Vy_px4_LP = Vy_px4_LP*1.0
	
	# fuse IMU velocity estimate with px4 velocity estimate
	#~ B_comp = 1.0 #0.9
	#~ Vx = (1.0 - B_comp)*(Vx + Ax*dt*9.81) + B_comp*Vx_px4_LP
	#~ Vy = (1.0 - B_comp)*(Vy + Ay*dt*9.81) - B_comp*Vy_px4_LP
	Vx = Vx_px4_LP
	Vy = Vy_px4_LP

	#print"{:12.9f}".format(Vx),
	#print"{:12.9f}".format(Vy),
	#print"{:12.9f}".format(Px),
	#print"{:12.9f}".format(Py)
	
	# reject bad sonar data
	if(abs(altitude_px4 - Pz_prev_re)>0.2):
		Pz_rejected = Pz_prev_re
		alt_counter = alt_counter + 1
		if (alt_counter == 35): #if 35 consecutive readings agree, probably true
			Pz_rejected = altitude_px4
			alt_counter = 0
	else:
		Pz_rejected = altitude_px4
		alt_counter = 0
	
	# filter rejected altitude estimate
	B_comp = 0.3 #0.008
	Pz = (1.0 - B_comp)*Pz_prev + B_comp*Pz_rejected

	Pz_prev = Pz
	Pz_prev_re = Pz_rejected
	altitude_px4_prev = altitude_px4


#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global q0, q1, q2, q3
	global roll, pitch, yaw
	global Ax_IMU, Ay_IMU, Az_IMU, Vx_IMU, Vy_IMU
	global Gx, Gy, Gz
	global Vx_px4, Vy_px4, altitude_px4, quality_px4
	global Vx_px4_LP, Vx_px4_LP_prev, Vy_px4_LP, Vy_px4_LP_prev
	global Vx, Vx_prev, Vy, Vy_prev
	global Px, Py, Pz, Pz_prev, altitude_px4_prev
	global Pz_prev_re
	global alt_counter
	global dt
	Vx_IMU = Vy_IMU = Vx_prev = Vy_prev = Vx_px4_LP = Vx_px4_LP_prev = Vy_px4_LP = Vy_px4_LP_prev = Gx = Gy = Gz = 0.0
	q1 = q2 = q3 = q0 = 0.0
	altitude_px4 = Vy_px4 = Vx_px4 = 0.0
	Px = Py = Pz = altitude_px4_prev = 0
	Pz_prev = 0.3
	Pz_prev_re = 0.3
	Vx = Vy = 0
	quality_px4 = 0
	alt_counter = 0
	
	# Initialize node
	rospy.init_node('odom', anonymous=True)
	rate = rospy.Rate(50) # Hz
	dt = 1.0/50.0
	
	# Initialize topics to publish
	pub_pose = rospy.Publisher('/odom/pose', PoseStamped, queue_size=1)
	pub_Vx = rospy.Publisher('/odom/Vel_x', Float64, queue_size=1)
	pub_Vy = rospy.Publisher('/odom/Vel_y', Float64, queue_size=1)
	
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/IMU/pose', PoseStamped, pose_feedback)
	rospy.Subscriber('/IMU/accel', AccelStamped, accel_feedback)
	rospy.Subscriber('/IMU/twist', TwistStamped, twist_feedback)
	rospy.Subscriber('/px4_data/quality', Int32, quality_feedback)
	rospy.Subscriber('/px4_data/x_vel', Float64, x_vel_feedback)
	rospy.Subscriber('/px4_data/y_vel', Float64, y_vel_feedback)
	rospy.Subscriber('/px4_data/altitude', Float64, altitude_feedback)
	rospy.Subscriber('/px4_data/quality', Int32, quality_feedback)
	
	
	while not rospy.is_shutdown():
		try:		
			# Do stuff 
			fuseData()

			# Publish to topics
			p = publishPose() #format data into posestamped struct
			pub_pose.publish(p)
			pub_Vx.publish(Vx)
			pub_Vy.publish(Vy)

		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in odometry.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


