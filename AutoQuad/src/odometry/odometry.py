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
	
def x_vel_feedback(data):
	global Vx_px4
	Vx_px4 = data.data
	
def y_vel_feedback(data):
	global Vy_px4
	Vy_px4 = data.data
	
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
	global Vx_px4, Vy_px4, altitude_px4, quality_px4
	global Vx_px4_LP, Vx_px4_LP_prev, Vy_px4_LP, Vy_px4_LP_prev
	global Vx, Vx_prev, Vy, Vy_prev
	global Px, Py, Pz, Pz_prev
	global dt
	
	# use orientation to get linear body accelerations 
	Ax = Ax_IMU + sin(pitch)
	Ay = Ay_IMU - sin(roll)
	Az = Az_IMU + abs(Ax_IMU*sin(roll)) + abs(Ay_IMU*sin(pitch)) - 1.0
	
	# integrate body accelerations for IMU velocity estimate
	Vx_IMU = Vx_IMU + Ax*dt*9.81
	Vy_IMU = Vy_IMU + Ay*dt*9.81
	
	# LP filter px4 velocity measurements
	B_px4 = 0.25
	Vx_px4_LP = (1.0 - B_px4)*Vx_px4_LP_prev + B_px4*Vx_px4
	Vy_px4_LP = (1.0 - B_px4)*Vy_px4_LP_prev + B_px4*Vy_px4
	Vx_px4_LP_prev = Vx_px4_LP
	Vy_px4_LP_prev = Vy_px4_LP
	
	# fuse IMU velocity estimate with px4 velocity estimate
	B_comp = .5
	Vx = (1.0 - B_comp)*(Vx_prev + Ax*dt*9.81) + B_comp*Vx_px4_LP
	Vy = (1.0 - B_comp)*(Vy_prev + Ay*dt*9.81) - B_comp*Vy_px4_LP
	Vx_prev = Vx
	Vy_prev = Vy
	
	# integrate fused velocity for position estimate 
	Px = Px + Vx*dt
	Py = Py + Vy*dt

	#print"{:8.4f}".format(Vx),
	#print"{:8.4f}".format(Vy),
	#print"{:8.4f}".format(Px),
	#print"{:8.4f}".format(Py)
	
	# fuse IMU altitude estimate with px4 altitude estimate
	B_comp = .5
	Pz = (1.0 - B_comp)*(Pz_prev + Az*dt*dt*9.81) + B_comp*altitude_px4
	Pz_prev = Pz
	#print"{:8.4f}".format(altitude_px4),
	#print"{:8.4f}".format(Az*dt*dt*1000.0),
	#print"{:8.4f}".format(Pz)
	

#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global q0, q1, q2, q3
	global roll, pitch, yaw
	global Ax_IMU, Ay_IMU, Az_IMU, Vx_IMU, Vy_IMU
	global Vx_px4, Vy_px4, altitude_px4, quality_px4
	global Vx_px4_LP, Vx_px4_LP_prev, Vy_px4_LP, Vy_px4_LP_prev
	global Vx, Vx_prev, Vy, Vy_prev
	global Px, Py, Pz, Pz_prev
	global dt
	Vx_IMU = Vy_IMU = Vx_prev = Vy_prev = Vx_px4_LP = Vx_px4_LP_prev = Vy_px4_LP = Vy_px4_LP_prev =  0.0
	Px = Py = Pz = Pz_prev = 0
	
	# Initialize node
	rospy.init_node('odom', anonymous=True)
	rate = rospy.Rate(500) # Hz
	dt = 1.0/500.0
	
	# Initialize topics to publish
	pub_pose = rospy.Publisher('/odom/pose', PoseStamped, queue_size=1)
	pub_Vx = rospy.Publisher('/odom/Vel_x', Float64, queue_size=1)
	pub_Vy = rospy.Publisher('/odom/Vel_y', Float64, queue_size=1)
	
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/IMU/pose', PoseStamped, pose_feedback)
	rospy.Subscriber('/IMU/accel', AccelStamped, accel_feedback)
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
			#traceback.print_exc()
			#rospy.loginfo('Some error ocurred in odometry.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


