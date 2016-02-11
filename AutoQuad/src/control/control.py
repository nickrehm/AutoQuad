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
	global altitude
	altitude = data.pose.position.z
	
def Vx_feedback(data):
	global Vx
	Vx = data.data
	
def Vy_feedback(data):
	global Vy
	Vy = data.data

def alt_des_feedback(data):
	global alt_des
	alt_des = data.data
	
def Vx_des_feedback(data):
	global Vx_des
	Vx_des = data.data

def Vy_des_feedback(data):
	global Vy_des
	Vy_des = data.data
	
def yaw_des_feedback(data):
	global yaw_des
	yaw_des = data.data
	
def accel_feedback(data):
	global Ax_IMU, Ay_IMU, Az_IMU
	Ax_IMU = data.accel.linear.x
	Ay_IMU = data.accel.linear.y
	Az_IMU = data.accel.linear.z
	
def constrain(val, min_val, max_val):

    if val < min_val: return min_val
    if val > max_val: return max_val
    return val
	
def controlAlt(Kp,Ki,Kd,K):
	global Az_IMU, dt
	global alt_des, altitude, integral_alt_prev, thro_pwm
	# PI controller for altitude, setpoint in meters
	error_limit = 0.3 #meters
	i_limit = 0.75 #arbitrary units
	PID_limit = 350.0 #+/- pwm 
	
	error = alt_des - altitude
	error = constrain(error, -error_limit, error_limit)
	integral = integral_alt_prev + error*dt
	integral = constrain(integral, -i_limit, i_limit) #prevent windup
	integral_alt_prev = integral
	derivative = (Az_IMU - 1.0)*dt*10.0
	PID = K*(Kp*error + Ki*integral - Kd*derivative)
	PID = constrain(PID,-PID_limit, PID_limit)
	
	# Convert command to int between 1000 and 2000
	thro_pwm = 1500.0 + PID
	thro_pwm = constrain(thro_pwm, 1000.0, 2000.0)
	thro_pwm = int(thro_pwm)
	
def controlVelocity(Kp,K):
	global Vx_des, Vy_des, Vx, Vy, roll_pwm, pitch_pwm
	# P controller for velocity, setpoint in m/s
	error_limit = 0.5 #m/s
	
	error = Vx_des - Vx
	error = constrain(error, -error_limit, error_limit)
	PIDx = K*(Kp*error)
	
	error = Vy_des - Vy
	error = constrain(error, -error_limit, error_limit)
	PIDy = K*(Kp*error)
	
	# Convert command to int between 1000 and 2000
	roll_pwm = 1500.0 - PIDy
	roll_pwm = constrain(roll_pwm, 1000.0, 2000.0)
	roll_pwm = int(roll_pwm)
	pitch_pwm = 1500.0 + PIDx
	pitch_pwm = constrain(pitch_pwm, 1000.0, 2000.0)
	pitch_pwm = int(pitch_pwm)
	

#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global Ax_IMU, Ay_IMU, Az_IMU
	global altitude, Vx, Vy
	global alt_des, Vx_des, Vy_des, yaw_des
	global thro_pwm, roll_pwm, pitch_pwm, yaw_pwm
	global integral_alt_prev
	global dt
	thro_pwm = 1000
	roll_pwm = pitch_pwm = yaw_pwm = 1500
	alt_des = Vx_des = Vy_des = yaw_des = 0
	integral_alt_prev = altitude = 0
	Vx = Vy = 0
	Az_IMU = 0
	
	alt_des = 1.0
	
	# Initialize node
	rospy.init_node('control', anonymous=True)
	rate = rospy.Rate(100) # Hz
	dt = 1.0/100.0
	
	# Initialize topics to publish
	pub_thro = rospy.Publisher('/control/thro_pwm', Int32, queue_size=1)
	pub_roll = rospy.Publisher('/control/roll_pwm', Int32, queue_size=1)
	pub_pitch = rospy.Publisher('/control/pitch_pwm', Int32, queue_size=1)
	pub_yaw = rospy.Publisher('/control/yaw_pwm', Int32, queue_size=1)
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/odom/pose', PoseStamped, pose_feedback)
	rospy.Subscriber('/odom/Vel_x', Float64, Vx_feedback)
	rospy.Subscriber('/odom/Vel_y', Float64, Vy_feedback)
	rospy.Subscriber('/control/alt_des', Float64, alt_des_feedback)
	rospy.Subscriber('/control/Vx_des', Float64, Vx_des_feedback)
	rospy.Subscriber('/control/Vy_des', Float64, Vy_des_feedback)
	rospy.Subscriber('/control/yaw_des', Float64, yaw_des_feedback)
	rospy.Subscriber('/IMU/accel', AccelStamped, accel_feedback)
	
	
	while not rospy.is_shutdown():
		try:		
			# Do stuff 
			controlAlt(Kp = 0.3, Ki = 0.1, Kd = 0.2, K = 1000.0)
			controlVelocity(Kp = 0.8, K = 1000.0)

			# Publish to topics
			pub_thro.publish(thro_pwm)
			pub_roll.publish(roll_pwm)
			pub_pitch.publish(pitch_pwm)
			pub_yaw.publish(yaw_pwm)
			

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

