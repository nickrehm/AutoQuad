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
	
def controlAlt(Kp,Ki,Kd,K,hov_pwm):
	global Az_IMU, dt
	global alt_des, altitude, integral_alt_prev, thro_pwm
	global altitude_prev
	global derivative_alt
	# PI controller for altitude, setpoint in meters
	error_limit = 0.5 #meters
	i_limit = 5.0 #arbitrary units

	error = alt_des - altitude
	error = constrain(error, -error_limit, error_limit)
	integral = integral_alt_prev + error*dt
	integral = constrain(integral, -i_limit, i_limit) #prevent windup
	integral_alt_prev = integral
	B_der = 0.008
	derivative_alt = (1 - B_der)*derivative_alt + B_der*(altitude - altitude_prev)*dt*1000.0
	altitude_prev = altitude
	PID = K*(Kp*error + Ki*integral - Kd*derivative_alt)
	
	#print"{:12.9f}".format(Kd*derivative)
	
	# Convert command to int between 1000 and 2000
	thro_pwm = hov_pwm + PID
	thro_pwm = constrain(thro_pwm, hov_pwm - 75.0, hov_pwm + 75.0)
	thro_pwm = int(thro_pwm)
	
def controlVelocity(Kp,Ki,K):
	global Vx_des, Vy_des, Vx, Vy, roll_pwm, pitch_pwm, integral_x, integral_y
	# P controller for velocity, setpoint in m/s
	error_limit = 0.5 #m/s
	i_limit = 0.1
	
	error = Vx_des - Vx
	error = constrain(error, -error_limit, error_limit)
	integral_x = integral_x + error
	integral_x = constrain(integral_x, -i_limit, i_limit)
	PIDx = K*(Kp*error + Ki*integral_x)
	
	error = Vy_des - Vy
	error = constrain(error, -error_limit, error_limit)
	integral_y = integral_y + error
	integral_y = constrain(integral_y, -i_limit, i_limit)
	PIDy = K*(Kp*error + Ki*integral_y)
	
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
	global integral_alt_prev, altitude_prev
	global derivative_alt
	global integral_x, integral_y
	global dt
	thro_pwm = 1000
	roll_pwm = pitch_pwm = yaw_pwm = 1500
	alt_des = Vx_des = Vy_des = yaw_des = 0
	integral_alt_prev = derivative_alt = altitude = altitude_prev = 0
	Vx = Vy = 0
	integral_x = integral_y = 0
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
			controlAlt(Kp = 0.005, Ki = 0.007, Kd = 0.3, K = 1000.0, hov_pwm = 1480.0) 
			controlVelocity(Kp = 0.16, Ki = 0.1, K = 1000.0)

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


