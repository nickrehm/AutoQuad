#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Empty, Int32
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, sqrt, asin, acos, atan
import numpy
import traceback
import time
from tf.transformations import quaternion_matrix


##################
# Initialization #
##################




#############
# Functions #
#############

def odom_pose_feedback(data):
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
	
def RC_feedback(data):
	global radio_command
	radio_command = data.data
	
def target_pose_feedback(data):
	global target_orientation, target_x, target_y, target_z
	x = data.pose.position.x
	y = data.pose.position.y
	z = data.pose.position.z
	q0 = data.pose.orientation.w
	q1 = data.pose.orientation.x
	q2 = data.pose.orientation.y
	q3 = data.pose.orientation.z
	
	matrix = quaternion_matrix([q1, q2, q3, q0])
	v = numpy.dot(numpy.transpose([x, y, z, 0.0]),matrix)
	target_x = -v[1]
	target_y = v[0]
	target_z = -v[2]
	
	pitch, roll, yaw = quaternion_to_euler(q0, q1, q2, q3)
	target_orientation = yaw*180.0/3.14159
	
def target_detected_feedback(data):
	global target_detected
	target_detected = data.data
	
def constrain(val, min_val, max_val):
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val
    
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
	
def controlAlt_odom(Kp,Ki,Kd,K,hov_pwm):
	global Az_IMU, dt
	global alt_des, altitude, integral_alt_prev, thro_pwm
	global altitude_prev
	global derivative_alt
	# PI controller for altitude, setpoint in meters
	error_limit = 0.3 #meters
	i_limit = 5.0 #arbitrary units

	error = alt_des - altitude
	error = constrain(error, -error_limit, error_limit)
	integral = integral_alt_prev + error*dt
	integral = constrain(integral, -i_limit, i_limit) #prevent windup
	if(radio_command == 1): #reset integral when entering alt hold 
		integral = 0.0
	integral_alt_prev = integral
	B_der = 0.25
	dz = altitude - altitude_prev
	derivative_alt = (1.0 - B_der)*derivative_alt + B_der*dz
	altitude_prev = altitude
	PID = K*(Kp*error + Ki*integral - Kd*derivative_alt)
	#print"{:12.4f}".format(K*Ki*integral)
	
	# Convert command to int between 1000 and 2000
	thro_pwm = hov_pwm + PID
	thro_pwm = constrain(thro_pwm, hov_pwm - 125.0, hov_pwm + 125.0)
	thro_pwm = int(thro_pwm)
	
def controlVelocity_odom(Kp,Ki,K):
	global Vx_des, Vy_des, Vx, Vy, roll_pwm, pitch_pwm, integral_x, integral_y
	# PI controller for velocity, setpoint in m/s
	error_limit = 0.8 #m/s
	i_limit = 250 #pwm
	
	# x-direction
	error = Vx_des - Vx
	error = constrain(error, -error_limit, error_limit)
	integral_x = integral_x + error
	if(radio_command == 1): #reset integral when entering autonomy 
		integral_x = 0.0
	PIDx = K*Kp*error + constrain(K*Ki*integral_x, -i_limit, i_limit)
	
	# y-direction
	error = Vy_des - Vy
	error = constrain(error, -error_limit, error_limit)
	integral_y = integral_y + error
	if(radio_command == 1): #reset integral when entering autonomy
		integral_y = 0.0
	PIDy = K*Kp*error + constrain(K*Ki*integral_y, -i_limit, i_limit)
	
	# Convert command to int between 1000 and 2000
	roll_pwm = 1500.0 - PIDy
	roll_pwm = constrain(roll_pwm, 1000.0, 2000.0)
	roll_pwm = int(roll_pwm)
	pitch_pwm = 1500.0 + PIDx
	pitch_pwm = constrain(pitch_pwm, 1000.0, 2000.0)
	pitch_pwm = int(pitch_pwm)
	
def controlAlt_target(Kp,Ki,Kd,K,hov_pwm):
	global dt
	global alt_des, target_z, integral_alt_prev, thro_pwm
	global altitude_prev
	global derivative_alt
	# PI controller for altitude, setpoint in meters
	error_limit = 0.3 #meters
	i_limit = 5.0 #arbitrary units

	error = alt_des - target_z
	error = constrain(error, -error_limit, error_limit)
	integral = integral_alt_prev + error*dt
	integral = constrain(integral, -i_limit, i_limit) #prevent windup
	if(radio_command == 1): #reset integral when entering alt hold 
		integral = 0.0
	integral_alt_prev = integral
	B_der = 0.2
	dz = target_z - altitude_prev
	derivative_alt = (1.0 - B_der)*derivative_alt + B_der*dz
	altitude_prev = target_z
	PID = K*(Kp*error**3 + Ki*integral - Kd*derivative_alt)
	
	altitude_prev = target_z
	
	# Convert command to int between 1000 and 2000
	thro_pwm = hov_pwm + PID
	thro_pwm = constrain(thro_pwm, hov_pwm - 125.0, hov_pwm + 125.0)
	thro_pwm = int(thro_pwm)
	
def controlPos_target(Kp,Ki,Kd,K,x_offset,y_offset):
	global dt
	global target_x, target_y, Vx, Vy, roll_pwm, pitch_pwm, integral_tx, integral_ty
	global roll_pwm, pitch_pwm
	global ex_prev, ey_prev
	# PID controller for position, setpoint x = 0.0m, y = 0.0m (plus offset)
	error_limit = 0.3 #meters
	i_limit = 50 #pwm
	B = 0.1
	
	# x-direction
	error = x_offset - target_x 
	error = (1.0 - B)*ex_prev + B*error
	error = constrain(error, -error_limit, error_limit)
	integral_tx = integral_tx + error
	if(Ki != 0.0):
		integral_tx = constrain(K*Ki*integral_tx, -i_limit, i_limit)/(K*Ki)
	if(radio_command == 1): #reset integral when entering autonomy
		integral_tx = 0.0
	derivative = Vx
	PIDx = K*Kp*error + constrain(K*Ki*integral_tx, -i_limit, i_limit) - K*Kd*derivative
	ex_prev = error
	
	# y-direction
	error = y_offset - target_y 
	error = (1.0 - B)*ey_prev + B*error
	error = constrain(error, -error_limit, error_limit)
	integral_ty = integral_ty + error
	if(Ki != 0.0):
		integral_ty = constrain(K*Ki*integral_ty, -i_limit, i_limit)/(K*Ki)
	if(radio_command == 1): #reset integral when entering autonomy
		integral_ty = 0.0
	derivative = Vy
	PIDy = K*Kp*error + constrain(K*Ki*integral_y, -i_limit, i_limit) - K*Kd*derivative
	ey_prev = error
	
	# Convert command to int between 1000 and 2000
	roll_pwm = 1500.0 - PIDy 
	roll_pwm = constrain(roll_pwm, 1000.0, 2000.0)
	roll_pwm = int(roll_pwm)
	pitch_pwm = 1500.0 + PIDx
	pitch_pwm = constrain(pitch_pwm, 1000.0, 2000.0)
	pitch_pwm = int(pitch_pwm)

	
def controlYaw(Kp,Ki,K):
	global dt
	global target_orientation, integral_yaw
	global radio_command
	global yaw_pwm
	# PI controller for orientation, setpoint in degrees (forced to 0.0)
	error_limit = 20.0 #degrees
	i_limit = 30 #pwm
	
	error = -target_orientation
	error = constrain(error, -error_limit, error_limit)
	integral_yaw = integral_yaw + error
	integral_yaw = constrain(K*Ki*integral_yaw, -i_limit, i_limit)/(K*Ki)
	if(radio_command == 1): #reset integral when entering autonomy
		integral_tx = 0.0
	PID = K*Kp*error + constrain(K*Ki*integral_yaw, -i_limit, i_limit)
	
	# Convert command to int between 1000 and 2000
	yaw_pwm = 1500.0 + PID
	yaw_pwm = constrain(yaw_pwm, 1000.0, 2000.0)
	yaw_pwm = int(yaw_pwm)
	
def request_target_velocity(Kp,B):
	global Vx_des, Vy_des
	global target_x, target_y
	global Vx_des_prev, Vy_des_prev
	v_limit = 0.07
	
	Vx_des = -Kp*target_x
	Vy_des = -Kp*target_y #check sign
	Vx_des = constrain(Vx_des,-v_limit, v_limit)
	Vy_des = constrain(Vy_des,-v_limit, v_limit)
	
	Vx_des = (1.0 - B)*Vx_des_prev + B*Vx_des
	Vy_des = (1.0 - B)*Vy_des_prev + B*Vy_des
	Vx_des_prev = Vx_des
	Vy_des_prev = Vy_des
	

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
	global integral_tx, integral_ty
	global radio_command
	global dt
	global target_orientation, target_x, target_y, target_z, target_detected 
	global integral_yaw
	global ex_prev, ey_prev
	global Vx_des_prev, Vy_des_prev
	thro_pwm = 1000
	roll_pwm = pitch_pwm = yaw_pwm = 1500
	alt_des = Vx_des = Vy_des = yaw_des = 0.0
	integral_alt_prev = derivative_alt = altitude = altitude_prev = 0.0
	Vx = Vy = 0.0
	integral_x = integral_y = 0.0
	integral_tx = integral_ty = 0.0
	integral_yaw = 0.0
	Az_IMU = 0.0
	ex_prev = ey_prev = 0.0
	Vx_des_prev = Vy_des_prev = 0.0
	radio_command = 1
	target_detected = 0
	alt_des = 1.0
	
	# Initialize node
	rospy.init_node('control', anonymous=True)
	rate = rospy.Rate(50) # Hz
	dt = 1.0/50.0
	
	# Initialize topics to publish
	pub_thro = rospy.Publisher('/control/thro_pwm', Int32, queue_size=1)
	pub_roll = rospy.Publisher('/control/roll_pwm', Int32, queue_size=1)
	pub_pitch = rospy.Publisher('/control/pitch_pwm', Int32, queue_size=1)
	pub_yaw = rospy.Publisher('/control/yaw_pwm', Int32, queue_size=1)
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/odom/pose', PoseStamped, odom_pose_feedback)
	rospy.Subscriber('/odom/Vel_x', Float64, Vx_feedback)
	rospy.Subscriber('/odom/Vel_y', Float64, Vy_feedback)
	rospy.Subscriber('/control/alt_des', Float64, alt_des_feedback)
	rospy.Subscriber('/control/Vx_des', Float64, Vx_des_feedback)
	rospy.Subscriber('/control/Vy_des', Float64, Vy_des_feedback)
	rospy.Subscriber('/control/yaw_des', Float64, yaw_des_feedback)
	rospy.Subscriber('/IMU/accel', AccelStamped, accel_feedback)
	rospy.Subscriber('/radio_command', Int32, RC_feedback)
	rospy.Subscriber('/target/pose', PoseStamped, target_pose_feedback)
	rospy.Subscriber('/target/target_detected', Int32, target_detected_feedback)
	
	
	
	while not rospy.is_shutdown():
		try:		
			# Do stuff 
			
			
			if(target_detected == 1):
				alt_des = alt_des - dt*.08 #descend
				alt_des = constrain(alt_des, 0.0, 1.5)
				if (alt_des < 0.3):
					alt_des = 0.0
					Vx_des = 0.0
					Vy_des = 0.0
					
				#controlAlt_target(Kp = 0.12, Ki = 0.015, Kd = 2.0, K = 1000.0, hov_pwm = 1400.0)
				#controlPos_target(Kp = 0.15, Ki = 0.005, Kd = 0.12, K = 1000.0, x_offset = 0.0 , y_offset = 0.0)
				controlAlt_odom(Kp = 0.15, Ki = .1, Kd = 3.5, K = 1000.0, hov_pwm = 1450.0)
				request_target_velocity(Kp = 0.07, B = 0.4)
				controlVelocity_odom(Kp = 0.15, Ki = 0.0033, K = 1000.0)
				controlYaw(Kp = 0.15, Ki = 0.0003, K = 10.0)
			else:
				if (alt_des < 0.4):
					alt_des = 0.0
					Vx_des = 0.0
					Vy_des = 0.0
				else:
					alt_des = alt_des + dt*.08 #ascend
					alt_des = constrain(alt_des, 0.0, 1.5)
					Vx_des = 0.1
					Vy_des = 0.0
				
				yaw_pwm = 1500
				controlAlt_odom(Kp = 0.15, Ki = .1, Kd = 3.5, K = 1000.0, hov_pwm = 1450.0) 
				controlVelocity_odom(Kp = 0.15, Ki = 0.0033, K = 1000.0)
			
			#print"{:12.9f}".format(Vx_des),
			#print"{:12.9f}".format(Vy_des)
		
			# Publish to topics
			pub_thro.publish(thro_pwm)
			pub_roll.publish(roll_pwm)
			pub_pitch.publish(pitch_pwm)
			pub_yaw.publish(yaw_pwm)
			

		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in control.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


