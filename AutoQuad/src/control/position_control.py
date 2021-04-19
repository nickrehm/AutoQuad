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

def alt_des_feedback(data):
	global alt_des
	alt_des = data.data
	
def Xpos_des_feedback(data):
	global Xpos_des
	Xpos_des = data.data

def Ypos_des_feedback(data):
	global Ypos_des
	Ypos_des = data.data
	
def yaw_des_feedback(data):
	global yaw_des
	yaw_des = data.data
	
def RC_feedback(data):
	global radio_command
	radio_command = data.data
	
def odom_pose_feedback(data):
	global yaw_angle, Xpos, Ypos, alt, dx, dy, dalt
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	
	dx = data.twist.twist.linear.x
	dy = -data.twist.twist.linear.y
	dalt = -data.twist.twist.linear.z
	
	#matrix = quaternion_matrix([q1, q2, q3, q0])
	#v = numpy.dot(numpy.transpose([x, y, z, 0.0]),matrix) #converts to body frame
	Xpos = -x #v[0]
	Ypos = -y #v[1]
	alt = z #-v[2]
	
	pitch, roll, yaw = quaternion_to_euler(q0, q1, q2, q3)
	yaw_angle = yaw*180.0/3.14159
	if yaw_angle < 0:
		yaw_angle = yaw_angle+180.0
	else:
		yaw_angle = yaw_angle-180.0
	
	
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
	
def controlAlt(Kp,Ki,Kd,K,hov_pwm):
	global dt
	global alt_des, alt, dalt, integral_alt_prev, thro_pwm
	global altitude_prev
	global derivative_alt
	# PID controller for altitude, setpoint in meters
	error_limit = 0.3 #meters
	i_limit = 5.0 #arbitrary units

	error = alt_des - alt
	error = constrain(error, -error_limit, error_limit)
	integral = integral_alt_prev + error*dt
	integral = constrain(integral, -i_limit, i_limit) #prevent windup
	if(radio_command == 1): #reset integral when entering alt hold 
		integral = 0.0
	integral_alt_prev = integral
	#B_der = 0.3
	#dz = alt - altitude_prev
	#derivative_alt = (1.0 - B_der)*derivative_alt + B_der*dz #LP filter derivative signal
	altitude_prev = alt
	PID = K*(Kp*error + Ki*integral - Kd*dalt)
	
	altitude_prev = alt
	
	# Convert command to int between 1000 and 2000
	thro_pwm = hov_pwm + PID
	thro_pwm = constrain(thro_pwm, hov_pwm - 200.0, hov_pwm + 200.0)
	thro_pwm = int(thro_pwm)
	
def controlXY(Kp,Ki,Kd,K,x_offset,y_offset):
	global dt
	global Xpos_des, Ypos_des, Xpos, Ypos, dx, dy, roll_pwm, pitch_pwm, integral_x, integral_y
	global roll_pwm, pitch_pwm
	global ex_prev, ey_prev
	# PID controller for position
	error_limit = 0.3 #meters
	i_limit = 50 #pwm
	B = 0.1
	
	# x-direction
	error = x_offset + Xpos - Xpos_des 
	#error = (1.0 - B)*ex_prev + B*error
	error = constrain(error, -error_limit, error_limit)
	integral_x = integral_x + error
	if(Ki != 0.0):
		integral_x = constrain(K*Ki*integral_x, -i_limit, i_limit)/(K*Ki)
	if(radio_command == 1): #reset integral when entering autonomy
		integral_x = 0.0
	derivative = dx
	PIDx = K*Kp*error + constrain(K*Ki*integral_x, -i_limit, i_limit) - K*Kd*derivative
	ex_prev = error
	
	# y-direction
	error = y_offset + Ypos - Xpos_des 
	#error = (1.0 - B)*ey_prev + B*error
	error = constrain(error, -error_limit, error_limit)
	integral_y = integral_y + error
	if(Ki != 0.0):
		integral_y = constrain(K*Ki*integral_y, -i_limit, i_limit)/(K*Ki)
	if(radio_command == 1): #reset integral when entering autonomy
		integral_y = 0.0
	derivative = dy
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
	global yaw_des, yaw_angle, integral_yaw
	global radio_command
	global yaw_pwm
	# PI controller for orientation, setpoint in degrees (forced to 0.0)
	error_limit = 20.0 #degrees
	i_limit = 30 #pwm
	
	error = yaw_des - yaw_angle
	error = constrain(error, -error_limit, error_limit)
	integral_yaw = integral_yaw + error
	integral_yaw = constrain(K*Ki*integral_yaw, -i_limit, i_limit)/(K*Ki)
	if(radio_command == 1): #reset integral when entering autonomy
		integral_yaw = 0.0
	PID = K*Kp*error + constrain(K*Ki*integral_yaw, -i_limit, i_limit)
	
	# Convert command to int between 1000 and 2000
	yaw_pwm = 1500.0 + PID
	yaw_pwm = constrain(yaw_pwm, 1000.0, 2000.0)
	yaw_pwm = int(yaw_pwm)
	

	

#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global alt, Xpos, Ypos
	global alt_des, Xpos_des, Ypos_des, yaw_des, yaw_angle, dx, dy, dalt
	global thro_pwm, roll_pwm, pitch_pwm, yaw_pwm
	global integral_alt_prev, altitude_prev
	global derivative_alt
	global integral_x, integral_y
	global radio_command
	global dt
	global integral_yaw
	global ex_prev, ey_prev
	thro_pwm = 1000
	roll_pwm = pitch_pwm = yaw_pwm = 1500
	radio_command = 1
	alt_des = Xpos_des = Ypos_des = yaw_des = 0.0
	Xpos = Ypos = alt = yaw_angle = 0.0
	dx = dy = dalt = 0.0
	
	integral_alt_prev = derivative_alt = altitude = altitude_prev = 0.0
	integral_x = integral_y = 0.0
	integral_yaw = 0.0
	ex_prev = ey_prev = 0.0
	
	
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
	rospy.Subscriber('/camera/odom/sample', Odometry, odom_pose_feedback)
	rospy.Subscriber('/control/alt_des', Float64, alt_des_feedback)
	rospy.Subscriber('/control/Xpos_des', Float64, Xpos_des_feedback) 
	rospy.Subscriber('/control/Ypos_des', Float64, Ypos_des_feedback) 
	rospy.Subscriber('/control/yaw_des', Float64, yaw_des_feedback)
	rospy.Subscriber('/radio_command', Int32, RC_feedback)

	
	
	
	while not rospy.is_shutdown():
		try:		
			# Do stuff 
			
			#IMPORTANT VARIABLES: yaw_angle, Xpos, Ypos, alt, alt_des, Xpos_des, Ypos_des, yaw_des, dx, dy, dalt
			yaw_des = 0.0; #do not change!!!
			alt_des = 1.0; #set constant for runs
			
			
			controlAlt(Kp = 0.12, Ki = 0.015, Kd = 0.1, K = 1000.0, hov_pwm = 1400.0) 
			controlXY(Kp = 0.15, Ki = 0.005, Kd = 0.12, K = 1000.0, x_offset = 0.0 , y_offset = 0.0)
			controlYaw(Kp = 0.15, Ki = 0.0003, K = 10.0)
			
			
			# Publish to topics
			pub_thro.publish(thro_pwm)
			pub_roll.publish(roll_pwm)
			pub_pitch.publish(pitch_pwm)
			pub_yaw.publish(yaw_pwm)
			

		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in position_control.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


