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
	port = '/dev/ttyAMA0',
	baudrate = 3000000,
	timeout=1)


#############
# Functions #
#############

def receiveData():
	global q0, q1, q2, q3, GyroX, GyroY, GyroZ, AccX, AccY, AccZ, radio_command
	
	if(ser.read() == 'a'):
		q0 = ser.readline()
		q0 = float(ser.readline().strip())
		if(ser.read()=='b'):
			q1 = ser.readline()
			q1 = float(ser.readline())
		if(ser.read()=='c'):
			q2 = ser.readline()
			q2 = float(ser.readline())
		if(ser.read()=='d'):
			q3 = ser.readline()
			q3 = float(ser.readline())
		if(ser.read()=='e'):
			GyroX = ser.readline()
			GyroX = float(ser.readline())
		if(ser.read()=='f'):
			GyroY = ser.readline()
			GyroY = float(ser.readline())
		if(ser.read()=='g'):
			GyroZ = ser.readline()
			GyroZ = float(ser.readline())
		if(ser.read()=='h'):
			AccX = ser.readline()
			AccX = float(ser.readline())
		if(ser.read()=='i'):
			AccY = ser.readline()
			AccY = float(ser.readline())
		if(ser.read()=='j'):
			AccZ = ser.readline()
			AccZ = float(ser.readline())
		if(ser.read()=='k'):
			radio_command = ser.readline()
			radio_command = int(ser.readline())
		
def sendData():
	ser.write('a')
	#ser.write('*')
	ser.write(bytes(auto_thro_pwm))
	ser.write('b')
	
	ser.write('b')
	#ser.write('*')
	ser.write(bytes(auto_roll_pwm))
	ser.write('c')
	
	ser.write('c')
	#ser.write('*')
	ser.write(bytes(auto_pitch_pwm))
	ser.write('d')
	
	ser.write('d')
	#ser.write('*')
	ser.write(bytes(auto_yaw_pwm))
	ser.write('e')
	
def checkData():
	global q0, q1, q2, q3, GyroX, GyroY, GyroZ, AccX, AccY, AccZ, radio_command
	global q0_prev, q1_prev, q2_prev, q3_prev, GyroX_prev, GyroY_prev, GyroZ_prev, AccX_prev, AccY_prev, AccZ_prev, radio_command_prev
	check = 0
	
	if(isinstance(q0, float) == False):
		check = check + 1
	if(isinstance(q1, float) == False):
		check = check + 1
	if(isinstance(q2, float) == False):
		check = check + 1
	if(isinstance(q3, float) == False):
		check = check + 1
	if(isinstance(GyroX, float) == False):
		check = check + 1
	if(isinstance(GyroY, float) == False):
		check = check + 1
	if(isinstance(GyroZ, float) == False):
		check = check + 1
	if(isinstance(AccX, float) == False):
		check = check + 1
	if(isinstance(AccY, float) == False):
		check = check + 1
	if(isinstance(AccZ, float) == False):
		check = check + 1
	if(isinstance(radio_command, int) == False):
		check = check + 1
	
	if(check > 0):
		# print("Bad data packet received")
		q0 = q0_prev
		q1 = q1_prev
		q2 = q2_prev
		q3 = q3_prev
		GyroX = GyroX_prev
		GyroY = GyroY_prev
		GyroZ = GyroZ_prev
		AccX = AccX_prev
		AccY = AccY_prev
		AccZ = AccZ_prev
		radio_command = radio_command_prev
	
	q0_prev = q0
	q1_prev = q1
	q2_prev = q2
	q3_prev = q3
	GyroX_prev = GyroX
	GyroY_prev = GyroY
	GyroZ_prev = GyroZ
	AccX_prev = AccX
	AccY_prev = AccY
	AccZ_prev = AccZ

def printData():
	print('  q0   '),
	print('q1   '),
	print('q2   '),
	print('q3   '),
	print('GX   '),
	print('GY   '),
	print('GZ  '),
	print('AccX '),
	print('AccY '),
	print('AccZ '),
	print('radio_command')
	print"{:5.2f}".format(q0),
	print"{:5.2f}".format(q1), 
	print"{:5.2f}".format(q2), 
	print"{:5.2f}".format(q3), 
	print"{:5.2f}".format(GyroX), 
	print"{:5.2f}".format(GyroY), 
	print"{:5.2f}".format(GyroZ), 
	print"{:5.2f}".format(AccX), 
	print"{:5.2f}".format(AccY), 
	print"{:5.2f}".format(AccZ),
	print"{:7}".format(radio_command)
	print('')
	
	
def publishPose():
	p = PoseStamped()
	p.header.frame_id = "map"
	p.header.stamp = rospy.Time.now()
	p.pose.position.x = 0.0 
	p.pose.position.y = 0.0
	p.pose.position.z = 0.0
	p.pose.orientation.x = q1
	p.pose.orientation.y = q2
	p.pose.orientation.z = q3
	p.pose.orientation.w = q0
	return p
	
def publishTwist():
	t = TwistStamped()
	t.header.frame_id = "map"
	t.header.stamp = rospy.Time.now()
	t.twist.linear.x = 0.0 
	t.twist.linear.y = 0.0
	t.twist.linear.z = 0.0
	t.twist.angular.x = GyroX
	t.twist.angular.y = GyroY
	t.twist.angular.z = GyroZ	
	return t

def publishAccel():
	a = AccelStamped()
	a.header.frame_id = "map"
	a.header.stamp = rospy.Time.now()
	a.accel.linear.x = AccX
	a.accel.linear.y = AccY
	a.accel.linear.z = AccZ
	return a

def thro_pwm_feedback(data):
	global auto_thro_pwm
	auto_thro_pwm = data.data

def roll_pwm_feedback(data):
	global auto_roll_pwm
	auto_roll_pwm = data.data

def pitch_pwm_feedback(data):
	global auto_pitch_pwm
	auto_pitch_pwm = data.data
	
def yaw_pwm_feedback(data):
	global auto_yaw_pwm
	auto_yaw_pwm = data.data

#############
# MAIN LOOP #
#############

def main():
	# Declare variables 
	global q0, q1, q2, q3, GyroX, GyroY, GyroZ, AccX, AccY, AccZ, radio_command
	global q0_prev, q1_prev, q2_prev, q3_prev, GyroX_prev, GyroY_prev, GyroZ_prev, AccX_prev, AccY_prev, AccZ_prev, radio_command_prev
	global auto_thro_pwm, auto_roll_pwm, auto_pitch_pwm, auto_yaw_pwm 
	auto_thro_pwm = 1000
	auto_roll_pwm = auto_pitch_pwm = auto_yaw_pwm = 1500
	q0 = q1 = q2 = q3 = GyroX = GyroY = GyroZ = AccX = AccY = AccZ = 0
	radio_command = 1
	
	# Initialize node
	rospy.init_node('serialComm', anonymous=True)
	rate = rospy.Rate(100) # Hz
	
	# Initialize topics to publish
	pub_pose = rospy.Publisher('/IMU/pose', PoseStamped, queue_size=1)
	pub_twist = rospy.Publisher('/IMU/twist', TwistStamped, queue_size=1)
	pub_accel = rospy.Publisher('/IMU/accel', AccelStamped, queue_size=1)
	pub_radio_command = rospy.Publisher('/radio_command', Int32, queue_size=1)
	
	# Subscribe to topics with reference to callback functions
	rospy.Subscriber('/control/thro_pwm', Int32, thro_pwm_feedback)
	rospy.Subscriber('/control/roll_pwm', Int32, roll_pwm_feedback)
	rospy.Subscriber('/control/pitch_pwm', Int32, pitch_pwm_feedback)
	rospy.Subscriber('/control/yaw_pwm', Int32, yaw_pwm_feedback)
	
	while not rospy.is_shutdown():
		try:		
			# receive/send data 
			ser.reset_input_buffer()
			ser.reset_output_buffer()
			receiveData()
			sendData()
			checkData()
			#printData()
			
			# Publish to topics
			p = publishPose() #format data into posestamped struct
			pub_pose.publish(p)
			t = publishTwist() #format data into twiststamped struct
			pub_twist.publish(t)
			a = publishAccel() #format data into accelstamped struct
			pub_accel.publish(a)
			pub_radio_command.publish(radio_command)


		except Exception:
			#traceback.print_exc()
			#rospy.loginfo('Some error ocurred in serialComm.py')
			indent = 1
		
		rate.sleep()
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


