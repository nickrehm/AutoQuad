#!/usr/bin/env python

import serial
import time
import math

# configure serial connection
ser = serial.Serial(
	port = '/dev/ttyAMA0',
	baudrate = 3000000,
	timeout=1)


# loop
q0 = q1 = q2 = q3 = GyroX = GyroY = GyroZ = AccX = AccY = AccZ = 0
auto_thro_pwm = 1010
auto_roll_pwm = 1500
auto_pitch_pwm = 1500
auto_pitch_pwm = 1500
auto_yaw_pwm = 1500
dt = .002
t = 0
while 1:
	t = t + dt
	auto_roll_pwm = 1500 + int(100*math.sin(2.0*(t + dt)))

	try:
		# receive data 
		if(ser.read() == 'a'):
			q0 = ser.readline()
			q0 = float(ser.readline())
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
			ser.reset_input_buffer()

		# print recieved data
		print('   q0   '),
		print('q1    '),
		print('q2    '),
		print('q3    '),
		print('GyroX '),
		print('GyroY '),
		print('GyroZ '),
		print('AccX  '),
		print('AccY  '),
		print('AccZ')
		print"{:6.2f}".format(q0),
		print"{:6.2f}".format(q1), 
		print"{:6.2f}".format(q2), 
		print"{:6.2f}".format(q3), 
		print"{:6.2f}".format(GyroX), 
		print"{:6.2f}".format(GyroY), 
		print"{:6.2f}".format(GyroZ), 
		print"{:6.2f}".format(AccX), 
		print"{:6.2f}".format(AccY), 
		print"{:6.2f}".format(AccZ)
		print('')
		
		# send data
		ser.write('a')
		ser.write(bytes(auto_thro_pwm))
		ser.write('b')
		ser.write('b')
		ser.write(bytes(auto_roll_pwm))
		ser.write('c')
		ser.write('c')
		ser.write(bytes(auto_pitch_pwm))
		ser.write('d')
		ser.write('d')
		ser.write(bytes(auto_yaw_pwm))
		ser.write('e')
		
		
		
		
		time.sleep(0.002) # 500Hz
	
	except Exception:
		pass

		

		

	


