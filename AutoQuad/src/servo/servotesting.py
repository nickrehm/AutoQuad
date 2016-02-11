import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(12,50) #pin, frequency
servo1.start(0) #initialize to 0 duty cycle


angle_des = 60
duty = angle_des/18 + 2

while True:
	servo1.ChangeDutyCycle(duty)


