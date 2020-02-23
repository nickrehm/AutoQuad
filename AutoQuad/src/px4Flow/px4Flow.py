#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2016 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################


import time
import string
#import sys
#import getopt
import math
import thread
#from array import *
import smbus



####################################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
####################################################################################################
class I2C:

    def __init__(self, address, bus=smbus.SMBus(1)):
        self.address = address
        self.bus = bus
        self.misses = 0

    def reverseByteOrder(self, data):
        "Reverses the byte order of an int (16-bit) or long (32-bit) value"
        # Courtesy Vishal Sapre
        dstr = hex(data)[2:].replace('L','')
        byteCount = len(dstr[::2])
        val = 0
        for i, n in enumerate(range(byteCount)):
            d = data & 0xFF
            val |= (d << (8 * (byteCount - i - 1)))
            data >>= 8
        return val

    def writeByte(self, value):
        while True:
            try:
                self.bus.write_byte(self.address, value)
                break
            except IOError, err:
                self.misses += 1

    def write8(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        while True:
            try:
                self.bus.write_byte_data(self.address, reg, value)
                break
            except IOError, err:
                self.misses += 1

    def writeList(self, reg, list):
        "Writes an array of bytes using I2C format"
        while True:
            try:
                self.bus.write_i2c_block_data(self.address, reg, list)
                break
            except IOError, err:
                self.misses += 1

    def readU8(self, reg):
        "Read an unsigned byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                return result
            except IOError, err:
                self.misses += 1

    def readS8(self, reg):
        "Reads a signed byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                if (result > 127):
                    return result - 256
                else:
                    return result
            except IOError, err:
                self.misses += 1

    def readU16(self, reg):
        "Reads an unsigned 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readS16(self, reg):
        "Reads a signed 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                if (hibyte > 127):
                    hibyte -= 256
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readList(self, reg, length):
        "Reads a a byte array value from the I2C device"
        result = self.bus.read_i2c_block_data(self.address, reg, length)
        return result

    def getMisses(self):
        return self.misses



####################################################################################################
#
#  Sensor Driver for PIX4FLOW sensor
#
####################################################################################################
class PX4FLOW:
#    i2c = None

    # Registers/etc.
    __PX4FLOW_REGISTER_BASE    = 0x0                  # 2 bytes unsigned - number
    __PX4FLOW_FRAME_COUNT      = 0x0                  # 2 bytes unsigned - number
    __PX4FLOW_PIXEL_FLOW_X     = 0x2                  # 2 bytes signed - latest x flow (pixels x 10)
    __PX4FLOW_PIXEL_FLOW_Y     = 0x4                  # 2 bytes signed - latest y flow (pixels x 10)
    __PX4FLOW_FLOW_COMP_M_X    = 0x6                  # 2 bytes signed - x velocity * 1000 (m/s)
    __PX4FLOW_FLOW_COMP_M_Y    = 0x8                  # 2 bytes signed - y velocity * 1000 (m/s)
    __PX4FLOW_QUAL_REG         = 0x0A                 # 2 bytes signed - optical flow quality (0:bad 255:max)
    __PX4FLOW_GYRO_X_RATE      = 0x0C                 # 2 bytes signed - gyro x rate (rad/sec)
    __PX4FLOW_GYRO_Y_RATE      = 0x0E                 # 2 bytes signed - gyro y rate (rad/sec)
    __PX4FLOW_GYRO_Z_RATE      = 0x10                 # 2 bytes signed - gyro z rate (rad/sec)
    __PX4FLOW_GYRO_RANGE       = 0x12                 # 1 byte unsigned - 0 - 7 = 50 - 2000 (degrees / second)
    __PX4FLOW_SONAR_TIMESTAMP  = 0x13                 # 1 byte unsigned - time since last sonar sample (ms)
    __PX4FLOW_GROUND_DISTANCE  = 0x14                 # 2 bytes signed  - ground distance (meters <0 = error)

    #--------------------------------------------------------------------------------------------------------
    # All integrals since the previous I2C read
    #--------------------------------------------------------------------------------------------------------
    __PX4FLOW_REGISTER_INTEGRAL           = 0x16      # 2 unsigned - number of reads since last I2C read
    __PX4FLOW_FRAME_COUNT_INTEGRAL        = 0x16      # 2 unsigned - number of reads since last I2C read
    __PX4FLOW_PIXEL_FLOW_X_INTEGRAL       = 0x02      # 2 signed - integrated flow around x axis (rad * 1000)
    __PX4FLOW_PIXEL_FLOW_Y_INTEGRAL       = 0x04      # 2 signed - integrated flow around y axis (rad * 1000)
    __PX4FLOW_GYRO_X_RATE_INTEGRAL        = 0x06      # 2 signed - integrated gyro X axis roll (rad * 1000)          # Would this be better for incremental roll rather than rate * dt
    __PX4FLOW_GYRO_Y_RATE_INTEGRAL        = 0x08      # 2 signed - integrated gyro Y axis pitch (rad * 1000)         # Would this be better for incremental roll rather than rate * dt
    __PX4FLOW_GYRO_Z_RATE_INTEGRAL        = 0x0A      # 2 signed - integrated gyro Z axis yaw (rad * 1000)           # Would this be better for incremental roll rather than rate * dt
    __PX4FLOW_TIMESPAN_INTEGRAL           = 0x0E      # 4 unsigned - integrated time lapse (microseconds)
    __PX4FLOW_SONAR_TIMESTAMP_2           = 0x12      # 4 unsigned - time since last sonar update (microseconds)
    __PX4FLOW_GROUND_DISTANCE_2           = 0x14      # 2 signed - ground distance (meters * 1000)
    __PX4FLOW_GYRO_TEMPERATURE            = 0x16      # 2 signed - temperature (celsius * 100)
    __PX4FLOW_QUALITY_AVERAGE             = 0x18      # 1 signed - averaged quality (0:bad 255:max)

    def __init__(self, address=0x42):
        self.i2c = I2C(address)
        self.address = address

    def read(self):
            #AB! Isn't SMBUS limited to 32 bytes?
            sensor_data =  self.i2c.readList(self.__PX4FLOW_REGISTER_BASE, 22)  # 22 if not using the integral registers

            hibyte = sensor_data[self.__PX4FLOW_FLOW_COMP_M_X + 1]
            if (hibyte > 127):
               hibyte -= 256
            x_velocity = ((hibyte << 8) + sensor_data[self.__PX4FLOW_FLOW_COMP_M_X]) 

            hibyte = sensor_data[self.__PX4FLOW_FLOW_COMP_M_Y + 1]
            if (hibyte > 127):
               hibyte -= 256
            y_velocity = ((hibyte << 8) + sensor_data[self.__PX4FLOW_FLOW_COMP_M_Y]) 

            hibyte = sensor_data[self.__PX4FLOW_QUAL_REG + 1]
            if (hibyte > 127):
               hibyte -= 256
            qual = (hibyte << 8) + sensor_data[self.__PX4FLOW_QUAL_REG]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_X_RATE + 1]
            if (hibyte > 127):
               hibyte -= 256
            pitch_rate = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_X_RATE]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_Y_RATE + 1]
            if (hibyte > 127):
               hibyte -= 256
            roll_rate = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_Y_RATE]

            hibyte = sensor_data[self.__PX4FLOW_GYRO_Z_RATE + 1]
            if (hibyte > 127):
               hibyte -= 256
            yaw_rate = (hibyte << 8) + sensor_data[self.__PX4FLOW_GYRO_Z_RATE]

            sonar_dt = (sensor_data[self.__PX4FLOW_SONAR_TIMESTAMP]) / 1000

            hibyte = sensor_data[self.__PX4FLOW_GROUND_DISTANCE + 1]
            if (hibyte > 127):
               hibyte -= 256
            ground_distance = ((hibyte << 8) + sensor_data[self.__PX4FLOW_GROUND_DISTANCE])


			#return sensor_data
            return x_velocity, y_velocity, ground_distance, qual
			#return pitch_rate, roll_rate, yaw_rate
