import os

import board
import sdioio
import storage
import digitalio
import adafruit_sdcard
import time
import busio
import adafruit_gps
import pwmio
from roboticsmasters_mpu9250 import MPU9250
from roboticsmasters_mpu6500 import MPU6500
from roboticsmasters_ak8963 import AK8963
import math


#Code to test MPU9250 magnetometer
i2c = busio.I2C(board.SCL, board.SDA)

mpu = MPU6500(i2c, address=0x69)
#ak = AK8963(i2c)

sensor = MPU9250(i2c)

#Calibrate magnetometer. Will take 51,2 seconds to complete
print("calibrating in 5")
time.sleep(5)
sensor.cal_mag()


print("Reading in data from IMU.")
print("reading calibrated magnetic in 5")
#time.sleep(5)

while True:    
    raw = sensor.magnetic
    print("\ncalibrated:")
    print(raw[0], ",", raw[1], ",", raw[2])
    phi = math.atan2(raw[1],raw[0]) * (180/math.pi)
    print("phi = ", phi)
    if(phi < 0):
        theta = -phi
    else:
        theta = 360 - phi
    print("theta = ", theta)
    #print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.acceleration))
    #print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.magnetic))
    #print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.gyro))
    #print('Temperature: {0:0.3f}C'.format(sensor.temperature))
    time.sleep(0.4)
    