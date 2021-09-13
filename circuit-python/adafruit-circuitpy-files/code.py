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

def bearing(sensor):
    mag = sensor.magnetic
    gamma = math.atan2(mag[1],mag[0]) * (180/math.pi)
    if(gamma < 0):
        return -gamma
    else:
        return (360 - gamma)

def bearing_tilt_comp(sensor):
    acc = sensor.acceleration
    mag = sensor.magnetic

    #calculate roll and pitch angles
    phi = math.atan2(acc[1],acc[2])
    theta = math.atan( (-acc[0])/(acc[1]*math.sin(phi) + acc[2]*math.cos(phi)))
    # print("phi = ", phi * (180/math.pi), "theta = ", theta * (180/math.pi))

    #Calculate tilt compensated bearing
    Bfy = (mag[1]*math.cos(phi) + mag[2]*math.sin(phi))
    Bfx =  (mag[0]*math.cos(theta) + mag[1]*math.sin(theta)*math.sin(phi) - mag[2]*math.sin(theta)*math.cos(phi) )
    gamma = math.atan2( -Bfy, Bfx) * (180/math.pi)
    
    #limit interval to o -> 360
    if(gamma < 0):
        gamma =  (360 + gamma)
    
    return (phi, theta, gamma)


# #Initialise and mount SD card filesystem using sdio. Note: using sdio gives OSError: [Errno 5] Input/output error for some reason
# sdcard = sdioio.SDCard(
#     clock=board.SDIO_CLOCK,
#     command=board.SDIO_COMMAND,
#     data=board.SDIO_DATA,
#     frequency=12000000)


# vfs = storage.VfsFat(sdcard)

# #Mount filesystem into circuitPython
# storage.mount(vfs, "/sd")

# #Setup led for debug purposes
# led = digitalio.DigitalInOut(board.LED)
# led.direction = digitalio.Direction.OUTPUT


# #Mounting sd card with spi
# SD_CS_PIN = board.D6
# spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
# sd_cs = digitalio.DigitalInOut(SD_CS_PIN)
# sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
# vfs = storage.VfsFat(sdcard)
# storage.mount(vfs, '/sd')    # Mount SD card under '/sd' path in filesystem.


# LOG_FILE = "/sd/gps.txt"    # Example for writing to SD card path /sd/gps.txt
# LOG_MODE = 'ab'


# #write newlines to file to differentiate between data_logs
# with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
#             file.write(bytes("\n\n\r", 'utf-16'))
#             #file.flush()


# #Initialise UART connection to gps module
# TX = board.TX
# RX = board.RX
# uart = busio.UART(TX, RX, baudrate = 9600, bits = 8, parity = None, stop = 1, timeout = 10)
# gps = adafruit_gps.GPS(uart)#, debug = False)

# #Not sure if this does anything
# gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

# gps_ident = bytes("$GNRMC", 'utf-16')




# #set up pwm for servos
# servo1 = pwmio.PWMOut(board.A2, frequency = 50, duty_cycle = (int)((75/1000)*(2**16)))



#Code to test MPU9250 magnetometer
i2c = busio.I2C(board.SCL, board.SDA)

mpu = MPU6500(i2c, address=0x69)
#ak = AK8963(i2c)

sensor = MPU9250(i2c)

#Calibrate magnetometer. Will take 51,2 seconds to complete
print("calibrating in 5")
time.sleep(5)
sensor.cal_mag()


# print("Reading in data from IMU.")
# print("reading calibrated magnetic in 5")
#time.sleep(5)

# #code to save start location
# count = 0

# while count <= 8:
#     gps.update()

#     if not gps.has_fix:
#         continue
#     else:
#         sentence = gps.readline()
#         if sentence[0 : 6] == bytes("$GNGLL", 'utf-16'):
#             # sentence = bytes(str(gps.latitude), 'utf-a6') + "," + bytes(str(gps.longitude), 'utf-16') + "\n"
#             # print(sentence)
#             with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
#                 file.write(sentence)
#                 #file.flush()
#             count += 1


# #Blink LED 3 times
# for i in range(3):
#     led.value = True
#     time.sleep(0.5)
#     led.value = False
#     time.sleep(0.5)

# current_time = time.monotonic()


while True:

    # phi, theta, gamma = bearing_tilt_comp(sensor)
    # print(phi * (180/math.pi),",", theta * (180/math.pi), "," ,gamma) 
    # sentence = bytes(str(phi), 'utf-16') + "," + bytes(str(theta), 'utf-16') + "," +bytes(str(gamma), 'utf-16') + "\n"

    # with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
    #         file.write(sentence)
    #         #file.flush()
    # print(time.monotonic() - current_time)
    # current_time = time.monotonic()
    # time.sleep(1 - 0.027)

    phi, theta, gamma = bearing_tilt_comp(sensor)
    print(phi * (180/math.pi),",", theta * (180/math.pi), "," ,gamma) 
    print("no-comp = ", bearing(sensor))
    time.sleep(0.4)
    
    
    
    # raw = sensor.magnetic
    # print("\ncalibrated:")
    # print(raw[0], ",", raw[1], ",", raw[2])
    # phi = math.atan2(raw[1],raw[0]) * (180/math.pi)
    # print("phi = ", phi)
    # if(phi < 0):
    #     theta = -phi
    # else:
    #     theta = 360 - phi
    # print("theta = ", theta)
    # #print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.acceleration))
    # #print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.magnetic))
    # #print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.gyro))
    # #print('Temperature: {0:0.3f}C'.format(sensor.temperature))
    # time.sleep(0.4)
    



# current_time = time.monotonic()
# #print(current_time)
# while True:
#     continue



#     #Reads and prints gps data to serial
#     gps.update()

#     if (time.monotonic() - current_time) >= 1.0:
#         if not gps.has_fix:
#             print("waiting for fix...")
#             #Note: continue keyword instructs next iteration of while loop to execute
#         else:
#             print("latitude: {0:.6f} degrees" .format(gps.latitude)) #
#             print("longitude: {0:.6f} degrees" .format(gps.longitude))
#             print(gps.track_angle_deg)

#             if gps.speed_knots is not None:
#                 print("Speed: {} knots" .format(gps.speed_knots))
#             if gps.track_angle_deg is not None:
#                 print("Track angle: {} degrees" .format(gps.track_angle_deg))

#         current_time = time.monotonic()


#     #reads and writes RMC data to sd card
#     sentence = gps.readline()
#     if not sentence:
#         continue
#     #print(sentence)#str(sentence, "utf-16").strip()) #this was from documentation, doesnt work

#     if sentence[0 : 6] == gps_ident:#(0x24, 0x47, 0x4E, 0x52, 0x4D, 0x43):
#         #Delay between acquiring RMC data is on avg 1 second
#         #print(sentence)

#         with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
#             file.write(sentence)
#             #file.flush()

