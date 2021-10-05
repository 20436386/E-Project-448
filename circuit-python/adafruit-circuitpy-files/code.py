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
import analogio
from roboticsmasters_mpu9250 import MPU9250
from roboticsmasters_mpu6500 import MPU6500
from roboticsmasters_ak8963 import AK8963
import math 

#Declination
DECLINATION = 25.77
#Earth radius (km)
R = 6371
#For Sail control system
PWM_sail_max = 120
PWM_sail_min = 72
ADC_min = 0.22
ADC_max = 0.96
angle_no_sail = 45

#For rudder control system (neutral is 75)
PWM_rudder_max = 102
PWM_rudder_min = 48

#Returns bearing wrt magnetic north, no tilt compensation
def bearing(sensor):
    mag = sensor.magnetic
    gamma = math.atan2(mag[1],mag[0]) * (180/math.pi)
    if(gamma < 0):
        return -gamma
    else:
        return (360 - gamma)


#Returns bearing wrt magnetic north, with tilt compensation
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

#This function will implement haversine formulae to calculate distance; returns distance in metres. Takes points in decimal degrees
def haversine(current, target):
    #current = (lat1, long1)
    #target = (lat2, long2)
    current_rad = [x * (math.pi/180) for x in current]
    target_rad = [x * (math.pi/180) for x in target]
    delta_lat = (target_rad[0] - current_rad[0]) 
    delta_long = (target_rad[1] - current_rad[1])


    a = math.sin( delta_lat/2 )**2 + math.cos(current_rad[0]) * math.cos(target_rad[0]) * math.sin( delta_long/2 )**2
    c = 2 * math.atan2( math.sqrt(a) , math.sqrt(1-a) )
    d = R*c

    return d * 1000

#This will compensate for the error that occurs when using offset = (2.5, 296.0, 52.0), scale = (1.01161, 1.0369, 0.955044) - found at dam.
def error_comp(mag_sample):
    comp =  ( 2.57600925e-06*mag_sample**3 - 5.29452811e-04*mag_sample**2 + 8.79993961e-01*mag_sample + 1.53253617e+01)
    if comp >=360:
        comp -= 360
    if comp < 0:
        comp += 360
    return comp

#return desired/reference bearing
def desired_bearing(current, target):
    #current = (lat1, long1)
    #target = (lat2, long2)
    current_rad = [x * (math.pi/180) for x in current]
    target_rad = [x * (math.pi/180) for x in target]

    # delta_lat = (target[0] - current[0]) * (math.pi/180)
    delta_long = (target_rad[1] - current_rad[1])

    numerator = math.sin(delta_long) * math.cos(target_rad[0])
    denominator = math.cos(current_rad[0]) * math.sin(target_rad[0]) - math.sin(current_rad[0]) * math.cos(target_rad[0]) * math.cos(delta_long)
    theta = math.atan2(numerator, denominator) * (180 / math.pi)

    #Limit theta to range (0, 360)
    if theta < 0:
        theta += 360

    return theta 

def blink(num, delay):
    for i in range(num):
        led.value = True
        time.sleep(delay)
        led.value = False
        time.sleep(delay)


# #Initialise and mount SD card filesystem using sdio. Note: using sdio gives OSError: [Errno 5] Input/output error for some reason
# sdcard = sdioio.SDCard(
#     clock=board.SDIO_CLOCK,
#     command=board.SDIO_COMMAND,
#     data=board.SDIO_DATA,
#     frequency=12000000)


# vfs = storage.VfsFat(sdcard)

# #Mount filesystem into circuitPython
# storage.mount(vfs, "/sd")

#Setup led for debug purposes
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT


# Mounting sd card with spi
SD_CS_PIN = board.D6
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
sd_cs = digitalio.DigitalInOut(SD_CS_PIN)
sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, '/sd')    # Mount SD card under '/sd' path in filesystem.


LOG_FILE = "/sd/error_sig_tst.txt"    # Example for writing to SD card path /sd/gps.txt
LOG_MODE = 'ab'


#Initialise UART connection to gps module
TX = board.TX
RX = board.RX
uart = busio.UART(TX, RX, baudrate = 9600, bits = 8, parity = None, stop = 1, timeout = 10)
gps = adafruit_gps.GPS(uart)#, debug = False)

#Turn on the basic GGA and RMC info
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
#This is supposed to set update rate to twice a second 2Hz
gps.send_command(b'PMTK220,1000')

# gps_ident = bytes("$GNRMC", 'utf-16')


# #set up pwm for servos
# rudder_servo = pwmio.PWMOut(board.A2, frequency = 50, duty_cycle = (int)((75/1000)*(2**16)))
# sail_servo = pwmio.PWMOut(board.D9, frequency = 50, duty_cycle = (int)((100/1000)*(2**16)))

# #Init ADC for wind sensor
# adc = analogio.AnalogIn(board.A3)

# Code initialise MPU9250 magnetometer
i2c = busio.I2C(board.SCL, board.SDA)
mpu = MPU6500(i2c, address=0x69)
#ak = AK8963(i2c)
sensor = MPU9250(i2c)

# # Calibrate magnetometer 
# print("calibrating in 5")
# time.sleep(5)
# sensor.cal_mag() 


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


#Blink LED 3 times
# blink(20, 0.3)

# current_time = time.monotonic()

# with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
#             file.write(bytes(str(sensor._akm._offset) , 'utf-16'))
#             file.write(bytes(str(sensor._akm._scale) , 'utf-16'))

# print("offset = ", sensor._akm._offset)
# print("scale = ", sensor._akm._scale)

# lat = []
# long = []

#This variable is for error signal test
target_pos = (-33.957047462 , 18.809661865) #bottom side dam
# target_pos = (-33.956742287 , 18.807343483) #right side dam

current_time = time.monotonic()
while True:

    # #This code obtains target gps coordinates
    # # print(gps.readline())
    # gps.update()

    # if (time.monotonic() - current_time) >= 1:
    #     if not gps.has_fix:
    #         print("waiting for fix...")
    #         #Note: continue keyword instructs next iteration of while loop to execute
    #     else:
    #         print("latitude: {0:.9f} degrees" .format(gps.latitude)) 
    #         print("longitude: {0:.9f} degrees" .format(gps.longitude), "\n\n")

    #     current_time = time.monotonic()

    # This code calculates and logs error signal and other data
    gps.update()
    
    if (time.monotonic() - current_time) >= 0.1:
        if not gps.has_fix:
            print("waiting for fix...")
        else:
            current_pos = (gps.latitude , gps.longitude)
            ref_bearing = desired_bearing(current_pos, target_pos)
            gamma = bearing_tilt_comp(sensor)[2]
            current_bearing = error_comp(gamma)
            current_bearing_true = current_bearing - DECLINATION
            if current_bearing_true < 0:
                current_bearing_true += 360
            error_sig = ref_bearing - current_bearing
            distance = haversine(current_pos, target_pos)

            log_sentence = str("{0:.9f}".format(current_pos[0])) + ',' + str("{0:.9f}".format(current_pos[1])) + ',' + str(distance) + ',' +  str(ref_bearing) + ',' + str(gps.track_angle_deg) + ',' + str(gps.speed_knots) + ',' + str(gamma) + ',' + str(current_bearing) + ',' + str(current_bearing_true) + ',' + str(error_sig) + '\n'
            print(log_sentence)

            with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
                    file.write(bytes(log_sentence , 'utf-16'))
                    #file.flush()
        current_time = time.monotonic()

    
    # #Code to test calculating desired bearing
    # current = (-33.929071, 18.862272)
    # # target = (-33.928935, 18.863529)
    # target = (-33.930480,  18.863745)


    # # current = (-33.9342191666667, 18.8628255)
    # # target = (-33.9334761666667, 18.8678281666667)


    # distance = haversine(current, target)
    # ref_bearing = desired_bearing(current, target)
    # print(ref_bearing)
    # print(distance, "\n")
    # time.sleep(1)

    # #Code to test distance calculation accuracy
    # # print(gps.readline())
    # gps.update()

    # if (time.monotonic() - current_time) >= 1:
    #     if not gps.has_fix:
    #         # print("waiting for fix...")\
    #         print("has fix = ", gps.has_fix)
    #         #Note: continue keyword instructs next iteration of while loop to execute
    #     else:
    #         print("latitude: {0:.9f} degrees" .format(gps.latitude)) 
    #         print("longitude: {0:.9f} degrees" .format(gps.longitude), "\n\n")

    #         # lat.append(gps.latitude)
    #         # long.append(gps.longitude)

    #         # if (len(lat) == 10 ):
    #         #     average_pos = ( ( sum(lat)/len(lat)) , ( sum(long)/len(long)) )
    #         #     print("average_lat = {0:.9f}" .format(average_pos[0]))
    #         #     print("average_long = {0:.9f}" .format(average_pos[1]))
    #         #     break             

    #         current_pos = (gps.latitude, gps.longitude )
    #         # target_pos = ( -33.929259777, 18.861877441)#average of 10 samples garden
    #         target_pos = ( -33.929276466 , 18.861862183)#one sample, garden
    #         # target_pos = ( -33.929245 , 18.861851)#from google earth

    #         distance = haversine( current_pos, target_pos)
    #         print("distance = ", distance , "m", "\n")

    #     current_time = time.monotonic()

    # #For Sail control system
    # print("adc_percentage = ",  (adc.value/2**16) * 100)
    # alpha = ( ( (adc.value/2**16) -ADC_min )/(ADC_max-ADC_min) )
    # print("alpha = ", alpha * 100)
    # wind_bearing = ( alpha * 360)
    # if (wind_bearing > 180):
    #     wind_bearing -= 360
    # print("wind bearing = ", wind_bearing, "\n")
    # if abs(wind_bearing) > 45:
    #     print("wind bearing = ", wind_bearing)
    #     PWM_val = ( (abs(wind_bearing) - angle_no_sail)/(180 - angle_no_sail) ) * (PWM_sail_max - PWM_sail_min) + PWM_sail_min
    #     print("PWM_val = ", PWM_val, "\n")
    #     sail_servo.duty_cycle = (int)((PWM_val/1000)*(2**16))
    # else:
    #     print("no-sail-zone, wind_bearing = ", wind_bearing)
    #     print((sail_servo.duty_cycle/2**16)*1000)

    # # print("adc.value = ", (adc.value/2**16))
    # # print("voltage = ", (adc.value/2**16) * 3.3, "\n")
    # time.sleep(1)


    # phi, theta, gamma = bearing_tilt_comp(sensor)
    # gamma = error_comp(gamma)
    # print(phi * (180/math.pi),",", theta * (180/math.pi), "," ,gamma) 
    # sentence = bytes(str(phi), 'utf-16') + "," + bytes(str(theta), 'utf-16') + "," +bytes(str(gamma), 'utf-16') + "\n"
    # time.sleep(1)

    # with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
    #         file.write(sentence)
    #         #file.flush()
    # print(time.monotonic() - current_time)
    # current_time = time.monotonic()
    # time.sleep(1 - 0.027)


    # phi, theta, gamma = bearing_tilt_comp(sensor)
    # print("gamma = ", gamma, "with comp = ", error_comp(gamma))
    # print(phi * (180/math.pi),",", theta * (180/math.pi), "," ,gamma) 
    # print("no-comp = ", bearing(sensor))
    # time.sleep(1)

    # blink(3, 0.2)
    
    
    
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
    # print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.acceleration))
    # print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.magnetic))
    # print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.gyro))
    # print('Temperature: {0:0.3f}C'.format(sensor.temperature))
    # time.sleep(0.4)
    



# current_time = time.monotonic()
# #print(current_time)
# while True:

    # #Reads and prints gps data to serial
    # gps.update()

    # if (time.monotonic() - current_time) >= 1.0:
    #     if not gps.has_fix:
    #         print("waiting for fix...")
    #         #Note: continue keyword instructs next iteration of while loop to execute
    #     else:
    #         print("latitude: {0:.8f} degrees" .format(gps.latitude)) #
    #         print("longitude: {0:.8f} degrees" .format(gps.longitude))
    #         print(gps.track_angle_deg)

    #         if gps.speed_knots is not None:
    #             print("Speed: {} knots" .format(gps.speed_knots))
    #         if gps.track_angle_deg is not None:
    #             print("Track angle: {} degrees" .format(gps.track_angle_deg))

    #     current_time = time.monotonic()


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