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

##For navigation
rmc_status = 'V'
# target_pos = (-33.957047462 , 18.809661865) #Bottom side dam
target_pos = (-33.956742287 , 18.807343483) #Right side dam
# target_pos = (-33.929276466 , 18.861862183) #Garden

##For Sail control system
PWM_sail_max = 120
PWM_sail_min = 72
ADC_min = 0.22
ADC_max = 0.96
angle_no_sail = 45

##For rudder control system (neutral is 75)
PWM_rudder_max = 102
PWM_rudder_min = 48
# k_p = 2.3
k_p = 1.5
k_i = 0.8
t_s = 0.1  #Need to measure this
I_k_prev = 0

##For digital filter:
#filter coefficients:
b = (0.02785977, 0.05571953, 0.02785977) 
a = (1. ,         -1.47548044,  0.58691951)
#flag for filled array
mag_full_flag = False
#Magnetic value arrays
#Note: mag_x_val[0] = inputs, mag_x_val[1] = outputs
mag_x_val = [[],[]]
mag_y_val = [[],[]]
mag_z_val = [[],[]]
#Acceleration value arrays
acc_x_val = [[],[]]
acc_y_val = [[],[]]
acc_z_val = [[],[]]

#returns size of global arrays
def is_full(array):
    if type(array) == float:
        return 1
    else:
        return len(array)

#Returns bearing wrt magnetic north, no tilt compensation
def bearing(sensor):
    mag = sensor.magnetic
    gamma = math.atan2(mag[1],mag[0]) * (180/math.pi)
    if(gamma < 0):
        return -gamma
    else:
        return (360 - gamma)


#Returns bearing wrt magnetic north, with tilt compensation
def bearing_tilt_comp(sensor, filter_mag=False, filter_acc=False):
    #Define global variables
    global mag_x_val
    global mag_y_val
    global mag_z_val
    global acc_x_val
    global acc_y_val
    global acc_z_val

    acc = sensor.acceleration
    mag = list(sensor.magnetic)

    if filter_acc:
        acc_x_val[0], acc_x_val[1] = butter_filter(acc[0], acc_x_val[0], acc_x_val[1])
        acc_y_val[0], acc_y_val[1] = butter_filter(acc[1], acc_y_val[0], acc_y_val[1])
        acc_z_val[0], acc_z_val[1] = butter_filter(acc[2], acc_z_val[0], acc_z_val[1])

        acc_fil = [0,0,0]
        length = is_full(acc_x_val[1])
        # print(length)
        acc_fil[0] = acc_x_val[1][length - 1]
        acc_fil[1] = acc_y_val[1][length - 1]
        acc_fil[2] = acc_z_val[1][length - 1]

        phi, theta = pitch_roll(acc_fil)
    else:
        phi, theta = pitch_roll(acc)

    if filter_mag:
        mag_x_val[0], mag_x_val[1] = butter_filter(mag[0], mag_x_val[0], mag_x_val[1])
        mag_y_val[0], mag_y_val[1] = butter_filter(mag[1], mag_y_val[0], mag_y_val[1])
        mag_z_val[0], mag_z_val[1] = butter_filter(mag[2], mag_z_val[0], mag_z_val[1])

        # #update values with filtered values
        length = is_full(mag_x_val[1])
        mag[0] = mag_x_val[1][length - 1]
        mag[1] = mag_y_val[1][length - 1]
        mag[2] = mag_z_val[1][length - 1]

        # #Calculate filtered bearing
        # mag_fil = [0,0,0]
        # length = is_full(mag_x_val[1])
        # mag_fil[0] = mag_x_val[1][length - 1]
        # mag_fil[1] = mag_y_val[1][length - 1]
        # mag_fil[2] = mag_z_val[1][length - 1]

        

    #Calculate tilt compensated bearing
    Bfy = (mag[1]*math.cos(phi) + mag[2]*math.sin(phi))
    Bfx =  (mag[0]*math.cos(theta) + mag[1]*math.sin(theta)*math.sin(phi) - mag[2]*math.sin(theta)*math.cos(phi) )
    gamma = math.atan2( -Bfy, Bfx) * (180/math.pi)

    # #Calculate tilt compensated bearing(mag filtered)
    # Bfy = (mag_fil[1]*math.cos(phi) + mag_fil[2]*math.sin(phi))
    # Bfx =  (mag_fil[0]*math.cos(theta) + mag_fil[1]*math.sin(theta)*math.sin(phi) - mag_fil[2]*math.sin(theta)*math.cos(phi) )
    # gamma_fil = math.atan2( -Bfy, Bfx) * (180/math.pi)
    
    # #limit interval to o -> 360
    # if(gamma_fil < 0):
    #     gamma_fil =  (360 + gamma_fil)
    
    #limit interval to o -> 360
    if(gamma < 0):
        gamma =  (360 + gamma)

    # if filter_mag:
    #     return (phi, theta, gamma, gamma_fil)
    # else:
    #     return (phi, theta, gamma)
    return (phi, theta, gamma)

def pitch_roll(acc):
    phi = math.atan2(acc[1],acc[2])
    theta = math.atan( (-acc[0])/(acc[1]*math.sin(phi) + acc[2]*math.cos(phi)))
    # print("phi = ", phi * (180/math.pi), "theta = ", theta * (180/math.pi))
    return phi, theta


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
    comp =  ( -2.16780203e-07 *mag_sample**3 - 5.37464909e-04*mag_sample**2 + 8.40257454e-01*mag_sample + 7.58334508e+00)
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

def butter_filter(sample, x_vals, y_vals):

    length = len(x_vals)
    # print(length)

    if length < 3:
        if(length == 0):
            x_vals.append(sample)
            y_vals.append((b[0]*x_vals[0]) / a[0])
            
        elif(length == 1):
            x_vals.append(sample)
            y_vals.append((b[0]*x_vals[1] + b[1]*x_vals[0] - a[1]*y_vals[0]) / a[0])
        
        elif(length == 2):
            x_vals.append(sample)
            y_vals.append((b[0]*x_vals[2] + b[1]*x_vals[1] + b[2]*x_vals[0] - a[1]*y_vals[1] - a[2]*y_vals[0]) / a[0])
        # x_idx = (x_idx + 1) % 3
        # y_idx = (y_idx + 1) % 3
    else:
        x_vals.append(sample)
        y_vals.append( ( b[0]*x_vals[3] + b[1]*x_vals[2] + b[2]*x_vals[1] - a[1]*y_vals[2] - a[2]*y_vals[1]  ) / a[0])
        x_vals.pop(0)
        y_vals.pop(0)

    return x_vals, y_vals

def blink(num, delay):
    for i in range(num):
        led.value = True
        time.sleep(delay)
        led.value = False
        time.sleep(delay)


#Initialise led for debug purposes
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT


#Initialise and mounting sd card with spi
SD_CS_PIN = board.D6
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
sd_cs = digitalio.DigitalInOut(SD_CS_PIN)
sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, '/sd')    # Mount SD card under '/sd' path in filesystem.


LOG_FILE = "/sd/log.txt"    # Example for writing to SD card path /sd/gps.txt
LOG_MODE = 'ab'


#Initialise UART connection to gps module
TX = board.TX
RX = board.RX
uart = busio.UART(TX, RX, baudrate = 9600, bits = 8, parity = None, stop = 1, timeout = 10)
gps = adafruit_gps.GPS(uart)#, debug = False)

# #These commands do nothing apparently
# #Turn on the basic GGA and RMC info
# gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# # gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# #This is supposed to set update rate to twice a second 2Hz
# gps.send_command(b'PMTK220,1000')

gps_ident = bytes("$GNRMC", 'utf-16')


#Initialise PWM for servos
rudder_servo = pwmio.PWMOut(board.A2, frequency = 50, duty_cycle = (int)((75/1000)*(2**16)))
#Set to 45 deg initially 
sail_servo = pwmio.PWMOut(board.D9, frequency = 50, duty_cycle = (int)((PWM_sail_max/1000)*(2**16))) #100

#Init ADC for wind sensor
adc = analogio.AnalogIn(board.A3)

# Initialise MPU9250 magnetometer
i2c = busio.I2C(board.SCL, board.SDA)
mpu = MPU6500(i2c, address=0x69)
sensor = MPU9250(i2c)

# # Calibrate magnetometer 
# print("calibrating in 5")
# time.sleep(5)
# sensor.cal_mag() 
# blink(20, 0.3)
# with open("/sd/calib.txt", LOG_MODE, encoding='utf-16') as file:
#             file.write(bytes(str(sensor._akm._offset) , 'utf-16'))
#             file.write(bytes(str(sensor._akm._scale) , 'utf-16'))

#These variables for plan-b
# gps_pos = [0]*3
# gps_idx = -1
# gps_bearing = 0


#This is to determine desired bearing before, instead of on every clock cycle
start_pos = (-33.929276466 , 18.861862183)
ref_bearing = desired_bearing(start_pos, target_pos)

# current_time = time.monotonic()
# #Note:
#Digital compass sampling period is on avg (1450 samples taken) 68.9764607586208 ms without calculating ref_bearing every time i.e. calculating before while loop, and with an error comp on bearing values, and without integral control
#Digital compass sampling period is on avg (1450 samples taken) 67.2002935862066 ms with calculating ref_bearing every time, and with an error comp on bearing values, and without integral control


while True:
        

#     # _,_,current_bearing = bearing_tilt_comp(sensor, filter_mag=True, filter_acc=True)
#     # current_bearing = error_comp(current_bearing)
#     # print(current_bearing)
#     # count += 1
#     # time.sleep(0.07)


#     # #This code obtains target gps coordinates
#     # # print(gps.readline())
#     # gps.update()

#     # if (time.monotonic() - current_time) >= 1:
#     #     if not gps.has_fix:
#     #         print("waiting for fix...")
#     #         #Note: continue keyword instructs next iteration of while loop to execute
#     #     else:
#     #         print("latitude: {0:.9f} degrees" .format(gps.latitude)) 
#     #         print("longitude: {0:.9f} degrees" .format(gps.longitude), "\n\n")

#     #     current_time = time.monotonic()

    nmea_sentence = gps.readline()
    # print(nmea_sentence)

    #This acquires and updates gps data every one second
    #This if statement takes of average 2.39702 ms to execute
    #Delay between acquiring RMC data is on avg 1 second
    if nmea_sentence[0 : 6] == gps_ident:#(0x24, 0x47, 0x4E, 0x52, 0x4D, 0x43):
        rmc_data = str(nmea_sentence).split(',')
        rmc_status = rmc_data[2]
        if rmc_data[2] == 'A':

            latitude = float(rmc_data[3])
            latitude = int(latitude/100) + ((latitude % 100)/60)
            # print("{0:.9f}".format(latitude))

            longitude = float(rmc_data[5])
            longitude = int(longitude/100) + ((longitude % 100)/60)

            if rmc_data[4] == 'S':
                latitude = -latitude

            if rmc_data[6] == 'W':
                longitude = -longitude

            speed_knots = rmc_data[7]
            track_angle_deg = rmc_data[8]

            #This was not included in execution time of if statement part of plan-b
            # gps_idx = (gps_idx + 1) % 3
            # gps_pos[gps_idx] = (latitude, longitude)
            # print(gps_pos)
            # print("{0:.9f}".format(latitude), "{0:.9f}".format(longitude), speed, track_angle)
        else:
            print("RMC void, fix not acquired")

    ##Rudder control system
    #This if statement takes 30.9774 ms to execute on average
    # if ((time.monotonic() - current_time) >= 1) and (rmc_status == 'A'):
    if (rmc_status == 'A'):

        #Check if distance is < 5m
        current_pos = (latitude , longitude)
        distance = haversine(current_pos, target_pos)

        if distance >= 5:
            #Calculate desired bearing(only use if calibration of compass is spot on)
            # ref_bearing = desired_bearing(current_pos, target_pos)

            #Sample actual bearing
            _,_,current_bearing = bearing_tilt_comp(sensor, filter_mag=True, filter_acc=True)
            # print(time.monotonic() - current_time)
            # current_time = time.monotonic()
            # current_bearing = error_comp(current_bearing) #Dont know if i should use this
            current_bearing_true = current_bearing - DECLINATION
            #Ensure range is 0 -> 360
            if current_bearing_true < 0:
                current_bearing_true += 360
            # print("current_bearing_true = ", current_bearing_true)
            # print(current_bearing)
            
            # #This calculates bearing using current coordinates and coordinates two back plan-b
            # if all(gps_pos):
            #     gps_bearing = desired_bearing(gps_pos[gps_idx - 2], gps_pos[gps_idx])
            #     # print(gps_bearing)

            #Calculate error signal
            error_sig = -(ref_bearing - current_bearing_true)

            ##Rudder Controller (proportional)
            #I_k = (k_i * error_sig * t_s)
            rudder_sig = (k_p * error_sig) #+ (I_k + I_k_prev)
            #I_k_prev = I_k
            # print("error_sig = ", error_sig)
            # print("rudder_sig = ",rudder_sig)

            PWM_rudder_val = ((rudder_sig / 60) * (PWM_rudder_max - PWM_rudder_min)) + ((PWM_rudder_max + PWM_rudder_min) / 2)

            #Limit rudder to range (-60, 60) 
            if PWM_rudder_val > PWM_rudder_max:
                PWM_rudder_val = PWM_rudder_max

            if PWM_rudder_val < PWM_rudder_min:
                PWM_rudder_val = PWM_rudder_min
            # print("PWM_rudder_val = ", PWM_rudder_val)

            #Apply control signal to Rudder actuator
            rudder_servo.duty_cycle = (int)((PWM_rudder_val/1000)*(2**16))

            ##Sail controller
            # print("adc_percentage = ",  (adc.value/2**16) * 100)
            alpha = ( ( (adc.value/2**16) -ADC_min )/(ADC_max-ADC_min) )
            # print("alpha = ", alpha * 100)
            app_wind = ( alpha * 360)
            if (app_wind > 180):
                app_wind -= 360
            # print("wind bearing = ", wind_bearing, "\n")
            if abs(app_wind) > 45:
                # print("wind bearing = ", app_wind)
                PWM_sail_val = ( (abs(app_wind) - angle_no_sail)/(180 - angle_no_sail) ) * (PWM_sail_max - PWM_sail_min) + PWM_sail_min
                # print("PWM_val = ", PWM_sail_val, "\n")
                sail_servo.duty_cycle = (int)((PWM_sail_val/1000)*(2**16))
            else:
                #Would ideally need to change to tacking mode here
                PWM_sail_val = PWM_sail_min
                print("no-sail-zone, wind_bearing = ", app_wind)
            #     print((sail_servo.duty_cycle/2**16)*1000)

            #Log data. Format: latitude, longitude, distance, desired bearing, GPS track angle, GPS speed, current bearing(true), error signal, rudder controller signal, rudder PWM value, alpha, apparent wind, sail PWM value
            log_sentence = str("{0:.9f}".format(current_pos[0])) + ',' + str("{0:.9f}".format(current_pos[1])) + ',' + str(distance) + ',' +  str(ref_bearing) + ',' + str(track_angle_deg) + ',' + str(speed_knots) + ',' + str(current_bearing_true) + ',' + str(error_sig) + ',' + str(rudder_sig) + ',' + str(PWM_rudder_val) + ',' + str(alpha) + ',' +  str(app_wind) + ',' + str(PWM_sail_val) + '\n'
            # print(log_sentence)
            with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
                    file.write(bytes(log_sentence , 'utf-16'))
                    #file.flush()
        else:
             with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
                    file.write(bytes("Target position acquired :)" , 'utf-16'))
            #         #file.flush()
