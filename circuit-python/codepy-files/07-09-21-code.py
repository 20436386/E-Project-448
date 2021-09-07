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

"""
#Initialise and mount SD card filesystem using sdio. Note: using sdio gives OSError: [Errno 5] Input/output error for some reason
sdcard = sdioio.SDCard(
    clock=board.SDIO_CLOCK,
    command=board.SDIO_COMMAND,
    data=board.SDIO_DATA,
    frequency=12000000)


vfs = storage.VfsFat(sdcard)

#Mount filesystem into circuitPython
storage.mount(vfs, "/sd")
"""
"""
#Mounting sd card with spi
SD_CS_PIN = board.D6
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
sd_cs = digitalio.DigitalInOut(SD_CS_PIN)
sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, '/sd')    # Mount SD card under '/sd' path in filesystem.
"""

LOG_FILE = "/sd/gps.txt"    # Example for writing to SD card path /sd/gps.txt
LOG_MODE = 'ab'

"""
#write newlines to file to differentiate between data_logs
with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
            file.write(bytes("\n\n\r", 'utf-16'))
            #file.flush()
"""
'''
#Initialise UART connection to gps module
TX = board.TX
RX = board.RX
uart = busio.UART(TX, RX, baudrate = 9600, bits = 8, parity = None, stop = 1, timeout = 10)
gps = adafruit_gps.GPS(uart)#, debug = False)

#Not sure if this does anything
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

gps_ident = bytes("$GNRMC", 'utf-16')

'''

"""
#set up pwm for servos
led = pwmio.PWMOut(board.A2, frequency = 50, duty_cycle = (int)((75/1000)*(2**16)))
"""


#Code to test MPU9250 magnetometer
i2c = busio.I2C(board.SCL, board.SDA)

mpu = MPU6500(i2c, address=0x69)
#ak = AK8963(i2c)

sensor = MPU9250(i2c)

#Calibrate magnetometer. Will take 51,2 seconds to complete
print("calibrating in 5")
#time.sleep(5)
#sensor.cal_mag()


#print("Reading in data from IMU.")
print("reading calibrated magnetic in 5")
time.sleep(5)
while True:

#for i in range(0, 256):
#    raw = sensor.magnetic
#    print(raw[0], ",", raw[1], ",", raw[2],"\n")
#    time.sleep(0.4)

    raw = sensor.magnetic
    print("\ncalibrated:")
    print(raw[0], ",", raw[1], ",", raw[2])
    phi = math.atan2(raw[0],raw[1]) * (180/math.pi)
    print("phi = ", phi)
    if(phi < 0):
        theta = 360+phi
    else:
        theta = phi
    print("theta = ", theta)

    #print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.acceleration))
    #print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.magnetic))
    #print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.gyro))
    #print('Temperature: {0:0.3f}C'.format(sensor.temperature))
    #time.sleep(0.2)


"""
current_time = time.monotonic()
#print(current_time)
while True:
    continue

"""
'''
    #Reads and prints gps data to serial
    gps.update()

    if (time.monotonic() - current_time) >= 1.0:
        if not gps.has_fix:
            print("waiting for fix...")
            #Note: continue keyword instructs next iteration of while loop to execute
        else:
            print("latitude: {0:.6f} degrees" .format(gps.latitude)) #
            print("longitude: {0:.6f} degrees" .format(gps.longitude))
            print(gps.track_angle_deg)

            if gps.speed_knots is not None:
                print("Speed: {} knots" .format(gps.speed_knots))
            if gps.track_angle_deg is not None:
                print("Track angle: {} degrees" .format(gps.track_angle_deg))

        current_time = time.monotonic()
'''
"""
    #reads and writes RMC data to sd card
    sentence = gps.readline()
    if not sentence:
        continue
    #print(sentence)#str(sentence, "utf-16").strip()) #this was from documentation, doesnt work

    if sentence[0 : 6] == gps_ident:#(0x24, 0x47, 0x4E, 0x52, 0x4D, 0x43):
        #Delay between acquiring RMC data is on avg 1 second
        #print(sentence)

        with open(LOG_FILE, LOG_MODE, encoding='utf-16') as file:
            file.write(sentence)
            #file.flush()
"""
