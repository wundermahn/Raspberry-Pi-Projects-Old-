#cameffects.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#This program actually uses the RbP camera and loops through the different effects.
#Based off of: https://www.raspberrypi.org/learning/getting-started-with-picamera/

from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.rotation = 180

camera.start_preview()
for effect in camera.IMAGE_EFFECTS:
    camera.image_effect = effect
    camera.annotate_text = "Current Effect: %s" % effect
    sleep(5)
camera.stop_preview()

##########################################################################

#camtest.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#This program utilizes the screen to show what the camera sees
#Based off of: https://www.raspberrypi.org/learning/getting-started-with-picamera/

from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.rotation = 180

camera.start_preview()
sleep(30)
camera.stop_preview()

##########################################################################

#camtest_picture.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#This program utilizes the camera to take a picture
#Based off of: https://www.raspberrypi.org/learning/getting-started-with-picamera/

from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.rotation = 180

camera.start_preview()
sleep(10)

camera.capture('/home/pi/Desktop/image_1.jpg')

camera.stop_preview()

##########################################################################

#camtest_video.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#This program utilizes the camera to take a video
#uses h264 format which is a RbP only format. Can easily be converted to mp4

#Based off of: https://www.raspberrypi.org/learning/getting-started-with-picamera/

from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.rotation = 180

camera.start_preview()
camera.start_recording('/home/pi/Desktop/video.h264')

sleep(10)

camera.stop_recording()
camera.stop_preview()

##########################################################################

#finalv1.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#this code actually uses the infrared sensor to wait for it to detect infrared, and when it does, to start recording
#based off of this: https://www.raspberrypi.org/learning/parent-detector/worksheet/

from gpiozero import MotionSensor
from picamera import PiCamera
from datetime import datetime

camera = PiCamera()
pir = MotionSensor(4)
while True:
    pir.wait_for_motion()
    filename = datetime.now().strftime("%Y-%m-%d_%H.%M.%S.mp4")
    camera.start_recording(filename)
    pir.wait_for_no_motion()
    camera.stop_recording()

##########################################################################

#motion.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#tests the functionality of the infrared motion sensor
#based off of: https://www.raspberrypi.org/learning/parent-detector/worksheet/

from gpiozero import MotionSensor

pir = MotionSensor(4)
while True:
    if pir.motion_detected:
        print("Motion detected!")

##########################################################################

#LEDButton.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#this code was all handwritten. The logic was taken from here: https://thepihut.com/blogs/raspberry-pi-tutorials/27968772-turning-on-an-led-with-your-raspberry-pis-gpio-pins
# turns on the LED by the push button. So while the button is TRUE or pressed...

#combined button.py and LED.py

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

switch_pin = 18
led_pin = 23

GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setwarnings(False)
#Added by Tony Kelly

GPIO.setup(led_pin, GPIO.OUT)

led_state = False
old_input_state = True # pulled-up

while True:
    new_input_state = GPIO.input(switch_pin)
    if new_input_state == False and old_input_state == True:
        led_state = not led_state
    old_input_state = new_input_state
    GPIO.output(led_pin, led_state)

##########################################################################

#LED.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#this code was all handwritten. The logic was taken from here: https://thepihut.com/blogs/raspberry-pi-tutorials/27968772-turning-on-an-led-with-your-raspberry-pis-gpio-pins
# turns on the LED

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)

#print "LED on"

GPIO.output(18, GPIO.HIGH)


#time.sleep(10)
#GPIO.output(18, GPIO.LOW)
#time.sleep(10)
#GPIO.output(18, False)

##########################################################################

#gettemp3.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#Much of this code was prepared and taken from: http://www.raspberrypi-spy.co.uk/2015/04/bmp180-i2c-digital-barometric-pressure-sensor/ and 
#https://bitbucket.org/MattHawkinsUK/rpispy-misc/raw/master/python/bmp180.py

#Some lines were edited, most of this was commented, however this was a modified version of my bmp180.py code

#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____          
#   / _ \/ _ \(_) __/__  __ __ 
#  / , _/ ___/ /\ \/ _ \/ // / 
# /_/|_/_/  /_/___/ .__/\_, /  
#                /_/   /___/   
#
#           bmp180.py
#  Read data from a digital pressure sensor.
#
# Author : Matt Hawkins
# Date   : 15/04/2015
#
# http://www.raspberrypi-spy.co.uk/
#
#--------------------------------------
import smbus
import time
from ctypes import c_short
 
DEVICE = 0x77 # Default device I2C address
 
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1 
 
def convertToString(data):
  # Simple function to convert binary data into
  # a string
  return str((data[1] + (256 * data[0])) / 1.2)

def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index] << 8) + data[index + 1]).value

def getUshort(data, index):
  # return two bytes from data as an unsigned 16-bit value
  return (data[index] << 8) + data[index + 1]

def readBmp180Id(addr=DEVICE):
  # Chip ID Register Address
  REG_ID     = 0xD0
  (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
  return (chip_id, chip_version)
  
def readBmp180(addr=DEVICE):
  # Register Addresses
  REG_CALIB  = 0xAA
  REG_MEAS   = 0xF4
  REG_MSB    = 0xF6
  REG_LSB    = 0xF7
  # Control Register Address
  CRV_TEMP   = 0x2E
  CRV_PRES   = 0x34 
  # Oversample setting
  OVERSAMPLE = 3    # 0 - 3
  
  # Read calibration data
  # Read calibration data from EEPROM
  cal = bus.read_i2c_block_data(addr, REG_CALIB, 22)

  # Convert byte data to word values
  AC1 = getShort(cal, 0)
  AC2 = getShort(cal, 2)
  AC3 = getShort(cal, 4)
  AC4 = getUshort(cal, 6)
  AC5 = getUshort(cal, 8)
  AC6 = getUshort(cal, 10)
  B1  = getShort(cal, 12)
  B2  = getShort(cal, 14)
  MB  = getShort(cal, 16)
  MC  = getShort(cal, 18)
  MD  = getShort(cal, 20)

  # Read temperature
  bus.write_byte_data(addr, REG_MEAS, CRV_TEMP)
  time.sleep(0.005)
  (msb, lsb) = bus.read_i2c_block_data(addr, REG_MSB, 2)
  UT = (msb << 8) + lsb

  # Read pressure
  bus.write_byte_data(addr, REG_MEAS, CRV_PRES + (OVERSAMPLE << 6))
  time.sleep(0.04)
  (msb, lsb, xsb) = bus.read_i2c_block_data(addr, REG_MSB, 3)
  UP = ((msb << 16) + (lsb << 8) + xsb) >> (8 - OVERSAMPLE)

  # Refine temperature
  X1 = ((UT - AC6) * AC5) >> 15
  X2 = (MC << 11) / (X1 + MD)
  B5 = X1 + X2
  temperature = (B5 + 8) >> 4

  # Refine pressure
  B6  = B5 - 4000
  B62 = B6 * B6 >> 12
  X1  = (B2 * B62) >> 11
  X2  = AC2 * B6 >> 11
  X3  = X1 + X2
  B3  = (((AC1 * 4 + X3) << OVERSAMPLE) + 2) >> 2

  X1 = AC3 * B6 >> 13
  X2 = (B1 * B62) >> 16
  X3 = ((X1 + X2) + 2) >> 2
  B4 = (AC4 * (X3 + 32768)) >> 15
  B7 = (UP - B3) * (50000 >> OVERSAMPLE)

  P = (B7 * 2) / B4

  X1 = (P >> 8) * (P >> 8)
  X1 = (X1 * 3038) >> 16
  X2 = (-7357 * P) >> 16
  pressure = P + ((X1 + X2 + 3791) >> 4)

  return (temperature/10.0,pressure/ 100.0)

def main():
    
  (chip_id, chip_version) = readBmp180Id()
  print "Chip ID     :", chip_id
  print "Version     :", chip_version

  print
  
  (temperature,pressure)=readBmp180()
  print "Temperature : ", temperature, "C"
  print "Pressure    : ", pressure, "mbar"
  
if __name__=="__main__":
   main()

##########################################################################

#button.py

#taken from https://github.com/simonmonk/raspberrypi_cookbook_ed2/blob/master/switch.py
#was pulled from our text

#he code reads that while the button is "True" or pressed...
 
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    input_state = GPIO.input(18)
    if input_state == False:
        print('Button Pressed')
        time.sleep(0.2)

##########################################################################

#bmp180.py

#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#Much of this code was prepared and taken from: http://www.raspberrypi-spy.co.uk/2015/04/bmp180-i2c-digital-barometric-pressure-sensor/ and 
#https://bitbucket.org/MattHawkinsUK/rpispy-misc/raw/master/python/bmp180.py

#Some lines were edited, most of this was commented

#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____          
#   / _ \/ _ \(_) __/__  __ __ 
#  / , _/ ___/ /\ \/ _ \/ // / 
# /_/|_/_/  /_/___/ .__/\_, /  
#                /_/   /___/   
#
#           bmp180.py
#  Read data from a digital pressure sensor.
#
# Author : Matt Hawkins
# Date   : 17/02/2017
#
# http://www.raspberrypi-spy.co.uk/
#
#--------------------------------------
import smbus
import time
from ctypes import c_short
 
DEVICE = 0x77 # Default device I2C address
 
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1 
 
def convertToString(data):
  # Simple function to convert binary data into
  # a string
  return str((data[1] + (256 * data[0])) / 1.2)

def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index] << 8) + data[index + 1]).value

def getUshort(data, index):
  # return two bytes from data as an unsigned 16-bit value
  return (data[index] << 8) + data[index + 1]

def readBmp180Id(addr=DEVICE):
  # Chip ID Register Address
  REG_ID     = 0xD0
  (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
  return (chip_id, chip_version)
  
def readBmp180(addr=DEVICE):
  # Register Addresses
  REG_CALIB  = 0xAA
  REG_MEAS   = 0xF4
  REG_MSB    = 0xF6
  REG_LSB    = 0xF7
  # Control Register Address
  CRV_TEMP   = 0x2E
  CRV_PRES   = 0x34 
  # Oversample setting
  OVERSAMPLE = 3    # 0 - 3
  
  # Read calibration data
  # Read calibration data from EEPROM
  cal = bus.read_i2c_block_data(addr, REG_CALIB, 22)

  # Convert byte data to word values
  AC1 = getShort(cal, 0)
  AC2 = getShort(cal, 2)
  AC3 = getShort(cal, 4)
  AC4 = getUshort(cal, 6)
  AC5 = getUshort(cal, 8)
  AC6 = getUshort(cal, 10)
  B1  = getShort(cal, 12)
  B2  = getShort(cal, 14)
  MB  = getShort(cal, 16)
  MC  = getShort(cal, 18)
  MD  = getShort(cal, 20)

  # Read temperature
  bus.write_byte_data(addr, REG_MEAS, CRV_TEMP)
  time.sleep(0.005)
  (msb, lsb) = bus.read_i2c_block_data(addr, REG_MSB, 2)
  UT = (msb << 8) + lsb

  # Read pressure
  bus.write_byte_data(addr, REG_MEAS, CRV_PRES + (OVERSAMPLE << 6))
  time.sleep(0.04)
  (msb, lsb, xsb) = bus.read_i2c_block_data(addr, REG_MSB, 3)
  UP = ((msb << 16) + (lsb << 8) + xsb) >> (8 - OVERSAMPLE)

  # Refine temperature
  X1 = ((UT - AC6) * AC5) >> 15
  X2 = (MC << 11) / (X1 + MD)
  B5 = X1 + X2
  temperature = int(B5 + 8) >> 4

  # Refine pressure
  B6  = B5 - 4000
  B62 = int(B6 * B6) >> 12
  X1  = (B2 * B62) >> 11
  X2  = int(AC2 * B6) >> 11
  X3  = X1 + X2
  B3  = (((AC1 * 4 + X3) << OVERSAMPLE) + 2) >> 2

  X1 = int(AC3 * B6) >> 13
  X2 = (B1 * B62) >> 16
  X3 = ((X1 + X2) + 2) >> 2
  B4 = (AC4 * (X3 + 32768)) >> 15
  B7 = (UP - B3) * (50000 >> OVERSAMPLE)

  P = (B7 * 2) / B4

  X1 = (int(P) >> 8) * (int(P) >> 8)
  X1 = (X1 * 3038) >> 16
  X2 = int(-7357 * P) >> 16
  pressure = int(P + ((X1 + X2 + 3791) >> 4))

  return (temperature/10.0,pressure/100.0)

def main():
    
  (chip_id, chip_version) = readBmp180Id()
  print("Chip ID     : {0}".format(chip_id))
  print("Version     : {0}".format(chip_version))

  print
  
  (temperature,pressure)=readBmp180()
  print("Temperature : {0} C".format(temperature))
  print("Pressure    : {0} mbar".format(pressure))
  
if __name__=="__main__":
   main()

##########################################################################

#this is a combination file of all of my code, including most with comments. Most of this code was taken and then edited, however
#all of it is sourced properly. Each individual file is saved as a python script, though most dont have /usr/bin available so it must be
#run through terminal if on Linux or through an IDLE, such as Pycharm, on Windows / OSX (or macOS).

#Anthony Kelly, Spring 2017