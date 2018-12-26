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