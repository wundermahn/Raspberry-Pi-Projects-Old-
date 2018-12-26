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
