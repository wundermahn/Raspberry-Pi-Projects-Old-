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
