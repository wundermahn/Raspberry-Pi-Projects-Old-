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
