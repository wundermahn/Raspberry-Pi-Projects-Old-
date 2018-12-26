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
