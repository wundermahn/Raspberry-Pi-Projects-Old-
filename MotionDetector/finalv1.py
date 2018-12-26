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
