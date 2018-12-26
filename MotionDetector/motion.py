#Anthony Kelly - La Salle University - B.S. I.T. May 2017 - Spring 2017, Research, Hardware Design Theory with RbP

#tests the functionality of the infrared motion sensor
#based off of: https://www.raspberrypi.org/learning/parent-detector/worksheet/

from gpiozero import MotionSensor

pir = MotionSensor(4)
while True:
    if pir.motion_detected:
        print("Motion detected!")
