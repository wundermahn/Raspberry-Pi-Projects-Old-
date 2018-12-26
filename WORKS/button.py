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
