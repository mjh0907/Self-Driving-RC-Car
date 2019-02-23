"""
firmware of the car, running on raspberry pi
"""
# External module imports
import RPi.GPIO as GPIO
import time
from threading import Thread
from pynput.keyboard import Key, Listener
import socket
import sys
import struct
from PIL import Image

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import YDLidarX4 as LaserModel
from roboviz import MapVisualizer
from roboviz import Visualizer

# enable self driving mode by default
auto_mode = True
# executor socket host/port, client.py(controller) should connect to this port,
# and send control cmd to this prot
HOST, PORT = "localhost", 50008
my_socket.bind((HOST, PORT))
my_socket.listen(5)
(controller_socket, address) = my_socket.accept()

#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.connect((HOST, PORT))

# Pin Definitons:
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
backPin = 2# Broadcom pin 23 (P1 pin 16)
frontPin = 3
leftPin = 10
rightPin = 17
butPin = 7 # Broadcom pin 17 (P1 pin 11)

freezeUntilTime = 0
GPIO.cleanup()
# Pin Setup:
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(leftPin, GPIO.OUT) # LED pin set as output
GPIO.setup(rightPin, GPIO.OUT) # LED pin set as output
GPIO.setup(frontPin, GPIO.OUT) # LED pin set as output
GPIO.setup(backPin, GPIO.OUT) # LED pin set as output
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(pwmPin, 50)  # Initialize PWM on pwmPin 100Hz frequency
GPIO.setup(butPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Button pin set as input w/ pull-up

# Initial state for LEDs:
GPIO.output(leftPin, GPIO.LOW)
GPIO.output(rightPin, GPIO.LOW)
GPIO.output(frontPin, GPIO.LOW)
GPIO.output(backPin, GPIO.LOW)

def send_cmd_to_car(pin, sec):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(sec)
    GPIO.output(pin, GPIO.LOW) 

def actionAsync(pin, sec):  
    thread = Thread(target = send_cmd_to_car, args = (pin, sec, )) 
    thread.start()
        
def autoDrive():
    global auto_mode
    global controller_socket
    while True:
        if auto_mode:
            time.sleep(1)
            continue
        control_data = controller_socket.recv(8000)
        cmd = control_data.decode()
        print("received cmd: " + cmd + ", applying cmd: " + cmd[-1])
        applyCmd(cmd[-1])

         
def applyCmd(char):
    print("apply cmd: "+char)
    global freezeUntilTime
    millis = int(round(time.time() * 1000))
    if (millis < freezeUntilTime):
        return
        
    actionDuration = 0.1
    freezeUntilTime = millis + actionDuration * 1000
    
    if (char == 'a'):
        actionAsync([leftPin], actionDuration)
    elif (char == 'd'):
        actionAsync([rightPin], actionDuration)
    elif (char == 'w'):
        actionAsync([frontPin], actionDuration)
    elif (char == 'x'):
        actionAsync([backPin], actionDuration)
    elif (char == 'q'):
        actionAsync([frontPin, leftPin], actionDuration)
    elif (char == 'e'):
        actionAsync([frontPin, rightPin], actionDuration)
    elif (char == 'z'):
        actionAsync([backPin, leftPin], actionDuration)
    elif (char == 'c'):
        actionAsync([backPin, rightPin], actionDuration)
    else:
        print("Unrecognized cmd " + char)
 
def on_press(key):
    global auto_mode
    if hasattr(key,'char'):
        if(key.char == 'm'):
            auto_mode = False
            print("enter manual mode")
        elif (key.char == 'i'): # intelligent mode
            auto_mode = True
            print("enter auto mode")
        else:
            applyCmd(key.char)

print("start")
# Collect events until released
with Listener(on_press=on_press) as listener:
    autoDrive()
    listener.join()

GPIO.cleanup() # cleanup all GPIO
