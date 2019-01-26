# External module imports
import RPi.GPIO as GPIO
import time
from threading import Thread
from pynput.keyboard import Key, Listener
import socket
import sys
import struct
from PIL import Image


MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 15


# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 200

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import YDLidarX4 as LaserModel
from roboviz import MapVisualizer
from roboviz import Visualizer

class CustomizedMapVisualizer(Visualizer):
    
    def __init__(self, map_size_pixels, map_size_meters, title='MapVisualizer', show_trajectory=False):

        # Put origin in lower left; disallow zero-angle setting
        Visualizer._init(self, map_size_pixels, map_size_meters, title, 0, show_trajectory, 0)

    def display(self, x_m, y_m, theta_deg, mapbytes, image_filename):

        self._setPose(x_m, y_m, theta_deg)

        mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        # Pause to allow display to refresh
        plt.pause(.001)

	# save to a png file
        fig.savefig(image_filename)

        return self._refresh()

# image file
image_filename = '/home/ubuntu/RCWeb/dash'

HOST, PORT = "localhost", 50007

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((HOST, PORT))
# Create an RMHC SLAM object with a laser model and optional robot model
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
trajectory = []

    # Initialize empty map
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # We will use these to store previous scan in case current scan is inadequate
previous_distances = None
previous_angles    = None

# Pin Definitons:
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
backPin = 2# Broadcom pin 23 (P1 pin 16)
frontPin = 3
leftPin = 10
rightPin = 17
butPin = 7 # Broadcom pin 17 (P1 pin 11)

freezeUntilTime = 0
dc = 95 # duty cycle (0-100) for PWM pin
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
# pwm.start(dc)

def doAction(pin, sec):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(sec)
    GPIO.output(pin, GPIO.LOW) 

def action(pin, sec):  
    thread = Thread(target = doAction, args = (pin, sec, )) 
    thread.start()
    	
stop = False

def autoDrive():
    global autoMode
    global stop
    idx = 0
    while True:
        #if idx > 500:
        #    break
	idx = idx + 1
        if stop:
            time.sleep(1)
            continue
	raw_data = s.recv(8000)
	num_floats = int(len(raw_data) / 4)
	format_str = 'f' * num_floats
	data = list(struct.unpack(format_str, raw_data[:num_floats * 4]))
        
	if (idx % 40) != 0: # use a big number!! or will read more than 720 points
	    #time.sleep(0.5)
	    continue
        # Extract distances and angles from tuple
        raw_dist = data[1::2]
        distances = [i*1000 for i in raw_dist]
        angles    = data[0::2]
        #distances = [1000*(idx % 5) for x in range(0,720)]#[i*1000 for i in raw_dist]
        #angles    = [x/2.0 for x in range(0,720)]#data[0::2]
	print("distances:")
	print(len(distances))
	#print(distances)
	print("angles:")
	#print(angles)
	print(len(angles))
	
	print("start: update"+repr(idx))
        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances) #, scan_angles_degrees=angles)
	print("end: update"+repr(idx))
            #previous_distances = distances#.copy()
            #previous_angles    = angles#.copy()

        # If not adequate, use previous
        #elif previous_distances is not None:
            #slam.update(previous_distances, scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = slam.getpos()
	# Add new position to trajectory
        trajectory.append((x, y))

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)
# Put trajectory into map as black pixels
	for coords in trajectory:
			
	    x_mm, y_mm = coords
    	    x_pix = mm2pix(x_mm)
	    y_pix = mm2pix(y_mm)
	    mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0;
 # Save map and trajectory as PNG file
	print("gen image..")
	image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), mapbytes, 'raw', 'L', 0, 1)
	image.save('%s.png' % image_filename)
	print("done gen image..")
        # Display map and robot pose, exiting gracefully if user closes it
	#print("display..")
        #if idx % 5 == 0 and  not viz.display(x/1000., y/1000., theta, mapbytes):
        #   exit(0)
	#doAction([leftPin], 0.1)
          #time.sleep(1)



def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
         
def on_press(key):
    global freezeUntilTime
    global stop
    millis = int(round(time.time() * 1000))
    if (millis < freezeUntilTime):
        return
        
    actionDuration = 0.1
    freezeUntilTime = millis + actionDuration * 1000
    if hasattr(key,'char'):
	#print("freeze until")
	#print(freezeUntilTime)
	if (key.char == 'a'):
	    action([leftPin], actionDuration)
	elif (key.char == 'd'):
	    action([rightPin], actionDuration)
	elif (key.char == 'w'):
	    action([frontPin], actionDuration)
	elif (key.char == 'x' or key.char == 's'):
	    action([backPin], actionDuration)
	elif (key.char == 'q'):
	    action([frontPin, leftPin], actionDuration)
	elif (key.char == 'e'):
	    action([frontPin, rightPin], actionDuration)
	elif (key.char == 'z'):
	    action([backPin, leftPin], actionDuration)
	elif (key.char == 'c'):
	    action([backPin, rightPin], actionDuration)
	elif (key.char == 'p'):
	    stop = not stop
	else:
	    print("Unrecognized key pressed")
	    print('{0} pressed'.format(key))
 


print("start")
#autoDrive()
# Collect events until released
with Listener(
        on_press=on_press) as listener:
    autoDrive()
    listener.join()


"""
# time.sleep(5)
print("Here we go! Press CTRL+C to exit")
try:
    while 1:
        if 0: # GPIO.input(butPin): # button is released
            pwm.ChangeDutyCycle(dc)
            GPIO.output(leftPin, GPIO.LOW)
        else: # button is pressed:
            # pwm.ChangeDutyCycle(100-dc)
            print("front")
            GPIO.output(frontPin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(frontPin, GPIO.LOW)
            time.sleep(0.5)
             

            print("back")
            GPIO.output(backPin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(backPin, GPIO.LOW)
            time.sleep(0.5)

            print("left")
            GPIO.output(leftPin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(leftPin, GPIO.LOW)
            time.sleep(0.5)

            print("right")
            GPIO.output(rightPin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(rightPin, GPIO.LOW)
            time.sleep(0.5)

except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
    # pwm.stop() # stop PWM
    GPIO.cleanup() # cleanup all GPIO

"""
GPIO.cleanup() # cleanup all GPIO
