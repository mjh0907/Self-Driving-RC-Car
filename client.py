# External module imports
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
image_filename = './dash'

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
freezeUntilTime = 0

def doAction(pin, sec):
    time.sleep(sec)

def action(pin, sec):  
    thread = Thread(target = doAction, args = (pin, sec, )) 
    thread.start()
    	

def autoDrive():
    global autoMode
    idx = 0
    search_start = True

    HOST, PORT = "192.168.0.18", 50007
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    feed_point = 720 # ydlidar x4, each scan contains 720 point
    left_over = []
    while True:
        idx = idx + 1

        '''
        test_angles    = [x/2.0 for x in range(0,720)]
        test_distances = [(idx % 5) for x in range(0,720)]
        zipped = zip(test_angles, test_distances)
        flatten = [item for sublist in zipped  for item in sublist]
        new_data = flatten
        '''
        print("receiving new data...")
        raw_data = s.recv(10000)
        num_floats = int(len(raw_data) / 4)
        format_str = 'f' * num_floats
        new_data = list(struct.unpack(format_str, raw_data[:num_floats * 4]))
        print("received new data", repr(len(new_data)))
        if search_start: # find start position
            for pos, val in enumerate(new_data[::4]):
                if val == -180:
                    left_over = new_data[pos:]
                    search_start = False
            print("located start position")
            continue

        data = left_over + new_data
            
        #if (idx % 50) != 0: # use a big number!! or will read more than 720 points
            #time.sleep(0.5)
        #    continue
            # Extract distances and angles from tuple
        
        #raw_dist = data[1::2]
        #distances = [i*1000 for i in raw_dist] # raw data is in meters, convert to mm
        #angles    = data[0::2]
        received_size = len(data) #min(len(distances), len(angles))
        distances = []
        angles = []
        feed_size = feed_point * 2
        if (received_size >= feed_size): # feed distance and angle data
            skipScan = len(data)/feed_size -1
            if skipScan > 0:
                print("*************************skipped " + repr(skipScan) + " scan*****")
            data = data[skipScan*feed_size:]
            angles = data[0:feed_size:2] 
            if (angles[0] != -180): # found invalid data, restart!
                print("left over is:") 
                print(left_over[::2])
                print("new angle is:") 
                print(angles)
                search_start = True
                left_over = []
                print("found invalid data, restart!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                continue
            distances = [i*1000 for i in data[1:feed_size:2]]
            left_over = data[feed_size:]
        else:
            left_over = left_over + data
            continue #wait until received enough sample
            # assume first package start from angle 0
            # todo assert data size is even

        #get data from latest 360 degree scan
        
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
        #image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), mapbytes, 'raw', 'L', 0, 1)
        #image.save('%s.png' % image_filename)
        print("done gen image..")
            # Display map and robot pose, exiting gracefully if user closes it
        print("display..")
        if not viz.display(x/1000., y/1000., theta, mapbytes):
             exit(0)
        print("done display")
        #doAction([leftPin], 0.1)
              #time.sleep(1)

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
         
def on_press(key):
    global freezeUntilTime
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
	else:
	    print("Unrecognized key pressed")
	    print('{0} pressed'.format(key))
 


print("start")
# Collect events until released
with Listener(
        on_press=on_press) as listener:
    autoDrive()
    listener.join()
