# External module imports
import argparse
import time
#from threading import Thread
import threading
from pynput.keyboard import Key, Listener
import socket
import sys
import struct
from PIL import Image
from Controller.map import CarApp
from Controller.map import getModelAction


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
MAP_QUALITY = 1
HOLE_SIZE_MM = 300
slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, MAP_QUALITY, HOLE_SIZE_MM)

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

# current lidar scan points of cloud (360 degree data), 
scan_lock = threading.RLock()
current_scan = []
pos_lock = threading.RLock()
current_x = 0
current_y = 0
current_theta = 0

# get obstacle distanct in lidar 360 degreescan data, 
# NOTE assume current_scan is ready and data starts from 0 degree, assume clockwise
# return distances(in mm) in 4 directions 
# direction: 0: front, 1: left, 2: right, 3:  back
def get_scan_distance():
    distances = {}
    total_angle = 360.0
    directions = ["front", "left", "right", "back"]
    # direction bounds
    angles = [
                [0, 10, 350, 360], # 0-10, 350-360 degree
                [300, 350],
                [10, 60],
                [170, 190]
             ]
    scan_lock.acquire()
    try:
        size = len(current_scan)  # ydlidar has 720 points per scan
        if size > 0:
            for i in angles:
                angle = angles[i]
                dist = 100000
                starts = angle[0::2]
                ends = angle[1::2]
                for j in range(0, len(starts)):
                    s = int(starts[j]/total_angle*size)
                    e = int(ends[j]/total_angle*size)
                    # todo may need to ignore noise
                    dist = min(dist, min(current_scan[s,e]))
                distances[directions[i]] = dist
    finally:
        scan_lock.release()
    return distances


def update_current_scan(_scan):
    scan_lock.acquire()
    try:
        current_scan = _scan
    finally:
        scan_lock.release()

def get_current_position():
    pos = []
    pos_lock.acquire()
    try:
        pos = (current_x, current_y, current_theta)
    finally:
        pos_lock.release()

    return pos

def update_current_position(x, y, theta):
    pos_lock.acquire()
    try:
        current_x = x
        current_y = y
        current_theta = theta
    finally:
        pos_lock.release()

def doAction(pin, sec):
    time.sleep(sec)

def action(pin, sec):  
    thread = Thread(target = doAction, args = (pin, sec, )) 
    thread.start()

def receive_lidar_data():
    idx = 0
    search_start = True
    # todo parameterize
    # lidar server address
    HOST, PORT = car_host, 50007
    lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lidar_socket.connect((HOST, PORT))
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
        raw_data = lidar_socket.recv(10000)
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
            update_current_scan(distances)
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
        update_current_position(x, y, theta)
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

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
         
cmd_socket=[]
def on_press(key):
    global cmd_socket
    global freezeUntilTime
    millis = int(round(time.time() * 1000))
    if (millis < freezeUntilTime):
        return
        
    actionDuration = 0.1
    freezeUntilTime = millis + actionDuration * 1000
    if hasattr(key,'char'):
        cmd_str = key.char
        print("send manual cmd "+ cmd_str)
        #cmd_socket.send(cmd_str.encode())
        
 
app = CarApp()
def runBrainModel():
    global app
    # load brain
    app.load('Controller/last_brain.pth')
    # load map, TODO prepare map
    app.load_map('map.mp')
    app.run()

def receive_lane_data():
    #create an INET, STREAMing socket
    global cmd_socket
    lane_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lane_host = "localhost"
    lane_port = 5553
    lane_socket.connect((lane_host, lane_port))
    CMD_PORT = 50008
    cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #cmd_socket.connect((car_host, CMD_PORT))

    while True:
        # get lastest lane data
        lane_data = server_socket.recv(2048)
        lane_str = lane_data.decode()
        print("receive lane string:" + lane_str)
        lanes = lane_data.split(";")[-1].split(",")
        print("parsed lane data:")
        print(lanes)

        # parse lidar data
        distances = get_scan_distance()
        print("current distance data:")
        print(distances)

        current_x, current_y, current_theta = get_current_pos()
        print("current pos data:")
        print(current_x)
        print(current_y)
        print(current_theta)

        # call control model here
        action = getModelAction([1,2,3,-1,2])
        print(action)

        #send command line to car
        action_str = "w"
        print("send auto cmd " + action_str)
        #cmd_socket.send(action_str.encode())
        # todo wait for car to act here?

"""def main():
parser = argparse.ArgumentParser()
parser.add_argument('-s', '--car_host',
                    help='car/raspberry pi IP address',
                    required=True)

args = parser.parse_args()

if args.car_host:
    car_host = args.car_host
"""

car_host = "localhost"
print("running brain model")
brain_thread = threading.Thread(target=runBrainModel)
brain_thread.daemon = True
brain_thread.start()

print("start thread to receive lane data")
lane_thread = threading.Thread(target=receive_lane_data)
lane_thread.daemon = True
lane_thread.start()
# Collect events until released
with Listener(
        on_press=on_press) as listener:
    # todo uncomment this
    #autoDrive()
    receive_lidar_data()
    listener.join()

#if __name__ == '__main__':
#    main()
