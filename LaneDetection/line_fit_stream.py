import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle
from combined_thresh import combined_thresh
from perspective_transform import perspective_transform
from Line import Line
from line_fit import line_fit, tune_fit, final_viz, calc_curve, calc_vehicle_offset
from moviepy.editor import VideoFileClip
import argparse
from time import sleep
import threading
import socket

import zmq

from VideoStream.constants import PORT
from VideoStream.utils import string_to_image
from RoadLaneLineDetection.lane_lines import annotate_image_array


class StreamViewer:
    def __init__(self, lane_server, lane_port, port=PORT):
        """
        Binds the computer to a ip address and starts listening for incoming streams.

        :param port: Port which is used for streaming
        """
        context = zmq.Context()
        self.footage_socket = context.socket(zmq.SUB)
        self.footage_socket.bind('tcp://*:' + port)
        self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))
        self.current_frame = None
        self.keep_running = True

        print("connecting to lane host..(please make sure you started car brain computer)"+repr(lane_host)+":"+repr(lane_port))
        self.lane_socket = socket.socket()
        #self.lane_socket.connect((lane_host, lane_port))

        self.lane_socket.bind((lane_host, lane_port))
        #become a server socket
        self.lane_socket.listen(5)

        print("connected to lane host")

        self.lane_data_buffer = []
        self.lock = threading.RLock()
        self.t = threading.Thread(target=self.send_lane_data_loop)
        self.t.daemon = True
        self.t.start()

    def send_lane_data_loop(self):
        print("Waiting controller to connect")
        #accept connections from outside
        (client_socket, address) = self.lane_socket.accept()
        print("Connected to controller and started send lane data loop")

        while True:
            self.lock.acquire()
            try:
                if len(self.lane_data_buffer) > 0:
                    # todo do we need to encode the data?
                    send_str = ""
                    it = iter(self.lane_data_buffer)
                    for l,r in zip(it, it):
                        send_str += "%s,%s;"%(str(l), str(r))
                    client_socket.send(send_str.encode())
                    self.lane_data_buffer = []
                    print("send lane data " + send_str)
            finally:
                self.lock.release()
            sleep(0.05)
        print("stopped send lane data")

   
    def append_lane_data(self, lane_data):
        #lock and send
        self.lock.acquire()
        try:
            self.lane_data_buffer.extend(lane_data)
            #self.lane_data_buffer.append(lane_data)
        finally:
            self.lock.release()

    def receive_stream(self, display=True):
        """
        Displays displayed stream in a window if no arguments are passed.
        Keeps updating the 'current_frame' attribute with the most recent frame, this can be accessed using 'self.current_frame'
        :param display: boolean, If False no stream output will be displayed.
        :return: None
        """
        self.keep_running = True
        idx = 0
        while self.footage_socket and self.keep_running:
            try:
                self.current_frame = mpimg.imread("messigray.bmp")
                """frame = self.footage_socket.recv_string()
                idx = (idx + 1)%10000000
                if idx % 24 == 0:
                    #print("received:"+frame)
                    self.current_frame = string_to_image(frame)
                """
                annotated_image = self.current_frame 
                #cv2.imshow("Stream", annotated_image)
                #cv2.imshow("Stream Org2", annotated_image)
                #cv2.waitKey(1)
                cv2.imshow("Stream Org", annotated_image)
                cv2.waitKey(1)
                #if idx % 100 == 0:
                try:
                    #print("annotated")
                    annotated_image, lane_distance = annotate_image_array(self.current_frame)
                    #print("done annotated: "+repr(idx))
                    #print(lane_distance)
                    self.append_lane_data(lane_distance)
                    #annotated_image = annotate_image(self.current_frame)
                    if display:
                        cv2.imshow("Stream", annotated_image)
                        cv2.waitKey(1)
                except:
                    annotated_image = self.current_frame 
                    #print("not found")

            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                break
        print("Streaming Stopped!")

    def stop(self):
        """
        Sets 'keep_running' to False to stop the running loop if running.
        :return: None
        """
        self.keep_running = False




# Global variables (just to make the moviepy video annotation work)
with open('calibrate_camera.p', 'rb') as f:
	save_dict = pickle.load(f)
mtx = save_dict['mtx']
dist = save_dict['dist']
window_size = 5  # how many frames for line smoothing
left_line = Line(n=window_size)
right_line = Line(n=window_size)
detected = False  # did the fast line fit detect the lines?
left_curve, right_curve = 0., 0.  # radius of curvature for left and right lanes
left_lane_inds, right_lane_inds = None, None  # for calculating curvature


# MoviePy video annotation will call this function
def annotate_image(img_in):
	"""
	Annotate the input image with lane line markings
	Returns annotated image
	"""
	global mtx, dist, left_line, right_line, detected
	global left_curve, right_curve, left_lane_inds, right_lane_inds

	# Undistort, threshold, perspective transform
	undist = cv2.undistort(img_in, mtx, dist, None, mtx)
	img, abs_bin, mag_bin, dir_bin, hls_bin = combined_thresh(undist)
	binary_warped, binary_unwarped, m, m_inv = perspective_transform(img)

	# Perform polynomial fit
	if not detected:
		# Slow line fit
		ret = line_fit(binary_warped)
		left_fit = ret['left_fit']
		right_fit = ret['right_fit']
		nonzerox = ret['nonzerox']
		nonzeroy = ret['nonzeroy']
		left_lane_inds = ret['left_lane_inds']
		right_lane_inds = ret['right_lane_inds']

		# Get moving average of line fit coefficients
		left_fit = left_line.add_fit(left_fit)
		right_fit = right_line.add_fit(right_fit)

		# Calculate curvature
		left_curve, right_curve = calc_curve(left_lane_inds, right_lane_inds, nonzerox, nonzeroy)

		detected = True  # slow line fit always detects the line

	else:  # implies detected == True
		# Fast line fit
		left_fit = left_line.get_fit()
		right_fit = right_line.get_fit()
		ret = tune_fit(binary_warped, left_fit, right_fit)
		left_fit = ret['left_fit']
		right_fit = ret['right_fit']
		nonzerox = ret['nonzerox']
		nonzeroy = ret['nonzeroy']
		left_lane_inds = ret['left_lane_inds']
		right_lane_inds = ret['right_lane_inds']

		# Only make updates if we detected lines in current frame
		if ret is not None:
			left_fit = ret['left_fit']
			right_fit = ret['right_fit']
			nonzerox = ret['nonzerox']
			nonzeroy = ret['nonzeroy']
			left_lane_inds = ret['left_lane_inds']
			right_lane_inds = ret['right_lane_inds']

			left_fit = left_line.add_fit(left_fit)
			right_fit = right_line.add_fit(right_fit)
			left_curve, right_curve = calc_curve(left_lane_inds, right_lane_inds, nonzerox, nonzeroy)
		else:
			detected = False

	vehicle_offset = calc_vehicle_offset(undist, left_fit, right_fit)

	# Perform final visualization on top of original undistorted image
	result = final_viz(undist, left_fit, right_fit, m_inv, left_curve, right_curve, vehicle_offset)

	return result


def annotate_stream(input_file, output_file):
	""" Given input_file video, save annotated video to output_file """
	video = VideoFileClip(input_file)
	annotated_video = video.fl_image(annotate_image)
	annotated_video.write_videofile(output_file, audio=False)


if __name__ == '__main__':
	# Annotate the video
    port = PORT

    parser = argparse.ArgumentParser()
    parser.add_argument('-lh', '--lane_host',
                        help='The host of lane data receiver', required=True)
    parser.add_argument('-lp', '--lane_port',
                        help='The port of lane data receiver', required=True)
    parser.add_argument('-p', '--port',
                        help='The port which you want the Streaming Viewer to use, default'
                             ' is ' + PORT, required=False)

    args = parser.parse_args()
    if args.port:
        port = args.port

    lane_host = args.lane_host
    lane_port = int(args.lane_port)
    stream_viewer = StreamViewer(lane_host, lane_port, port)
    stream_viewer.receive_stream()
'''
    # Show example annotated image on screen for sanity check
    img_file = 'test_images/test7.jpg'
    #img_file = 'test_images/test2.jpg'
    img = mpimg.imread(img_file)
    result = annotate_image(img)
    result = annotate_image(img)
    result = annotate_image(img)
    plt.imshow(result)
    plt.show()
'''
