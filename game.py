
# Importing the Dqn object from our AI in ai.py
from aii import Dqn
import random
import math
import matplotlib.pyplot as plt
import time
import matplotlib.patches as patches


def eu_distance(a, b):
	return math.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)

class Map:
	def __init__(self, length, width, origin_point, end_point, delta):
		self.length = length
		self.width = width

		self.origin_point = origin_point
		self.end_point = end_point
		self.delta = delta

		self.min_x = self.origin_point[0]
		self.min_y = self.origin_point[1]

		self.max_x = self.min_x + length
		self.max_y = self.min_y + width

	def is_in_map(self, car):
		if car.x >= self.min_x and car.x <= self.max_x and car.y >= self.min_y and car.y <= self.max_y:
			return 1
		else:
			return 0

	def is_successful(self, car):
		if abs(car.x - self.end_point[0]) < self.delta and abs(car.y - self.end_point[1]) < self.delta:
			return 1
		else:
			return 0

	def get_reward(self, car):
		if len(car.way) < 2:
			return car.reward

		if not self.is_in_map(car):
			car.reward = -10.0
			return car.reward

		if self.is_successful(car):
			car.reward = 10.0
			return car.reward 

		current_point = car.way[-1]
		previous_point = car.way[-2]
		change = eu_distance(current_point, self.end_point) - eu_distance(previous_point, self.end_point)

		if change > 0:
			car.reward = 2.0
			return car.reward
		else:
			car.reward = -2.0
			return car.reward



def get_move(action):
	# 0 is moving forward
	if action == 0:
		return [1, 0]
	# 1 is moving down
	elif action == 1:
		return [0, -1]
	#2 is moving backwards
	elif action == 2:
		return [-1, 0]
	elif action == 3:
		return [0, 1]


class Car():
	def __init__(self, initial_point):
		self.initial_point = initial_point
		self.reset()
		self.way.append([self.x, self.y])

	def move(self, action):
		a, b = get_move(action)
		new_x = self.x + a
		new_y = self.y + b
		print ("Car moved from {0}, {1} to {2}, {3}".format(self.x, self.y, new_x, new_y))
		self.x = new_x
		self.y = new_y
		self.way.append([self.x, self.y])

	def reset(self):
		self.reset_pos()
		self.reward = 0

	def reset_pos(self):
		self.x = self.initial_point[0]
		self.y = self.initial_point[1]
		self.way = []

	def set_current_reward(self, reward):
		self.reward = reward


class Game():
	def run(self):
		new_car = Car([0, 10])
		new_map = Map(100, 20, [0,0], [100, 10], 2)

		brain = Dqn(3, 4, 0.9)

		for epoch in range(100):
			print ("Epoch # {}".format(epoch))
			new_car.reset()

			xdata = []
			xdata.append(new_car.x)
			ydata = []
			ydata.append(new_car.y)

			plt.show()
			axes = plt.gca()
			axes.set_xlim(-10, 100)
			axes.set_ylim(-10, 50)
			line, = axes.plot(xdata, ydata, 'r-')

			axes.add_patch(plt.Rectangle((0, 0), 100, 20, fill=False, edgecolor='b', linewidth=3))

			while not new_map.is_successful(new_car):
				
				is_in_map = new_map.is_in_map(new_car)
				current_signal = [new_car.x, new_car.y, is_in_map]

				print("Current reward is {}".format(new_car.reward))
				print("Current signal is {}".format(current_signal))

				action = brain.update(new_car.reward, current_signal).item()

				print("Current action is: {}".format(action))
				new_car.move(action)
				reward = new_map.get_reward(new_car)
				new_car.set_current_reward(reward)

				if not new_map.is_in_map(new_car):
					new_car.reset_pos()

				elif new_map.is_successful(new_car):
					break

				if (len(xdata) > 10):
					xdata.pop(0)
					ydata.pop(0)
				xdata.append(new_car.x)
				ydata.append(new_car.y)

				line.set_xdata(xdata)
				line.set_ydata(ydata)
				plt.draw()

				plt.pause(1e-17)
				time.sleep(0.01)

			plt.close()	



if __name__ == "__main__":
	Game().run()

