# Self Driving Car

# Importing the libraries
import numpy as np
from random import random, randint
import random
import matplotlib.pyplot as plt
import time
import sys

# Importing the Kivy packages
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.graphics import Color, Ellipse, Line, Rectangle
from kivy.config import Config
from kivy.properties import NumericProperty, ReferenceListProperty, ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock

# Importing the Dqn object from our AI in ai.py
from aii import Dqn

# Adding this line if we don't want the right click to put a red point
Config.set('input', 'mouse', 'mouse,multitouch_on_demand')

# Introducing last_x and last_y, used to keep the last point in memory when we draw the sand on the map
last_x = 0   # axis x is horizontal, y is vertical, left bottom coordinates are (0,0)
last_y = 0
n_points = 0 # number of sand points in the current sand line
length = 0 # length of current sand line

# Getting our AI, which we call "brain", and that contains our neural network that represents our Q-function
brain = Dqn(5,3,0.9) # 7 inputs, 4 outputs
# keep 0 at first
action2rotation = [0,20,-20, 180] # action rotations, go straigth, left turn 20 degree, right turn 20 degree, reverse
last_reward = 0
scores = []

# Initializing the map
first_update = True
def init():
    global sand
    global goal_x
    global goal_y
    global first_update
    sand = np.zeros((longueur,largeur))
    goal_x = 20
    goal_y = largeur - 20
    first_update = False
    
def getModelAction(last_signal):
        #last_signal = [self.car.signal1, self.car.signal2, self.car.signal3, orientation, -orientation]
        #last_signal = [self.car.signal1, self.car.signal2, self.car.signal3, self.car.signal4, orientation, -orientation, distance_to_lane]
        brain.update(last_reward, last_signal)

# Initializing the last distance
continus_turn_times = 0
last_distance = 0
last_on_sand = False
last_action = -1

# Creating the car class

max_pixel_density = 16
car_size = 10
#car_size = 40 
class Car(Widget):
    #size = (car_size, 5)
    #color = (0, 100, 0)

    angle = NumericProperty(0) # current heading direction
    rotation = NumericProperty(0) # current action rotation
    velocity_x = NumericProperty(0)
    velocity_y = NumericProperty(0)
    velocity = ReferenceListProperty(velocity_x, velocity_y)
    # todo use list instead of 4 sensor objects
    sensor1_x = NumericProperty(0)
    sensor1_y = NumericProperty(0)
    sensor1 = ReferenceListProperty(sensor1_x, sensor1_y)
    sensor2_x = NumericProperty(0)
    sensor2_y = NumericProperty(0)
    sensor2 = ReferenceListProperty(sensor2_x, sensor2_y)
    sensor3_x = NumericProperty(0)
    sensor3_y = NumericProperty(0)
    sensor3 = ReferenceListProperty(sensor3_x, sensor3_y)
    sensor4_x = NumericProperty(0)
    sensor4_y = NumericProperty(0)
    sensor4 = ReferenceListProperty(sensor4_x, sensor4_y)
    signal1 = NumericProperty(0)  # signal from sensor1, pixel density of sensor, 0-255
    signal2 = NumericProperty(0) # signal from sensor2
    signal3 = NumericProperty(0) # signal from sensor3
    signal4 = NumericProperty(0) # signal from sensor4

    def move(self, action):
        #print("position: (" + str(self.x) + ", " + str(self.y) + ")")
       
        rotation = action2rotation[action]
        self.pos = Vector(*self.velocity) + self.pos
        self.rotation = rotation
        self.angle = self.angle + self.rotation
        side_detect_angle = 30 # left/right sensor angle to the current heading direction
        detect_area_range = 30 # sensor detection distance, TODO, set to detect_area_length/2?
        detect_area_length = 10 # sensor detection length
        detect_area_width = 10 # sensor detection width, e.g, for the example below, 
                               # the detection range is 6 pixel ahead, length is 4 pixel, width is 1 pixel, 
                               # (dots within [] represents detected area),  i.e, ....[..|..]
        # TODO use car size length, width
        sand_count1 = float(np.count_nonzero(sand[int(self.sensor1_x)-detect_area_length:int(self.sensor1_x)+detect_area_length, int(self.sensor1_y)-detect_area_width:int(self.sensor1_y)+detect_area_width]))
        sand_count2 = float(np.count_nonzero(sand[int(self.sensor2_x)-detect_area_length:int(self.sensor2_x)+detect_area_length, int(self.sensor2_y)-detect_area_width:int(self.sensor2_y)+detect_area_width]))
        sand_count3 = float(np.count_nonzero(sand[int(self.sensor3_x)-detect_area_length:int(self.sensor3_x)+detect_area_length, int(self.sensor3_y)-detect_area_width:int(self.sensor3_y)+detect_area_width]))
        sand_count4 = float(np.count_nonzero(sand[int(self.sensor3_x)-detect_area_length:int(self.sensor4_x)+detect_area_length, int(self.sensor4_y)-detect_area_width:int(self.sensor4_y)+detect_area_width]))
        total_pixel = (detect_area_length * 2) * (detect_area_width * 2)
        # avoid divide by 0 and filter small obstacle and noise
        sand_pixel_count_threshold = total_pixel * 0.05
        if (sand_count1 < sand_pixel_count_threshold):
            sand_count1 = sys.maxint 
        if (sand_count2 < sand_pixel_count_threshold):
            sand_count2 = sys.maxint 
        if (sand_count3 < sand_pixel_count_threshold) :
            sand_count3 = sys.maxint 
        if (sand_count4 < sand_pixel_count_threshold) :
            sand_count4 = sys.maxint 
        self.sensor1 = Vector(detect_area_range, 0).rotate(self.angle) + self.pos  # front sensor
        self.sensor2 = Vector(detect_area_range, 0).rotate((self.angle+side_detect_angle)%360) + self.pos  # left sensor
        self.sensor3 = Vector(detect_area_range, 0).rotate((self.angle-side_detect_angle)%360) + self.pos # right sensor
        self.sensor4 = Vector(detect_area_range, 0).rotate((self.angle-180)%360) + self.pos # back sensor
        # avg non zero pixel density, as simple avg all pixel will not be able to detect small obstacles
        # todo may need to improve it
        # python int is big enough to avoid overflow
        self.signal1 = int(np.sum(sand[int(self.sensor1_x)-detect_area_length:int(self.sensor1_x)+detect_area_length, int(self.sensor1_y)-detect_area_width:int(self.sensor1_y)+detect_area_width]))/sand_count1
        self.signal2 = int(np.sum(sand[int(self.sensor2_x)-detect_area_length:int(self.sensor2_x)+detect_area_length, int(self.sensor2_y)-detect_area_width:int(self.sensor2_y)+detect_area_width]))/sand_count2
        self.signal3 = int(np.sum(sand[int(self.sensor3_x)-detect_area_length:int(self.sensor3_x)+detect_area_length, int(self.sensor3_y)-detect_area_width:int(self.sensor3_y)+detect_area_width]))/sand_count3
        self.signal4 = int(np.sum(sand[int(self.sensor4_x)-detect_area_length:int(self.sensor4_x)+detect_area_length, int(self.sensor4_y)-detect_area_width:int(self.sensor4_y)+detect_area_width]))/sand_count4
        # if detection range is out of the map, signal set to 1
        if self.sensor1_x>longueur-car_size or self.sensor1_x<car_size or self.sensor1_y>largeur-car_size or self.sensor1_y<car_size:
            self.signal1 = -max_pixel_density
        if self.sensor2_x>longueur-car_size or self.sensor2_x<car_size or self.sensor2_y>largeur-car_size or self.sensor2_y<car_size:
            self.signal2 = -max_pixel_density
        if self.sensor3_x>longueur-car_size or self.sensor3_x<car_size or self.sensor3_y>largeur-car_size or self.sensor3_y<car_size:
            self.signal3 = -max_pixel_density
        if self.sensor4_x>longueur-car_size or self.sensor4_x<car_size or self.sensor4_y>largeur-car_size or self.sensor4_y<car_size:
            self.signal4 = -max_pixel_density

class Ball1(Widget):
    #size = (5, 5)
    pass
class Ball2(Widget):
    #size = (5, 5)
    pass
class Ball3(Widget):
    #size = (5, 5)
    pass

# Creating the game class

car_speed_per_unit= 6 # car speed per unit
car_slow_speed_per_unit= 3 # car slow speed per unit
class Game(Widget):

    car = ObjectProperty(None) #Car()
    ball1 = ObjectProperty(None) ##Ball1()
    ball2 = ObjectProperty(None) ##Ball2()
    ball3 = ObjectProperty(None) ##Ball3()

    def serve_car(self):
        global car_speed_per_unit
        self.car.center = self.center
        #TODO set car velocity
        self.car.velocity = Vector(car_speed_per_unit, 0)
        #self.car.color = (1, 0.3, 0.4)
        # self.car.size = (5, 5)
        #self.add_widget(self.car)
        #self.add_widget(self.ball1)
        #self.add_widget(self.ball2)
        #self.add_widget(self.ball3)

    def update(self, dt):
        global brain
        global last_reward # reward of last action
        global scores
        global last_distance
        global last_on_sand
        global last_action
        global goal_x
        global goal_y
        global longueur # length of map
        global largeur # width of map
        global car_speed_per_unit, car_slow_speed_per_unit
        global car_size
        global continus_turn_times 

        longueur = self.width
        largeur = self.height
        if first_update:
            self.steps = 0
            self.last_steps = 0
            init()

        xx = goal_x - self.car.x
        yy = goal_y - self.car.y
        orientation = Vector(*self.car.velocity).angle((xx,yy))/180.
        distance_to_lane = 0
        # TODO set distance to lane
        last_signal = [self.car.signal1, self.car.signal2, self.car.signal3, orientation, -orientation]
        #last_signal = [self.car.signal1, self.car.signal2, self.car.signal3, self.car.signal4, orientation, -orientation, distance_to_lane]
        action = brain.update(last_reward, last_signal)
        wall_reward = -500 # 
        last_reward = 0 # init
        if self.car.signal1 < 0 and action == 0:
            action = 3
            last_reward = wall_reward
        if self.car.signal2 < 0 and action == 1:
            action = 3
            last_reward = wall_reward
        if self.car.signal3 < 0 and action == 2:
            action = 3
            last_reward = wall_reward
        #print("score: "+repr(brain.score()))
        scores.append(brain.score())
         # TODO send cmd to car
        # todo make sure signal4 is 0
        self.car.move(action)
        distance = np.sqrt((self.car.x - goal_x)**2 + (self.car.y - goal_y)**2)
        self.ball1.pos = self.car.sensor1
        self.ball2.pos = self.car.sensor2
        self.ball3.pos = self.car.sensor3
        self.steps += 1
        
        sand_reward = 10 #10 # 
        to_no_sand_reward = -500# 
        to_sand_reward = 0.1 # 
        step_reward = 1# 0l2 #0.1 # basic reward each step
        forward_reward = 0.0 # 
        #sand_reward = -50
        half_car_size = car_size / 2
        total = int(np.sum(sand[int(self.car.x)-half_car_size: int(self.car.x)+half_car_size,  int(self.car.y)-half_car_size :int(self.car.y)+half_car_size ]))
        #if sand[int(self.car.x),int(self.car.y)] > obstacle_density_threshold:
        if total > 0:
            # hit sand
            # TODO should stop or slowdown, backward? now backward, should choise left/right/backward/forawrd according to signal
            #self.car.velocity = Vector(1, 0).rotate(self.car.angle)
            #print("on lane")
            # slowdown to get more bad samples?
            self.car.velocity = Vector(car_slow_speed_per_unit, 0).rotate(self.car.angle)
            last_reward += sand_reward # sand reward
            if (last_on_sand == False):
                last_reward += to_sand_reward # sand reward
            last_on_sand = True
            #print("on sand")
        elif total < 0:
            print ("hit wall")
            if (last_on_sand == True):
                last_reward += to_no_sand_reward # sand reward
                #print("to no sand")
            last_on_sand = False
            # obstacle
            self.car.velocity = Vector(car_slow_speed_per_unit, 0).rotate(self.car.angle)
            last_reward += wall_reward # sand reward
        else: # otherwise
            #print ("unpaved road")
            self.car.velocity = Vector(car_speed_per_unit, 0).rotate(self.car.angle)
            if (last_on_sand == True):
                last_reward += to_no_sand_reward # sand reward
                #print("to no sand")
            last_on_sand = False
            if distance < last_distance:
                last_reward += step_reward * (last_distance - distance) # driving towards objective reward
            else:
                last_reward += -step_reward * (last_distance - distance) # if driving away from objective reward
        
        if last_action == 0:
            last_reward += forward_reward
            continus_turn_times = 0
        else:
            continus_turn_times += 1
            #print("turning "+repr(continus_turn_times) + " times")
        last_reward += -0.5* max(0, continus_turn_times-5) # each time we turn 20 degree

        last_reward += -0.005*self.steps #
            # TODO consider distance to obstacle, consider driving angle, lane departion, prefer go straight

        # TODO allow go out of boundary may lead to run time error?
        if self.car.x < car_size:
            self.car.x = car_size
            last_reward += wall_reward # too close to edges of the wall reward
        if self.car.x > self.width - car_size:
            self.car.x = self.width - car_size
            last_reward += wall_reward #
        if self.car.y < car_size:
            self.car.y = car_size
            last_reward +=  wall_reward #
        if self.car.y > self.height - car_size:
            self.car.y = self.height - car_size
            last_reward += wall_reward 
        #print("last reward: "+repr(last_reward))

        # TODO tune this
        reach_goal_threshold = car_size * 4
        if distance < reach_goal_threshold:
            print("reach goal in " + repr(self.steps) + ", increase by " + repr(self.last_steps - self.steps))
            goal_x = self.width-goal_x
            goal_y = self.height-goal_y
            #last_reward = self.last_steps - self.steps # reward for reaching the objective faster than last round (TODO may want to scale this)
            # TODO is it appropriate to punish last step by mistakes we made from all the previous steps?
            #last_reward += (self.last_steps - self.steps) #* step_reward # reward for reaching the objective faster than last round
            self.last_steps = self.steps 
            print("score: "+repr(brain.score())+ " steps: " + repr(self.steps))
            self.steps = 0
            brain.clear_score()
        last_action = action
        last_distance = distance


# Adding the painting tools
min_obstacle_density = 1
default_obstacle_density = 10
max_obstacle_density = 16
obstacle_density_threshold = 1 # pixel density
sand_line_width = 20
class MyPaintWidget(Widget):
    
    def load_map(self, mp):
        print("todo, load map here")
        #TODO load mp

    def on_touch_down(self, touch):
        global length, n_points, last_x, last_y, min_obstacle_density, max_obstacle_density, default_obstacle_density 
        with self.canvas:
            if touch.button == 'left':
                Color(0.8,0.7,0)
            else:
                Color(0,0.8,0.7)
            d = 10.
            touch.ud['line'] = Line(points = (touch.x, touch.y), width = default_obstacle_density)
            last_x = int(touch.x)
            last_y = int(touch.y)
            n_points = 0
            length = 0
            #print("draw sand")
            if touch.button == 'left':
                sand[int(touch.x),int(touch.y)] = default_obstacle_density 
            else:
                sand[int(touch.x),int(touch.y)] = -default_obstacle_density 

    def on_touch_move(self, touch):
        global length, n_points, last_x, last_y, min_obstacle_density, max_obstacle_density 
        touch.ud['line'].points += [touch.x, touch.y]
        x = int(touch.x)
        y = int(touch.y)
        length += np.sqrt(max((x - last_x)**2 + (y - last_y)**2, 2))
        n_points += 1.
        avg_points = n_points/(length)
        density = int(20 * avg_points + 1)
        touch.ud['line'].width = density
        w = sand_line_width / 2
        pixel_depth = max_obstacle_density 
        if density < pixel_depth:
            pixel_depth = density
        #print("draw sand")
        if touch.button == 'left':
            #print("right mouse clicked")
            sand[int(touch.x) - w : int(touch.x) + w, int(touch.y) - w : int(touch.y) + w] = pixel_depth 
        else:
            sand[int(touch.x) - w : int(touch.x) + w, int(touch.y) - w : int(touch.y) + w] = -pixel_depth 
        last_x = x
        last_y = y

# Adding the API Buttons (clear, save and load)

class CarApp(App):
    simulate_map = True
    def __init__(self):
        self.painter = MyPaintWidget()
    def load_map(self, mp):
        self.simulate_map = False
        self.painter.load_map(mp)    
        # TODO load map from path mp
    def build(self):
        parent = Game()
        parent.serve_car()
        if self.simulate_map:
            Clock.schedule_interval(parent.update, 1.0/150.0)
        clearbtn = Button(text = 'clear')
        savebtn = Button(text = 'save', pos = (parent.width, 0))
        loadbtn = Button(text = 'load', pos = (2 * parent.width, 0))
        clearbtn.bind(on_release = self.clear_canvas)
        savebtn.bind(on_release = self.save)
        loadbtn.bind(on_release = self.load)
        parent.add_widget(self.painter)
        parent.add_widget(clearbtn)
        parent.add_widget(savebtn)
        parent.add_widget(loadbtn)
        return parent

    def clear_canvas(self, obj):
        global sand
        self.painter.canvas.clear()
        sand = np.zeros((longueur,largeur))

    def save(self, obj):
        print("saving brain...")
        brain.save()
        plt.plot(scores)
        plt.show()

    def load(self, obj):
        print("loading last saved brain...")
        brain.load()

# Running the whole thing
if __name__ == '__main__':
    CarApp().run()
