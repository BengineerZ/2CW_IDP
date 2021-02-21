#!/home/ben/anaconda3/bin/python3.8
from controller import Robot
from controller import Emitter
from controller import Receiver
import numpy as np
import struct
import matplotlib.pyplot as plt
from skimage.draw import line, rectangle, rectangle_perimeter, ellipse, polygon
from skimage.feature import blob_dog
from skimage.filters import gaussian
from pathfinder import findpath, h
from math import hypot
import scipy.interpolate as si
from scipy.signal import find_peaks
from scipy.stats import entropy
from robot_manager import *
from environment_manager import *
from utils import visualiser
from state_manager import robot_state_manager

### master controller initialisation:
TIME_STEP = 64
master_robot = Robot()
receiver = master_robot.getDevice('receiver')
emitter = master_robot.getDevice('emitter')
receiver.enable(TIME_STEP)
emitter.setChannel(Emitter.CHANNEL_BROADCAST)
receiver.setChannel(Receiver.CHANNEL_BROADCAST)

# Robot, robot state and environment manager objects
red_bot = robot_manager(0, emitter)
blue_bot = robot_manager(1, emitter)
environment = environment_manager()
red_bot_state_manager = robot_state_manager(master_robot)
blue_bot_state_manager = robot_state_manager(master_robot)

# Visualiser objects
visualiser1 = visualiser(2, ["Occupancy Grid","Block Grid"], k=1)
visualiser2 = visualiser(2, ["Blue bot","Red bot"], k=2)
red_visualiser = np.full((80,80), 0)
blue_visualiser = np.full((80,80), 0)

# Simulation step counter
n = 0

#### MAIN LOOP ####
while master_robot.step(TIME_STEP) != -1:
	# Comms
	initiate_data_transfer(emitter)
	robot_data, update = await_state_data(receiver)

	# If received data is good
	if len(robot_data) == 2:
		red_bot.robot_data = robot_data
		blue_bot.robot_data = robot_data

		# Debugging output
		print("====")
		print("Blue bot current task: ", blue_bot_state_manager.current_task)
		print("Blue bot state: ", blue_bot.state)
		#print("Blue Current target: ", blue_current_target)
		print("Red bot current task: ", red_bot_state_manager.current_task)
		print("Red bot state: ", red_bot.state)
		#print("Red Current target: ", red_current_target)
		print("Blocks found: \n", environment.blocks)

		driving_grid, reduced_driving_grid = environment.driving_grid()
		blue_bot.update_driveable_area(driving_grid, reduced_driving_grid)
		red_bot.update_driveable_area(driving_grid, reduced_driving_grid)

		obver, _ = environment.find_blocks()
		environment.update_binary_occupancy_grid(robot_data)

		# Update manager objects
		environment()
		red_bot(environment)
		blue_bot(environment)

		# red_bot.set_state("sweep", start= -np.pi/2 +0.5, end= np.pi - 0.5, speed= 0.5, n= 2)
		# blue_bot.set_state("sweep", start= -np.pi/2 +0.5, end= np.pi - 0.5, speed= 0.5, n= 2)

		red_bot_state_manager.update_state(environment, red_bot)
		blue_bot_state_manager.update_state(environment, blue_bot)

		### visualisation:
		red_visualiser = np.copy(red_bot.driving_grid)
		for i in range(len(red_bot.current_path)):
			red_visualiser[int(red_bot.current_path[i][0]), int(red_bot.current_path[i][1])] = 0.5

		blue_visualiser = np.copy(blue_bot.driving_grid)
		for i in range(len(blue_bot.current_path)):
			blue_visualiser[int(blue_bot.current_path[i][0]), int(blue_bot.current_path[i][1])] = 0.5

		#point = environment.explore_uncertainty()

		#rr, cc = ellipse(point[0] , point[1], 2,2)

		#red_visualiser[rr, cc] = 0.5

		visualiser1(environment.occupancy_grid, obver)
		visualiser2(blue_visualiser, red_visualiser)

		# Step counter
		n = n + 1

