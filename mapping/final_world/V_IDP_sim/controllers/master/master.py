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

red_bot = robot_manager(0, emitter)
blue_bot = robot_manager(1, emitter)

environment = environment_manager()

visualiser = visualiser(3)

red_bot_state_manager = robot_state_manager(master_robot)
blue_bot_state_manager = robot_state_manager(master_robot)


red_visualiser = np.full((80,80), 0)
blue_visualiser = np.full((80,80), 0)

n = 0
while master_robot.step(TIME_STEP) != -1:

	initiate_data_transfer(emitter)
	robot_data, update = await_state_data(receiver)

	if len(robot_data) == 2:
		red_bot.robot_data = robot_data
		blue_bot.robot_data = robot_data

		# if n == 10:
		# 	### main loop 
		# 	#blue_bot.set_state('go_to_target', target = (40,40), obstacle_grid = test, early_stop = 5, grip = 0)
		# 	blue_bot.set_state('sweep', start = np.pi/2 - 0.5, end = -np.pi +0.5, speed = 0.5, n=2)
		# 	red_bot.set_state('sweep', start = -np.pi/2 + 0.5, end = np.pi - 0.5, speed = 0.5, n=2)

		# if n == 200:
		# 	target = red_bot_state_manager.optimum_block(red_bot, environment)
		# 	red_bot.set_state('go_to_target', target = (int(target[0]), int(target[1])), early_stop = 5, grip = 0, block= True, empty = False, away = 1, optimise_heading =  False, speed=1.5)

		# if n == 500:
		# 	red_bot.set_state('block_extent_routine', target = (int(target[0]), int(target[1])), env=environment)

		# if n == 800:
		# 	red_bot.set_state("block_update_routine", grip=1, env=environment)
		# 	red_bot.set_state('go_to_target', target = (int(target[0]), int(target[1])), early_stop = 5, grip = 0, block= True, empty = True, away = -1, optimise_heading =  False, speed=1)

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

		_, _ = environment.find_blocks()
		environment.update_binary_occupancy_grid(robot_data)

		environment()
		red_bot(environment)
		blue_bot(environment)



		red_bot_state_manager.update_state(environment, red_bot)
		blue_bot_state_manager.update_state(environment, blue_bot)



		### visualisation:

		red_visualiser = np.copy(red_bot.driving_grid)
		for i in range(len(red_bot.current_path)):
			red_visualiser[int(red_bot.current_path[i][0]), int(red_bot.current_path[i][1])] = 0.5

		blue_visualiser = np.copy(blue_bot.driving_grid)
		for i in range(len(blue_bot.current_path)):
			blue_visualiser[int(blue_bot.current_path[i][0]), int(blue_bot.current_path[i][1])] = 0.5

		visualiser(environment.occupancy_grid, blue_visualiser, red_visualiser)

		n = n + 1

