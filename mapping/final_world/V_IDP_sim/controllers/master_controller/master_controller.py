#!/home/ben/anaconda3/bin/python3.8
from controller import Robot
from controller import Emitter
from controller import Receiver
import numpy as np
import struct
import matplotlib.pyplot as plt

TIME_STEP = 64

robot = Robot()

receiver = robot.getDevice('receiver')

receiver.enable(TIME_STEP)

def log_odds(map):
	return np.log(np.divide(map, np.subtract(1, map)))

def inv_lodds(lmap):
	#  1 - 1/(1+np.exp(l))
	return np.subtract(1, np.divide(1, np.add(1, np.exp(lmap))))

def bound(coord, a=0, b=79):
	k = min(coord, b)
	k = max(k, a)
	return k

def pad_grid(occupancy_grid, iterations):

	for i in range(iterations):
		obstacle_coords = np.where(occupancy_grid == 1)
		# print(len(obstacle_coords[0]))
		for i in range(len(obstacle_coords[0])):
			occupancy_grid[bound(obstacle_coords[0][i] + 1)][bound(obstacle_coords[1][i])] = 1
			occupancy_grid[bound(obstacle_coords[0][i] - 1)][bound(obstacle_coords[1][i])] = 1
			occupancy_grid[bound(obstacle_coords[0][i])][bound(obstacle_coords[1][i] + 1)] = 1
			occupancy_grid[bound(obstacle_coords[0][i])][bound(obstacle_coords[1][i] - 1)] = 1
	
	return occupancy_grid

while robot.step(TIME_STEP) != -1:
	
	queue_length = receiver.getQueueLength()
	#print(queue_length)

	if queue_length == 2:
		message=receiver.getData()
		dataList=struct.unpack("51200s",message)
		data_string = dataList[0]
		occupancy_grid_1 = np.frombuffer(data_string)
		#print(occupancy_grid_1)
		# plt.imshow(occupancy_grid_1.reshape(80,80))
		# plt.show()
		receiver.nextPacket()
		message=receiver.getData()
		dataList=struct.unpack("51200s",message)
		data_string = dataList[0]
		occupancy_grid_2 = np.frombuffer(data_string)

		final_grid = inv_lodds(np.add(log_odds(occupancy_grid_1),log_odds(occupancy_grid_2)))
		final_grid[final_grid > 0.6] = 1
		final_grid[final_grid <= 0.6] = 0
		final_grid = final_grid.reshape(80,80)
		#final_grid = pad_grid(final_grid, )

		plt.imshow(final_grid, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")
		plt.show()




