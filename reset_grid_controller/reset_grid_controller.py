#!/home/ben/anaconda3/bin/python3.8
from controller import Robot
from controller import Emitter
from controller import Receiver
from controller import Supervisor
import numpy as np
import struct
import matplotlib.pyplot as plt
import random

from sac_torch import Agent
from utils import plot_learning_curve

TIME_STEP = 64

#robot = Robot()
supervisor = Supervisor()

block_nodes = []
for i in range(4):
	block_node = supervisor.getFromDef('Block_R' + str(int(i+1)))
	block_nodes.append(block_node)
for i in range(4):
	block_node = supervisor.getFromDef('Block_B' + str(int(i+1)))
	block_nodes.append(block_node)

robot_nodes = [supervisor.getFromDef('bot_red'), supervisor.getFromDef('bot_blue')]

def reset_block_position(block_nodes):

	positions = []

	def get_new_random():
		return [np.random.uniform(-1, 1)*1.13, 0.025, np.random.uniform(-1, 1)*1.13]

	for i in range(len(block_nodes)):

		#position_vect = get_new_random()
		while True:

			position_vect = get_new_random()
			#print(position_vect)
			if (position_vect[0] > 0.75 and position_vect[2] > 0.75) or (position_vect[0] > 0.75 and position_vect[2] < -0.75):
				#position_vect = get_new_random()
				print('region overlap')
			else:
				if len(positions) > 0:
					dist = []
					for j in range(len(positions)):
						dist.append(np.linalg.norm(np.array(position_vect) - positions[j]))

					if min(dist) > 0.08:
						positions.append(position_vect)
						break
					else:
						position_vect = get_new_random()
						print('block overlap')
				else:
					positions.append(position_vect)
					break

		trans = block_nodes[i].getField('translation')
		trans.setSFVec3f(position_vect)
		block_nodes[i].resetPhysics()

	return positions

def reset_robots(robot_nodes):

	bot_position = [[0.98, 0.04, 0.95], [0.98, 0.04, -0.95]]
	bot_orientation = [[0,1,0, np.pi],[0,1,0, 0]]

	for i in range(2):
		trans = robot_nodes[i].getField('translation')
		trans.setSFVec3f(bot_position[i])

		orient = robot_nodes[i].getField('rotation')
		orient.setSFRotation(bot_orientation[i])

		robot_nodes[i].resetPhysics()

	return robot_nodes[0].getPosition(), robot_nodes[1].getPosition()

reset_robots(robot_nodes)
reset_block_position(block_nodes)

while supervisor.step(TIME_STEP) != -1:
	pass