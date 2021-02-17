#!/home/ben/anaconda3/bin/python3.8
import numpy as np
from skimage.draw import line, rectangle, rectangle_perimeter, ellipse, polygon
from skimage.feature import blob_dog
from skimage.filters import gaussian
from utils import *

### recursive environment defs - used for generating the occupancy grid
def log_odds_ratio(p):
	return np.log(p/(1-p))

def inv_log_odds_ratio(l):
	return 1 - 1/(1+np.exp(l))

def update_occupied(grid, coord, p = 0.6):
	return inv_log_odds_ratio(log_odds_ratio(grid[coord[0]][coord[1]]) + log_odds_ratio(p))

def update_free(grid, coord, position, p=0.4):
	rr, cc = line(position[0], position[1], coord[0], coord[1])
	updates = []
	for i in range(len(rr)-1):
		updates.append(inv_log_odds_ratio(log_odds_ratio(grid[rr[i], cc[i]]) + log_odds_ratio(p)))

	return rr[:-1], cc[:-1], np.array(updates)

def block_padding(blocks, occupancy_grid, size=9):
	for i in range(len(blocks)):
		rr, cc = ellipse(int(blocks[i][0]) , int(blocks[i][1]), size, size)
		for i in range(len(rr)):
			occupancy_grid[bound(rr[i]), bound(cc[i])] = 1

	return occupancy_grid

def pad_grid(occupancy_grid, iterations):

	occupancy_grid = gaussian(occupancy_grid, sigma = 2.5)
	occupancy_grid[occupancy_grid > 0.04] = 1
	occupancy_grid[occupancy_grid <= 0.04] = 0
	
	return occupancy_grid


### main environment_manager class:
class environment_manager:

	def __init__(self):
		print('initialise environment')
		self.occupancy_grid = np.full((80,80), 0.5)
		self.detected_blocks = []
		self.blocks = np.array([])
		self.total_blocks_detected = 0
		self.total_blocks_removed = 0
		self.red_dropped_off = 0
		self.blue_dropped_off = 0

		self._extent = np.array([[6, -4],[-7, -4],[-7, 4],[6, 3]])

	def __call__(self):
		#print('occupancy_grid update')

		self.combine_coord_sets()
		self.occupancy_grid = np.clip(self.occupancy_grid, 0.08, 0.995)
		self.total_blocks_detected = len(self.blocks) + self.total_blocks_removed


	def update_binary_occupancy_grid(self, robot_data):
		max_range = 1.3
		for i in [0,1]:
			test_position = robot_data[i][1:4]
			psValues = robot_data[i][4:6]
		
			local_coordinate1 = [-psValues[0]*0.707,psValues[0]*0.707]
			local_coordinate2 = [psValues[1]*0.707,psValues[1]*0.707]

			coord1 = np.rint((transform_local_coords(test_position, local_coordinate1)*[100, -100] + np.array([119, 119]))/3)
			coord2 = np.rint((transform_local_coords(test_position, local_coordinate2)*[100, -100] + np.array([119, 119]))/3)

			pos = convert_to_grid_space(robot_data[(i+1)%2][1:3])
			rad = robot_data[(i+1)%2][3]
			grid_robot_state = np.array([pos[0], pos[1], rad])
			robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
			for i in range(len(self._extent)):
				robot_extent_global[i] = np.rint((transform_local_coords(grid_robot_state, self._extent[i])))
			robot_extent_global = np.transpose(robot_extent_global)
			rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
			check_coords = np.array([rr, cc]).T

			if np.array([bound(int(coord1[1]),bound(int(coord1[0])))]) not in check_coords and psValues[0] < max_range:
				self.occupancy_grid[bound(int(coord1[1]))][bound(int(coord1[0]))] = update_occupied(self.occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], p=(0.95 - 0.14*psValues[0]))
			if np.array([bound(int(coord2[1]),bound(int(coord2[0])))]) not in check_coords and psValues[1] < max_range:
				self.occupancy_grid[bound(int(coord2[1]))][bound(int(coord2[0]))] = update_occupied(self.occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], p=(0.95 - 0.14*psValues[1]))

			current_pos = np.rint((np.array([test_position[0], test_position[1]])*[100, -100] + np.array([119, 119]))/3)

			rr1, cc1, updates1 = update_free(self.occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.4)
			self.occupancy_grid[rr1, cc1] = updates1

			rr2, cc2, updates2 = update_free(self.occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.4)
			self.occupancy_grid[rr2, cc2] = updates2

	def find_blocks(self):

		final_grid = gaussian(np.copy(self.occupancy_grid), sigma = 0.2)

		final_grid[final_grid > 0.95] = 1
		final_grid[final_grid <= 0.95] = 0

		rr, cc = rectangle_perimeter((1,1), (78,78), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		rr, cc = rectangle((1,1), (13,13), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		rr, cc = rectangle((1,65), (13,78), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		final_grid = gaussian(final_grid, sigma = 0.4)

		self.detected_blocks = blob_dog((final_grid),min_sigma=1.5, max_sigma=2.3, threshold=0.01, overlap = 0.4)
		
		return final_grid, self.detected_blocks

	def driving_grid(self):

		final_grid = np.copy(self.occupancy_grid)
	
		final_grid[final_grid > 0.4] = 1
		final_grid[final_grid <= 0.4] = 0

		rr, cc = rectangle_perimeter((2,2), (77,77), shape=final_grid.shape)
		final_grid[rr, cc] = 1

		final_grid = pad_grid(final_grid, 1)

		rr, cc = rectangle((4,4), (13,13), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		rr, cc = rectangle((4,65), (13,74), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		reduced_grid = np.copy(final_grid)
		reduced_grid = block_padding(self.blocks, reduced_grid, size=5)

		final_grid = block_padding(self.blocks, final_grid)

		# rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 9,9, shape=final_grid.shape)
		# for i in range(len(rr)):
		#   final_grid[bound(rr[i]), bound(cc[i])] = 1

		return final_grid, reduced_grid

	def remove_block(self, target):

		combined_set = np.copy(self.blocks)
		dist = []
		for j in range(len(self.blocks)):
			dist.append(np.linalg.norm(target - self.blocks[j][0:2]))

		if len(dist) > 0:
			if min(dist) <= 4:
				#combined_set[dist.index(min(dist))] = coords_col
				combined_set = np.delete(combined_set, dist.index(min(dist)), axis = 0)
				self.blocks = combined_set
				self.total_blocks_removed += 1

				rr, cc = ellipse(target[0] , target[1], 4,4)
				for i in range(len(rr)):
					self.occupancy_grid[bound(rr[i]), bound(cc[i])] = 0.2


	def combine_coord_sets(self):
		# allows for order preservation as well as minor deviations in block position
		new_blocks = np.copy(self.detected_blocks)
		combined_set = np.copy(self.blocks)
		for i in range(len(new_blocks)):
			dist = []
			for j in range(len(self.blocks)):
				dist.append(np.linalg.norm(new_blocks[i][0:2] - self.blocks[j][0:2]))

			if len(dist) > 0:
				if min(dist) >  4:
					combined_set = np.concatenate((combined_set, [np.append(new_blocks[i][0:2], 0)]), axis=0)
				elif min(dist) <= 4:
					combined_set[dist.index(min(dist))][0:2] = new_blocks[i][0:2]

		if len(self.blocks) == 0:
			#print('intialise block set')
			combined_set = np.concatenate((new_blocks[:,0:2], np.zeros( (len(new_blocks), 1), dtype=int ) ), axis=1)

		self.blocks = combined_set

	def update_block(self, block_state):
		# block state - coord, colour, pickup
		coord = block_state[0]
		colour = block_state[1]
		pickup = block_state[2]

		print('updating block state')
		print(coord)
		print(colour)

		if (coord[0] <= 1 or coord[0] >= 78 or coord[1] <= 1 or coord[1] >= 78) and colour == 0:
			self.remove_block(coord)
			return
		else:
			pass

		if pickup == True:
			self.remove_block(coord)
		else:
			coords_col = np.array([coord[0], coord[1], colour])
			combined_set = np.copy(self.blocks)
			dist = []
			for j in range(len(self.blocks)):
				dist.append(np.linalg.norm(coords_col[0:2] - self.blocks[j][0:2]))

			if len(dist) >0:
				if min(dist) <= 4:
					combined_set[dist.index(min(dist))] = coords_col
					self.blocks = combined_set

	def explore_uncertainty(self):
		flattened = np.copy(self.occupancy_grid)
		flattened = flattened.flatten()
		avg_uncertainty = np.average(flattened*np.log2(1/flattened))

		uncertainty_grid = self.occupancy_grid*np.log2(1/self.occupancy_grid)
		uncertainty_grid[uncertainty_grid > 0.3] = 1
		uncertainty_grid[uncertainty_grid <= 0.3] = 0

		# detect on uncertainty grid
		rr, cc = rectangle_perimeter((0,0), (79,79), shape=uncertainty_grid.shape)
		uncertainty_grid[rr, cc] = 0

		rr, cc = rectangle((0,0), (13,13), shape=uncertainty_grid.shape)
		uncertainty_grid[rr, cc] = 0

		rr, cc = rectangle((0,65), (13,79), shape=uncertainty_grid.shape)
		uncertainty_grid[rr, cc] = 0

		uncertainty_grid = gaussian(uncertainty_grid, sigma = 0.8)

		coords = np.where(uncertainty_grid > 0.2)

		if len(coords) > 0:
			explore_coordinate = np.average(coords, weights = uncertainty_grid[coords] ,axis = 1)
			return explore_coordinate
		else:
			return None



	def optimum_heading(self, target, additional_coords, display=False):

		flattened = np.copy(self.occupancy_grid)

		flattened[flattened > 0.8] = 1
		flattened[flattened <= 0.8] = 0

		rr, cc = rectangle_perimeter((1,1), (78,78), shape=flattened.shape)
		flattened[rr, cc] = 0

		rr, cc = rectangle((0,0), (13,13), shape=flattened.shape)
		flattened[rr, cc] = 0

		rr, cc = rectangle((0,65), (13,79), shape=flattened.shape)
		flattened[rr, cc] = 0

		# coords = np.where(flattened > 0.8)

		# rr, cc = ellipse(target[0][0] , target[1][0], 4,4)
		# #print(np.array(coords).shape)
		# for i in range(len(rr)):
		# 	k1 = np.where(coords[0] == rr[i], True, False)
		# 	k2 = np.where(coords[1] == cc[i], True, False)
		# 	inter = np.logical_not(np.logical_and(k1, k2))
		# 	#print(inter)
		# 	coords = np.compress(inter, coords, axis = 1)

		coords = []
		rr, cc = ellipse(target[0] , target[1], 4,4)

		for i in range(len(self.blocks)):
			if np.linalg.norm(target[0:2] - self.blocks[i][0:2]) > 4:
				coords.append([self.blocks[i][0], self.blocks[i][1]])
		#coords.append(additional_coords)
		coords = np.array(coords)

		

		# vectors = []
		# for i in range(len(self.blocks)):
		# 	vectors.append([target[0] - self.blocks[i][0], target[1] - self.blocks[i][1]])

		# vectors.append([target[0], 0])
		# vectors.append([0, target[1]])
		# vectors.append([79 - target[0], 0])
		# vectors.append([0, 79 - target[1]])

		if len(coords) > 0:
			vectors = np.subtract(target, coords)
			vectors = np.concatenate((vectors, np.array([[target[0], 0]])))
		else:
			vectors = np.array([[target[0], 0]])
		vectors = np.concatenate((vectors, np.array([[0, target[1]]])))
		vectors = np.concatenate((vectors, np.array([[-79 + target[0], 0]])))
		vectors = np.concatenate((vectors, np.array([[0, -79 + target[1]]])))
		
		
		#print(vectors)

		#distances = np.sqrt(np.add(np.power(vectors[0,:], 2), np.power(vectors[1,:], 2)))
		distances = np.linalg.norm(vectors, axis = 1)
		#print(distances)
		w = 200
		distance_weighting = 1 - np.tanh(np.divide(np.power(distances, 2), w))
		#probability_weighting = (np.exp(np.power(self.occupancy_grid[coords[0], coords[1]], 6)) - 1)/1.718
		probability_weighting = 100
		overall_weighting = distance_weighting*probability_weighting + 0.0000001
		overall_weighting = np.expand_dims(overall_weighting, axis = 1)
		#print(overall_weighting)
		normed = []
		for i in range(len(vectors)):
			norm = vectors[i]/np.linalg.norm(vectors[i])
			normed.append(norm)
		normed = np.array(normed)
		optimum_heading = np.sum(normed*overall_weighting, axis=0)/(len(overall_weighting))

		scaling_factor = 8
		normed_optimum_heading = scaling_factor*optimum_heading/np.linalg.norm(optimum_heading)



		#print(optimum_heading)
		optimum_target = np.array([target[0] + normed_optimum_heading[0] + 0.00001, target[1] + normed_optimum_heading[1] + 0.00001])
		#print(optimum_target)

		
		#flattened[rr,cc] = 0.5

		#flattened[int(optimum_target[0]), int(optimum_target[1])] = 0.8

		#print('target_danger: ' + str())

		target_danger = np.linalg.norm(optimum_heading)
		if np.isnan(target_danger):
			target_danger = 0

		#print(target_danger)

		test = flattened

		if display == False:
			return optimum_target, target_danger
		else:
			return optimum_target, target_danger, flattened