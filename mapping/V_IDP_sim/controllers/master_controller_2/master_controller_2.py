#!/home/ben/anaconda3/bin/python3.8
from controller import Robot
from controller import Emitter
from controller import Receiver
import numpy as np
import struct
import matplotlib.pyplot as plt
from skimage.draw import line, rectangle, rectangle_perimeter, ellipse
from skimage.feature import blob_dog
from skimage.filters import gaussian
from pathfinder import findpath
import scipy.interpolate as si

TIME_STEP = 64

robot = Robot()

receiver = robot.getDevice('receiver')
emitter = robot.getDevice('emitter')
receiver.enable(TIME_STEP)

emitter.setChannel(Emitter.CHANNEL_BROADCAST)
receiver.setChannel(Receiver.CHANNEL_BROADCAST)

update = False
### useful defs

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

	# for i in range(iterations):
	# 	obstacle_coords = np.where(occupancy_grid == 1)
	# 	# print(len(obstacle_coords[0]))
	# 	for i in range(len(obstacle_coords[0])):
	# 		occupancy_grid[bound(obstacle_coords[0][i] + 1)][bound(obstacle_coords[1][i])] = 1
	# 		occupancy_grid[bound(obstacle_coords[0][i] - 1)][bound(obstacle_coords[1][i])] = 1
	# 		occupancy_grid[bound(obstacle_coords[0][i])][bound(obstacle_coords[1][i] + 1)] = 1
	# 		occupancy_grid[bound(obstacle_coords[0][i])][bound(obstacle_coords[1][i] - 1)] = 1

	occupancy_grid = gaussian(occupancy_grid, sigma = 1)
	occupancy_grid[occupancy_grid > 0.1] = 1
	occupancy_grid[occupancy_grid <= 0.1] = 0
	
	return occupancy_grid

def initiate_data_transfer():
	message = struct.pack('i', 1)
	emitter.send(message)
	#print('initiate')

def set_robot_state(robot_id, wheel_v, gripper):
	message = struct.pack('iddi', robot_id, wheel_v[0], wheel_v[1],  gripper)
	emitter.send(message)

def await_state_data():

	#initiate_data_transfer()

	queue_length = receiver.getQueueLength()

	if queue_length == 0:
		print('...')

	state_update = False
	robot_data = []
	while queue_length > 0:
		message=receiver.getData()
		dataList=struct.unpack("idddddi",message)
		robot_data.append(dataList)
		#print('recieved state from: ' + str(dataList[0]))
		receiver.nextPacket()
		queue_length = receiver.getQueueLength()
		state_update = True
		pass

	
	return robot_data, state_update

def transform_local_coords(robot_position, local_coordinate):
	# robot_position = [x, y, theta]
	# local coordinate = [x', y']

	rot_matrix = np.array([[np.cos(robot_position[2]), np.sin(robot_position[2])], [-np.sin(robot_position[2]), np.cos(robot_position[2])]])
	offset_vector = np.array([robot_position[0],robot_position[1]])
	
	return np.matmul(rot_matrix, local_coordinate) + offset_vector

def log_odds_ratio(p):
	return np.log(p/(1-p))

def inv_log_odds_ratio(l):
	return 1 - 1/(1+np.exp(l))

def update_occupied(map, coord, p = 0.6):
	return inv_log_odds_ratio(log_odds_ratio(map[coord[0]][coord[1]]) + log_odds_ratio(p))

def update_free(map, coord, position, p=0.4):
	rr, cc = line(position[0], position[1], coord[0], coord[1])
	updates = []
	for i in range(len(rr)-1):
		updates.append(inv_log_odds_ratio(log_odds_ratio(occupancy_grid[rr[i], cc[i]]) + log_odds_ratio(p)))

	return rr[:-1], cc[:-1], np.array(updates)

def convert_to_grid_coords(coord):
	return np.flip(np.rint((np.array(coord)*[100, -100] + np.array([119, 119]))/3))

def convert_to_grid_space(coord):
	return np.flip((np.array(coord)*[100, -100] + np.array([119, 119]))/3)

def update_binary_occupancy_grid(occupancy_grid, robot_data):
	for i in [0,1]:
		test_position = robot_data[i][1:4]
		psValues = robot_data[i][4:6]
	
		local_coordinate1 = [-psValues[0]*0.707,psValues[0]*0.707]
		local_coordinate2 = [psValues[1]*0.707,psValues[1]*0.707]

		coord1 = np.rint((transform_local_coords(test_position, local_coordinate1)*[100, -100] + np.array([119, 119]))/3)
		coord2 = np.rint((transform_local_coords(test_position, local_coordinate2)*[100, -100] + np.array([119, 119]))/3)

		#print(coord1)
		#occupancy_grid[bound(int(coord1[1]))][bound(int(coord1[0]))] = 1
		#occupancy_grid[bound(int(coord2[1]))][bound(int(coord2[0]))] = 1

		occupancy_grid[bound(int(coord1[1]))][bound(int(coord1[0]))] = update_occupied(occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], p=(0.95 - 0.14*psValues[0]))
		occupancy_grid[bound(int(coord2[1]))][bound(int(coord2[0]))] = update_occupied(occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], p=(0.95 - 0.14*psValues[1]))


		current_pos = np.rint((np.array([test_position[0], test_position[1]])*[100, -100] + np.array([119, 119]))/3)

		rr1, cc1, updates1 = update_free(occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.4)
		occupancy_grid[rr1, cc1] = updates1

		rr2, cc2, updates2 = update_free(occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.4)
		occupancy_grid[rr2, cc2] = updates2

		# centre = convert_to_grid_coords(robot_data[0][1:3])
		# rr, cc = ellipse(centre[0], centre[1], 3, 3)
		# occupancy_grid[rr, cc] = 0.2

	return occupancy_grid

def compute_uncertainty_grid(occupancy_grid):
	uncertainty_grid = 16 * occupancy_grid ** 2 * (1 - occupancy_grid) **2
	overall_uncertainty = np.average(uncertainty_grid)
	return uncertainty_grid, overall_uncertainty


def find_blocks(occupancy_grid):
	final_grid = gaussian(occupancy_grid, sigma = 0.2)

	final_grid[final_grid > 0.99] = 1
	final_grid[final_grid <= 0.99] = 0

	rr, cc = rectangle_perimeter((1,1), (78,78), shape=final_grid.shape)
	final_grid[rr, cc] = 0

	rr, cc = rectangle((1,1), (13,13), shape=final_grid.shape)
	final_grid[rr, cc] = 0

	rr, cc = rectangle((1,65), (13,78), shape=final_grid.shape)
	final_grid[rr, cc] = 0

	final_grid = gaussian(final_grid, sigma = 0.4)

	blobs_dog= blob_dog((final_grid),min_sigma=1.5, max_sigma=2.3, threshold=0.01, overlap = 0.4)
	
	return final_grid, blobs_dog

def robot_motion(state, target, pd, pc):

	current_orientation = np.array([-np.cos(state[2]), 0, -np.sin(state[2])])
	current_position = convert_to_grid_space(state[0:2])
	
	heading_vector = np.subtract(target,current_position)
	heading_vector = np.array([heading_vector[0], 0, -heading_vector[1]])

	dot = current_orientation.dot(heading_vector)
	cross = -np.cross(current_orientation, heading_vector)[1]

	leftSpeed = pd*dot + pc*cross
	rightSpeed = pd*dot - pc*cross

	return leftSpeed, rightSpeed

def block_padding(blocks, occupancy_grid):
	for i in range(len(blocks)):
		rr, cc = ellipse(int(blocks[i][0]) , int(blocks[i][1]), 9, 9)
		occupancy_grid[rr, cc] = 1

	return occupancy_grid


import scipy.interpolate as si


def bspline(cv, n=100, degree=3, periodic=False):
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
                  False - Curve is open
    """

    # If periodic, extend the point array by count+degree+1
    cv = np.asarray(cv)
    count = len(cv)
    if periodic:
        factor, fraction = divmod(count+degree+1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree,1,degree)
    # If opened, prevent degree from exceeding count-1
    else:
        degree = np.clip(degree,1,count-1)
    # Calculate knot vector
    kv = None
    if periodic:
        kv = np.arange(0-degree,count+degree+degree-1)
    else:
        kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)
    # Calculate query range
    u = np.linspace(periodic,(count-degree),n)
    # Calculate result
    return np.array(si.splev(u, (kv,cv.T,degree))).T

class robot_manager:
	def __init__(self):
		self.data = None

	def __call__(self):
		initiate_data_transfer()
		self.data , update = await_state_data()




### occupancy defs

occupancy_grid = np.full((80,80), 0.5)
final_grid = np.full((80,80), 0.5)
uncertainty_graph = []

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
img = ax1.imshow(occupancy_grid, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")
fig.canvas.draw()   # note that the first draw comes before setting data 
# cache the background
axbackground = fig.canvas.copy_from_bbox(ax1.bbox)



plt.show(block=False)




n=0
spin0 = True
spin1 = True
path0 = []
path1 = []
blocks = []
overall_uncertainty = 1
block_i = 0

while robot.step(TIME_STEP) != -1:

	
	initiate_data_transfer()
	
	robot_data, update = await_state_data()
	if len(robot_data) == 2:
		#print('id: ' + str(robot_data[0][0]) + ': angle : ' + str(robot_data[0][3]))


		# red robot spin
		if robot_data[0][3] > -np.pi/2 +0.5 and robot_data[0][3] < 0 - 0.5 and not(spin0):
			spin0 = not(spin0)
		elif robot_data[0][3] < np.pi - 0.5 and robot_data[0][3] > np.pi/2 - 0.5 and spin0:
			spin0 = not(spin0)

		if spin0:
			set_robot_state(0, [-0.5, 0.5], 0)
		else:
			set_robot_state(0, [0.5, -0.5], 0)

		# blue robot spin
		if robot_data[1][3] < np.pi/2 - 0.5 and robot_data[1][3] > 0 - 0.5 and not(spin1):
			spin1 = not(spin1)
		elif robot_data[1][3] > -np.pi + 0.5 and robot_data[1][3] < -np.pi/2 + 0.5 and spin1:
			spin1 = not(spin1)

		if spin1:
			set_robot_state(1, [0.5, -0.5], 0)
		else:
			set_robot_state(1, [-0.5, 0.5], 0)

		# if n > 400:
		# 	set_robot_state(1, [0, 0], 0)


		# 	fig2 = plt.figure()
		# 	ax2 = fig2.add_subplot(1, 1, 1)
		# 	graph = ax2.plot(uncertainty_graph)
		# 	fig2.canvas.draw()
		# 	plt.show(block=True)

		
			
			

			

		# position0 = robot_data[0][1:4]
		# current_pos0 = np.rint((np.array([position0[0], position0[1]])*[100, -100] + np.array([119, 119]))/3)
		# current_pos0 = (int(current_pos0[1]), int(current_pos0[0]))
		
		# final_pos = (40,40)
		# if n%20 == 1:
		# 	sgrid0, path0 = findpath(current_pos0, final_pos, final_grid)
		

		# position1 = robot_data[1][1:4]
		# current_pos1 = np.rint((np.array([position1[0], position1[1]])*[100, -100] + np.array([119, 119]))/3)
		# current_pos1 = (int(current_pos1[1]), int(current_pos1[0]))
		
		# if n%20 == 1:
		# 	sgrid1, path1 = findpath(current_pos1, final_pos, final_grid)
		
		

		if len(blocks) > 0  and overall_uncertainty < 0.3:
			final_grid = np.copy(occupancy_grid)
	
			final_grid[final_grid > 0.4] = 1
			final_grid[final_grid <= 0.4] = 0

			rr, cc = rectangle_perimeter((1,1), (78,78), shape=final_grid.shape)
			final_grid[rr, cc] = 1

			rr, cc = rectangle((1,1), (13,13), shape=final_grid.shape)
			final_grid[rr, cc] = 0

			rr, cc = rectangle((1,65), (13,78), shape=final_grid.shape)
			final_grid[rr, cc] = 0

			final_grid = pad_grid(final_grid, 1)


			final_grid = block_padding(blocks, final_grid)

			current_pos0 = convert_to_grid_coords(robot_data[0][1:3])
			current_pos0 = (int(current_pos0[0]), int(current_pos0[1]))
			rr, cc = ellipse(int(blocks[block_i][0]) , int(blocks[block_i][1]), 9,9)
			final_grid[rr, cc] = 0
			path_temp = findpath(current_pos0, (int(blocks[block_i][0]), int(blocks[block_i][1])), final_grid)

			if path_temp:
				path0 = path_temp
				#print(np.asarray(path0))
				path0 = bspline(np.asarray(path_temp), int(np.sqrt((current_pos0[0] - blocks[block_i][0])**2 + (current_pos0[1] - blocks[block_i][1])**2)/2), 9)

			if path0.any():
				if len(path0) > 5:
					set_robot_state(0, robot_motion(robot_data[0][1:4], path0[1], 1.5, 1), 0)
					#set_robot_state(0, robot_motion(robot_data[0][1:4], blocks[0][0:2], 0, 0.1), 0)
				else:

					#set_robot_state(0, robot_motion(robot_data[0][1:4], blocks[block_i][0:2], 0, 1), 0)#
					set_robot_state(0, [0.5, -0.5], 0)

			
			print(robot_data[0][6])

			if path0.any():
				for i in range(len(path0)):
					final_grid[int(path0[i][0]), int(path0[i][1])] = 0.5


			

		#print(update)
		# if update and not(len(blocks) > 0 and overall_uncertainty < 0.3):
		occupancy_grid = update_binary_occupancy_grid(occupancy_grid, robot_data)
			


	# if robot_data


	# if n%40 == 1:
	# 	set_robot_state(0, [-1, 1], 0)
	# 	set_robot_state(1, [1, -1], 0)
	# 	pass
	# elif n%40 == 20:
	# 	set_robot_state(0, [1, -1], 0)
	# 	set_robot_state(1, [-1, 1], 0)
	# 	pass

	# if datalist_0:
	# 	print(datalist_0[6])

	# final_grid = np.copy(occupancy_grid)
	
	# # final_grid[final_grid > 0.4] = 1
	# # final_grid[final_grid <= 0.4] = 0

	# # final_grid = gaussian(final_grid, sigma = 0.5)
	
	# # # final_grid[final_grid > 0.5] = 0
	# # # final_grid[final_grid <= 0.3] = 0

	# # #uncertainty_grid = compute_uncertainty_grid(occupancy_grid)

	# rr, cc = rectangle_perimeter((1,1), (78,78), shape=final_grid.shape)
	# final_grid[rr, cc] = 1

	# rr, cc = rectangle((1,1), (13,13), shape=final_grid.shape)
	# final_grid[rr, cc] = 0

	# rr, cc = rectangle((1,65), (13,78), shape=final_grid.shape)
	# final_grid[rr, cc] = 0

	#final_grid = pad_grid(final_grid, 3)

	# rr, cc = rectangle_perimeter((1,1), (78,78), shape=final_grid.shape)
	# final_grid[rr, cc] = 0

	# print(final_grid[79,79])

	# if path0:
	# 	for i in range(len(path0)):
	# 		final_grid[path0[i]] = 0.5

	# if path1:
	# 	for i in range(len(path1)):
	# 		final_grid[path1[i]] = 0.5

	
	#final_grid = gaussian(final_grid, sigma = 2)

	uncertainty_grid = gaussian(occupancy_grid, sigma = 0.2)

	

	rr, cc = rectangle_perimeter((1,1), (78,78), shape=uncertainty_grid.shape)
	uncertainty_grid[rr, cc] = 0

	rr, cc = rectangle((1,1), (13,13), shape=uncertainty_grid.shape)
	uncertainty_grid[rr, cc] = 0

	rr, cc = rectangle((1,65), (13,78), shape=uncertainty_grid.shape)
	uncertainty_grid[rr, cc] = 0


	


	# final_grid = gaussian(final_grid, sigma = 0.4)

	# blobs_dog= blob_dog((final_grid),min_sigma=1.5, max_sigma=2.3, threshold=0.01, overlap = 0.4)
	# print(blobs_dog)
	# for i in range(len(blobs_dog)):
	# 	if blobs_dog[i][0] != 0 and blobs_dog[i][0] != 79 and blobs_dog[i][1] != 79 and blobs_dog[i][1] != 0:
	# 		print(blobs_dog[i])

	vis1, overall_uncertainty = compute_uncertainty_grid(uncertainty_grid)
	print(overall_uncertainty)

	uncertainty_graph.append(overall_uncertainty)

	vis, blocks = find_blocks(occupancy_grid)
	print(blocks)

	if overall_uncertainty > 0.3:
		img.set_data(occupancy_grid)
	else:
		img.set_data(final_grid)
	fig.canvas.restore_region(axbackground)
	# redraw just the points
	ax1.draw_artist(img)
	# fill in the axes rectangle
	fig.canvas.blit(ax1.bbox)
	fig.canvas.flush_events()

	n = n+1
		
		



