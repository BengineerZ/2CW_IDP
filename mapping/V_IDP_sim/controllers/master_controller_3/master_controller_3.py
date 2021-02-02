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
from pathfinder import findpath
import scipy.interpolate as si
from scipy.signal import find_peaks

TIME_STEP = 64

robot = Robot()

receiver = robot.getDevice('receiver')
emitter = robot.getDevice('emitter')
receiver.enable(TIME_STEP)

emitter.setChannel(Emitter.CHANNEL_BROADCAST)
receiver.setChannel(Receiver.CHANNEL_BROADCAST)

### robot_extent:

#robot_extent = np.mgrid[-3:4, 6:-7:-1].reshape(2, -1).T

robot_extent = np.array([[6, -4],[-7, -4],[-7, 4],[6, 3]])
print(robot_extent)





### comms functionality:

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

	if len(robot_data) > 0:
		if robot_data[0][0] == 1:
			temp = robot_data[0]
			robot_data[0] = robot_data[1]
			robot_data[1] = temp
	return robot_data, state_update

#### grid functionality

def bound(coord, a=0, b=79):
	k = min(coord, b)
	k = max(k, a)
	return k

def transform_local_coords(robot_position, local_coordinate):
	# robot_position = [x, y, theta]
	# local coordinate = [x', y']

	rot_matrix = np.array([[np.cos(robot_position[2]), np.sin(robot_position[2])], [-np.sin(robot_position[2]), np.cos(robot_position[2])]])
	offset_vector = np.array([robot_position[0],robot_position[1]])
	
	return np.matmul(rot_matrix, local_coordinate) + offset_vector

def convert_to_grid_coords(coord):
	return np.flip(np.rint((np.array(coord)*[100, -100] + np.array([119, 119]))/3))

def convert_to_grid_space(coord):
	return np.flip((np.array(coord)*[100, -100] + np.array([119, 119]))/3)

#### generic robot motion defs
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
####



#### main robot_manager class:
class robot_manager:

	def __init__(self, robot_id):
		print('initialise robot: ' + str(robot_id))
		self.robot_id = robot_id
		self.state = ['idle', {}]
		self.set_state('idle')
		self.robot_data = None
		self.driving_grid = np.full((80,80), 0)
		self.current_path = []



		# state utility vars
		self._spin = False
		self._dist_temp = [1000,1000,1000,1000, None, None, None, None]
		self._block_found = False
		self._block_pos_temp = np.array([0,0])

		self._angle_index = 0
		self._dist_array = []
		

	def __call__(self):

		getattr(self, self.state[0])(self.state[1] if len(self.state[1]) > 0 else None)
		

	def set_state(self, state_function, **args):
		self.state[0] = state_function
		self.state[1] = args

	# convention for state defs: use dict of args even if not needed
	def idle(self, args):
		set_robot_state(self.robot_id, [0,0], 0)

	def sweep(self, args):

		start_heading = np.array([-np.cos(args['start']), 0, -np.sin(args['start'])])
		end_heading = np.array([-np.cos(args['end']), 0, -np.sin(args['end'])])
		current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		
		cross_start = -np.cross(current_orientation, start_heading)[1]
		cross_end = -np.cross(current_orientation, end_heading)[1]

		#print(cross_end)
		if np.abs(cross_start) < 0.1 and self._spin:
			self._spin = not(self._spin)
		elif np.abs(cross_end) < 0.1 and not(self._spin):
			self._spin = not(self._spin)
		#print(self.current_angle() + np.pi)

		if self._spin:
			set_robot_state(self.robot_id, [np.sign(cross_start)*args['speed'], -np.sign(cross_start)*args['speed']], 0)
		else:
			set_robot_state(self.robot_id, [np.sign(cross_end)*args['speed'], -np.sign(cross_end)*args['speed']], 0)

	def go_to_target(self, args):
		current_pos = self.current_position()
		current_pos = (int(current_pos[0]), int(current_pos[1]))
		#obstacle_grid = args['obstacle_grid']
		obstacle_grid = self.driving_grid
		if args['block'] == True:
			rr, cc = ellipse(args['target'][0] , args['target'][1], 9,9, shape=self.driving_grid.shape)
			for i in range(len(rr)):
				obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
		if args['empty'] == True:

			rad = blue_bot.current_angle()
			robot_state = np.array([current_pos[0], current_pos[1], rad])
			robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
			for i in range(len(robot_extent)):
				robot_extent_global[i] = np.rint((transform_local_coords(robot_state, robot_extent[i])))
			robot_extent_global = np.transpose(robot_extent_global)
			rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
			obstacle_grid[rr, cc] = 0

		path_temp = findpath(current_pos, args['target'], obstacle_grid)
		#print(path_temp)
		if path_temp:
			if len(path_temp) > 1:
				path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - args['target'][0])**2 + (current_pos[1] - args['target'][1])**2)/2), 9)
				self.current_path = path
				if len(path) > args['early_stop']:
						set_robot_state(self.robot_id, robot_motion(robot_data[self.robot_id][1:4], path[1], 1.5, 1), args['grip'])
				else:
					if 'state' in args:
						#self.set_state(args['state'])
						set_robot_state(self.robot_id, [0,0], args['grip'])
					else:
						set_robot_state(self.robot_id, robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, 1), args['grip'])
			else:
				#set_robot_state(self.robot_id, robot_motion(robot_data[self.robot_id][1:4], args['target'], 0, 1), args['grip'])
				self.set_state('idle')
		else:
			#print(args['target'])
			print('no path possible')

	def block_extent_routine(self, args):

		def check_block_size():
			if self._dist_temp[5] > 0 and self._dist_temp[4] < 0 and np.abs(self._dist_temp[4]) < np.abs(self._dist_temp[5]):
				block_size = np.abs(self._dist_temp[5] - self._dist_temp[4] + 2*np.pi)
				print('test')
			elif self._dist_temp[5] > 0 and self._dist_temp[4] < 0 and np.abs(self._dist_temp[4]) > np.abs(self._dist_temp[5]):
				block_size = np.abs(self._dist_temp[5] - self._dist_temp[4] - 2*np.pi)
				print('test')
			else:
				block_size = np.abs(self._dist_temp[5] - self._dist_temp[4])

			return block_size

		original_block_angle = np.arctan2((args['target'][1] - self.current_position(grid=False)[1]),(-args['target'][0] + self.current_position(grid=False)[0])) - np.pi/4
		#print(original_block_angle)

		sweep_angles = np.linspace(original_block_angle-0.4, original_block_angle+0.4, 20)

		trigger_value = 0.2

		if self._angle_index < len(sweep_angles) - 1:
			if sweep_angles[self._angle_index] > np.pi:
				desired_angle = sweep_angles[self._angle_index] - np.pi*2
			elif sweep_angles[self._angle_index] < -np.pi:
				desired_angle = sweep_angles[self._angle_index] + np.pi*2
			else:
				desired_angle = sweep_angles[self._angle_index]

			if self._angle_index == 0:
				self.set_heading(desired_angle, 5)
			else:
				self.set_heading(desired_angle, 15)

			

			

			current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
			desired_orientation = np.array([-np.cos(desired_angle), 0, -np.sin(desired_angle)])
			cross_start = -np.cross(current_orientation, desired_orientation)[1]
			if np.abs(cross_start) < 0.01:
				print(self._angle_index)
				self._angle_index = self._angle_index + 1
				self._dist_array.append(self.robot_data[self.robot_id][5])
		else:

			# rising_mask = ((np.array(self._dist_array[:-1]) < trigger_value) & (np.array(self._dist_array[1:]) > trigger_value))
			# falling_mask = ((np.array(self._dist_array[:-1]) > trigger_value) & (np.array(self._dist_array[1:]) < trigger_value))
			# print(np.flatnonzero(rising_mask) +1)
			# print(np.flatnonzero(falling_mask) +1)
			print(np.array(self._dist_array)*-1)
			peaks, _ = find_peaks(np.array(self._dist_array)*-1, height=(-0.35, 0), width=3)
			print(self._dist_array[peaks[0]])

			
			if sweep_angles[peaks[0]] + np.pi/4 > np.pi:
				exact_angle = sweep_angles[peaks[0]] + np.pi/4 - np.pi*2
			elif sweep_angles[peaks[0]] + np.pi/4 < -np.pi:
				exact_angle = sweep_angles[peaks[0]] + np.pi/4 + np.pi*2
			else:
				exact_angle = sweep_angles[peaks[0]] + np.pi/4

			exact_position = convert_to_grid_coords(transform_local_coords([self.robot_data[self.robot_id][1], self.robot_data[self.robot_id][2], exact_angle], [0,self._dist_array[peaks[0]]]))
			self.set_heading(exact_angle, 5)
			print(exact_position)
			self._block_pos_temp = np.array([int(exact_position[0]), int(exact_position[1])])

	def get_block_colour(self, args):
		pass


	def raw(self, args):
		set_robot_state(self.robot_id, args['wheels'], args['gripper'])
		
	# convention for utility funcs: args like normal
	def current_position(self, grid=True):
		if (grid):
			return convert_to_grid_coords(robot_data[self.robot_id][1:3])
		else:
			return convert_to_grid_space(robot_data[self.robot_id][1:3])

	def current_angle(self):
		return robot_data[self.robot_id][3]

	def update_driveable_area(self, overall_driving_grid):
		self.driving_grid = np.copy(overall_driving_grid)
		other_bot = convert_to_grid_coords(robot_data[(self.robot_id+1)%2][1:3])
		rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 12,12, shape=overall_driving_grid.shape)
		for i in range(len(rr)):
			self.driving_grid[bound(rr[i]), bound(cc[i])] = 1

		return self.driving_grid

	def set_heading(self, desired_angle, speed):
		if desired_angle > np.pi:
			desired_angle = desired_angle - np.pi*2
		elif desired_angle < -np.pi:
			desired_angle = desired_angle + np.pi*2
		current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		desired_orientation = np.array([-np.cos(desired_angle), 0, -np.sin(desired_angle)])
		cross_start = -np.cross(current_orientation, desired_orientation)[1]
		set_robot_state(self.robot_id, [cross_start*speed, -cross_start*speed], 0)

### environment defs


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

def block_padding(blocks, occupancy_grid):
	for i in range(len(blocks)):
		rr, cc = ellipse(int(blocks[i][0]) , int(blocks[i][1]), 9, 9)
		for i in range(len(rr)):
			occupancy_grid[bound(rr[i]), bound(cc[i])] = 1

	return occupancy_grid

def pad_grid(occupancy_grid, iterations):

	occupancy_grid = gaussian(occupancy_grid, sigma = 2)
	occupancy_grid[occupancy_grid > 0.05] = 1
	occupancy_grid[occupancy_grid <= 0.05] = 0
	
	return occupancy_grid

class environment_manager:

	def __init__(self):
		print('initialise environment')
		self.occupancy_grid = np.full((80,80), 0.5)
		self.blocks = []

	def __call__(self, ):
		#print('occupancy_grid update')
		pass

	def update_binary_occupancy_grid(self, robot_data):
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
			for i in range(len(robot_extent)):
				robot_extent_global[i] = np.rint((transform_local_coords(grid_robot_state, robot_extent[i])))
			robot_extent_global = np.transpose(robot_extent_global)
			rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
			check_coords = np.array([rr, cc]).T

			if np.array([bound(int(coord1[1]),bound(int(coord1[0])))]) not in check_coords:
				self.occupancy_grid[bound(int(coord1[1]))][bound(int(coord1[0]))] = update_occupied(self.occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], p=(0.95 - 0.14*psValues[0]))
			if np.array([bound(int(coord2[1]),bound(int(coord2[0])))]) not in check_coords:
				self.occupancy_grid[bound(int(coord2[1]))][bound(int(coord2[0]))] = update_occupied(self.occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], p=(0.95 - 0.14*psValues[1]))

			current_pos = np.rint((np.array([test_position[0], test_position[1]])*[100, -100] + np.array([119, 119]))/3)

			rr1, cc1, updates1 = update_free(self.occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.4)
			self.occupancy_grid[rr1, cc1] = updates1

			rr2, cc2, updates2 = update_free(self.occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.4)
			self.occupancy_grid[rr2, cc2] = updates2

	def find_blocks(self):

		final_grid = gaussian(np.copy(self.occupancy_grid), sigma = 0.2)

		final_grid[final_grid > 0.99] = 1
		final_grid[final_grid <= 0.99] = 0

		rr, cc = rectangle_perimeter((1,1), (78,78), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		rr, cc = rectangle((1,1), (13,13), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		rr, cc = rectangle((1,65), (13,78), shape=final_grid.shape)
		final_grid[rr, cc] = 0

		final_grid = gaussian(final_grid, sigma = 0.4)

		self.blocks = blob_dog((final_grid),min_sigma=1.5, max_sigma=2.3, threshold=0.01, overlap = 0.4)
		
		return final_grid, self.blocks

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

		

		final_grid = block_padding(self.blocks, final_grid)

		# rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 9,9, shape=final_grid.shape)
		# for i in range(len(rr)):
		# 	final_grid[bound(rr[i]), bound(cc[i])] = 1

		return final_grid






environment = environment_manager()

test = np.full((80,80), 0)
occupancy_grid = np.full((80,80), 0)
driving_grid = np.full((80,80), 0)

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
img = ax1.imshow(environment.occupancy_grid, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")
fig.canvas.draw()   # note that the first draw comes before setting data 
# cache the background
axbackground = fig.canvas.copy_from_bbox(ax1.bbox)

fig2 = plt.figure()
ax2 = fig2.add_subplot(1, 1, 1)
img2 = ax2.imshow(test, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")
fig2.canvas.draw()   # note that the first draw comes before setting data 
# cache the background
ax2background = fig2.canvas.copy_from_bbox(ax2.bbox)



plt.show(block=False)





### comms initialisation:
# initiate_data_transfer()
# robot_data, update = await_state_data()
###

### robot initialisation:
red_bot = robot_manager(0)
blue_bot = robot_manager(1)
n = 0

blocks = []

while robot.step(TIME_STEP) != -1:

	initiate_data_transfer()
	robot_data, update = await_state_data()

	if len(robot_data) == 2:
		red_bot.robot_data = robot_data
		blue_bot.robot_data = robot_data



		
		if n == 10:
			### main loop 
			#blue_bot.set_state('go_to_target', target = (40,40), obstacle_grid = test, early_stop = 5, grip = 0)
			blue_bot.set_state('sweep', start = np.pi/2 - 0.5, end = -np.pi +0.5, speed = 0.5)
			red_bot.set_state('sweep', start = -np.pi/2 + 0.5, end = np.pi - 0.5, speed = 0.5)

		if n == 200:
			first_block = np.array([int(blocks[0][0]), int(blocks[0][1])])
			blue_bot.set_state('go_to_target', target = first_block, obstacle_grid = blue_bot.driving_grid, early_stop = 5, grip = 0, block= True, empty=False, state='idle')
			print(first_block)
		if n == 280:
			#blue_bot.set_state('go_to_target', target = (5,5), obstacle_grid = blue_bot.driving_grid, early_stop = 2, grip = 1, block= False, empty=True)
			blue_bot.set_state('block_extent_routine', target = first_block)

		if n == 500:
			blue_bot.set_state('go_to_target', target = blue_bot._block_pos_temp, obstacle_grid = blue_bot.driving_grid, early_stop = 1, grip = 0, block= True, empty=False, state='idle')

		if n == 520:
			blue_bot.set_state('go_to_target', target = blue_bot._block_pos_temp, obstacle_grid = blue_bot.driving_grid, early_stop = 1, grip = 1, block= True, empty=False, state='idle')

			rr, cc = ellipse(int(blue_bot._block_pos_temp[0]) , int(blue_bot._block_pos_temp[1]), 4,4)
			for i in range(len(rr)):
				environment.occupancy_grid[bound(rr[i]), bound(cc[i])] = 0.2

		if n == 600:
			blue_bot.set_state('go_to_target', target = (8,8), obstacle_grid = blue_bot.driving_grid, early_stop = 1, grip = 1, block= False, empty=True, state='idle')


		if n > 700:
			blue_bot.set_state('go_to_target', target = (8,8), obstacle_grid = blue_bot.driving_grid, early_stop = 2, grip = 0, block= False, empty=True)
			


		#print(robot_data[0][6])
		# pos = blue_bot.current_position(grid = False)
		# rad = blue_bot.current_angle()
		# robot_state = np.array([pos[0], pos[1], rad])
		# #if n == 40:
		# robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
		# occupancy_grid = np.full((80,80), 0)
		# for i in range(len(robot_extent)):
		# 	robot_extent_global[i] = np.rint((transform_local_coords(robot_state, robot_extent[i])))
		# robot_extent_global = np.transpose(robot_extent_global)
		# rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
		# print(np.array([rr, cc]).T)
		# occupancy_grid[rr, cc] = 1



		
		
		test = np.copy(blue_bot.driving_grid)
		for i in range(len(blue_bot.current_path)):
			test[int(blue_bot.current_path[i][0]), int(blue_bot.current_path[i][1])] = 0.5
		driving_grid = environment.driving_grid()

		blue_bot.update_driveable_area(driving_grid)
		red_bot.update_driveable_area(driving_grid)

		occupancy_grid, blocks = environment.find_blocks()
		environment.update_binary_occupancy_grid(robot_data)
		red_bot()
		blue_bot()


	img.set_data(environment.occupancy_grid)
	fig.canvas.restore_region(axbackground)
	# redraw just the points
	ax1.draw_artist(img)
	# fill in the axes rectangle
	fig.canvas.blit(ax1.bbox)
	fig.canvas.flush_events()



	img2.set_data(test)
	fig2.canvas.restore_region(ax2background)
	# redraw just the points
	ax2.draw_artist(img2)
	# fill in the axes rectangle
	fig2.canvas.blit(ax2.bbox)
	fig2.canvas.flush_events()
	n = n+1
		
		



