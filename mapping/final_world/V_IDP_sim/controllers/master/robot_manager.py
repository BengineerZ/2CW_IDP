#!/home/ben/anaconda3/bin/python3.8
import numpy as np
import struct
from skimage.draw import ellipse, polygon, line
from pathfinder import findpath, h
from pathfinder import escape_path as obstacle_path
import scipy.interpolate as si
from scipy.signal import find_peaks
from utils import *

#### comms functionality:
def initiate_data_transfer(emitter):
	message = struct.pack('i', 1)
	emitter.send(message)
	#print('initiate')

def await_state_data(receiver):

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

#### generic robot motion defs
def robot_motion(state, target, pd, pc, info=False, normalise=False, magnitude=True):

	current_orientation = np.array([-np.cos(state[2]), 0, -np.sin(state[2])])
	current_position = convert_to_grid_space(state[0:2])
	
	heading_vector = np.subtract(target,current_position)
	heading_vector = np.array([heading_vector[0], 0, -heading_vector[1]])

	if normalise and magnitude:
		heading_vector = heading_vector/np.abs(np.linalg.norm(heading_vector))
		current_orientation = current_orientation/np.abs(np.linalg.norm(current_orientation))
	if normalise and not(magnitude):
		heading_vector = heading_vector/np.linalg.norm(heading_vector)
		current_orientation = current_orientation/np.linalg.norm(current_orientation)
		print(current_orientation)

	dot = current_orientation.dot(heading_vector)
	cross = -np.cross(current_orientation, heading_vector)[1]

	leftSpeed = pd*dot + pc*cross
	rightSpeed = pd*dot - pc*cross

	if info:
		return leftSpeed, rightSpeed, dot, cross
	else:
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

#### main robot_manager class:
class robot_manager:

	def __init__(self, robot_id, emitter):
		print('initialise robot: ' + str(robot_id))
		self.robot_id = robot_id
		self.state = ['idle', {}]
		self.set_state('idle')
		self.robot_data = None
		self.driving_grid = np.full((80,80), 0)
		self.reduced_driving_grid = np.full((80,80), 0)
		self.current_path = []
		self._block_found_temp = False
		self.extent = np.array([[6, -4],[-7, -4],[-7, 4],[6, 3]])

		self.emitter = emitter

		# state utility vars
		self._spin = False
		self._sweep_counter = 0

		self._block_pos_temp = np.array([0,0])
		self._angle_index = 0
		self._dist_array = []
		self._block_col_temp = 0

		self.carrying = False
		self.blocks_dropped = 0

		self.toggle_rev = True
		self._rev = 1

		self._wait_counter = 0
		self._goto_wait_counter = 0
		self._goto_wait = False

		self.specific_blocks_removed = 0

		self._reduce_grid = False
		

	def __call__(self, env):

		if self.carrying and self.get_colour() == 0:
			self.set_state('blocked', grip = 0)
			self.carrying = False
			self.blocks_dropped = self.blocks_dropped + 1

		# for i in range(len(env.blocks)):
		# 	if h(self.current_position(), env.blocks[i][0:2]) < 3:
		# 		env.remove_block(self.current_position())



		#print('number of blocks dropped: ' + str(self.blocks_dropped))

		return getattr(self, self.state[0])(self.state[1] if len(self.state[1]) > 0 else None)

	def set_robot_state(self, wheel_v, gripper):
		message = struct.pack('iddi', self.robot_id, np.clip(wheel_v[0], -10,10), np.clip(wheel_v[1],-10,10),  gripper)
		self.emitter.send(message)

	def set_state(self, state_function, *args ,**kwargs):

		if state_function:
			self.state[0] = state_function

			if len(args) > 0:
				self.state[1] = args[0]
			else:
				self.state[1] = kwargs

	# convention for state defs: use dict of args even if not needed
	def idle(self, args):
		if args:
			if 'grip' in args:
				self.set_robot_state([0,0], args['grip'])
			else:
				self.set_robot_state([0,0], 0)
		else:
			self.set_robot_state([0,0], 0)

	def blocked(self, args):
		if args:
			if 'grip' in args:
				self.set_robot_state([0,0], args['grip'])
			else:
				self.set_robot_state([0,0], 0)
		else:
			self.set_robot_state([0,0], 0)

	def sweep(self, args):

		start_heading = np.array([-np.cos(args['start']), 0, -np.sin(args['start'])])
		end_heading = np.array([-np.cos(args['end']), 0, -np.sin(args['end'])])
		current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		
		cross_start = -np.cross(current_orientation, start_heading)[1]
		cross_end = -np.cross(current_orientation, end_heading)[1]

		dot_start = current_orientation.dot(start_heading)
		dot_end = current_orientation.dot(end_heading)

		#print(cross_end)
		if np.abs(cross_start) < 0.1 and dot_start > 0 and self._spin:
			self._spin = not(self._spin)
		elif np.abs(cross_end) < 0.1 and dot_end > 0 and not(self._spin):
			self._spin = not(self._spin)
			self._sweep_counter = self._sweep_counter + 1
		#print(self.current_angle() + np.pi)

		if self._spin:
			self.set_robot_state([np.sign(cross_start)*args['speed'], -np.sign(cross_start)*args['speed']], 0)
		else:
			self.set_robot_state([np.sign(cross_end)*args['speed'], -np.sign(cross_end)*args['speed']], 0)
		if self._sweep_counter == args['n']:

			self._spin = not(self._spin)
			self._sweep_counter = 0
			self.set_state('idle')

#	def go_to_target(self, args):

		# if args['grip'] == 1:
		# 	self.carrying == True
		# else:
		# 	self.carrying == False

		# if "speed" in args:
		# 	speed = args['speed']
		# else:
		# 	speed = 1.5

		# if 'look_at' in args:
		# 	look_at = args['look_at']
		# else:
		# 	look_at = True

		# if 'reverse' in args:
		# 	reverse = args['reverse']
		# 	if reverse:
		# 		rev = -1
		# 	else:
		# 		rev = 1
		# else:
		# 	rev= 1

		# if 'optimise_heading' in args:
		# 	optimise_heading = args['optimise_heading']
		# else:
		# 	optimise_heading = False

		# current_pos = self.current_position()
		# current_pos = (int(current_pos[0]), int(current_pos[1]))
		# #obstacle_grid = args['obstacle_grid']
		# obstacle_grid = self.driving_grid
		# reduced_obstacle_grid = self.reduced_driving_grid

		# if args['block'] == True:
		# 	rr, cc = ellipse(args['target'][0] , args['target'][1], 9,9, shape=self.driving_grid.shape)
		# 	for i in range(len(rr)):
		# 		obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
		# 		reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0

		# if args['empty'] == True:
		# 	rad = self.current_angle()
		# 	robot_state = np.array([current_pos[0], current_pos[1], rad])
		# 	robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
		# 	for i in range(len(self.extent)):
		# 		robot_extent_global[i] = np.rint((transform_local_coords(robot_state, self.extent[i])))
		# 	robot_extent_global = np.transpose(robot_extent_global)
		# 	rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
		# 	for i in range(len(rr)):
		# 		obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
		# 		reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0

		# other_bot = convert_to_grid_coords(self.robot_data[(self.robot_id+1)%2][1:3])
		# rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 12,12, shape=self.driving_grid.shape)
		# for i in range(len(rr)):
		# 	self.driving_grid[bound(rr[i]), bound(cc[i])] = 1
		# 	self.reduced_driving_grid[bound(rr[i]), bound(cc[i])] = 1
		# 	#obstacle_grid[rr, cc] = 0
		# 	#reduced_obstacle_grid[rr, cc] = 0
		# #print(obstacle_grid[args['target'][0], args['target'][1]])
		# path_temp = findpath(current_pos, args['target'], obstacle_grid)
		# #print(path_temp)
		# if path_temp:
		# 	if len(path_temp) > 1:
		# 		path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - args['target'][0])**2 + (current_pos[1] - args['target'][1])**2)/2), 9)
		# 		self.current_path = path
		# 		if np.sign(speed)*len(path) > np.sign(speed)*args['early_stop']:
		# 			if optimise_heading:
		# 				# compare robot heading and path:
						
		# 				path_heading = np.array([path[1][0] - current_pos[0], 0, path[1][1] - current_pos[1]])
		# 				path_heading = path_heading/np.linalg.norm(path_heading)
		# 				current_heading = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		# 				current_heading = current_heading/np.linalg.norm(current_heading)
						
		# 				dot_bot = -1*current_heading.dot(path_heading)

		# 				cross_bot = -np.cross(current_heading, path_heading)[1]

		# 				#print(cross_bot)
		# 				#print(dot_bot)
		# 				if np.abs(cross_bot) < 0.9 and dot_bot < 0 and self.toggle_rev:
		# 					self._rev = np.sign(dot_bot)
		# 					self.toggle_rev = False

		# 				rev = self._rev

		# 			self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[1], speed, rev*1), args['grip'])
		# 		else:
		# 			if look_at:
		# 				motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, rev*1, info=True)
		# 				self.set_robot_state([np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
		# 				if np.abs(motion[2]) < 0.1:
		# 					self.set_state('idle', grip = args['grip'])
		# 					self.toggle_rev = True
		# 					self.current_path = []
		# 			else:
		# 				self.set_state('idle', grip = args['grip'])
		# 				self.toggle_rev = True
		# 				self.current_path = []
		# 	else:
		# 		self.set_state('idle', grip = args['grip'])
		# 		self.toggle_rev = True
		# 		self.current_path = []
		# else:
		# 	#print(args['target'])
		# 	obstacle_count = 0
		# 	if len(self.current_path) > 0:
		# 		for i in range(len(self.current_path) if len(self.current_path) <= 6 else 6):
		# 			if obstacle_grid[int(self.current_path[i][0]), int(self.current_path[i][1])] == 1:
		# 				obstacle_count = obstacle_count + 1
		# 	#print(obstacle_count)

		# 	if len(self.current_path) > 0 and 3 < obstacle_count < 5:
		# 		print('path obstruction!')
		# 		# motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, rev*1, info=True)
		# 		# self.set_robot_state([np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
		# 		self.set_state('blocked', grip = args['grip'])
		# 	elif obstacle_count >= 5:
		# 		print('no path possible ... reducing driving grid')

		# 		path_temp = findpath(current_pos, args['target'], reduced_obstacle_grid)
		# 		#print(path_temp)
		# 		if path_temp:
		# 			if len(path_temp) > 1:
		# 				path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - args['target'][0])**2 + (current_pos[1] - args['target'][1])**2)/2), 9)
		# 				self.current_path = path
		# 				if np.sign(speed)*len(path) > np.sign(speed)*args['early_stop']:
		# 					if optimise_heading:
		# 						# compare robot heading and path:
								
		# 						path_heading = np.array([path[1][0] - current_pos[0], 0, path[1][1] - current_pos[1]])
		# 						path_heading = path_heading/np.linalg.norm(path_heading)
		# 						current_heading = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		# 						current_heading = current_heading/np.linalg.norm(current_heading)
								
		# 						dot_bot = -1*current_heading.dot(path_heading)

		# 						cross_bot = -np.cross(current_heading, path_heading)[1]

		# 						if np.abs(cross_bot) < 0.9 and dot_bot < 0 and self.toggle_rev:
		# 							self._rev = np.sign(dot_bot)
		# 							self.toggle_rev = False

		# 						rev = self._rev

		# 					self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[1], speed, rev*1), args['grip'])
		# 				else:
		# 					if look_at:
		# 						motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, rev*1, info=True)
		# 						self.set_robot_state([np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
		# 						if np.abs(motion[2]) < 0.1:
		# 							self.set_state('idle', grip = args['grip'])
		# 							self.toggle_rev = True
		# 							self.current_path = []
		# 					else:
		# 						self.set_state('idle', grip = args['grip'])
		# 						self.toggle_rev = True
		# 						self.current_path = []
		# 			else:
		# 				self.set_state('idle', grip = args['grip'])
		# 				self.toggle_rev = True
		# 				self.current_path = []
		# 		else:
		# 			print('no reduced path possible, state set to: "blocked"')
		# 			self.toggle_rev = True
		# 			self.set_state('blocked', grip = args['grip'])
		# 			self.current_path = []
		# 	else:
		# 		print('NOPE')
		# 		self.set_state('idle', grip = args['grip'])

		# 	# if len(self.current_path) > 10 and obstacle_count <= 3:
		# 	# 	print('returning to path')
		# 	# 	print('path obstruction!')
		# 	# 	# motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, rev*1, info=True)
		# 	# 	# self.set_robot_state([np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
		# 	# 	self.set_state('blocked', grip = args['grip'])
		# 	# 	self.current_path = []
		# 	# 	self.toggle_rev = True

		# 	# elif 3 < obstacle_count <= 5:
		# 	# 	print('no path possible ... reducing driving grid')

		# 	# 	path_temp = findpath(current_pos, args['target'], reduced_obstacle_grid)
		# 	# 	#print(path_temp)
		# 	# 	if path_temp:
		# 	# 		if len(path_temp) > 1:
		# 	# 			path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - args['target'][0])**2 + (current_pos[1] - args['target'][1])**2)/2), 9)
		# 	# 			self.current_path = path
		# 	# 			if np.sign(speed)*len(path) > np.sign(speed)*args['early_stop']:
		# 	# 				if optimise_heading:
		# 	# 					# compare robot heading and path:
								
		# 	# 					path_heading = np.array([path[1][0] - current_pos[0], 0, path[1][1] - current_pos[1]])
		# 	# 					path_heading = path_heading/np.linalg.norm(path_heading)
		# 	# 					current_heading = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		# 	# 					current_heading = current_heading/np.linalg.norm(current_heading)
								
		# 	# 					dot_bot = -1*current_heading.dot(path_heading)

		# 	# 					cross_bot = -np.cross(current_heading, path_heading)[1]

		# 	# 					if np.abs(cross_bot) < 0.9 and dot_bot < 0 and self.toggle_rev:
		# 	# 						self._rev = np.sign(dot_bot)
		# 	# 						self.toggle_rev = False

		# 	# 					rev = self._rev

		# 	# 				self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[1], speed, rev*1), args['grip'])
		# 	# 			else:
		# 	# 				if look_at:
		# 	# 					motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, rev*1, info=True)
		# 	# 					self.set_robot_state([np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
		# 	# 					if np.abs(motion[2]) < 0.1:
		# 	# 						self.set_state('idle', grip = args['grip'])
		# 	# 						self.toggle_rev = True
		# 	# 						self.current_path = []
		# 	# 				else:
		# 	# 					self.set_state('idle', grip = args['grip'])
		# 	# 					self.toggle_rev = True
		# 	# 					self.current_path = []
		# 	# 		else:
		# 	# 			self.set_state('idle', grip = args['grip'])
		# 	# 			self.toggle_rev = True
		# 	# 			self.current_path = []
		# 	# 	else:
		# 	# 		print('no reduced path possible, state set to: "blocked"')
		# 	# 		self.toggle_rev = True
		# 	# 		self.set_state('blocked', grip = args['grip'])
		# 	# 		self.current_path = []
				
		# 	# elif len(self.current_path) > 10 and obstacle_count > 5:
		# 	# 	print('path obstruction!')
		# 	# 	# motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, rev*1, info=True)
		# 	# 	# self.set_robot_state([np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
		# 	# 	self.set_state('blocked', grip = args['grip'])
		# 	# 	self.current_path = []
		# 	# 	self.toggle_rev = True

		# 	# else:
		# 	# 	print('NOPE')
		# 	# 	self.set_state('idle', grip = args['grip'])
		# 	# 	self.toggle_rev = True
		# 	# 	self.current_path = []

	#### utility functions for go_to_target():

	def straight_line_path(self, obstacle_grid, target):
		### check straight line path:
		cur_pos = self.current_position()
		rr, cc = line(int(cur_pos[0]), int(cur_pos[1]), int(target[0]), int(target[1]))
		straight_line_path = True
		for i in range(len(rr)):
			if obstacle_grid[bound(rr[i]), bound(cc[i])] == 1:
				straight_line_path = False

		### if straight line path is possible, generate the path
		if straight_line_path:
			return np.array([list(x) for x in zip(*[rr, cc])])
		else:
			return np.array([None])

	def smart_path(self, obstacle_grid, target):
		current_pos = self.current_position()
		current_pos = (int(current_pos[0]), int(current_pos[1]))
		path_temp = findpath(current_pos, target, obstacle_grid)
		if path_temp:
			if len(path_temp) > 1:
				path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - target[0])**2 + (current_pos[1] - target[1])**2)/2), 9)
				return path
			else:
				return np.array(path_temp)
		else:
			return np.array([None])

	def escape_path(self, obstacle_grid, threshold=10):
		current_pos = self.current_position()
		current_pos = (int(current_pos[0]), int(current_pos[1]))
		path_temp = obstacle_path(current_pos, obstacle_grid)

		if path_temp:
			if len(path_temp) <= threshold:
				return np.array(path_temp)
			else:
				return np.array([None])
		else:
			return np.array([None])

	def straight_line_follow(self, args, path_temp, speed_factor=1):
		cur_pos = self.current_position()
		#print(len(path_temp))
		if len(path_temp) > 2:
			#print(path_temp)
			path = bspline(path_temp, int(np.sqrt((cur_pos[0] - args['target'][0])**2 + (cur_pos[1] - args['target'][1])**2)/2), 9)
			if np.sign(args['away'])*len(path) > np.sign(args['away'])*args['early_stop']:
				
				# compare robot heading and path:
				path_heading = np.array([args['target'][0] - cur_pos[0], 0, args['target'][1] - cur_pos[1]])
				path_heading = path_heading/np.linalg.norm(path_heading)
				current_heading = np.array([np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
				current_heading = current_heading/np.linalg.norm(current_heading)

				dot_bot = -1*current_heading.dot(path_heading)
				cross_bot = -np.cross(current_heading, path_heading)[1]

				if np.abs(cross_bot) < 0.8 and dot_bot < 0 and self.toggle_rev and args['optimise_heading']:
					self._rev = -1
					self.toggle_rev = False
					#print('reversing!!!')

				#print(len(path))
				rev = self._rev
				if len(path) > 10:
					m = si.interp1d([0, 1],[8*speed_factor, 0])
					m1 = si.interp1d([0, 114], [0.1,1])
					self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[-1], float(m(np.abs(cross_bot)))*args['away'], rev*3*speed_factor, normalise=True), args['grip'])
				elif len(path) > 1:
					self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[1], 1.5*args['away'], rev*1, normalise=False), args['grip'])
				else:
					return
				return
			else:
				print('path completed')
				if args['look_at']:
					motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, self._rev*1, info=True ,normalise=True)
					self.set_robot_state([np.sign(motion[3])*2, -np.sign(motion[3])*2], args['grip'])
					if np.abs(motion[3]) < 0.1 and np.sign(motion[2]) > 0:
						self.set_state('idle', grip = args['grip'])
						self.toggle_rev = True
						self._rev = 1
						self.current_path = []
						print('look at completed')
						return
				else:
					self.set_state('idle', grip = args['grip'])
					self.toggle_rev = True
					self._rev = 1
					self.current_path = []
					print('path completed')
					return
		else:
			self.set_state('idle', grip = args['grip'])
			self.toggle_rev = True
			self._rev = 1
			self.current_path = []
			print('path completed')
			return

	def smart_path_follow(self, args, path, speed_factor=1):
		#print('smart path follow')
		cur_pos = self.current_position()
		if np.sign(args['away'])*len(path) > np.sign(args['away'])*args['early_stop']:
			
			# compare robot heading and path:
			path_heading = np.array([path[1][0] - cur_pos[0], 0, path[1][1] - cur_pos[1]])
			path_heading = path_heading/np.linalg.norm(path_heading)
			current_heading = np.array([np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
			current_heading = current_heading/np.linalg.norm(current_heading)

			dot_bot = -1*current_heading.dot(path_heading)
			cross_bot = -np.cross(current_heading, path_heading)[1]

			if np.abs(cross_bot) < 0.8 and dot_bot < 0 and self.toggle_rev and args['optimise_heading']:
					self._rev = -1
					self.toggle_rev = False
					#print('reversing!!!')

			rev = self._rev

			if len(path) > 5:
				m = si.interp1d([0, 1],[8*speed_factor, 0])
				self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[1], 4*speed_factor*args['away'], rev*3*speed_factor, normalise=True), args['grip'])
			else:
				self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[1], 4*speed_factor*args['away'], rev*3*speed_factor, normalise=False), args['grip'])
			return
		else:
			print('path completed')
			if args['look_at']:
				motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, self._rev*1, info=True ,normalise=True)
				self.set_robot_state([np.sign(motion[3])*3, -np.sign(motion[3])*3], args['grip'])
				if np.abs(motion[3]) < 0.1 and np.sign(motion[2]) > 0:
					self.set_state('idle', grip = args['grip'])
					self.toggle_rev = True
					self._rev = 1
					self.current_path = []
					print('look at completed')
					return
			else:
				self.set_state('idle', grip = args['grip'])
				self.toggle_rev = True
				self._rev = 1
				self.current_path = []
				print('path completed')
				return

	def escape_path_follow(self, args, path, speed_factor=1):
		cur_pos = self.current_position()

		rev = self._rev
		
		self.set_robot_state(robot_motion(self.robot_data[self.robot_id][1:4], path[-1], 3*speed_factor*args['away'], rev*3*speed_factor, normalise=True), args['grip'])
		
	def go_to_target(self, args):

		if self._goto_wait:
			if self._goto_wait_counter < 50:
				self._goto_wait_counter += 1
				self.set_robot_state([0,0], args['grip'])
				return
			else:
				self._goto_wait_counter = 0
				self._goto_wait = False

		cur_pos = self.current_position()#

		### argument setup:
		rev = 1

		if "speed" in args:
			speed = args['speed']
		else:
			speed = 1

		if 'away' not in args:
			args['away'] = 1
		if 'optimise_heading' not in args:
			args['optimise_heading'] = False
		if 'look_at' not in args:
			args['look_at'] = True

		### driving grid step:
		current_pos = cur_pos
		current_pos = (int(current_pos[0]), int(current_pos[1]))
		#obstacle_grid = args['obstacle_grid']
		obstacle_grid = self.driving_grid
		reduced_obstacle_grid = self.reduced_driving_grid

		if args['block'] == True:
			rr, cc = ellipse(args['target'][0] , args['target'][1], 9,9, shape=self.driving_grid.shape)
			for i in range(len(rr)):
				obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
				reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0

		if args['empty'] == True:
			rad = self.current_angle()
			robot_state = np.array([current_pos[0], current_pos[1], rad])
			robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
			for i in range(len(self.extent)):
				robot_extent_global[i] = np.rint((transform_local_coords(robot_state, self.extent[i])))
			robot_extent_global = np.transpose(robot_extent_global)
			rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
			for i in range(len(rr)):
				obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
				reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0

		other_bot = convert_to_grid_coords(self.robot_data[(self.robot_id+1)%2][1:3])
		rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 12,12, shape=self.driving_grid.shape)
		for i in range(len(rr)):
			obstacle_grid[bound(rr[i]), bound(cc[i])] = 1
			reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 1
		
		############

		### if straight line path is possible, turn towards it and follow the straight line!
		straight_line_path = self.straight_line_path(obstacle_grid, args['target'])
		if straight_line_path.any() != None:
			self.current_path = straight_line_path
			self.straight_line_follow(args, straight_line_path, speed_factor=speed)
			#print('following straight line')
		else:
			#print('straight line path blocked, finding new path')
			smart_path = self.smart_path(obstacle_grid, args['target'])
			if smart_path.any() != None:
				self.current_path = smart_path
				self.smart_path_follow(args, smart_path, speed_factor=speed)
				self._reduce_grid = False
			elif smart_path.any() == None and len(self.current_path) > 0 and self._reduce_grid == False:
				print('entered blocked zone, attempting to escape!')
				escape_path = self.escape_path(obstacle_grid)
				if escape_path.any() != None:
					
					self.escape_path_follow(args, escape_path, speed_factor=speed)
					obstacle_count = 0
					if len(self.current_path) > 0:
						for i in range(len(self.current_path) if len(self.current_path) <= 1 else 1):
							if obstacle_grid[int(self.current_path[i][0]), int(self.current_path[i][1])] == 1:
								obstacle_count = obstacle_count + 1
					print(obstacle_count)

					if obstacle_count == 0:
						#print('target obstruction, choosing new target')
						if args['grip'] == 0:
							self.set_state('blocked', grip = args['grip'])
							self.toggle_rev = True
							self._rev = 1
							self.current_path = []
							print('obstacle blocked')
						else:
							self._goto_wait = True
							print('WAIT1')
				else:
					print('Definitely blocked!!!')
					pass
			else:
				print('smart path blocked, reducing driving_grid')
				obstacle_count = 0
				if len(self.current_path) > 0:
					for i in range(len(self.current_path) if len(self.current_path) <= 1 else 1):
						if obstacle_grid[int(self.current_path[i][0]), int(self.current_path[i][1])] == 1:
							obstacle_count = obstacle_count + 1
				#print(obstacle_count)

				# if args['grip'] == 0 and self.robot_id == 1:
				# 	self.set_state('blocked', grip = args['grip'])
				# 	self.toggle_rev = True
				# 	self._rev = 1
				# 	self.current_path = []
				# 	print('End blocked')
				# else:
				# 	self._goto_wait = True
				# 	print('WAIT2')

				reduced_smart_path = self.smart_path(reduced_obstacle_grid, args['target'])
				if reduced_smart_path.any() != None:
					self.current_path = reduced_smart_path
					self.smart_path_follow(args, reduced_smart_path, speed_factor=speed)
					self._reduce_grid = True
				else:
					print('no reduced path')
					if args['grip'] == 0:
						self.set_state('blocked', grip = args['grip'])
						self.toggle_rev = True
						self._rev = 1
						self.current_path = []
						print('End blocked')
					else:
						self._goto_wait = True
						print('WAIT2')



				








			#self.set_state('blocked', grip = args['grip'])
			#self.toggle_rev = True
			#self.current_path = []

		

	def block_extent_routine(self, args):

		self._block_pos_temp = np.array([0,0])
		self._block_found_temp = False

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
				self.set_heading(desired_angle, 0.5)
			else:
				self.set_heading(desired_angle, 0.5)

			current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
			desired_orientation = np.array([-np.cos(desired_angle), 0, -np.sin(desired_angle)])
			cross_start = -np.cross(current_orientation, desired_orientation)[1]
			if np.abs(cross_start) < 0.01:
				#print(self._angle_index)
				self._angle_index = self._angle_index + 1
				self._dist_array.append(self.robot_data[self.robot_id][5])
		else:

			peaks, properties = find_peaks(np.array(self._dist_array)*-1, height=(-0.35, 0), width=3)

			if len(peaks) == 0:
				print('no block found')
				self._angle_index = 0
				self._dist_array = []
				self.set_state('idle')
				self._block_found_temp = False
				args['env'].update_block((args['target'], 0, True))

			else:
				self._block_found_temp = True

				sweep1 = sweep_angles[int(properties["left_ips"][0])]
				sweep2 = sweep_angles[int(properties["right_ips"][0])]

				exact_angle = (sweep1 + sweep2)/2

				if exact_angle + np.pi/4 > np.pi:
					exact_angle = exact_angle + np.pi/4 - np.pi*2
				elif sweep_angles[peaks[0]] + np.pi/4 < -np.pi:
					exact_angle = exact_angle + np.pi/4 + np.pi*2
				else:
					exact_angle = exact_angle + np.pi/4

				
				cross = self.set_heading(exact_angle, 0.5, info=True)
				#print(exact_position)
				if np.abs(cross) < 0.04:
					exact_position = convert_to_grid_coords(transform_local_coords([self.robot_data[self.robot_id][1], self.robot_data[self.robot_id][2], exact_angle], [0,self._dist_array[peaks[0]]]))
					self._angle_index = 0
					self._dist_array = []
					self._block_pos_temp = np.array([int(exact_position[0]), int(exact_position[1])])
					self.set_state('go_to_target', target = self._block_pos_temp, early_stop = 2, grip = 0, block= True, empty=False, look_at = False, speed = 1)


	def block_update_routine(self, args):

		
		colour = self.robot_data[self.robot_id][6]
		coord = self._block_pos_temp

		#self._block_pos_temp = np.array([0,0])

		if args['grip'] == 1:
			args['env'].update_block((coord, colour, True))
			self.specific_blocks_removed += 1
		else:
			args['env'].update_block((coord, colour, False))

		
		self.set_state('idle', grip = 0)

	def wait(self, args):
		if self._wait_counter < args['steps']:
			self.set_robot_state([0,0], args['grip'])
			self._wait_counter += 1
		else:
			self.set_state('idle', grip = args['grip'])
			self._wait_counter = 0

	def raw(self, args):
		self.set_robot_state(args['wheels'], args['gripper'])
		
	# convention for utility funcs: args like normal
	def current_position(self, grid=True):
		if (grid):
			return convert_to_grid_coords(self.robot_data[self.robot_id][1:3])
		else:
			return convert_to_grid_space(self.robot_data[self.robot_id][1:3])

	def current_angle(self):
		return self.robot_data[self.robot_id][3]

	def update_driveable_area(self, driving_grid, reduced_driving_grid):
		self.driving_grid = np.copy(driving_grid)
		self.reduced_driving_grid = np.copy(self.reduced_driving_grid)
		other_bot = convert_to_grid_coords(self.robot_data[(self.robot_id+1)%2][1:3])
		rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 12,12, shape=self.driving_grid.shape)
		for i in range(len(rr)):
			self.driving_grid[bound(rr[i]), bound(cc[i])] = 1
			self.reduced_driving_grid[bound(rr[i]), bound(cc[i])] = 1

		#return self.driving_grid

	def set_heading(self, desired_angle, speed, info=False):
		if desired_angle > np.pi:
			desired_angle = desired_angle - np.pi*2
		elif desired_angle < -np.pi:
			desired_angle = desired_angle + np.pi*2
		current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
		desired_orientation = np.array([-np.cos(desired_angle), 0, -np.sin(desired_angle)])
		cross_start = -np.cross(current_orientation, desired_orientation)[1]
		self.set_robot_state([np.sign(cross_start)*speed, -np.sign(cross_start)*speed], 0)
		if info:
			return cross_start

	def get_block_state(self, pickup=False):
		colour = self.robot_data[self.robot_id][6]
		coord = self._block_pos_temp

		self._block_col_temp = 0
		self._block_pos_temp = np.array([0,0])

		return coord, colour, pickup
	
	def get_colour(self):
		return self.robot_data[self.robot_id][6]

	def path_possible(self, target):

		current_pos = self.current_position()
		current_pos = (int(current_pos[0]), int(current_pos[1]))
		#obstacle_grid = args['obstacle_grid']
		obstacle_grid = self.driving_grid
		reduced_obstacle_grid = self.reduced_driving_grid

		
		rr, cc = ellipse(target[0] , target[1], 9,9, shape=self.driving_grid.shape)
		for i in range(len(rr)):
			obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
			reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0

		
		rad = self.current_angle()
		robot_state = np.array([current_pos[0], current_pos[1], rad])
		robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
		for i in range(len(self.extent)):
			robot_extent_global[i] = np.rint((transform_local_coords(robot_state, self.extent[i])))
		robot_extent_global = np.transpose(robot_extent_global)
		rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
		for i in range(len(rr)):
			obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
			reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0

		other_bot = convert_to_grid_coords(self.robot_data[(self.robot_id+1)%2][1:3])
		rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 12,12, shape=self.driving_grid.shape)
		for i in range(len(rr)):
			obstacle_grid[bound(rr[i]), bound(cc[i])] = 1
			reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 1
			#obstacle_grid[rr, cc] = 0
			#reduced_obstacle_grid[rr, cc] = 0
		#print(obstacle_grid[args['target'][0], args['target'][1]])
		path_temp = findpath(current_pos, target, obstacle_grid)

		if path_temp:
			return True
		else:
			return False