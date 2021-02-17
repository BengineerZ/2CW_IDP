import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import line
import time

occupancy_grid = np.full((80,80), 0)

occupancy_grid[40][40] = 1

occupancy_grid[30][70] = 1

for i in range(40):
	occupancy_grid[60][i] = 1
# plt.imshow(occupancy_grid)
# plt.show()

def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('%r  %2.2f ms' % \
                  (method.__name__, (te - ts) * 1000))
        return result
    return timed


def bound(coord, a=0, b=79):
	k = min(coord, b)
	k = max(k, a)
	return k

@timeit
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





# class robot_manager:

# 	def __init__(self, robot_id):
# 		self.id = 0
# 		self.state = ['idle', {}]
# 		self.set_state('idle')
# 		print('initialise')

# 	def __call__(self):
# 		print('called')
# 		print(self.state[0])
# 		getattr(self, self.state[0])(self.state[1] if len(self.state[1]) > 0 else None)
		

# 	def idle(self, args):
# 		print('....')

# 	def sweep(self, args):
# 		print(args['speed'])
# 		print(args['angle'])
		
# 	def set_state(self, state_function, **args):
# 		self.state[0] = state_function
# 		self.state[1] = args


	

# robot_0 = robot_manager()
# robot_0()
# for i in range(3):
	
# 	robot_0.set_state(0, 'sweep', speed = 2, angle = np.pi/2)
# 	robot_0()

#blocks0 = np.array([[72, 32],[1, 30],[55, 61],[72, 4]])

#blocks_set = np.array([[55, 60, 0],[1, 30, 1]])

def combine_coord_sets(blocks, new_blocks):
	# allows for order preservation as well as minor deviations in block position

	combined_set = blocks
	for i in range(len(new_blocks)):
		dist = []
		for j in range(len(blocks)):
			dist.append(np.linalg.norm(new_blocks[i] - blocks[j][0:2]))

		if min(dist) >  4:
			combined_set = np.concatenate((combined_set, [np.append(new_blocks[i], 0)]), axis=0)
		elif min(dist) <= 4:
			combined_set[dist.index(min(dist))][0:2] = new_blocks[i]

	return combined_set








# current_position = gps.getValues()
# current_orientation = np.array([compass.getValues()[2], 0, compass.getValues()[0]])

# #convert current_position to grid val
# x = int(current_position[0] // cellwidth)
# z = int(current_position[2] // cellwidth)
# row = 40 - x
# col = 40 + z

# start = (row, col)
# sgrid, path = pathfinder.findpath(start, Target, grid)

# next_pos = path[1]

# new_heading_vec = np.subtract(next_pos,start)
# new_heading_vec = np.array([-new_heading_vec[0], 0, new_heading_vec[1]])

# dot = current_orientation.dot(new_heading_vec)
# cross = np.cross(current_orientation, new_heading_vec)[1]

# leftSpeed = pd*dot - pc*cross
# rightSpeed = pd*dot + pc*cross

# occupancy_grid = pad_grid(occupancy_grid, 3)

# plt.imshow(occupancy_grid)
# plt.show()

from collections import deque



class overall_state_manager:

	def __init__(self):
		self.task_list = deque([['go to target', {'target': (20,20)}], ['idle', {'grip': 0}]])
		
	def __call__(self, args):
		# for task in self.task_list:
		# 	print(task[0])
		# 	print(task[1])
		determine_next_state(args)

	def get_next_task(self, current_state):
		if thing2:
			return self.task_list.popleft()
		else:
			return None

	def add_to_queue(self, task, args, position=0):
		self.task_list.insert([task, args])

	def thing:
		return True
		return False

manager = overall_state_manager()

class robot:
	def __init__(self):
		self.state = 'idle'

	def __call__(self):
		print(self.state)

		if self.state == 'go to target':
			self.set_state('idle')

	def set_state(self, state):
		if state:
			print('state set to : ' + state)
			self.state = state
		else:
			print('cannot set state, robot occupied')

bot = robot()

for i in range(10):
	print(i)

	

	
	if i == 5:

		bot.set_state(manager.get_next_task(bot.state)[0])

	# if i == 7:
	# 	bot.set_state(manager.get_next_task()[0])


	bot()
	manager()
	print('\n')



		
		