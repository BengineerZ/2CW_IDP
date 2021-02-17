#!/home/ben/anaconda3/bin/python3.8
import numpy as np
import matplotlib.pyplot as plt

### General utility function defs:
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

### Visualiser Class:
class visualiser:
	def __init__(self, num_of_displays):
		self.num_of_displays = num_of_displays
		self.fig = plt.figure()
		placeholder = np.full((80,80), 0)
		for i in range(num_of_displays):
			exec('self.ax{} = self.fig.add_subplot(1, {}, {})'.format(i, self.num_of_displays, i+1))
			exec('self.display{} = self.ax{}.imshow(np.full((80,80), 0), vmin=-1, vmax=1, interpolation="None", cmap="RdBu")'.format(i, i))
			self.fig.canvas.draw()
			exec('self.axbackground{} = self.fig.canvas.copy_from_bbox(self.ax{}.bbox)'.format(i, i))
		plt.show(block = False)

	def __call__(self, *args):
		if len(args) != self.num_of_displays:
			raise ValueError('displays and display number must match')
		else:
			for i in range(self.num_of_displays):
				exec('self.display{}.set_data(args[{}])'.format(i, i))
				exec('self.fig.canvas.restore_region(self.axbackground{})'.format(i))
				exec('self.ax{}.draw_artist(self.display{})'.format(i,i))
				exec('self.fig.canvas.blit(self.ax{}.bbox)'.format(i))
				self.fig.canvas.flush_events()
