#!/home/ben/anaconda3/bin/python3.8
from controller import Robot, Motor
from controller import GPS
from controller import Emitter
import struct
import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import line

occupancy_grid = np.full((80,80), 0.5)

def transform_local_coords(robot_position, local_coordinate):
	# robot_position = [x, y, theta]
	# local coordinate = [x', y']

	rot_matrix = np.array([[np.cos(robot_position[2]), np.sin(robot_position[2])], [-np.sin(robot_position[2]), np.cos(robot_position[2])]])
	offset_vector = np.array([robot_position[0],robot_position[1]])
	
	return np.matmul(rot_matrix, local_coordinate) + offset_vector

def compute_bresenham(start, end):
	pass

def bound(coord, a=0, b=79):
	k = min(coord, b)
	k = max(k, a)
	return k

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



test_position = [10,10, np.pi/2]
local_coordinate = [0,5]

print(transform_local_coords(test_position, local_coordinate))

#test 
TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# get the motor devices
leftMotor = robot.getDevice('wheel1')
rightMotor = robot.getDevice('wheel2')
# set the target position of the motors
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

ps = []
psNames = [
	'ds_right', 'ds_left'
]

for i in range(2):
	ps.append(robot.getDevice(psNames[i]))
	ps[i].enable(TIME_STEP)

main_gps = robot.getDevice('gps_main')
main_gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)


emitter = robot.getDevice('emitter')
#.enable(TIME_STEP)

# read sensors outputs

n = 0




fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
img = ax1.imshow(occupancy_grid, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")
fig.canvas.draw()   # note that the first draw comes before setting data 
# cache the background
axbackground = fig.canvas.copy_from_bbox(ax1.bbox)
plt.show(block=False)



while robot.step(TIME_STEP) != -1:
	psValues = []
	for i in range(2):
		psValues.append(ps[i].getValue())
	#print(psValues)
	#print(main_gps.getValues())


	


	n = n+1

	#print(psValues[0])
	
	rad = np.arctan2(compass.getValues()[0],compass.getValues()[2])
	
	if n%50 == 1:
		leftMotor.setVelocity(-2)
		rightMotor.setVelocity(2)
	elif n%50 == 25:
		leftMotor.setVelocity(2)
		rightMotor.setVelocity(-2)
	# elif n%50 == 30:
	# 	leftMotor.setVelocity(-2)
	# 	rightMotor.setVelocity(2)
	


	#print(rad)	

	test_position = [main_gps.getValues()[2],main_gps.getValues()[0], rad]
	local_coordinate1 = [-psValues[0]*0.707,psValues[0]*0.707]
	local_coordinate2 = [psValues[1]*0.707,psValues[1]*0.707]

	coord1 = np.rint((transform_local_coords(test_position, local_coordinate1)*[100, -100] + np.array([119, 119]))/3)
	coord2 = np.rint((transform_local_coords(test_position, local_coordinate2)*[100, -100] + np.array([119, 119]))/3)

	#print(coord1)
	#occupancy_grid[bound(int(coord1[1]))][bound(int(coord1[0]))] = 1
	#occupancy_grid[bound(int(coord2[1]))][bound(int(coord2[0]))] = 1

	occupancy_grid[bound(int(coord1[1]))][bound(int(coord1[0]))] = update_occupied(occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], p=0.8)
	occupancy_grid[bound(int(coord2[1]))][bound(int(coord2[0]))] = update_occupied(occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], p=0.8)


	current_pos = np.rint((np.array([test_position[0], test_position[1]])*[100, -100] + np.array([119, 119]))/3)

	rr1, cc1, updates1 = update_free(occupancy_grid, [bound(int(coord1[1])),bound(int(coord1[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.2)
	occupancy_grid[rr1, cc1] = updates1

	rr2, cc2, updates2 = update_free(occupancy_grid, [bound(int(coord2[1])),bound(int(coord2[0]))], [bound(int(current_pos[1])), bound(int(current_pos[0]))], p=0.2)
	occupancy_grid[rr2, cc2] = updates2


	#rr1, cc1 = line(bound(int(current_pos[1])), bound(int(current_pos[0])), bound(int(coord1[1])), bound(int(coord1[0])))

	#occupancy_grid[rr1[:-1], cc1[:-1]] = 0

	#rr2, cc2 = line(bound(int(current_pos[1])), bound(int(current_pos[0])), bound(int(coord2[1])), bound(int(coord2[0])))

	#occupancy_grid[rr2[:-1], cc2[:-1]] = 0

	img.set_data(occupancy_grid)
	fig.canvas.restore_region(axbackground)
	# redraw just the points
	ax1.draw_artist(img)
	# fill in the axes rectangle
	fig.canvas.blit(ax1.bbox)
	fig.canvas.flush_events()


	if n == 200:
		# occupancy_grid[occupancy_grid >= 0.6] = 1
		# occupancy_grid[occupancy_grid < 0.6] = 0
		print(occupancy_grid.flatten())
		print(len(occupancy_grid.flatten().tostring()))
		message = struct.pack('51200s', occupancy_grid.flatten().tostring())
		#message = np.array2string(occupancy_grid.flatten(), separator=',')
		emitter.send(message)
		#plt.show(block=True)
		