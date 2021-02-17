#!/home/ben/anaconda3/bin/python3.8
from controller import Robot, Motor
from controller import GPS
from controller import Emitter
from controller import Camera
import struct
import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import line

#test 
TIME_STEP = 64

# create the Robot instance.
robot = Robot()

gripper_left_motor = robot.getDevice("Lmotor")
gripper_right_motor = robot.getDevice("Rmotor")

# gripper_right_motor.setMinPosition(0)
# gripper_right_motor.setMaxPosition(0.9)
# gripper_left_motor.setMinPosition(0)
# gripper_left_motor.setMaxPosition(0.9)

gripper_right_motor.setAvailableTorque(0.2)
gripper_left_motor.setAvailableTorque(0.2)

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
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)

colour_sensor = robot.getDevice('camera')
colour_sensor.enable(TIME_STEP)

robot_name = robot.getName()
robot_id = 0

if robot_name == 'red':
	robot_id = 0
	emitter.setChannel(0)
	receiver.setChannel(0)
else:
	robot_id = 1
	emitter.setChannel(1)
	receiver.setChannel(1)

n = 0

def send_robot_state(robot_id, state, distance, colour):
	message = struct.pack('idddddi', robot_id, state[0], state[1], state[2], distance[0], distance[1], colour)
	emitter.send(message)

def grip(grip_pos):
	if grip_pos == 1:
		gripper_left_motor.setPosition(0.1)
		gripper_right_motor.setPosition(0.1)
	else:
		gripper_left_motor.setPosition(0.9)
		gripper_right_motor.setPosition(0.9)

grip(0)

while robot.step(TIME_STEP) != -1:

	psValues = []
	for i in range(2):
		psValues.append(ps[i].getValue())
	
	rad = np.arctan2(compass.getValues()[0],compass.getValues()[2])

	global_position = [main_gps.getValues()[2],main_gps.getValues()[0], rad]

	image = colour_sensor.getImageArray()
	red   = image[0][0][0]
	green = image[0][0][1]
	blue  = image[0][0][2]
	gray  = (red + green + blue) / 3
	
	if red > 1.5*gray:
		#print('red')
		detected_colour = 2
	elif blue > 1.5*gray:
		detected_colour = 1
		#print('blue')
	else:
		detected_colour = 0
		#print('nothing')

	queue_length = receiver.getQueueLength()
	#print(queue_length)

	while queue_length > 0:
		message = receiver.getData()
		size = receiver.getDataSize()
		if size == 4:
			#print(robot_name + ' - received')
			dataList=struct.unpack("i",message)
			if dataList[0] == 1:
				send_robot_state(robot_id, global_position, psValues, detected_colour)
		else:
			dataList=struct.unpack("iddi",message)
			if dataList[0] == robot_id:
				leftMotor.setVelocity(dataList[1])
				rightMotor.setVelocity(dataList[2])
				grip(dataList[3])
		receiver.nextPacket()
		queue_length = receiver.getQueueLength()

	n = n+1
		