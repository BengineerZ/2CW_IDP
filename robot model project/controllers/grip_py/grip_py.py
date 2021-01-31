from controller import Robot, Motor
import math

TIME_STEP = 32

robot = Robot()


left_motor = robot.getDevice("Lmotor")
right_motor = robot.getDevice("Rmotor")
left_motor.setPosition(0.1)
right_motor.setPosition(0.1)

velocity=[0.1,0.8]

i=0
while robot.step(TIME_STEP) != -1:
    i+=1
    left_motor.setPosition(math.sin((i%50)/50))
    right_motor.setPosition(math.sin((i%50)/50))



