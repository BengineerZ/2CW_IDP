#!/usr/bin/python3
from controller import Robot, Motor
from controller import GPS
from controller import Emitter
import numpy as np

# Constants
l_dist_sensor_angle = -np.pi / 4 # angle of left dist. sensor rel. to robot
r_dist_sensor_angle = np.pi / 4 # angle of right dist. sensor rel. to robot
wheel_radius = 0.04

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get the motor devices
leftMotor = robot.getDevice('wheel1')
rightMotor = robot.getDevice('wheel2')
# set the target position of the motors
leftMotor.setPosition(float("Inf"))
rightMotor.setPosition(float("Inf"))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(-0.0)

main_gps = robot.getDevice('gps_main')
main_gps.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)
emitter = robot.getDevice('emitter')

dist_sensor = [robot.getDevice("ds_left"), robot.getDevice("ds_right")]
for d in dist_sensor:
    d.enable(timestep)

# Main loop:
def approach_test_pickup_block():
    def motors_rotate(dir, speed):
        leftMotor.setPosition(float("Inf"))
        rightMotor.setPosition(float("Inf"))
        leftMotor.setVelocity(dir * speed)
        rightMotor.setVelocity(-dir * speed)
    
    def motors_forward(speed):
        #leftMotor.setPosition(dist / wheel_radius)
        #rightMotor.setPosition(dist / wheel_radius)
        leftMotor.setVelocity(speed)
        rightMotor.setVelocity(speed)
    
    def motors_stop():
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
    
    prev_heading = None
    total_rotation = 0
    
    motor_speed = 0
    rotation_dir = 0 # 1 - clockwise, -1 - counterclockwise, 0 - stop
    dist_travelled = 0
    prev_rotation = 0
    
    distance = [] # distance measurement by left dist_sensor
    l_angle = [] # left dist_sensor heading
    
    dist_threshold = 0.5 # look for blocks within this distance
    edge_threshold = 0.05 # discontinuity in distance measurement must be at least this large to be considered an edge
    approach_dist = 0.15 # approach block within this distance
    width_estimate = 0 # object width estimate
    block_max_width = 0.4
    rising_edge_angle = None
    falling_edge_angle = None
    
    robot_state = "init"
    
    while robot.step(timestep) != -1:
        comp = compass.getValues()
        heading = np.pi + np.arctan2(comp[0], comp[2]) # heading angle from -pi to pi
        if prev_heading == None:
            prev_heading = heading
        if np.abs(heading - prev_heading) < 1.9 * np.pi: # ignore discontinuities in heading angle
            total_rotation += heading - prev_heading
        prev_heading = heading
        
        if robot_state == "init":
            distance = []
            l_angle = []
            width_estimate = 0
            rising_edge_angle = None
            falling_edge_angle = None
            robot_state = "searching_for_block"
        elif robot_state == "searching_for_block":
            # Find an object within a distance of dist_threshold
            motors_rotate(-1, 1.5)
            if dist_sensor[0].getValue() < dist_threshold:
                prev_rotation = total_rotation
                robot_state = "preparing_to_scan_block"
        elif robot_state == "preparing_to_scan_block":
            # Get in position to scan block
            if total_rotation < prev_rotation + 0.1:
                motors_rotate(1, 1.5)
            else:
                motors_stop()
                robot_state = "scanning_block"
        elif robot_state == "scanning_block":
            # Scan block
            motors_rotate(-1, 0.5)
            distance.append(dist_sensor[0].getValue())
            l_angle.append(total_rotation - l_dist_sensor_angle)
            
            if len(l_angle) > 2:
                width_estimate += np.abs(l_angle[-1] - l_angle[-2]) * distance[-1]
            
            if width_estimate > block_max_width:
                print("Detected object is too large to be a block, looking for more objects...")
                robot_state = "init"
            
            if len(distance) > 2 and distance[-1] < dist_threshold and distance[-2] - distance[-1] > edge_threshold:
                rising_edge_angle = total_rotation - l_dist_sensor_angle
                print("Found rising edge at", rising_edge_angle, ", magnitude", distance[-2] - distance[-1])
            if len(distance) > 2 and distance[-2] < dist_threshold and distance[-1] - distance[-2] > edge_threshold:
                falling_edge_angle = total_rotation - l_dist_sensor_angle
                print("Found falling edge at", falling_edge_angle, ", magnitude", distance[-1] - distance[-2])
                
            if rising_edge_angle != None and falling_edge_angle != None:
                print("Detected object width is approx.", width_estimate)
                robot_state = "preparing_for_approach"
        elif robot_state == "preparing_for_approach":
            # Face the block
            if total_rotation < (rising_edge_angle + falling_edge_angle) / 2 + 0.05:
                motors_rotate(1, 1.5)
            else:
                motors_stop()
                robot_state = "approaching_block"
        elif robot_state == "approaching_block":
            # Approach
            dist_target = min(distance) - approach_dist
            speed = np.clip(1000 * (dist_target - dist_travelled) * dist_travelled + 0.1, 0, 5)
            dist_travelled += speed * wheel_radius * (timestep / 1000)
            motors_forward(speed)
            
            if dist_travelled >= dist_target:
                motors_stop()
                robot_state = "testing_color"
        elif robot_state == "testing_color":
            # TODO: test color
            print("Ready to test block color.")
            robot_state = "grabbing_block"
            pass
        elif robot_state == "grabbing_block":
            # TODO: call block pickup routine
            pass
        else:
            print("Robot entered unknown state, restarting... (this should never happen)")
            robot_state = "init"

approach_test_pickup_block()