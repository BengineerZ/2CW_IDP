import sys
packages = [r'C:\\Users\\sahil\\Documents\\Python_environments\\env', r'C:\\Users\\sahil\\Documents\\Python_environments\\env\\lib\\site-packages']
for path in packages:
    sys.path.append(path)
from controller import Robot
from controller import Emitter
from controller import Receiver
import numpy as np
import struct
import matplotlib.pyplot as plt
from skimage.draw import line, rectangle, rectangle_perimeter, ellipse, polygon
from skimage.feature import blob_dog
from skimage.filters import gaussian
from pathfinder import findpath, h
from math import hypot
import scipy.interpolate as si
from scipy.signal import find_peaks


TIME_STEP = 64

master_robot = Robot()

receiver = master_robot.getDevice('receiver')
emitter = master_robot.getDevice('emitter')
receiver.enable(TIME_STEP)

emitter.setChannel(Emitter.CHANNEL_BROADCAST)
receiver.setChannel(Receiver.CHANNEL_BROADCAST)

### robot_extent:

#robot_extent = np.mgrid[-3:4, 6:-7:-1].reshape(2, -1).T

robot_extent = np.array([[6, -4],[-7, -4],[-7, 4],[6, 3]])
#print(robot_extent)


blue_current_target = (0,0) 
red_current_target = (0,0)


### comms functionality:

def initiate_data_transfer():
    message = struct.pack('i', 1)
    emitter.send(message)
    #print('initiate')

def set_robot_state(robot_id, wheel_v, gripper):
    message = struct.pack('iddi', robot_id, np.clip(wheel_v[0], -10,10), np.clip(wheel_v[1],-10,10),  gripper)
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
def robot_motion(state, target, pd, pc, info=False):

    current_orientation = np.array([-np.cos(state[2]), 0, -np.sin(state[2])])
    current_position = convert_to_grid_space(state[0:2])
    
    heading_vector = np.subtract(target,current_position)
    heading_vector = np.array([heading_vector[0], 0, -heading_vector[1]])

    dot = current_orientation.dot(heading_vector)
    cross = -np.cross(current_orientation, heading_vector)[1]

    leftSpeed = pd*dot + pc*cross
    rightSpeed = pd*dot - pc*cross

    if info:
        return leftSpeed, rightSpeed, cross
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
        self.reduced_driving_grid = np.full((80,80), 0)
        self.current_path = []
        self._block_found_temp = False


        # state utility vars
        self._spin = False
        self._sweep_counter = 0

        self._block_pos_temp = np.array([0,0])
        self._angle_index = 0
        self._dist_array = []
        self._block_col_temp = 0
        self.carrying = False
        

    def __call__(self):

        return getattr(self, self.state[0])(self.state[1] if len(self.state[1]) > 0 else None)

        
        
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
                set_robot_state(self.robot_id, [0,0], args['grip'])
            else:
                set_robot_state(self.robot_id, [0,0], 0)
        else:
            set_robot_state(self.robot_id, [0,0], 0)

    def blocked(self, args):
        if args:
            if 'grip' in args:
                set_robot_state(self.robot_id, [0,0], args['grip'])
            else:
                set_robot_state(self.robot_id, [0,0], 0)
        else:
            set_robot_state(self.robot_id, [0,0], 0)

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
            set_robot_state(self.robot_id, [np.sign(cross_start)*args['speed'], -np.sign(cross_start)*args['speed']], 0)
        else:
            set_robot_state(self.robot_id, [np.sign(cross_end)*args['speed'], -np.sign(cross_end)*args['speed']], 0)

        if self._sweep_counter == args['n']:
            self._spin = not(self._spin)
            self._sweep_counter = 0
            self.set_state('idle')

    def go_to_target(self, args):

        if "speed" in args:
            speed = args['speed']
        else:
            speed = 1.5

        if 'look_at' in args:
            look_at = args['look_at']
        else:
            look_at = True

        current_pos = self.current_position()
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
            rad = blue_bot.current_angle()
            robot_state = np.array([current_pos[0], current_pos[1], rad])
            robot_extent_global = np.array([[0,0],[0,0],[0,0],[0,0]])
            for i in range(len(robot_extent)):
                robot_extent_global[i] = np.rint((transform_local_coords(robot_state, robot_extent[i])))
            robot_extent_global = np.transpose(robot_extent_global)
            rr, cc = polygon(robot_extent_global[0], robot_extent_global[1])
            for i in range(len(rr)):
                obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
                reduced_obstacle_grid[bound(rr[i]), bound(cc[i])] = 0
            #obstacle_grid[rr, cc] = 0
            #reduced_obstacle_grid[rr, cc] = 0
        #print(obstacle_grid[args['target'][0], args['target'][1]])
        path_temp = findpath(current_pos, args['target'], obstacle_grid)
        #print(path_temp)
        if path_temp:
            if len(path_temp) > 1:
                path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - args['target'][0])**2 + (current_pos[1] - args['target'][1])**2)/2), 9)
                self.current_path = path
                if np.sign(speed)*len(path) > np.sign(speed)*args['early_stop']:
                        set_robot_state(self.robot_id, robot_motion(robot_data[self.robot_id][1:4], path[1], speed, 1), args['grip'])
                else:
                    if look_at:
                        motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, 1, info=True)
                        set_robot_state(self.robot_id, [np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
                        if np.abs(motion[2]) < 0.1:
                            self.set_state('idle', grip = args['grip'])
                            self.current_path = []
                    else:
                        self.set_state('idle', grip = args['grip'])
                        self.current_path = []
            else:
                self.set_state('idle', grip = args['grip'])
                self.current_path = []
        else:
            #print(args['target'])
            obstacle_count = 0
            if len(self.current_path) > 0:
                for i in range(len(self.current_path) if len(self.current_path) <= 6 else 6):
                    if obstacle_grid[int(self.current_path[i][0]), int(self.current_path[i][1])] == 1:
                        obstacle_count = obstacle_count + 1
            print(obstacle_count)

            if obstacle_count >= 3:
                print('no path possible ... reducing driving grid')

                path_temp = findpath(current_pos, args['target'], reduced_obstacle_grid)
                #print(path_temp)
                if path_temp:
                    if len(path_temp) > 1:
                        path = bspline(np.asarray(path_temp), int(np.sqrt((current_pos[0] - args['target'][0])**2 + (current_pos[1] - args['target'][1])**2)/2), 9)
                        self.current_path = path
                        if np.sign(speed)*len(path) > np.sign(speed)*args['early_stop']:
                                set_robot_state(self.robot_id, robot_motion(robot_data[self.robot_id][1:4], path[1], speed, 1), args['grip'])
                        else:
                            if look_at:
                                motion = robot_motion(self.robot_data[self.robot_id][1:4], args['target'], 0, 1, info=True)
                                set_robot_state(self.robot_id, [np.sign(motion[2])*0.4, -np.sign(motion[2])*0.4], args['grip'])
                                if np.abs(motion[2]) < 0.1:
                                    self.set_state('idle', grip = args['grip'])
                                    self.current_path = []
                            else:
                                self.set_state('idle', grip = args['grip'])
                                self.current_path = []
                    else:
                        self.set_state('idle', grip = args['grip'])
                        self.current_path = []
                else:
                    print('no reduced path possible, state set to: "blocked"')
                    self.set_state('blocked', grip = args['grip'])
                    self.current_path = []

            else:
                print('returning to path')
                #self.set_state('blocked', grip = args['grip'])
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
                self.set_heading(desired_angle, 5)
            else:
                self.set_heading(desired_angle, 15)

            current_orientation = np.array([-np.cos(self.current_angle()), 0, -np.sin(self.current_angle())])
            desired_orientation = np.array([-np.cos(desired_angle), 0, -np.sin(desired_angle)])
            cross_start = -np.cross(current_orientation, desired_orientation)[1]
            if np.abs(cross_start) < 0.01:
                #print(self._angle_index)
                self._angle_index = self._angle_index + 1
                self._dist_array.append(self.robot_data[self.robot_id][5])
        else:

            # rising_mask = ((np.array(self._dist_array[:-1]) < trigger_value) & (np.array(self._dist_array[1:]) > trigger_value))
            # falling_mask = ((np.array(self._dist_array[:-1]) > trigger_value) & (np.array(self._dist_array[1:]) < trigger_value))
            # print(np.flatnonzero(rising_mask) +1)
            # print(np.flatnonzero(falling_mask) +1)
            #print(np.array(self._dist_array)*-1)
            peaks, properties = find_peaks(np.array(self._dist_array)*-1, height=(-0.35, 0), width=3)
            #print(self._dist_array[peaks[0]])

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

                
                cross = self.set_heading(exact_angle, 5, info=True)
                #print(exact_position)
                if np.abs(cross) < 0.04:
                    exact_position = convert_to_grid_coords(transform_local_coords([self.robot_data[self.robot_id][1], self.robot_data[self.robot_id][2], exact_angle], [0,self._dist_array[peaks[0]]]))
                    self._angle_index = 0
                    self._dist_array = []
                    self._block_pos_temp = np.array([int(exact_position[0]), int(exact_position[1])])
                    self.set_state('go_to_target', target = self._block_pos_temp, early_stop = 2, grip = 0, block= True, empty=False, look_at = True, speed = 0.6)


    def block_update_routine(self, args):
        colour = self.robot_data[self.robot_id][6]
        coord = self._block_pos_temp

        #self._block_pos_temp = np.array([0,0])

        if args['grip'] == 1:
            args['env'].update_block((coord, colour, True))
        else:
            args['env'].update_block((coord, colour, False))
        
        self.set_state('idle', grip = args['grip'])

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

    def update_driveable_area(self, driving_grid, reduced_driving_grid):
        self.driving_grid = np.copy(driving_grid)
        self.reduced_driving_grid = np.copy(reduced_driving_grid)
        other_bot = convert_to_grid_coords(robot_data[(self.robot_id+1)%2][1:3])
        rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 12,12, shape=driving_grid.shape)
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
        set_robot_state(self.robot_id, [cross_start*speed, -cross_start*speed], 0)
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

    occupancy_grid = gaussian(occupancy_grid, sigma = 2.5)
    occupancy_grid[occupancy_grid > 0.04] = 1
    occupancy_grid[occupancy_grid <= 0.04] = 0
    
    return occupancy_grid


class environment_manager:

    def __init__(self):
        print('initialise environment')
        self.occupancy_grid = np.full((80,80), 0.5)
        self.detected_blocks = []
        self.blocks = np.array([])
        #self.block_dict = 

    def __call__(self):
        #print('occupancy_grid update')

        self.combine_coord_sets()

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

        final_grid = block_padding(self.detected_blocks, final_grid)

        # rr, cc = ellipse(int(other_bot[0]) , int(other_bot[1]), 9,9, shape=final_grid.shape)
        # for i in range(len(rr)):
        # 	final_grid[bound(rr[i]), bound(cc[i])] = 1

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

                rr, cc = ellipse(target[0] , target[1], 4,4)
                for i in range(len(rr)):
                    environment.occupancy_grid[bound(rr[i]), bound(cc[i])] = 0.2


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
            print('intialise block set')
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

# =================================================================================
#block picker 
def choose_block(block_list, robot, factor = 0.5):
    cur_pos = robot.current_position()
    new_block_list = []
    for block in block_list:
        p = (block[0],block[1])
        dis = h(p, cur_pos)
        if robot.robot_id ==0:
            other_target = blue_current_target
        else:
            other_target = red_current_target
        if h(other_target, p) <15:
            continue
        if robot.robot_id + block[2] ==2:
            new_block_list.append([block[0],block[1],dis*factor])
        elif block[2] ==0:
            new_block_list.append([block[0], block[1], dis])
    if new_block_list == []:
        return None
    optblock = min(new_block_list, key=lambda x: x[2])
    return (optblock[0],optblock[1])

def robot_at_start(robot):
    row,col = robot.current_position()
    if robot.robot_id == 0 and row <13 and col > 67:
        return True
    elif robot.robot_id == 1 and row <13 and col <13:
        return True
    else:
        return False 

class robot_state_manager:
    def __init__(self):
        # All available tasks
        #self.available_tasks = ["start", "sweep", "go_to_block", "approach_block", "test_colour_and_grab", "return"]
        
        # Each task tree node has a name, name of next node, name of alt. next node and a condition
        # Condition is a lambda function with arguments env and robot
        # If it evaluates to True, go to next node, if False, go to alt. next node
        self.task_tree = {}
        
        self.task_tree["start"] = {
            "next": "sweep",
            "next_alt": "go_to_block",
            "condition": lambda env, robot: choose_block(env.blocks,robot) == None
        }
        
        self.task_tree["sweep"] = {
            "next": "go_to_block",
            "next_alt": "return_home",
            "condition": lambda env, robot: choose_block(env.blocks,robot) != None
        }
        
        self.task_tree["go_to_block"] = {
            "next": "start_to_block",
            "next_alt": "block_to_block",
            "condition": lambda env, robot: robot_at_start(robot)
        }

        self.task_tree["start_to_block"]= {
            "next": "approach_block",
            "next_alt": "",
            "condition": lambda env, robot: True
        }

        self.task_tree["block_to_block"]= {
            "next": "approach_block",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        
        self.task_tree["approach_block"] = {
            "next": "test_color",
            "next_alt": "block_to_block",
            "condition": lambda env, robot: robot._block_found_temp 
        }
        
        self.task_tree["test_color"] = {
            "next": "grab_block",
            "next_alt": "reverse",
            "condition": lambda env, robot: robot.get_colour() + robot.robot_id == 2
        }
        self.task_tree["reverse"] = {
            "next": "start",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        
        self.task_tree["grab_block"] = {
            "next": "grab_block2",
            "next_alt": "reverse",
            "condition": lambda env, robot: robot.get_colour() + robot.robot_id ==2
        }
        self.task_tree["grab_block2"] = {
            "next": "return",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        
        self.task_tree["return"] = {
            "next": "start_reverse",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        self.task_tree["start_reverse"] = {
            "next": "start",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        self.task_tree["final_check"] = {
            "next": "final_sweep",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        self.task_tree["final_sweep"] = {
            "next": "return_home",
            "next_alt": "start",
            "condition": lambda env, robot: choose_block(env.blocks, robot) == None
        }
        self.task_tree["return_home"] = {
            "next": "sweep",
            "next_alt": "",
            "condition": lambda env, robot: True
        }
        self.task_tree["timeout"] = {
            "next": "start",
            "next_alt": "",
            "condition": lambda env, robot: True
        }

        
        self.current_task = "start"
        self.last_update_time = master_robot.getTime()
    
    def make_robot_state_from_task(self, env, robot):
        global red_current_target
        global blue_current_target
        if self.current_task == "start":
            return ["idle", {}]

        elif self.current_task == "sweep":
            if robot.robot_id == 0 :
                return ["sweep", {"start": -np.pi/2 +0.5, "end": np.pi - 0.5, "speed": 0.5, "n": 2}]
            else:
                return ["sweep", {"start": np.pi/2 - 0.5, "end": np.pi + 0.5, "speed": 0.5, "n": 2}]

        elif self.current_task == "go_to_block":
            return ["idle", {}]

        elif self.current_task == "start_to_block":
            block = choose_block(env.blocks, robot, 0.5)
            if robot.robot_id ==0:
                red_current_target = block
            else:
                blue_current_target = block
            return ["go_to_target", {"target": np.array([int(block[0]), int(block[1])]), "early_stop": 5, "grip": 0, "block": True, "empty": False}]

        elif self.current_task == "block_to_block":
            block = choose_block(env.blocks, robot, 0.5)
            if robot.robot_id ==0:
                red_current_target = block
            else:
                blue_current_target = block
            return ["go_to_target", {"target": np.array([int(block[0]), int(block[1])]), "early_stop": 5, "grip": 0, "block": True, "empty": True}]

        elif self.current_task == "approach_block":
            block = choose_block(env.blocks, robot, 0.5)
            return ["block_extent_routine", {"target": np.array([int(block[0]), int(block[1])]), 'env' : env}]
            
        elif self.current_task == "test_color":
            return ["block_update_routine", {"grip":0, "env": env}]

        elif self.current_task == "reverse":
            return ["go_to_target", {"target": robot._block_pos_temp, "early_stop": 5, "grip": 0, "speed": -0.5, "block": True, "empty": False}]

        elif self.current_task =="grab_block":
            return ["block_update_routine", {"grip":1, "env": env}]

        elif self.current_task =="grab_block2":
            return ["go_to_target", {"target": robot._block_pos_temp, "early_stop": 1, "grip": 1, "block": True, "empty": False}]

        elif self.current_task == "return":
            if robot.robot_id == 0:
                red_current_target = (7,73)
                return ["go_to_target", {"target": (7, 73), "early_stop": 2, "grip": 1, "block": False, "empty": True, "look_at": False}]
            else:
                blue_current_target = (7,7)
                return ["go_to_target", {"target": (7, 7), "early_stop": 2, "grip": 1, "block": False, "empty": True, "look_at": False}]
        
        elif self.current_task == "start_reverse":
            if robot.robot_id ==0:
                return ["go_to_target", {"target": (7, 73), "early_stop": 5, "grip": 0, "speed": -0.5, "block": True, "empty": False}]
            else:
                return ["go_to_target", {"target": (7, 7), "early_stop": 5, "grip": 0, "speed": -0.5, "block": True, "empty": False}]

        elif self.current_task == "final_check":
            if robot.robot_id == 0:
                return ["go_to_target", {"target": (20, 40), "early_stop": 1, "grip": 0, "block": False, "empty": True}]
            else:
                return ["go_to_target", {"target": (60, 40), "early_stop": 1, "grip": 0, "block": False, "empty": True}]

        elif self.current_task == "final_sweep":
            return ["sweep", {"start": -np.pi/2 +0.5, "end": np.pi - 0.5, "speed": 0.5, "n": 2}]
        
        elif self.current_task == "return_home":
            if robot.robot_id == 0:
                red_current_target = (10,70)
                return ["go_to_target", {"target": (10, 70), "early_stop": 1, "grip": 0, "block": True, "empty": False}]
            else:
                blue_current_target = (10,10)
                return ["go_to_target", {"target": (10, 10), "early_stop": 1, "grip": 0, "block": True, "empty": False}]



#red_bot.set_state('go_to_target', target = second_block, early_stop = 5, grip = 0, block= True, empty=False)
    
    def update_current_task(self, env, robot):
        condition_value = self.task_tree[self.current_task]["condition"](env, robot) # Evaluate condition function
        #print("aaaa", len(env.blocks))
        if condition_value == True:
            self.current_task = self.task_tree[self.current_task]["next"]
        else:
            self.current_task = self.task_tree[self.current_task]["next_alt"]
    
    def update_state(self, env, robot):
        if master_robot.getTime() - self.last_update_time > 30:
            self.current_state = "timeout"
            
        # If busy, do not interrupt
        # TODO: restart if robot times out (takes too long to do task)
        if robot.state[0] != "idle" and robot.state[0] != 'blocked':
            return
        
        self.last_update_time = master_robot.getTime()
        self.update_current_task(env, robot)
        state = self.make_robot_state_from_task(env, robot)
        print("State manager says: ", self.current_task, "state: ", state)
        robot.set_state(state[0], state[1])
        #print("Current task: ", self.current_task)
        #print("Condition eval: ", self.task_tree[self.current_task]["condition"](env, robot))
        #robot.set_state('sweep', start = np.pi/2 - 0.5, end = -np.pi +0.5, speed = 0.5)
# =================================================================================


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

red_bot_state_manager = robot_state_manager()
blue_bot_state_manager = robot_state_manager()

#blocks = []

while master_robot.step(TIME_STEP) != -1:

    initiate_data_transfer()
    robot_data, update = await_state_data()

    if len(robot_data) == 2:
        red_bot.robot_data = robot_data
        blue_bot.robot_data = robot_data



        '''
        if n == 10:
            ### main loop 
            #blue_bot.set_state('go_to_target', target = (40,40), obstacle_grid = test, early_stop = 5, grip = 0)
            blue_bot.set_state('sweep', start = np.pi/2 - 0.5, end = -np.pi +0.5, speed = 0.5, n=1)
            red_bot.set_state('sweep', start = -np.pi/2 + 0.5, end = np.pi - 0.5, speed = 0.5, n=1)

        if n == 200:
            first_block = np.array([int(environment.blocks[0][0]), int(environment.blocks[0][1])])
            second_block = np.array([int(environment.blocks[1][0]), int(environment.blocks[1][1])])
            blue_bot.set_state('go_to_target', target = first_block, early_stop = 5, grip = 0, block= True, empty=False)
            red_bot.set_state('go_to_target', target = second_block, early_stop = 5, grip = 0, block= True, empty=False)
            print(first_block)
        if n == 350:
            #blue_bot.set_state('go_to_target', target = (5,5), obstacle_grid = blue_bot.driving_grid, early_stop = 2, grip = 1, block= False, empty=True)
            blue_bot.set_state('block_extent_routine', target = first_block)
            red_bot.set_state('block_extent_routine', target = second_block)

        if n == 500:
            environment.update_block(blue_bot.get_block_state(pickup=False))
            environment.update_block(red_bot.get_block_state(pickup=False))
            #blue_bot.set_state('go_to_target', target = blue_bot._block_pos_temp, obstacle_grid = blue_bot.driving_grid, early_stop = 2, grip = 0, block= True, empty=False, state='idle')
            pass
        #if n == 520:
            #blue_bot.set_state('block_colour_pickup_routine')

        # 	blue_bot.set_state('go_to_target', target = blue_bot._block_pos_temp, obstacle_grid = blue_bot.driving_grid, early_stop = 1, grip = 1, block= True, empty=False, state='idle')

        # 	rr, cc = ellipse(int(blue_bot._block_pos_temp[0]) , int(blue_bot._block_pos_temp[1]), 4,4)
        # 	for i in range(len(rr)):
        # 		environment.occupancy_grid[bound(rr[i]), bound(cc[i])] = 0.2

        # if n == 600:
        # 	blue_bot.set_state('go_to_target', target = (8,8), obstacle_grid = blue_bot.driving_grid, early_stop = 1, grip = 1, block= False, empty=True, state='idle')


        # if n > 700:
        # 	blue_bot.set_state('go_to_target', target = (8,8), obstacle_grid = blue_bot.driving_grid, early_stop = 2, grip = 0, block= False, empty=True)'''
            


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



        #print(f"blue_bot_State: {blue_bot.state[0]}, angle: {blue_bot.current_angle()}")
        
        test = np.copy(blue_bot.driving_grid)
        for i in range(len(blue_bot.current_path)):
            test[int(blue_bot.current_path[i][0]), int(blue_bot.current_path[i][1])] = 0.5

        driving_grid, reduced_driving_grid = environment.driving_grid()
        blue_bot.update_driveable_area(driving_grid, reduced_driving_grid)
        red_bot.update_driveable_area(driving_grid, reduced_driving_grid)

        occupancy_grid, _ = environment.find_blocks()
        environment.update_binary_occupancy_grid(robot_data)
        
        #print(blocks)
        print("====")
        print("Blue bot current task: ", blue_bot_state_manager.current_task)
        print("Blue bot state: ", blue_bot.state)
        print("Blue Current target: ", blue_current_target)
        print("Red bot current task: ", red_bot_state_manager.current_task)
        print("Red bot state: ", red_bot.state)
        print("Red Current target: ", red_current_target)
        print("Blocks found: \n", environment.blocks)

        environment()
        red_bot()
        blue_bot()
    
        red_bot_state_manager.update_state(environment, red_bot)
        blue_bot_state_manager.update_state(environment, blue_bot)


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
        
        



