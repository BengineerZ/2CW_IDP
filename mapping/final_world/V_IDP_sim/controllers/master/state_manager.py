#!/home/ben/anaconda3/bin/python3.8
import numpy as np
from pathfinder import h, escape_path
from utils import *

# =================================================================================
#block picker 


def robot_at_start(robot):
	row,col = robot.current_position()
	if robot.robot_id == 0 and row <13 and col > 67:
		return True
	elif robot.robot_id == 1 and row <13 and col <13:
		return True
	else:
		return False 

def near_wall(block_pos):
	if block_pos[0] <6 or block_pos[0] > 72 or block_pos[1] < 6 or block_pos[1] > 72:
		return True
	else:
		return False

class robot_state_manager:
	def __init__(self, master_robot):
		
		self.master_robot = master_robot
		self.blue_current_target = (0,0)
		self.red_current_target = (0,0)

		# Each task tree node has a name, name of next node, name of alt. next node and a condition
		# Condition is a lambda function with arguments env and robot
		# If it evaluates to True, go to next node, if False, go to alt. next node
		self.task_tree = {}
		
		self.task_tree["start"] = {
			"next": "sweep",
			"next_alt": "pre_approach",
			"condition": lambda env, robot: self.optimum_block(robot, env) == None
		}
		
		self.task_tree["sweep"] = {
			"next": "pre_approach",
			"next_alt": "return_home",
			"condition": lambda env, robot: self.optimum_block(robot, env) != None and self.master_robot.getTime() < 270
			#"condition": lambda env, robot: True

		}
		
		self.task_tree["go_to_block"] = {
			"next": "start_to_block",
			"next_alt": "block_to_block",
			"condition": lambda env, robot: robot_at_start(robot)
		}

		self.task_tree["start_to_block"]= {
			"next": "approach_block",
			"next_alt": "start",
			"condition": lambda env, robot: self.optimum_block(robot, env) != None
		}

		self.task_tree["block_to_block"]= {
			"next": "approach_block",
			"next_alt": "start",
			"condition": lambda env, robot: self.optimum_block(robot, env) != None
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
			"next_alt": "start",
			"condition": lambda env, robot: robot.get_colour() != 0
		}
		
		self.task_tree["return"] = {
			"next": "return_spin",
			"next_alt": "",
			"condition": lambda env, robot: True
		}

		self.task_tree["return_spin"] = {
			"next": "return_push",
			"next_alt": "",
			"condition": lambda env, robot: True
		}

		self.task_tree["return_push"] = {
			"next": "start_reverse",
			"next_alt": "",
			"condition": lambda env, robot: True
		}

		self.task_tree["start_reverse"] = {
			"next": "start",
			"next_alt": "return_home",
			"condition": lambda env, robot: self.master_robot.getTime() < 270
		}
		self.task_tree["final_check"] = {
			"next": "final_sweep",
			"next_alt": "",
			"condition": lambda env, robot: True
		}
		self.task_tree["final_sweep"] = {
			"next": "return_home",
			"next_alt": "start",
			"condition": lambda env, robot: self.optimum_block(robot, env) == None
		}
		self.task_tree["return_home"] = {
			"next": "sweep",
			"next_alt": "explore",
			"condition": lambda env, robot: (env.total_blocks_detected >= 8 and env.total_blocks_removed == env.total_blocks_detected and env.red_dropped_off >= 4 and env.blue_dropped_off >= 4) or self.master_robot.getTime() > 270
		}
		self.task_tree["timeout"] = {
			"next": "return",
			"next_alt": "",
			"condition": lambda env, robot: True
		}
		self.task_tree["explore"] = {
			"next": "sweep",
			"next_alt": "",
			"condition": lambda env, robot: True
		}
		self.task_tree['pre_approach'] = {
			"next": "go_to_block",
			"next_alt": "start",
			"condition": lambda env, robot: self.optimum_block(robot, env) != None
		}
		self.task_tree['blocked'] = {
			"next": "pre_approach",
			"next_alt": "start",
			"condition": lambda env, robot: self.optimum_block(robot, env) != None
		}

		self.task_tree['test'] = {
			"next": "",
			"next_alt": "",
			"condition": lambda env, robot: True
		}


		
		self.current_task = "start"
		self.last_update_time = self.master_robot.getTime()
	
	def make_robot_state_from_task(self, env, robot):
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
			block = self.optimum_block(robot, env)
			if block:
				if robot.robot_id ==0:
					self.red_current_target = block
				else:
					self.blue_current_target = block
				return ["go_to_target", {"target": np.array([int(block[0]), int(block[1])]), "early_stop": 5, "grip": 0, "block": True, "empty": True, "speed": 4}]
			else:
				return ["idle", {}]

		elif self.current_task == "block_to_block":
			block = self.optimum_block(robot, env)
			if block:
				if robot.robot_id ==0:
					self.red_current_target = block
				else:
					self.blue_current_target = block
				return ["go_to_target", {"target": np.array([int(block[0]), int(block[1])]), "early_stop": 5, "grip": 0, "block": True, "empty": True, "speed": 4}]
			else:
				return ["idle", {}]

		elif self.current_task == "approach_block":
			### CHANGE
			#block = self.optimum_block(robot, env)
			if robot.robot_id ==0:
				targ = self.red_current_target
			else:
				targ = self.blue_current_target
			return ["block_extent_routine", {"target": np.array([int(targ[0]), int(targ[1])]), 'env' : env}]
			
		elif self.current_task == "test_color":
			return ["block_update_routine", {"grip":0, "env": env}]

		elif self.current_task == "reverse":
			return ["go_to_target", {"target": robot._block_pos_temp, "early_stop": 5, "grip": 0, "speed": 1, "block": True, "empty": True, 'away': -1, 'optimise_heading': False}]

		elif self.current_task =="grab_block":
			return ["block_update_routine", {"grip":1, "env": env}]

		elif self.current_task =="grab_block2":
			if near_wall(robot._block_pos_temp):
				return ["go_to_target", {"target": robot._block_pos_temp, "early_stop": 2, "grip": 1, "block": True, "empty": False}]
			else:
				return ["go_to_target", {"target": robot._block_pos_temp, "early_stop": 1, "grip": 1, "block": True, "empty": False}]

		elif self.current_task == "return":
			if robot.robot_id == 0:
				self.red_current_target = (7,73)
				return ["go_to_target", {"target": (7, 73), "early_stop": 5, "grip": 1, "block": True, "empty": True, "look_at": False, 'optimise_heading': False, 'speed': 1}]
			else:
				self.blue_current_target = (7,7)
				return ["go_to_target", {"target": (7, 7), "early_stop": 5, "grip": 1, "block": True, "empty": True, "look_at": False, 'optimise_heading': False, 'speed': 1}]

		elif self.current_task == "return_spin":
			if robot.robot_id == 0:
				self.red_current_target = (7,73)
				return ["go_to_target", {"target": (7, 73), "early_stop": 4, "grip": 1, "block": False, "empty": True, "look_at": True}]
			else:
				self.blue_current_target = (7,7)
				return ["go_to_target", {"target": (7, 7), "early_stop": 4, "grip": 1, "block": False, "empty": True, "look_at": True}]

		elif self.current_task == "return_push":
			if robot.robot_id == 0:
				env.red_dropped_off += 1
				self.red_current_target = (7,73)
				return ["go_to_target", {"target": (7, 73), "early_stop": 2, "grip": 0, "block": False, "empty": True, "look_at": True}]
			else:
				env.blue_dropped_off += 1
				self.blue_current_target = (7,7)
				return ["go_to_target", {"target": (7, 7), "early_stop": 2, "grip": 0, "block": False, "empty": True, "look_at": True}]
		
		elif self.current_task == "start_reverse":
			if robot.robot_id ==0:
				return ["go_to_target", {"target": (7, 73), "early_stop": 5, "grip": 0, "speed": 1, "block": True, "empty": True, 'away': -1}]
			else:
				return ["go_to_target", {"target": (7, 7), "early_stop": 5, "grip": 0, "speed": 1, "block": True, "empty": True, 'away': -1}]

		elif self.current_task == "final_check":
			if robot.robot_id == 0:
				return ["go_to_target", {"target": (20, 40), "early_stop": 1, "grip": 0, "block": False, "empty": True}]
			else:
				return ["go_to_target", {"target": (60, 40), "early_stop": 1, "grip": 0, "block": False, "empty": True}]

		elif self.current_task == "final_sweep":
			return ["sweep", {"start": -np.pi/2 +0.5, "end": np.pi - 0.5, "speed": 0.5, "n": 2}]
		
		elif self.current_task == "return_home":
			if robot.robot_id == 0:
				self.red_current_target = (10,70)
				return ["go_to_target", {"target": (10, 70), "early_stop": 1, "grip": 0, "block": True, "empty": False}]
			else:
				self.blue_current_target = (10,10)
				return ["go_to_target", {"target": (10, 10), "early_stop": 1, "grip": 0, "block": True, "empty": False}]
		elif self.current_task == 'explore':
			print('detected ' + str(env.total_blocks_detected))
			print(env.total_blocks_removed)
			if robot.robot_id == 0:
				self.red_current_target = (20,60)
				#red_current_target = environment.explore_uncertainty()
				return ["go_to_target", {"target": (int(self.red_current_target[0]), int(self.red_current_target[1])), "early_stop": 20, "grip": 0, "block": False, "empty": False}]
			else:
				point = self.explore_coord(env.explore_uncertainty(),robot.driving_grid)
				self.blue_current_target = [int(point[0]), int(point[1])]
				return ["go_to_target", {"target": self.blue_current_target, "early_stop": 1, "grip": 0, "block": False, "empty": False}]
		elif self.current_task == "pre_approach":
			block = self.optimum_block(robot, env)
			targ, danger = env.optimum_heading([block[0],block[1]], convert_to_grid_coords(robot.robot_data[(robot.robot_id+1)%2][1:3]))
			if danger < 6:
				return ["idle", {}]
			else:
				print('danger!! - performing intermediate step')
				if robot.robot_id == 0:
					self.red_current_target= (int(targ[0]), int(targ[1]))
				else:
					self.blue_current_target = (int(targ[0]), int(targ[1]))
				return ["go_to_target", {"target": (int(targ[0]), int(targ[1])), "early_stop": 1, "grip": 0, "block": True, "empty": True, 'look_at': False, "speed": 4}]

		elif self.current_task == 'test':
			if robot.robot_id == 0:
				self.red_current_target = (40,20)
				return ["go_to_target", {"target": (40, 20), "early_stop": 1, "grip": 0, "block": True, "empty": False, 'away': 1, 'optimise_heading': False}]
			else:
				self.blue_current_target = (40,60)
				return ["blocked", {}]
				#return ["go_to_target", {"target": (40, 60), "early_stop": 1, "grip": 0, "block": True, "empty": False, 'away': 1, 'optimise_heading': False}]

		if self.current_task == "blocked":
			return ["idle", {}]




#red_bot.set_state('go_to_target', target = second_block, early_stop = 5, grip = 0, block= True, empty=False)
	
	def update_current_task(self, env, robot):
		condition_value = self.task_tree[self.current_task]["condition"](env, robot) # Evaluate condition function
		#print("aaaa", len(env.blocks))
		if condition_value == True:
			self.current_task = self.task_tree[self.current_task]["next"]
		else:
			self.current_task = self.task_tree[self.current_task]["next_alt"]
	
	def update_state(self, env, robot):
		if self.master_robot.getTime() - self.last_update_time > 30:
			self.current_task = "timeout"
		#print(master_robot.getTime() - self.last_update_time)
			
		# If busy, do not interrupt
		# TODO: restart if robot times out (takes too long to do task)
		if robot.state[0] == "idle":
			self.last_update_time = self.master_robot.getTime()
			self.update_current_task(env, robot)
			state = self.make_robot_state_from_task(env, robot)
			print("State manager says: ", self.current_task, "state: ", state)
			robot.set_state(state[0], state[1])
		elif robot.state[0] == 'blocked':
			print('blocked state, choosing new block')
			self.current_task = "blocked"
			self.update_current_task(env, robot)
			state = self.make_robot_state_from_task(env, robot)
			print("State manager says: ", self.current_task, "state: ", state)
			robot.set_state(state[0], state[1])
		else:
			pass

	def choose_block(self, block_list, robot, factor = 0.5, env=None):

		cur_pos = robot.current_position()
		new_block_list = []
		for block in block_list:
			p = (block[0],block[1])
			dis = h(p, cur_pos)
			if env:
				_, danger = env.optimum_heading([block[0],block[1]], convert_to_grid_coords(robot.robot_data[(robot.robot_id+1)%2][1:3]))
			else:
				danger = 0

			print('dis: ' + str(dis))
			print('danger: ' + str(danger))

			if robot.robot_id ==0:
				other_target = self.blue_current_target
			else:
				other_target = self.red_current_target
			if h(other_target, p) <20:
				continue
			if robot.robot_id + block[2] ==2:
				new_block_list.append([block[0],block[1],(dis+danger*2)*factor])
			elif block[2] ==0:
				new_block_list.append([block[0], block[1], dis+danger*2])
		if new_block_list == []:
			return None
		optblock = min(new_block_list, key=lambda x: x[2])
		return (optblock[0],optblock[1])

	def optimum_block(self, robot, env):

		block_list = env.blocks

		if len(block_list) == 1:
			if robot.robot_id ==0 and env.red_dropped_off < env.blue_dropped_off and env.blue_dropped_off > 2:
				return (block_list[0][0], block_list[0][1])
			elif robot.robot_id == 1 and env.red_dropped_off > env.blue_dropped_off and env.red_dropped_off > 2:
				return (block_list[0][0], block_list[0][1])
			elif env.red_dropped_off == env.blue_dropped_off:
				pass
			else:
				return None

		new_block_list = []
		cur_pos = robot.current_position()
		for block in block_list:
			p = (block[0],block[1])
			block_position = (block[0], block[1])

			if robot.robot_id ==0:
				other_target = self.blue_current_target
			else:
				other_target = self.red_current_target

			if h(other_target, p) <20:
				continue
			if not(robot.path_possible([int(block[0]),int(block[1])])):
				continue
			else:
				distance = h(p, cur_pos)
				other_robot_distance = h(convert_to_grid_coords(robot.robot_data[(robot.robot_id+1)%2][1:3]), p)
				danger_coord, danger = env.optimum_heading([block[0],block[1]], convert_to_grid_coords(robot.robot_data[(robot.robot_id+1)%2][1:3]))

				if danger >= 6:
					if not(robot.path_possible([int(danger_coord[0]),int(danger_coord[1])])):
						print('No danger path')
						continue


				combined_heuristic = distance*2 + -other_robot_distance*1 + danger*1 - h(other_target, p)
				print(combined_heuristic)

				if robot.robot_id + block[2] ==2:
					new_block_list.append([block[0],block[1],(combined_heuristic)*0.3])
				elif block[2] ==0:
					new_block_list.append([block[0], block[1], combined_heuristic])
		if new_block_list == []:
			return None
		optblock = min(new_block_list, key=lambda x: x[2])
		return (optblock[0],optblock[1])

	def explore_coord(self, cur_exp, obstacle_grid):
		x, y = cur_exp.astype(int)
		print(cur_exp)
		if obstacle_grid[x,y] ==0:
			return cur_exp
		else:
			path = escape_path((x, y), obstacle_grid)
			return path[-1]