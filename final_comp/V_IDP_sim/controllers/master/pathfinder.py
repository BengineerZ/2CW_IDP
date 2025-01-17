from queue import PriorityQueue
import numpy as np
from math import hypot
from matplotlib import pyplot as plt

# to use this just import this file into your script and use the findpath function which takes 
# in a binary grid of obstacles, start and end grid coordiantes and outputs a list of grid positions which is the path to follow
# you also have the option to turn this into a colorised map which can then be viewed using the visualise_grid function and matplotlib 

#STATES
EMPTY = 0
START = 1
END = 2
OBSTACLE = 3
PATH =4

coldict = {0:(255,255, 255),1:(255,0, 0),2:(0,255, 0),3:(0,0, 0),4:(128,0, 128) } # WHITE, RED, GREEN, BLACk, PURPLE 

neighbors = ((1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
              (1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)))


class Node:
    ''' Node object to store information about a grid cell in the occupancy grid
    attributes:
        int: row - the cell's row.
        int: col - the cell's column.
        float: g - represents the shortest path found to the node from the start position so far.
        float: h - straight line distance from the node to the end position.
        int tuple[2]: parent - grid coordinates of the previous node in the shortest path to this node coordiante so far.
    '''
    def __init__(self, row, col, g=float("inf"), h=0, parent=None):
        self.row = row
        self.col = col
        self.g = g
        self.h = h
        self.parent = parent

    def update(self, new_parent, new_g):
        ''' if a shorter path to this node is found this will update g to reflect the length of the shorter path and the new parent node it has come from
        args:
            int tuple[2]: new_parent - coordinate of the new previous node in the shortest path to this node coordinate so fa
            float: new_g - lenght of new shortest path to this point so far.
        '''
        self.parent = new_parent
        self.g = new_g
    
    def f(self):
        ''' f score which is a metric that combines the shortest path to a point and the straight line distance to the end
        return:
              float: the sum of h and g
        '''
        return self.h + self.g
    
    def get_pos(self):
        '''
            return:
            int: row and column of the node
        '''
        return self.row, self.col
    
    def __eq__(self, other):
        ''' equality operator overided to refelct node object equality based on same grid positon
        return:
            bool: if nodes are equal or not
        ''' 
        return self.row == other.row and self.col == other.col

    def __lt__(self, other):
        ''' less than operator overided to refelct node object size based on the f attribute
        return:
            bool: if one node has a lower f score than the other
        ''' 
        return self.f() < other.f()


def h(p1, p2):
    ''' straight line distance between two points
        args:
            int tuple[2]: p1- coordinates of the first point
            int tuple[2]: p2 - coordinates of the other point
        return:
            float: the straight line distance between the points
    '''
    x1, y1 = p1
    x2, y2 = p2
    return hypot(x2-x1, y2-y1)

def EmptyCheck(row, col, grid, total_rows, total_colums):
    ''' Checks if a grid coordinate can be used as part of a path by checking it is within the grid and that there is no obstacle at this point
        args:
            int : row- the cell's row
            int : col - the cell's column
            numpy array: grid - the occupancy grid of the robot
            int: total_row - the total number of rows in the grid
            int: total_colums - the total number of columns in the grid
        return:
            Bool: if the cell is usable or not for the path
    '''
    if 0 <= row < total_rows and 0 <= col < total_colums:
        return grid[row][col] == 0
    return False 

def findpath(start, end, obstgrid):
    ''' Shortest path between two points for a given grid of obstacles using A*
        args:
            int tuple[2]: start- coordinates of the start point.
            int tuple[2]: end - coordinates of the end point.
            numpy arrary: obstgrid: the occupancy grid of the robot.
        return:
            list: a list of grid coordinates which make up the shortest path from the start to then end.
    '''
    obstgrid = list(obstgrid)
    rows = len(obstgrid)
    colums = len(obstgrid[0])

    grid = []
    for i in range(rows):
        grid.append([])
        for j in range(colums):
            grid[i].append(None)
    
    startNode = Node(start[0], start[1], g=0, h = h(start,end))
    grid[start[0]][start[1]] = startNode
    endNode = Node(end[0],end[1])
    grid[end[0]][end[1]] = endNode

    count =0
    open_set = PriorityQueue()
    open_set.put(startNode)

    found = False
    while not found:

        current = open_set.get()

        for neighbor in neighbors:
            r, c = current.row + neighbor[0], current.col + neighbor[1]
            if not EmptyCheck(r,c,obstgrid, rows, colums):
                continue

            if grid[r][c] is None:
                grid[r][c] = Node(r,c, h= h((r,c),end))
            
            temp_g = grid[current.row][current.col].g + neighbor[2]
            if temp_g < grid[r][c].g:
                grid[r][c].update(current.get_pos(), temp_g)
                open_set.put(grid[r][c])
            if grid[r][c] == endNode:
                found = True
                break
        if open_set.empty():
            return None
        
    path = []
    while True:
        path.append(current.get_pos())
        if current.parent is not None:
            current = grid[current.parent[0]][current.parent[1]]
        else:
            break
    return path[::-1]
        
'''
def produce_grid(start, end, obstacle_grid, path):
    grid = np.array([[0 for i in range(len(obstacle_grid))]for i in range(len(obstacle_grid[0]))])
    for point in path:
        row, column = point
        grid[row][column] = 4
    for i in range(len(obstacle_grid)):
        for j in range(len(obstacle_grid[0])):
            if obstacle_grid[i][j] == 1:
                grid[i][j] = 3
    grid[start[0]][start[1]] = 1
    grid[end[0]][end[1]] = 2

    return grid

def visualise_grid(grid):
    colgrid = []
    for i in range(len(grid)):
        colgrid.append([])
        for j in range(len(grid[0])):
            colgrid[i].append(coldict[grid[i][j]])
    return colgrid
'''

def escape_path(robot_pos, obstacle_grid):
    ''' Shortest path to get to free space from blocked space using Dijkstra.
        args:
            int tuple[2]: robot_pos- coordinates of the point the robot is stuck at
            numpy arrary: obstgrid: the occupancy grid of the robot.
        return:
            list: a list of grid coordinates which make up the shortest path from the blocked position to free space.
    '''
    rows = len(obstacle_grid)
    colums = len(obstacle_grid[0])
    count = 0

    g_grid = np.array([[float("inf") for _ in range(colums)] for _ in range(rows)])
    g_grid[robot_pos[0],robot_pos[1]] = 0
    came_from = np.array([[None for _ in range(colums)] for _ in range(rows)])
    open_set = PriorityQueue()
    open_set.put((0,count, robot_pos))

    found = False
    while not found:
        count +=1
        current = open_set.get()[2]

        for neighbor in neighbors:
            r, c = current[0] + neighbor[0], current[1]+neighbor[1]
            if 0 <= r < rows and 0 <= c < colums:
                temp__g = g_grid[current[0],current[1]] + neighbor[2]
                if temp__g < g_grid[r,c]:
                    came_from[r,c] = current
                    g_grid[r,c] = temp__g
                    open_set.put((g_grid[r,c],count,(r,c)))
                if obstacle_grid[r,c] == 0:
                    current = (r,c)
                    found = True
                    break
        if open_set.empty():
            return None

    path = []
    while current != None:
        path.append(current)
        current = came_from[current[0],current[1]]
    esc_path = path[::-1]

    return esc_path

#TEST CASE

'''
start = (40,40)
end = (5,5)
grid = np.array([[0 for i in range(80)]for i in range(80)])
barrier = [(37-i,23+i) for i in range(20)]
barrier2 = [(38-i,23+i) for i in range(22)]
for point in barrier + barrier2:
    row, y = point
    grid[row,y] = 1
path = findpath(start, end, grid)
bgrid = produce_grid(start, end, grid, path)
colgrid = visualise_grid(bgrid)
plt.imshow(np.array(colgrid))
plt.show()
'''