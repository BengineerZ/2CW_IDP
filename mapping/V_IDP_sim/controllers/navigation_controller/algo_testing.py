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
		
occupancy_grid = pad_grid(occupancy_grid, 3)

plt.imshow(occupancy_grid)
plt.show()