import time
from matplotlib import pyplot as plt
import numpy as np


def live_update_demo(blit = False):
    occupancy_grid = np.full((80,80), 0.5)
    
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    

    img = ax1.imshow(occupancy_grid, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")


    fig.canvas.draw()   # note that the first draw comes before setting data 


    if blit:
        # cache the background
        axbackground = fig.canvas.copy_from_bbox(ax1.bbox)

    plt.show(block=False)


    t_start = time.time()
    k=0.

    for i in np.arange(1000):
        occupancy_grid[int(i/20)][60] = 1
        img.set_data(occupancy_grid)
        
        
        #print tx
        k+=0.11
        if blit:
            # restore background
            fig.canvas.restore_region(axbackground)
            

            # redraw just the points
            ax1.draw_artist(img)
            

            # fill in the axes rectangle
            fig.canvas.blit(ax1.bbox)
            

            # in this post http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
            # it is mentionned that blit causes strong memory leakage. 
            # however, I did not observe that.

        else:
            # redraw everything
            fig.canvas.draw()

        fig.canvas.flush_events()
        #alternatively you could use
        #plt.pause(0.000000000001) 
        # however plt.pause calls canvas.draw(), as can be read here:
        #http://bastibe.de/2013-05-30-speeding-up-matplotlib.html


live_update_demo(True)   # 175 fps
#live_update_demo(False) # 28 fps