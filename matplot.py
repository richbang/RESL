import time
import threading

import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

import matplotlib.image as mpimg

# Set the figure dimensions
fig = plt.figure()
ax = plt.axes(xlim=(0,441*3), ylim=(0,445*3))

# Plot an image containing the room layout
img = mpimg.imread('C:/Users/OWNER/Desktop/hall3.png')
#imgplot = ax.imshow(img, extent=(-500, 4301-500, -500, 3987-500))
imgplot = ax.imshow(img, extent=(0, 441*3, 0, 445*3))
file = open("C:/Users/OWNER/Desktop/rtls_log/lg.txt", 'r')
x_point, y_point = file.readline().split(" ")[0], file.readline().split(" ")[1].rstrip('\n')
#print(x_point, y_point)

ax.scatter(np.array(x_point), np.array(y_point), c='blue')
scatter = ax.scatter(0, 0, c='red')

def our(particles):
    global x_point, y_point
    x_point, y_point = file.readline().split(" ")[0], file.readline().split(" ")[1].rstrip('\n')
    print(x_point, y_point)
    data = np.array((x_point, y_point))
    # list_x.append(file.readline())
    # list_y.append(file.readline())
    scatter.set_offsets(data)

    return scatter,

anim = FuncAnimation(fig, our, interval = 1,blit=True)

plt.show()

file.close()
anim.save('C:/Users/OWNER/Desktop/animation.mp4')