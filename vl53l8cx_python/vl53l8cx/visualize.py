import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
from collections import deque
import time
from datetime import datetime
from .data_collect import *



sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)



def updatefig(frame, sensor, im, text_objects):
    distance_values = sensor.get_data() # import get_data from data_collect.py
    
    if distance_values:
        distance_value = [0]*64
        zone = [0]*64
        status = [0]*64
        
        for i in range(len(distance_values)):
            zone[i] = distance_values[i]['zone']
            status[i] = distance_values[i]['Status']
            if status[i]==5:
                distance_value[i] = distance_values[i]['Distance(mm)']
            else:
                distance_value[i] = 4000
    
        # Update the distance data grid
        data = np.array(distance_value).reshape(8, 8)
        flipped_data = np.fliplr(data)
        im.set_array(flipped_data)

        # Update the text on each cell with the distance value
        for i in range(8):
            for j in range(8):
                text_objects[i][j].set_text(f"{int(flipped_data[i, j])}")
    else: # if get no data show the last data
        print('Failed to get new data, reinit sensor')
        sensor._initialize_driver()


    
    return [im] + [text for row in text_objects for text in row]


def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    cmap = 'plasma'
    norm = plt.Normalize(vmin=0, vmax=4000)
    fig, ax = plt.subplots(figsize=(6, 6))
    im = ax.imshow(np.zeros((8, 8)), cmap=cmap, norm=norm)
    plt.colorbar(im, label="Distance (mm)")
    ax.set_xticks(np.arange(7,-1,-1))
    ax.set_yticks(np.arange(8))
    ax.set_xticklabels(np.arange(8))
    ax.set_yticklabels(np.arange(8))
    ax.set_title("8x8 Distance Zone")

    # Create text objects to display distance on each grid cell
    text_objects = [[ax.text(j, i, "", ha="center", va="center", color="white", fontsize=8) for j in range(8)] for i in range(8)]

    # Set up the animation
    anim = animation.FuncAnimation(fig, updatefig, fargs=(sensor, im, text_objects), interval=1000/15, blit=True, cache_frame_data=False)

    plt.show()

if __name__ == "__main__":
    main()

