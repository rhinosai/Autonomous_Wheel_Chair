import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
from collections import deque
import time

# Check initialzing time
# driver = VL53L8CX(bus_id=0)

# alive = driver.is_alive()
# if not alive:
#     raise IOError("VL53L8CX Device is not alive")

# print("Initialising...")
# t = time.time()
# driver.init()
# print(f"Initialised ({time.time() - t:.1f}s)")

class TOFSensor:
    def __init__(self, resolution=VL53L8CX_RESOLUTION_8X8, frequency=15):
        self.driver = VL53L8CX(bus_id=0)
        self.resolution = resolution
        self.frequency = frequency
        self.res = int(np.sqrt(resolution))
        self.data_queue = deque(maxlen=64)
        self.flag = False
        self._initialize_driver()

    def _initialize_driver(self):
        self.flag = True # start initiating
        self.driver.init()
        self.driver.set_resolution(self.resolution)
        self.driver.set_ranging_frequency_hz(self.frequency)
        self.driver.start_ranging()
        self.flag = False # finish initiating

    def get_data(self):
        if self.flag:
            return []
        """Collect the latest data from the sensor."""
        if self.driver.check_data_ready():
            try:
                ranging_data = self.driver.get_ranging_data()
                for i in range(self.resolution):
                    status = ranging_data.target_status[self.driver.nb_target_per_zone * i]
                    distance = ranging_data.distance_mm[self.driver.nb_target_per_zone * i]

                    data_entry = {
                        "zone": i,
                        "Status": status,
                        "Distance(mm)": int(distance)
                    } 
                    self.data_queue.append(data_entry)
                    with open('data_entries.json', 'a') as f:
                        json.dump(data_entry, f)
                        f.write('\n')
                    #print(data_entry)
                return list(self.data_queue)
            except Exception as e:
                self._initialize_driver() # reinitializing
        return []
    

def updatefig(frame, sensor, im, text_objects):
    data_list = sensor.get_data()
    
    if data_list:
        distance_values = [0] * 64
        for entry in data_list:
            zone = entry['zone']
            distance_values[zone] = entry['Distance(mm)']
    

        # Update the distance data grid
        data = np.array(distance_values).reshape(8, 8)
        flipped_data = np.fliplr(data)
        im.set_array(flipped_data)

        # Update the text on each cell with the distance value
        for i in range(8):
            for j in range(8):
                text_objects[i][j].set_text(f"{int(flipped_data[i, j])}")
    else: # if get no data show the last data
        print('reinit...')
    
    return [im] + [text for row in text_objects for text in row]

def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    cmap = 'plasma'
    norm = plt.Normalize(vmin=0, vmax=2000)
    fig, ax = plt.subplots(figsize=(6, 6))
    im = ax.imshow(np.zeros((8, 8)), cmap=cmap, norm=norm)
    plt.colorbar(im, label="Distance (mm)")
    ax.set_xticks(np.arange(7,-1,-1))
    ax.set_yticks(np.arange(8))
    ax.set_xticklabels(np.arange(7,-1,-1))
    ax.set_yticklabels(np.arange(8))
    ax.set_title("8x8 Distance Grid")

    # Create text objects to display distance on each grid cell
    text_objects = [[ax.text(j, i, "", ha="center", va="center", color="white", fontsize=8) for j in range(8)] for i in range(8)]

    # Set up the animation
    anim = animation.FuncAnimation(fig, updatefig, fargs=(sensor, im, text_objects), interval=1000/15, blit=True)

    plt.show()

if __name__ == "__main__":
    main()

      
