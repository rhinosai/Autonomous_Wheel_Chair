import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
from collections import deque
import time

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
        if self.flag: # if init completes flag is False
            return []
        try:
            if self.driver.check_data_ready():
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
        
        except OSError as e:
            print(f"I2C error occurred: {e}")
            self._initialize_driver()  
            return []
        except Exception as e:
            print(f"Unexpected error: {e}")
            self._initialize_driver()  
            return []


def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    while True:
        sensor.get_data()


if __name__ == "__main__":
    main()