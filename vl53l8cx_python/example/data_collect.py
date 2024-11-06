import time
import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
import json

class TOFSensor:
    def __init__(self, resolution=VL53L8CX_RESOLUTION_8X8):
        self.driver = VL53L8CX()
        self.resolution = resolution
        self.res = int(np.sqrt(resolution)) 
        self._initialize_driver()

    def _initialize_driver(self):
        self.driver.init()
        self.driver.set_resolution(self.resolution)
        self.driver.start_ranging()

    def get_coord(self):
        self.driver.set_resolution(self.resolution)
        ranging_data = self.driver.get_ranging_data()
        distance_mm = np.array(ranging_data.distance_mm[:self.resolution]).reshape(self.res, self.res)
        buf = np.empty((self.res, self.res, 3), dtype=np.float32)
        per_px = np.deg2rad(45) / self.res
        it = np.nditer(distance_mm, flags=["multi_index"])
        for e in it:
            w, h = it.multi_index # (w,h) = (0,0) ~ (7,7)
            e = max(e, 0) 
            x = e * np.cos(w * per_px - np.deg2rad(45) / 2 - np.deg2rad(90)) / 1000  # 2d x 좌표
            y = e * np.sin(h * per_px - np.deg2rad(45) / 2) / 1000 # 2d y 좌표
            z = e / 1000 # depth = distance
            buf[w, h] = [x, y, z]
        return buf

    def run(self, num_data=10):
        """Collect data from the ToF sensor and save it to a JSON file."""
        loop = 0
        with open("tof_data.json", "a") as json_file:
            while loop < num_data:
                if self.driver.check_data_ready():  
                    ranging_data = self.driver.get_ranging_data()
                    buf = self.get_coord()  
                    
                    for i in range(self.resolution):
                        status = ranging_data.target_status[self.driver.nb_target_per_zone * i]
                        distance = ranging_data.distance_mm[self.driver.nb_target_per_zone * i]
                        w = i // self.res
                        h = i % self.res
                        x_coord = round(float(buf[w, h, 0]), 5)
                        y_coord = round(float(buf[w, h, 1]), 5)
                        z_coord = round(float(buf[w, h, 2]), 5)

                        print(f"Zone : {i: >3d}, "
                              f"Status : {status: >3d}, "
                              f"Distance : {distance: >4.0f} mm, "
                              f"X: {x_coord}, "
                              f"Y: {y_coord}, "
                              f"Z: {z_coord}")

                        data_entry = {
                            "data no": loop,
                            "zone": i,
                            "Status": status,  # Status indicating the measurement validity (5 & 9 means ranging OK)
                            "Distance(mm)": distance,
                            "X": x_coord,
                            "Y": y_coord,
                            "Z": z_coord
                        }

                        json.dump(data_entry, json_file)
                        json_file.write('\n')
                    
                    loop += 1

if __name__ == '__main__':
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    sensor.run(num_data=1)
    