import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
import json
from collections import deque
import time
from datetime import datetime
import os

# driver = VL53L8CX(bus_id=0)

# alive = driver.is_alive()
# if not alive:
#     raise IOError("VL53L8CX Device is not alive")

# print("Initialising...")
# t = time.time()
# driver.init()
# print(f"Initialised ({time.time() - t:.1f}s)")

file_name= f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

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
                distance_values = {}
                for i in range(self.resolution):
                    status = ranging_data.target_status[self.driver.nb_target_per_zone * i]
                    distance = ranging_data.distance_mm[self.driver.nb_target_per_zone * i]
                    distance_values[i] = {"zone": i,"Status": status,"Distance(mm)": int(distance)}
            
                
                                    
                return distance_values

        
        except OSError as e:
            print(f"I2C error occurred: {e}")
            self._initialize_driver()  
            return []
        
        except Exception as e:
            print(f"Unexpected error: {e}")
            self._initialize_driver()  
            return []
    
    
    def save_data(self,distance_values, file_name = file_name) -> None :  
        data_entry = {"time": datetime.now().isoformat(), "data": distance_values}
    
        try:
            with open(file_name, 'r') as f:
                existing_data = json.load(f)
                
        except (FileNotFoundError, json.JSONDecodeError):
            # 파일이 없거나 비어 있으면 빈 리스트로 초기화
            existing_data = []
        
        if data_entry['data']: # data 가 None 일 경우는 저장하지 않음
            existing_data.append(data_entry)
            
        # 수정된 데이터를 파일에 덮어쓰기
        try:
            with open(file_name, 'w') as f:
                json.dump(existing_data, f)
                
        except KeyboardInterrupt :
            if existing_data:
                existing_data.pop()
            with open(file_name, 'w') as f:
                json.dump(existing_data, f)
            raise


def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    while True:
        distance_values = sensor.get_data()
        sensor.save_data(distance_values,file_name)


if __name__ == "__main__":
    main()