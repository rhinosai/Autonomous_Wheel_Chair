import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
from .data_collect import *
from McLumk_Wheel_Sports import *
from Raspbot_Lib import Raspbot
bot = Raspbot()
bot.Ctrl_Servo(1, 28)
bot.Ctrl_Servo(2, 45)



def check_data(data):
    if data:
        return len(data) == VL53L8CX_RESOLUTION_8X8
    

# filter_data : status 5인 유효한 값만 필터링. 그 이외의 값은 4000으로 설정정


def filter_data(data):
    distance_value = [item["Distance(mm)"] if item["Status"] == 5 else 4000 for item in data.values()]
    numpy_value = np.array(distance_value).reshape(8, 8)
    return numpy_value


# classify_data : 8*8 ToF 영역에서 왼쪽열 3열을 left_area, 오른쪽열 3열을 right_area로 설정
# 거리에 따라 action을 return


def classify_data(filtered_data):
    left_area = filtered_data[:,:3]
    right_area = filtered_data[:,5:]

    #left_min = np.min(left_area)
    #right_min = np.min(right_area)

    left_mean = np.mean(left_area)
    right_mean = np.mean(right_area)
    
    overall_min = np.min(filtered_data)

    if overall_min <= 300:
        
        if left_mean > right_mean:
            return "turn_left"
        
        else:
            return "turn_right"
        
    elif overall_min <= 1000:
        return "slow_down"
    
    else:
        return "go_straight"
    
# get_direction: action들을 모터로 조향향


def get_direction(control_action):
    if control_action == "turn_left":         
        rotate_left(speed=20)

    elif control_action == "turn_right":
        rotate_right(speed=20)

    elif control_action == "slow_down":
        move_forward(speed=10)
    
    elif control_action == "go_straight":
        move_forward(speed=30)
    
    elif control_action == 'go_back':
        move_backward(speed=10)
    
# auto_drive : ToF 데이터만으로 장애물을 피해 주행행

def auto_drive(sensor):
    data = sensor.get_data()
    if check_data(data):
            filtered_data = filter_data(data)
            print(np.min(filtered_data))
            control_action = classify_data(filtered_data)
            print(control_action)
            get_direction(control_action)


# main : I2C 에러가 발생하면 센서를 reinit. ctrl+c로 멈춤춤

def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    try:
        while True:
                try:
                    auto_drive(sensor)
                except OSError:
                    print("I2C Error, Start reinit")
                    sensor._initialize_driver()
                    auto_drive(sensor)
    except KeyboardInterrupt:
        print("Finished autodrive")
        move_forward(speed=0)

                    

if __name__ == "__main__":
    main()
