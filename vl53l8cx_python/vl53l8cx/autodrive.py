import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
from .data_collect import *
from .raspbot.motor_control import *

sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)


# 연결 확인
sensor._initialize_driver()

# data 가져옴, 데이터 확인
data = sensor.get_data()


def check_data():
    return len(data) == VL53L8CX_RESOLUTION_8X8


# data 전처리 : status 5만 남김 else : NaN


def filter_data(data):
    distance_value = [
        item["Distance(mm)"] if item["status"] == 5 else None for item in data
    ]
    numpy_value = np.array(distance_value).reshape(8, 8)
    return np.fliplr(numpy_value)


# data 판단 : 50cm 이내(정지 후 turn around), 1m 이내 (감속), 그 외 => 직진


def classify_data(data):
    filtered_data = filter_data(data)
    if np.min(filtered_data) <= 300:
        return "turn_around"
    elif np.min(filtered_data) <= 1000:
        return "slow_down"
    else:
        return "go_straight"


def auto_drive(control_action):
    if control_action == "turn_around":
        stop_robot()
        time.sleep(0.2)         
        while control_action == "turn_around": # 물체로부터 300mm 이상 멀어질때까지 회전전
            rotate_left(speed=5)
            data = sensor.get_data()  
            if check_data(data):
                valid_data = filter_data(data)
                control_action = classify_data(valid_data)

    elif control_action == "go_straight":
        move_forward(speed=5)

    elif control_action == "slow_down":
        move_forward(speed=1)


def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    sensor._initialize_driver()
    while True:
        data = sensor.get_data()
        if check_data():
            valid_data = filter_data(data)
            control_action = classify_data(valid_data)
            auto_drive(control_action)


if __name__ == "__main__":
    main()
