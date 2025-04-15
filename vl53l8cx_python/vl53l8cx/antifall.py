import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from .api import *
from .data_collect import *
from McLumk_Wheel_Sports import *
from Raspbot_Lib import Raspbot

bot = Raspbot()
bot.Ctrl_Servo(1, 28)
bot.Ctrl_Servo(2, 0)


import numpy as np
from collections import deque

# 최근 15프레임 중 몇 개가 낙하위험이면 알람을 울릴지
FRAME_WINDOW = 15
TRIGGER_COUNT = 5  # 예: 최근 15프레임 중 5개 이상 위험하면 경고
Threshold = 1000 # 48개 데이터(64개에서 맨 위 16 제외)중 최대값이 1000mm 이상이면 위험 count
# 프레임 결과를 저장할 큐
fall_history = deque(maxlen=FRAME_WINDOW)

def check_data(data):
    return data and len(data) == VL53L8CX_RESOLUTION_8X8

def filter_data(data):
    distance_value = [item["Distance(mm)"] if item["Status"] == 5 else 0 for item in data.values()]
    numpy_value = np.array(distance_value).reshape(8, 8)
    return numpy_value

def detect_fall(filtered_data):
    bottom = filtered_data[2:, :]
    valid_values = bottom[bottom > 0]
    return valid_values.size > 0 and np.max(valid_values) >= Threshold

def antifall(sensor):
    data = sensor.get_data()
    if check_data(data):
        filtered_data = filter_data(data)
        print(filtered_data)

        # 낙하 위험 여부 판단
        danger = detect_fall(filtered_data)
        fall_history.append(1 if danger else 0)

        # 최근 프레임 중 위험 판단이 일정 수 이상이면 경고
        if sum(fall_history) >= TRIGGER_COUNT:
            bot.Ctrl_BEEP_Switch(1)
        else:
            bot.Ctrl_BEEP_Switch(0)

def main():
    sensor = TOFSensor(resolution=VL53L8CX_RESOLUTION_8X8)
    try:
        while True:
            try:
                antifall(sensor)
            except OSError:
                print("I2C Error, Start reinit")
                sensor._initialize_driver()
                antifall(sensor)
    except KeyboardInterrupt:
        bot.Ctrl_BEEP_Switch(0)

if __name__ == "__main__":
    main()
