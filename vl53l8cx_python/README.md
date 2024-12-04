# VL53L8CX_python

Converting VL53L8CX code from C to Python

# VL53L8Cx driver install

vl53l8cx_python 폴더에서
sudo python3 setup.py install

# tof 모듈 설치

tof_ws/src 에서 tof_imager_ros를 복사 후 빌드
colcon build

# tof 모듈 설정

config/sensor_params.yaml 참고

# 테스트

터미널 1
ros2 launch tof_imager_ros tof_imager_launch.py

터미널 2
ros2 lifecycle set /tof_imager activate
ros2 topic list
ros2 topic echo /pointcloud
