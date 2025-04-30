# Anti-Fall Detection System (ROS 2 Package)

This package, `antifall`, is designed to detect falls using ToF sensor data VL53L8CX. 
It alerts buzzer when fall is detected.

---

## 🛠️ How to Run

```bash
cd ~/antifall
colcon build
source install/setup.bash
ros2 launch antifall antifall_launch.py
