import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from collections import deque

from std_msgs.msg import Bool
from Raspbot_Lib import Raspbot

FRAME_WINDOW = 15
TRIGGER_COUNT = 5
THRESHOLD = 1000

class AntiFallNode(Node):
    def __init__(self):
        super().__init__('antifall_subscriber')
        self.bot = Raspbot()
        self.bot.Ctrl_Servo(1,28)
        self.bot.Ctrl_Servo(0,0)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'tof_data',
            self.listener_callback,
            10
        )
        self.fall_history = deque(maxlen=FRAME_WINDOW)

    def listener_callback(self, msg):
        try:
            data = np.array(msg.data).reshape((8, 8)).astype(int)
            bottom = data[2:, :]  # 하단 6행 
            valid_values = bottom[bottom > 0]
            danger = valid_values.size > 0 and np.max(valid_values) >= THRESHOLD
            self.fall_history.append(1 if danger else 0)

            self.get_logger().info(f"Danger: {danger}, Fall History: {list(self.fall_history)}")

            # 낙하 위험 판단
            alert = sum(self.fall_history) >= TRIGGER_COUNT

            if alert:
                self.get_logger().warn("🚨 Fall Risk Detected! Published Beep ON")
                self.get_logger().info(f"{data}")
                self.bot.Ctrl_BEEP_Switch(1) # msg : if it detects fall danger 1 else 0
                
            else:
                self.get_logger().info("✅ Safe. Published Beep OFF")
                self.bot.Ctrl_BEEP_Switch(0)

        except Exception as e:
            self.get_logger().error(f"Error processing ToF data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AntiFallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
