import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from collections import deque

FRAME_WINDOW = 15
TRIGGER_COUNT = 5
THRESHOLD = 1000

class AntiFallNode(Node):
    def __init__(self):
        super().__init__('antifall_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'tof_data',
            self.listener_callback,
            10
        )
        self.alert_publisher = self.create_publisher(Bool, 'beep_cmd', 10)  # 부저 명령용 publisher
        self.fall_history = deque(maxlen=FRAME_WINDOW)

    def listener_callback(self, msg):
        try:
            data = np.array(msg.data).reshape((8, 8)).astype(int)
            bottom = data[2:, :]  # 하단 6행 (48개)
            valid_values = bottom[bottom > 0]
            danger = valid_values.size > 0 and np.max(valid_values) >= THRESHOLD
            self.fall_history.append(1 if danger else 0)

            self.get_logger().info(f"Danger: {danger}, Fall History: {list(self.fall_history)}")

            # 낙하 위험 판단
            alert = sum(self.fall_history) >= TRIGGER_COUNT
            self.alert_publisher.publish(Bool(data=alert))

            if alert:
                self.get_logger().warn("🚨 Fall Risk Detected! Published Beep ON")
            else:
                self.get_logger().info("✅ Safe. Published Beep OFF")

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
