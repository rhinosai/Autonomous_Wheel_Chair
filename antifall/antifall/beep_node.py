import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from Raspbot_Lib import Raspbot

class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.subscription = self.create_subscription(
            Bool,
            'beep_cmd',
            self.beep_callback,
            10
        )
        self.bot = Raspbot()
        self.get_logger().info("🚨 BeepNode started and waiting for alerts...")

    def beep_callback(self, msg): 
        self.bot.Ctrl_BEEP_Switch(msg.data) # msg : if it detects fall danger 1 else 0
        if msg.data:
            self.get_logger().warn("🔊 BEEP ON")
        else:
            self.get_logger().info("🔇 BEEP OFF")

def main(args=None):
    rclpy.init(args=args)
    node = BeepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()