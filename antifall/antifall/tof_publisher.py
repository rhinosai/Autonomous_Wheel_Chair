import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from vl53l8cx.vl53l8cx import VL53L8CX
from vl53l8cx.api import *
from vl53l8cx.data_collect import *
import time

class ToFPublisher(Node):
    def __init__(self,node_name='tof_publisher'):
        super().__init__(node_name)
        # parameter.yaml
        self.declare_parameters(
        namespace='',
        parameters=[
            ('bus_id', 0),
            ('frame_id', 'tof_frame'),
            ('resolution', 8),
            ('mode', 1), # 1 is continuous, 3 is autonomous
            ('ranging_freq', 15),
            ('timer_period', 0.1)
        ])

        self.res = self.get_parameter('resolution').value 
        self.mode = self.get_parameter('mode').value
        self.freq = self.get_parameter('ranging_freq').value
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'tof_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz 주기
        self.sensor = TOFSensor()
        try:
            self.sensor._initialize_driver()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensor: {e}")
            
        self.get_logger().info('Initialized')

    def check_data(self, data):
        return data and len(data) == VL53L8CX_RESOLUTION_8X8

    def filter_data(self, data):
        distance_value = [item["Distance(mm)"] if item["Status"] == 5 else 0 for item in data.values()]
        return np.array(distance_value).reshape(8, 8)
        
    def timer_callback(self):
        try:
            data = self.sensor.get_data()
            if self.check_data(data):
                filtered = self.filter_data(data)
                msg = Float32MultiArray()
                msg.data = filtered.flatten().tolist()
                self.publisher_.publish(msg)
                self.get_logger().debug('Published filtered ToF data.')
            else:
                self.get_logger().debug('Failed passing check_data')
        except:
            self.sensor._initialize_driver()
            


def main(args=None):
    rclpy.init(args=args)
    node = ToFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
