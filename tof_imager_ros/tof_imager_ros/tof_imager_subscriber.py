# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class ToFImagerSubscriber(Node):
    def __init__(self, node_name='tof_imager_subscriber'):
        super().__init__(node_name)
        self.pcl_sub = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.pcl_callback,
            qos_profile_sensor_data
        )
        
    def pcl_callback(self, msg: PointCloud2):
        self.get_logger().info("PointCloud2 message received.")
        pc_data = read_points(msg)
        print(pc_data)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ToFImagerSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
