import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
import struct


class PCDToCSVNode(Node):
    def __init__(self):
        super().__init__('pcd_to_csv_node')
        self.declare_parameter('pcd_file', '/mnt/e/IITH/LiDAR/fastlio_ws/src/unilidar_fastlio_ros2/PCD/scans.pcd')
        self.declare_parameter('csv_file', 'scans.csv')
        pcd_file = self.get_parameter('pcd_file').get_parameter_value().string_value
        csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.convert(pcd_file, csv_file)
    
    def convert(self, pcd_file, csv_file):
        with open(pcd_file, 'rb') as f:
            while True:
                line = f.readline().decode('ascii').strip()
                if line.startswith('DATA'):
                    break
            point_data = []
            while True:
                data = f.read(32)
                if len(data) < 32:
                    break
                values = struct.unpack('ffffffff', data)
                point_data.append(values)
        
        points_array = np.array(point_data)
        df = pd.DataFrame({
            'x': points_array[:, 0],
            'y': points_array[:, 1],
            'z': points_array[:, 2],
            'intensity': points_array[:, 3]
        })

        df.to_csv(csv_file, index = False)
        self.get_logger().info(f"Converted {pcd_file} to {csv_file}")

    
def main(args = None):
    rclpy.init(args=args)
    node = PCDToCSVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()