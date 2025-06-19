import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
import os

class DistCalcNode(Node):
    def __init__(self):
        super().__init__('dist_calc_node')
        self.declare_parameter('csv_file', '/mnt/e/IITH/LiDAR/lidar_ws/human_cluster.csv')
        self.csv_path = self.get_parameter('csv_file').get_parameter_value().string_value

        if not os.path.exists(self.csv_path):
            self.get_logger().error('CSV file doesnt exist')
            return
        
        self.calculate_distance_from_csv(self.csv_path)

    def calculate_distance_from_csv(self, csv_path):
        df = pd.read_csv(csv_path)
        points = df[['x', 'y', 'z']].values

        centroid = np.mean(points, axis = 0)
        self.get_logger().info(f"Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})")

        distance_to_origin = np.linalg.norm(centroid)
        self.get_logger().info(f"Euclidean distance to centroid: {distance_to_origin:.3f}")


def main(args = None):
    rclpy.init(args = args)
    node = DistCalcNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
