#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
from sklearn.decomposition import PCA
from scipy.spatial import cKDTree
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class RealtimeHumanDetectNode(Node):
    def __init__(self):
        super().__init__('realtime_human_detect_node')
        
        self.declare_parameter('x_min', -5.0)
        self.declare_parameter('x_max', 5.0)
        self.declare_parameter('y_min', -5.0)
        self.declare_parameter('y_max', 5.0)
        self.declare_parameter('z_min', -2.0)
        self.declare_parameter('z_max', 3.0)
        self.declare_parameter('voxel_size', 0.05)
        
        self.x_min = self.get_parameter('x_min').get_parameter_value().double_value
        self.x_max = self.get_parameter('x_max').get_parameter_value().double_value
        self.y_min = self.get_parameter('y_min').get_parameter_value().double_value
        self.y_max = self.get_parameter('y_max').get_parameter_value().double_value
        self.z_min = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_max = self.get_parameter('z_max').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        
        self.current_human_clusters = []
        
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10
        )
        self.colored_cloud_pub = self.create_publisher(
            PointCloud2,
            '/human_detection/colored_cloud',
            10
        )
        
        self.timer = self.create_timer(10.0, self.timer_callback)

        self.get_logger().info(f'Human detection region: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}], Z[{self.z_min}, {self.z_max}]')
        self.get_logger().info('Realtime Human Detection Node initialized')
    
    def cloud_callback(self, msg):
        try:
            points = self.pointcloud2_to_array(msg)

            if len(points) == 0:
                self.get_logger().warn('Empty point cloud received')
                return
            filtered_points = self.filter_points_to_region(points)
            
            if len(filtered_points) > 0:
                human_points_in_region, non_human_points_in_region = self.detect_humans(filtered_points)
            else:
                human_points_in_region, non_human_points_in_region = np.array([]).reshape(0,3), np.array([]).reshape(0,3)
            
            outside_region_mask = ~(
                (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max) &
                (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max) &
                (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
            )
            points_outside_region = points[outside_region_mask]
            
            if len(non_human_points_in_region) > 0 and len(points_outside_region) > 0:
                all_non_human_points = np.vstack([non_human_points_in_region, points_outside_region])
            elif len(non_human_points_in_region) > 0:
                all_non_human_points = non_human_points_in_region
            else:
                all_non_human_points = points_outside_region

            self.publish_colored_cloud(human_points_in_region, all_non_human_points, msg.header)
            
            self.get_logger().debug(f'Total points: {len(points)}, Human points: {len(human_points_in_region)}, Non-human points: {len(all_non_human_points)}')
            
        except Exception as e:
            self.get_logger().error(f'Error in cloud_callback: {str(e)}')
    
    def pointcloud2_to_array(self, cloud_msg):
        points_list = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list)
        
    def filter_points_to_region(self, points):
        mask = (
            (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max) &
            (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max) &
            (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        )
        return points[mask]
    
    def timer_callback(self):

        if len(self.current_human_clusters) == 1:
            human_cluster = self.current_human_clusters[0]
            centroid = np.mean(human_cluster, axis=0)
            
            distance = np.linalg.norm(centroid)

            self.get_logger().info(f'Single human detected - Centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}], Distance from origin: {distance:.3f}m')
    
    def detect_humans(self, points):
        if len(points) < 10:
            self.current_human_clusters = []
            return np.array([]), points
        
        voxels = self.voxelize(points, self.voxel_size)
        
        objects = self.group_voxels(voxels, self.voxel_size)
        
        human_points = []
        non_human_points = []
        human_cluster_count = 0
        human_clusters = []
        
        for obj in objects:
            obj_array = np.array(obj)

            votes = [
                self.shape_classifier(obj_array),
                self.normal_classifier(obj_array),
                self.shadow_classifier(obj_array, points)
            ]

            if self.shape_classifier(obj_array) and (self.shadow_classifier(obj_array, points) or self.normal_classifier(obj_array)):
                human_points.extend(obj)
                human_clusters.append(obj_array)
                human_cluster_count += 1
            else:
                non_human_points.extend(obj)
        
        self.current_human_clusters = human_clusters
        self.get_logger().info(f'Total number of human clusters detected: {human_cluster_count}')
        
        return np.array(human_points), np.array(non_human_points)
    
    def voxelize(self, points, voxel_size):
        voxels = {}
        indices = np.floor(points / voxel_size).astype(int)
        for idx, pt in zip(indices, points):
            key = tuple(idx)
            voxels.setdefault(key, []).append(pt)
        return voxels

    def group_voxels(self, voxels, voxel_size):
        if not voxels:
            return []
        
        centers = np.array([np.mean(v, axis=0) for v in voxels.values()])
        tree = cKDTree(centers)
        labels = [-1] * len(centers)
        current_label = 0
        
        for i in range(len(centers)):
            if labels[i] != -1:
                continue
            queue = [i]
            labels[i] = current_label
            while queue:
                idx = queue.pop()
                neighbors = tree.query_ball_point(centers[idx], voxel_size * 2)
                for n in neighbors:
                    if labels[n] == -1:
                        labels[n] = current_label
                        queue.append(n)
            current_label += 1

        objects = {}
        for label, voxel in zip(labels, voxels.values()):
            objects.setdefault(label, []).extend(voxel)
        return list(objects.values())
    
    def shape_classifier(self, pts):
        if len(pts) < 10:
            return 0
        dims = pts.max(axis=0) - pts.min(axis=0)
        height = dims[2]
        width = max(dims[0], dims[1])
        aspect_ratio = width / height if height > 0 else 1
        return int(1.3 <= height <= 2.0 and aspect_ratio < 1)
    
    def normal_classifier(self, pts):
        if len(pts) < 10:
            return 0
        try:
            pca = PCA(n_components=3)
            pca.fit(pts)
            normal = pca.components_[-1]
            cos_angle = abs(np.dot(normal, np.array([0, 0, 1])))
            return int(cos_angle < 0.80)
        except:
            return 0   
        
    def shadow_classifier(self, pts, all_pts):
        if len(pts) < 5:
            return 0
        centroid = np.mean(pts, axis=0)
        tree = cKDTree(all_pts)
        count = len(tree.query_ball_point(centroid, 0.5))
        return int(count > 200)
    
    def publish_colored_cloud(self, human_points, non_human_points, header):
        if len(human_points) == 0 and len(non_human_points) == 0:
            return
        all_points = []
        
        for point in human_points:
            all_points.append([point[0], point[1], point[2], 
                             self.pack_rgb(255, 0, 0)]) 
        
        for point in non_human_points:
            all_points.append([point[0], point[1], point[2], 
                             self.pack_rgb(0, 0, 255)]) 
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)        ]
        
        cloud_msg = pc2.create_cloud(header, fields, all_points)
        self.colored_cloud_pub.publish(cloud_msg)
    
    def pack_rgb(self, r, g, b):
        return (r << 16) | (g << 8) | b

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeHumanDetectNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
