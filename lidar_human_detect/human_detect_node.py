import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial import cKDTree
import open3d as o3d
import os

class HumanDetectNode(Node):
    def __init__(self):
        super().__init__('human_detect_node')

        self.declare_parameter('csv_file', '/mnt/e/IITH/LiDAR/lidar_ws/scans.csv')
        self.declare_parameter('output_file', 'human_cluster.csv')
        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        if not os.path.exists(self.csv_file):
            self.get_logger().error(f"Input CSV file {self.csv_file} does not exist.")
            return

        self.detect_and_visualize()

    def detect_and_visualize(self):
        df = pd.read_csv(self.csv_file)

        # Crop bounds (hardcoded for now)
        df = df[df['x'].between(0, 3)]
        df = df[df['y'].between(-0.5, 0.5)]
        df = df[df['z'].between(0.05, 5)]

        points = df[['x', 'y', 'z']].values

        voxels = self.voxelize(points, 0.05)
        objects = self.group_voxels(voxels, 0.05)

        human_points = []
        non_human_points = []

        for obj in objects:
            votes = [
                self.shape_classifier(obj),
                self.normal_classifier(obj),
                self.shadow_classifier(obj, points)
            ]
            if sum(votes) > 2:
                human_points.extend(obj)
            else:
                non_human_points.extend(obj)

        if human_points:
            human_df = pd.DataFrame(human_points, columns=['x', 'y', 'z'])
            human_df.to_csv(self.output_file, index=False)
            self.get_logger().info(f"Saved {len(human_points)} human points to {self.output_file}")
        else:
            self.get_logger().warn("No human points detected to save.")

        self.visualize_point_cloud(np.array(human_points), np.array(non_human_points))

    def voxelize(self, points, voxel_size):
        voxels = {}
        indices = np.floor(points / voxel_size).astype(int)
        for idx, pt in zip(indices, points):
            key = tuple(idx)
            voxels.setdefault(key, []).append(pt)
        return voxels

    def group_voxels(self, voxels, voxel_size):
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
                neighbors = tree.query_ball_point(centers[idx], voxel_size * 1.5)
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
        pts = np.array(pts)
        if len(pts) < 10:
            return 0
        dims = pts.max(axis=0) - pts.min(axis=0)
        height = dims[2]
        width = max(dims[0], dims[1])
        aspect_ratio = width / height if height > 0 else 1
        return int(1.3 <= height <= 2.5 and aspect_ratio < 1.2)

    def normal_classifier(self, pts):
        pts = np.array(pts)
        if len(pts) < 10:
            return 0
        pca = PCA(n_components=3)
        pca.fit(pts)
        normal = pca.components_[-1]
        cos_angle = abs(np.dot(normal, np.array([0, 0, 1])))
        return int(cos_angle < 0.95)

    def shadow_classifier(self, pts, all_pts):
        centroid = np.mean(pts, axis=0)
        tree = cKDTree(all_pts)
        count = len(tree.query_ball_point(centroid, 0.5))
        return int(count > 20)

    def visualize_point_cloud(self, human_pts, non_human_pts):
        if human_pts.size == 0 and non_human_pts.size == 0:
            self.get_logger().warn("No points to display.")
            return

        if human_pts.size == 0:
            human_pts = np.zeros((0, 3))
        if non_human_pts.size == 0:
            non_human_pts = np.zeros((0, 3))

        all_points = np.vstack([human_pts, non_human_pts])
        colors = np.vstack([
            np.tile([1.0, 0.0, 0.0], (len(human_pts), 1)),  # red for human
            np.tile([0.0, 0.0, 1.0], (len(non_human_pts), 1))  # blue for non-human
        ]).astype(np.float32)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



