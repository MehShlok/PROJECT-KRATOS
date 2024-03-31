#!/usr/bin/env python3
import os
import numpy as np
import open3d as o3d
from PIL import Image

# Path to the folder containing point cloud binary files
pointcloud_folder = "/home/ubuntu/Downloads/second_try/second\\/pointcloud_bin"

# Path to the folder containing RGB images
rgb_image_folder = "/home/ubuntu/Downloads/second_try/second\\/rgb"

# Get a list of point cloud binary files in the folder
pointcloud_files = [os.path.join(pointcloud_folder, filename) for filename in os.listdir(pointcloud_folder) if filename.endswith(".bin")]
count=0
# Iterate through each point cloud file and its corresponding RGB image
for pointcloud_file in pointcloud_files:
    # Load point cloud data
    # print (pointcloud_file)
    points = np.fromfile(pointcloud_file, dtype=np.float64)
    num_points = len(points)
    num_trimmed_points = num_points - (num_points % 3)
    trimmed_points = points[:num_trimmed_points]
    reshaped_points = trimmed_points.reshape(-1, 3)
    # Load corresponding RGB image
    image_filename = os.path.splitext(os.path.basename(pointcloud_file))[0] + ".jpeg"
    print (image_filename)
    rgb_image_path = os.path.join(rgb_image_folder, image_filename)
    rgb_image = Image.open(rgb_image_path)
    # Create Open3D point cloud and assign colors from RGB image
    pcd = o3d.geometry.PointCloud()
    
    pcd.points = o3d.utility.Vector3dVector(reshaped_points)
    colors = np.array(rgb_image)  # Normalize RGB values to range [0, 1]
    print(colors)
    num_points = len(colors) // 3
    pcd.colors = o3d.utility.Vector3dVector(colors.reshape(921600,3))

    # Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
    
    
