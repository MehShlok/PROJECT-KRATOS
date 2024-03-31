#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import numpy as np
import cv2
import struct
import os

def read_images(image_folder):
    image_files = [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith('.jpg')]
    point_cloud_data = []
    for image_file in image_files:
        image = cv2.imread(image_file)
        # Convert the image to point cloud data (depth values) using camera calibration parameters
        # For example, assume depth values are in the blue channel of the image
        depth_values = image[:, :, 0].astype(float)  # Extract depth values from blue channel
        # Convert pixel values to 3D coordinates using camera calibration parameters
        # Assuming fx and fy are focal lengths and cx, cy are principal points
        fx, fy, cx, cy = 500, 500, 320, 240  # Adjust these values based on your camera calibration
        points = []
        for y in range(depth_values.shape[0]):
            for x in range(depth_values.shape[1]):
                z = depth_values[y, x]
                if z > 0:  # Non-zero depth indicates a valid point
                    x_ = (x - cx) * z / fx
                    y_ = (y - cy) * z / fy
                    points.append([x_, y_, z])
        point_cloud_data.extend(points)
    return np.array(point_cloud_data, dtype=np.float32)

def read_binary_files(binary_folder):
    binary_files = [os.path.join(binary_folder, f) for f in os.listdir(binary_folder) if f.endswith('.bin')]
    point_cloud_data = []
    for binary_file in binary_files:
        with open(binary_file, 'rb') as f:
            binary_data = f.read()
        # Assuming each point is represented by 3 floats (x, y, z)
        point_size = 3  # number of floats per point
        points = struct.unpack('f' * (len(binary_data) // 4 // point_size), binary_data)
        points_array = np.array(points, dtype=np.float32).reshape(-1, 3)
        point_cloud_data.extend(points_array)
    return np.array(point_cloud_data, dtype=np.float32)

def main():
    rospy.init_node('point_cloud_publisher')
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

    # Read data from folders
    image_folder = '/home/ubuntu/Downloads/second_try/second\\/rgb/'
    binary_folder = '/home/ubuntu/Downloads/second_try/second\\/pointcloud_bin'
    image_point_cloud = read_images(image_folder)
    binary_point_cloud = read_binary_files(binary_folder)

    # Combine point cloud data from both sources
    combined_point_cloud = np.concatenate((image_point_cloud, binary_point_cloud), axis=0)

    # Define PointCloud2 message
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'  # Adjust the frame_id as needed

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    pc2_msg = pc2.create_cloud_xyz32(header, combined_point_cloud)
    
    rate = rospy.Rate(10)  # Publish at 10 Hz
    while not rospy.is_shutdown():
        pub.publish(pc2_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
