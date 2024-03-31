#!/usr/bin/env python3
import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sensor_msgs.msg import PointCloud2
import std_msgs.msg

# Step 2: Extract Point Cloud Data from ROS Bag
bag_file = '2023-11-16-23-34-46.bag'
pointcloud_topic = '/zed2i/zed_node/point_cloud/cloud_registered'

bag = rosbag.Bag(bag_file)

x_coords = []
y_coords = []
z_coords = []

for topic, msg, t in bag.read_messages(topics=[pointcloud_topic]):
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    for point in pc_data:
        x_coords.append(point[0])
        y_coords.append(point[1])
        z_coords.append(point[2])

bag.close()

# Step 4: Create Elevation Map
resolution = 0.1
x_range = np.arange(min(x_coords), max(x_coords), resolution)
y_range = np.arange(min(y_coords), max(y_coords), resolution)

hist, x_edges, y_edges = np.histogram2d(x_coords, y_coords, bins=[x_range, y_range], weights=z_coords)
elevation_map = hist.T

# Step 3: Publish Elevation Map as PointCloud2
rospy.init_node('elevation_map_publisher', anonymous=True)
elevation_map_topic = '/elevation_map'
elevation_map_pub = rospy.Publisher(elevation_map_topic, PointCloud2, queue_size=10)

elevation_map_msg = pc2.create_cloud_xyz32(header=std_msgs.msg.Header(), xyz=np.column_stack((x_coords, y_coords, z_coords)))
elevation_map_pub.publish(elevation_map_msg)

# Step 5: Visualize in RViz
rospy.sleep(1)  # wait for RViz to initialize
rospy.spin()
