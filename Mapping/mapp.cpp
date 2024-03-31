#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>


// decalre pointer for storing point cloud data representing segmented obstacles
pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_obstacles;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Apply VoxelGrid and PassThrough filters to the point cloud downsample the point cloud and limit the z-axis range 
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01); // Sample leafsize values ~ adjust accordingly
    voxel_grid.filter(*filtered_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0); // Sample limits ~ needs to be adjusted
    pass.filter(*filtered_cloud);

    // Perform obstacle segmentation using EuclideanClusterExtraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // Adjust cluster tolerance as needed
    ec.setMinClusterSize(100);  // Minimum points required for a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    ec.extract(cluster_indices);

    segmented_obstacles.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Loop through cluster_indices to work with individual obstacles
    for (const pcl::PointIndices& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        for (const int& index : indices.indices) {
            obstacle->push_back((*filtered_cloud)[index]);
        }

        // Merge obstacle point clouds
        *segmented_obstacles += *obstacle;
    }
}

void publishOccupancyGrid(ros::Publisher& map_publisher)
{
    if (!segmented_obstacles) {
        return;
    }

    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = "map"; // set fram id

    // Set map resolution and dimensions
    occupancy_grid.info.resolution = 0.05; 
    occupancy_grid.info.width = 1000; 
    occupancy_grid.info.height = 1000;

    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, -1); // Initialize all cells as unknown

    for (const pcl::PointXYZ& point : *segmented_obstacles) {
        // Convert 3D point to 2D grid coordinates
        int x = static_cast<int>(point.x / occupancy_grid.info.resolution) + occupancy_grid.info.width / 2;
        int y = static_cast<int>(point.y / occupancy_grid.info.resolution) + occupancy_grid.info.height / 2;

        // Ensure the cell is within map boundaries
        if (x >= 0 && x < occupancy_grid.info.width && y >= 0 && y < occupancy_grid.info.height) {
            int index = x + y * occupancy_grid.info.width;
            occupancy_grid.data[index] = 100; 
        }
    }

    map_publisher.publish(occupancy_grid);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_obstacle_detection");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud_topic", 1, pointCloudCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);

    
    ros::Rate loop_rate(10); 

    while (ros::ok()) {
        ros::spinOnce();
        publishOccupancyGrid(map_publisher);
        loop_rate.sleep();
    }

    return 0;
}
