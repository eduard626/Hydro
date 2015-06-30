/**
 * Renders the cloud from sensor.
 */

#include <iostream>
//ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//point cloud includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/registration/icp.h>
//pcl filter includes
#include <pcl/filters/filter.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//pcl::visualization::CloudViewer viewer("Cloud viewer");
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb (const PointCloud::ConstPtr& cloud)
{
    ROS_INFO("Is dense? %d, Width: %d, Height: %d",cloud->is_dense,cloud->width,cloud->height);
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_new, mapping);
    ROS_INFO("Is dense? %d, Width: %d, Height: %d",cloud_new->is_dense,cloud_new->width,cloud_new->height);
  //  viewer.showCloud(cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "viewer");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("input", 1, cloud_cb);

  // Spin
  ros::spin ();
}
