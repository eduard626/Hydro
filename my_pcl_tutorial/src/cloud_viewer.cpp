/**
 * Renders the cloud from sensor.
 */

#include <iostream>
//ros includes
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
//point cloud includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/registration/icp.h>
//pcl filter includes
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

pcl::visualization::CloudViewer viewer("Cloud viewer");
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZRGB>);

void cloud_cb (const PointCloud::ConstPtr& cloud)
{
    ROS_INFO("Obtained the point cloud for filtering");
    //*cloud_target = *cloud;

    //passthrough filter taking only point between 0.55-1.0 meters
 //   pcl::PassThrough<pcl::PointXYZRGB> pass;
   // pass.setInputCloud (cloud_target);
  //  pass.setFilterFieldName ("z");
   // pass.setFilterLimits (0.0, 1.0);
 //   pass.filter (*cloud_target);

    //Affine transformation
    Eigen::Affine3f transform=Eigen::Affine3f::Identity();
    transform.translation()<<0.0,0.0,0.0;
    float theta=M_PI;
    transform.rotate(Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitX()));

  //Execute transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud,*transformed_cloud,transform);
    uint8_t r = 0, g = 255, b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    transformed_cloud->at(320,240).rgb = *reinterpret_cast<float*>(&rgb);
    viewer.showCloud(transformed_cloud);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "viewer");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("input", 1, cloud_cb);

  // Spin
  ros::spin ();
}
