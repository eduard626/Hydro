
//ros includes
#include <ros/ros.h>

#include<pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>

#include <pcl/conversions.h>

#include <pcl/point_types.h>

#include <pcl/io/vtk_lib_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ply_viewer");
  //Node handle
  ros::NodeHandle nh;
  //Setup publisher
  ros::Publisher pub = nh.advertise<PointCloud>("/vertices",1);
  PointCloud::Ptr msg (new PointCloud);
  PointCloud::Ptr cloud_xyz (new PointCloud);
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY("/home/eduardo/test2.ply",mesh);
  pcl::fromPCLPointCloud2(mesh.cloud,*cloud_xyz);
  msg=cloud_xyz;
  msg->header.frame_id="cloud_frame";
  ros::Rate loop_rate(50);
  while(nh.ok())
  {
	  msg->header.stamp=ros::Time::now().toNSec();
	  pub.publish(msg);
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  //pcl::visualization::CloudViewer viewer("Cloud viewer");
  //viewer.showCloud(cloud_xyz);

}
