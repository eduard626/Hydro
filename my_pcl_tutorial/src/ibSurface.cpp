
//ros includes
#include <ros/ros.h>

#include<pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>

#include <pcl/conversions.h>

#include <pcl/point_types.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>

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
  PointCloud::Ptr vVertices (new PointCloud);
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY("/home/eduardo/test2.ply",mesh);
  pcl::fromPCLPointCloud2(mesh.cloud,*cloud_xyz);
  //Convex hull thing
  PointCloud::Ptr cloud_hull (new PointCloud);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_xyz);
  chull.setAlpha (0.5);
  chull.setVoronoiCenters(vVertices);
  chull.reconstruct(*cloud_hull);
  ROS_INFO("%d",vVertices->size());
  msg=cloud_hull;
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
