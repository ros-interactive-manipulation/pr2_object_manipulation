#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "write_transformed_pcd");
  sensor_msgs::PointCloud2 transformed_cloud;

  if(argc < 3)
  {
    ROS_ERROR("Usage: write_transformed_pcd <point_cloud_topic> <frame_id>");
    exit(-1);
  }
  tf::TransformListener tfl(ros::Duration(20.0));
  //need to let the transform listener build up some transforms
  ros::Duration(1.0).sleep();

  sensor_msgs::PointCloud2::ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(argv[1], ros::Duration(5.0));
  if(!cloud)
  {
    ROS_INFO("No message received on topic %s", argv[1]);
    exit(-2);
  }
  tfl.waitForTransform(argv[2], cloud->header.frame_id, cloud->header.stamp, ros::Duration(2.0));
  pcl_ros::transformPointCloud(argv[2], *cloud, transformed_cloud, tfl);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(transformed_cloud, *cloud_out);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("cloud.pcd", *cloud_out, false);
  return 0;
  
}
