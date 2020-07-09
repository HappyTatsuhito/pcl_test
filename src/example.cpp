#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>//filter
#include <pcl/filters/voxel_grid.h>//downsample
#include <pcl/segmentation/sac_segmentation.h>//segmentation

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2 &cloud_input_ros)
{
  /*pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);*/
  pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;
  pcl::PointCloud<pcl::PointXYZ> cloud_downsampled_pcl;
  sensor_msgs::PointCloud2 cloud_ros;


  // Convert to PCL data type
  //pcl_conversions::toPCL(*input, *cloud);
  pcl::fromROSMsg(cloud_input_ros, cloud_input_pcl);

  // Filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
  statFilter.setInputCloud(cloud_input_pcl.makeShared());
  statFilter.setMeanK(10);
  statFilter.setStddevMulThresh(0.2);
  statFilter.filter(cloud_filtered_pcl);

  // Downsample
  pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
  voxelSampler.setInputCloud(cloud_filtered_pcl.makeShared());
  voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
  voxelSampler.filter(cloud_downsampled_pcl);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //pcl_conversions::moveFromPCL(cloud, output);
  //output.header.frame_id = frame_id;
  pcl::toROSMsg(cloud_downsampled_pcl, cloud_ros);

  // Publish the data
  pub.publish (cloud_ros);
}

int main (int argc, char **argv)
{
  //Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_sample_output", 1);

  // Spin
  ros::spin ();
}
