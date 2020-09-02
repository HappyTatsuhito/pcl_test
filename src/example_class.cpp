#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>//filter
#include <pcl/filters/voxel_grid.h>//downsample
//#include <pcl/segmentation/sac_segmentation.h>//segmentation

class CloudTester
{
private:
  ros::Subscriber test_sub;
  ros::Subscriber realsense_sub;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;
  pcl::PointCloud<pcl::PointXYZ> cloud_downsampled_pcl;
  sensor_msgs::PointCloud2 cloud_ros;
  
public:
  explicit CloudTester(ros::NodeHandle& n):
	nh(n){
	test_sub = nh.subscribe("/test",1,&CloudTester::TestCB,this);
	realsense_sub = nh.subscribe("/camera/depth_registered/points",1,&CloudTester::CloudCB,this);
	pcl_sub = nh.subscribe("/centroid_req",1,&CloudTester::ReqCB,this);
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_sample_output",1);
  }

  void TestCB(std_msgs::Bool msg)
  {
	ROS_INFO("complete");
  }
  
  void CloudCB(const sensor_msgs::PointCloud2 &cloud_input_ros)
  {
	//ROS_INFO("ok");
	pcl::fromROSMsg(cloud_input_ros, cloud_input_pcl);
  }

  void ReqCB(std_msgs::Bool flg)
  {
	ROS_INFO("req");
	FilteringCloud();
	DownsamplingCloud();
	PCL2ROS(cloud_downsampled_pcl);
	pcl_pub.publish(cloud_ros);
  }

  void FilteringCloud()
  {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
	statFilter.setInputCloud(cloud_input_pcl.makeShared());
	statFilter.setMeanK(10);
	statFilter.setStddevMulThresh(0.2);
	statFilter.filter(cloud_filtered_pcl);
  }

  void DownsamplingCloud()
  {
	pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
	voxelSampler.setInputCloud(cloud_filtered_pcl.makeShared());
	voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
	voxelSampler.filter(cloud_downsampled_pcl);
  }

  void PCL2ROS(pcl::PointCloud<pcl::PointXYZ> cloud_pcl)
  {
	pcl::toROSMsg(cloud_pcl, cloud_ros);
  }

  void hoge()
  {
	ROS_INFO("hoge");
  }
};

int main (int argc, char **argv)
{
  //Initialize ROS
  ros::init (argc, argv, "pcl_example");
  ros::NodeHandle nh("~");
  ROS_INFO("start");
  CloudTester* tester = new CloudTester(nh);
  //tester->hoge();
  // Spin
  ros::spin ();
}
