// Std Stuff
#include <iostream>
#include <fstream>
// PCL Stuff
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
// Ros Stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// Wall_Features
#include "wall_features/wall_damage_estimation.h"
#include "wall_features/wall_damage_estimation.hpp"
#include "wall_features/point_wall_damage.h"
#include "wall_features/wall_damage_histogram.h"
// Primitive_Search
#include <pointcloud_primitive_search/primitive_process.h>
#include <pointcloud_primitive_search/primitive_process_creation.h>
#include <pointcloud_primitive_search/primitive_process_publisher.h>

int main (int argc, char **argv)
{ 
  // --------------------------------------------- Basic ROS Stuffs ---------------------------------------------
  ros::init(argc, argv, "wall_features_client");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); 

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  

  ros::NodeHandle nh;

  // --------------------------------------------- Initializing Things ---------------------------------------------

  // Publishers for Clouds
  ros::Publisher input_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/input_cloud", 1);
  ros::Publisher voxelized_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/voxelized_cloud", 1);
  ros::Publisher damage_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/damage_cloud", 1);
  ros::Publisher hsitogram_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/histogram_cloud", 1);

  // ROS Msg Clouds
  sensor_msgs::PointCloud2 input_msg;
  sensor_msgs::PointCloud2 voxelized_msg;
  sensor_msgs::PointCloud2 damage_msg;
  sensor_msgs::PointCloud2 histogram_msg;

  // PCL Clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWallDamage>::Ptr wall_damage_cloud (new pcl::PointCloud<pcl::PointWallDamage>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelized_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::WallDamageHistogram>::Ptr histogram_cloud (new pcl::PointCloud<pcl::WallDamageHistogram>);

  // Feature Estimators 
  pcl::WallDamagePointwiseEstimation<pcl::PointXYZRGB, pcl::PointWallDamage> point_damage_estimator;
  pcl::WallDamageHistogramEstimation<pcl::PointWallDamage, pcl::PointXYZRGB, pcl::WallDamageHistogram> damage_histogram_estimator;

  // Primitive Search Stuff
  pointcloud_primitive_search::primitive_process wall_process;
  PrimitiveProcessCreation::createProcesses(&wall_process, "primitive_search");
  PrimitiveProcessPublisher primitive_pub(nh, wall_process);
  ros::ServiceClient wall_finder = nh.serviceClient<pointcloud_primitive_search::primitive_process>("primitive_search");

  // --------------------------------------------- Bag Stuff ---------------------------------------------
  std::string bag_topic = "/laser_stitcher/output_cloud";
  std::string bag_name = "stitched_pointcloud.bag";
  nh.getParam("wall_features/bag_topic", bag_topic);
  nh.getParam("wall_features/bag_name", bag_name);
  ROS_INFO_STREAM("[RegistrationClient] Loading clouds from bag files, using bag name: " << bag_name << " and topic name: " << bag_topic << ".");
  rosbag::Bag input_bag; 
  input_bag.open(bag_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(bag_topic);
  rosbag::View view_1(input_bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view_1)
  {
      sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
      if (cloud_ptr != NULL)
          input_msg = *cloud_ptr;
      else
        ROS_ERROR_STREAM("[RegistrationClient] Cloud caught for first cloud is null...");
  }
  input_bag.close();

  ROS_INFO_STREAM("[WallFeatures] Input cloud loaded from bag of size " << input_msg.height*input_msg.width);

  // --------------------------------------------- Find Wall Within Cloud ---------------------------------------------
  wall_process.request.pointcloud = input_msg;
  if(!wall_finder.call(wall_process))
    ROS_ERROR_STREAM("[Wall Features] Failed to perform Primitive_Search on input cloud. Continuing with whole cloud...");
  else
  {
    int process_size = wall_process.response.outputs.size();
    input_msg = wall_process.response.outputs[process_size-1].task_results[1].task_pointcloud;
    ROS_INFO_STREAM("[WallFeatures] Successfully called PrimitiveSearch on input cloud. Output wall cloud size: " << input_msg.height*input_msg.width);
  }
  pcl::fromROSMsg(input_msg, *input_cloud);

  // --------------------------------------------- Wall Damage Estimation ---------------------------------------------
  point_damage_estimator.compute(*input_cloud, *wall_damage_cloud);
  ROS_INFO_STREAM("[WallFeatures] Performed pointwise damage estimation - output cloud size is " << wall_damage_cloud->points.size());


  // --------------------------------------------- Voxelization ---------------------------------------------
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  *voxelized_cloud = *input_cloud;
  vg.setInputCloud(voxelized_cloud);
  float leaf_size = 1.0;
  nh.getParam("wall_features/leaf_size", leaf_size);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  pcl::PointCloud<pcl::PointXYZRGB> temp_pcp;
  vg.filter(temp_pcp);
  *voxelized_cloud = temp_pcp; 
  ROS_INFO_STREAM("[WallFeatures] Performed voxelization of input cloud - output cloud size is " << voxelized_cloud->points.size());

  damage_histogram_estimator.compute(*wall_damage_cloud, *voxelized_cloud, *histogram_cloud);
  ROS_INFO_STREAM("[WallFeatures] Performed histogram cloud estimation.");

  std::ofstream output_file;
  output_file.open ("histogram_point.csv");
  output_file << "Point 1\n";
  int histogram_size = 80;
  for(int i=0; i<10; i++)
  {
    for(int j=0; j<histogram_size-1; j++)
      output_file << histogram_cloud->points[i].histogram[j] << ", ";
    output_file << histogram_cloud->points[i].histogram[histogram_size-1] << "\n";
  }
  output_file.close();

  pcl::toROSMsg(*voxelized_cloud, voxelized_msg);
  pcl::toROSMsg(*wall_damage_cloud, damage_msg);
  pcl::toROSMsg(*histogram_cloud, histogram_msg);

  while(ros::ok())
  {
    input_pub.publish(input_msg);
    voxelized_pub.publish(voxelized_msg);
    damage_pub.publish(damage_msg);
    hsitogram_pub.publish(histogram_msg);
    primitive_pub.publish(wall_process);
    ros::Duration(1.0).sleep();
  }

  return 0;
}
