
// Ros Stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// Std Stuff
#include <iostream>
#include <fstream>
// PCL Stuff
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
// Wall_Features
#include "wall_features/wall_damage_estimation.h"
#include "wall_features/wall_damage_estimation.hpp"
#include "wall_features/point_wall_damage.h"
#include "wall_features/wall_damage_histogram.h"
#include "wall_features/wall_damage_service.h"
// Primitive_Search
#include <pointcloud_primitive_search/primitive_process.h>
#include <pointcloud_primitive_search/primitive_process_creation.h>
#include <pointcloud_primitive_search/primitive_process_publisher.h>


class WallDamageEstimatorServer
{
public:
	WallDamageEstimatorServer();

	typedef typename pcl::PointXYZRGB PointType;
	typedef typename pcl::PointXYZRGBNormal PointNormalType;
	typedef typename pcl::PointCloud<PointType> PC;
	typedef typename pcl::PointCloud<PointType>::Ptr PCP;

private:
	bool resetServer(std::string primitive_search_name);
	bool wallDamageCallback(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res);
	bool inputFromBag(wall_features::wall_damage_service::Request &req);
	bool voxelizeCloud(PCP input_cloud_ptr, PCP output_cloud_ptr, float leaf_size, std::string cloud_name);
	bool planeSegmentation(wall_features::wall_damage_service::Response &res);
	bool estimateWallDamage(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res);
	bool estimateWallDamageHistogram(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res);
	bool publishClouds(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res);

	ros::NodeHandle nh_;
	ros::Publisher input_cloud_pub_;
	ros::Publisher damage_cloud_pub_;
	ros::Publisher damage_histogram_cloud_pub_;

	ros::ServiceServer damage_estimator_server_;

	// Baseline Input Cloud
	PCP input_cloud_ptr_;	
	
	// Voxelized Input Clouds
	sensor_msgs::PointCloud2 segmentation_input_cloud_;
	sensor_msgs::PointCloud2 wall_damage_input_cloud_;
	sensor_msgs::PointCloud2 wall_damage_histogram_input_cloud_;
	PCP segmentation_input_ptr_;
	PCP wall_damage_input_ptr_;
	PCP wall_damage_histogram_input_ptr_;	

	// Wall Estimation Information
	pointcloud_primitive_search::primitive_process wall_process_;
	PrimitiveProcessPublisher primitive_pub_;
  	ros::ServiceClient wall_finder_;
  	sensor_msgs::PointCloud2 wall_cloud_;
  	PCP wall_cloud_ptr_;

  	// Feature Estimation
	pcl::WallDamagePointwiseEstimation<PointType, PointNormalType, pcl::PointWallDamage> point_damage_estimator_;
	pcl::WallDamageHistogramEstimation<pcl::PointWallDamage, PointType, pcl::WallDamageHistogram> damage_histogram_estimator_;
	pcl::PointCloud<pcl::PointWallDamage>::Ptr wall_damage_ptr_;
	pcl::PointCloud<pcl::WallDamageHistogram>::Ptr wall_damage_histogram_ptr_;

};