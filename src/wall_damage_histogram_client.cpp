#include <ros/ros.h>
#include <pcl/point_types.h>
#include "wall_features/wall_damage_estimation.h"
#include "wall_features/point_wall_damage.h"
#include "wall_features/wall_damage_histogram.h"
#include <pcl/filters/voxel_grid.h>

int main (int argc, char **argv)
{ 
  ros::init(argc, argv, "test_segmentation");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); /*

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<PointWallDamage>::Ptr wall_damage_cloud (new pcl::PointCloud<PointWallDamage>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<WallDamageHistogram>::Ptr histogram_cloud (new pcl::PointCloud<WallDamageHistogram>);

  pcl::WallDamagePointwiseEstimation<pcl::PointXYZ, PointWallDamage> point_damage_estimator;

  point_damage_estimator.compute(input_cloud, wall_damage_cloud);

  // Create a voxelized version of the input
  pcl::VoxelGrid<PCLPoint> vg;
  *voxelized_cloud = *input_cloud;
  vg.setInputCloud(voxelized_cloud);
  vg.setLeafSize(0.1, 0.1, 0.1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pcp (new pcl::PointCloud<pcl::PointXYZ>());
  vg.filter(*temp_pcp);
  *voxelized_cloud = *temp_pcp; */

  pcl::WallDamageHistogramEstimation<pcl::PointWallDamage, pcl::WallDamageHistogram> damage_histogram_estimator;
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  const pcl::PointCloud<pcl::PointWallDamage> wall_damage_cloud;
  pcl::PointCloud<pcl::PointXYZ> voxelized_cloud;
  pcl::PointCloud<pcl::WallDamageHistogram> histogram_cloud;
  //damage_histogram_estimator.compute(wall_damage_cloud, histogram_cloud);

  return 0;
}
