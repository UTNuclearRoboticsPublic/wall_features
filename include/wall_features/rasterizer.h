
#ifndef RASTERIZER_H
#define RASTERIZER_H

#include <ros/ros.h>
// Bags
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Basic PCL Stuff
#include <pcl_conversions/pcl_conversions.h> 			// fromROSMsg(), toROSMsg() 
#include <pcl/kdtree/kdtree.h>
// Segmentation
#include <pcl/segmentation/sac_segmentation.h> 			// SACSegmentation
#include <pcl/filters/extract_indices.h> 				// Extract target points
// PCL Filters 
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "wall_features/rasterizer_service.h"

#include <pointcloud_processing_server/pointcloud_transforms.h>

// CVBridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <Eigen/Dense>

template <typename PointType> 
class Rasterizer
{
public:
	//usefule PCL typedefs
	typedef typename pcl::PointCloud<PointType> PC;
	typedef typename pcl::PointCloud<PointType>::Ptr PCP;

	Rasterizer();
	bool rasterizer_service(wall_features::rasterizer_service::Request &req, wall_features::rasterizer_service::Response &res);

private:

};

#endif //RASTERIZER_H