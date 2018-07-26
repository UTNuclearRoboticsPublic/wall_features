
#include <ros/ros.h>
// Bags
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
// Pointcloud Processing Server
#include <pointcloud_processing_server/pointcloud_process_publisher.h>
#include <pointcloud_processing_server/pointcloud_process.h>
#include <pointcloud_processing_server/pointcloud_task_creation.h>
// Primitive Search
#include <pointcloud_primitive_search/primitive_process.h>
#include <pointcloud_primitive_search/primitive_process_creation.h>
#include <pointcloud_primitive_search/primitive_process_publisher.h>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Basic PCL Stuff
#include <pcl_conversions/pcl_conversions.h> 			// fromROSMsg(), toROSMsg() 
// Segmentation
#include <pcl/segmentation/sac_segmentation.h> 			// SACSegmentation
#include <pcl/filters/extract_indices.h> 				// Extract target points
// PCL Filters 
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "wall_features/rasterizer_service.h"

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