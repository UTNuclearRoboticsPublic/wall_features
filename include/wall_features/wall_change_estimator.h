
#ifndef WALL_CHANGE_ESTIMATOR_H
#define WALL_CHANGE_ESTIMATOR_H

#include <ros/ros.h>

#include "wall_features/wall_change.h"
#include "wall_features/wall_change_pointcloud.h"

// CVBridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Registration
#include <pointcloud_registration_server/pointcloud_registration.h>
#include <pointcloud_registration_server/registration_service.h>
#include <pointcloud_registration_server/reg_creation.h>

#include <pcl/filters/voxel_grid.h>

class WallChangeEstimator
{
public:
	WallChangeEstimator();
private:
	bool wall_change_pointcloud(wall_features::wall_change_pointcloud::Request &req, wall_features::wall_change_pointcloud::Response &res);
	bool wall_change(wall_features::wall_change::Request &req, wall_features::wall_change::Response &res);
};

#endif //WALL_CHANGE_ESTIMATOR_H