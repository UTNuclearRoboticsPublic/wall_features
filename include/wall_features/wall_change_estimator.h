
#ifndef WALL_CHANGE_ESTIMATOR_H
#define WALL_CHANGE_ESTIMATOR_H

#include <ros/ros.h>

#include "wall_features/wall_change.h"

// CVBridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

class WallChangeEstimator
{
public:
	WallChangeEstimator();
private:
	bool wall_change(wall_features::wall_change::Request &req, wall_features::wall_change::Response &res);
};

#endif //WALL_CHANGE_ESTIMATOR_H