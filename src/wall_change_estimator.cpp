
#include "wall_features/wall_change_estimator.h"

WallChangeEstimator::WallChangeEstimator()
{
	ros::NodeHandle nh;
	ros::ServiceServer service_server = nh.advertiseService("wall_change_estimation", &WallChangeEstimator::wall_change, this);
	ros::ServiceServer service_server_pointcloud = nh.advertiseService("wall_change_estimation_pointcloud", &WallChangeEstimator::wall_change_pointcloud, this);

	ros::ServiceClient registration_client = nh.serviceClient<pointcloud_registration_server::registration_service>("pointcloud_registration");

	ros::spin();	
}

bool WallChangeEstimator::wall_change_pointcloud(wall_features::wall_change_pointcloud::Request &req, wall_features::wall_change_pointcloud::Response &res)
{
	ROS_INFO_STREAM("[WallChange] Received request to perform wall change analysis.");
	ROS_INFO_STREAM("[WallChange]   First cloud size: " << req.first_cloud.height*req.first_cloud.width);
	ROS_INFO_STREAM("[WallChange]   Second cloud size: " << req.second_cloud.height*req.second_cloud.width);
	ROS_INFO_STREAM("[WallChange]   Using thresholds: acceptability = " << req.acceptable_distance_threshold << " and outlier = " << req.outlier_distance_threshold);

	pcl::PointCloud<pcl::PointXYZI>::Ptr first_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr second_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr first_cloud_ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr second_cloud_ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr difference_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(req.first_cloud, *first_cloud_ptr);
	pcl::fromROSMsg(req.second_cloud, *second_cloud_ptr);

	res.min_deviation = 10000;
	res.max_deviation = 0;
	res.mean_deviation = 0;
	res.mean_deviation_abs = 0;
	int valid_points = 0;

	// voxelize?
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(first_cloud_ptr);
	vg.setLeafSize(0.008, 0.008, 0.008);
	vg.filter(*first_cloud_ptr_voxelized);

	vg.setInputCloud(second_cloud_ptr);
	vg.filter(*second_cloud_ptr_voxelized);

	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(first_cloud_ptr_voxelized);
	std::vector<float> nearest_dist_squareds;
	std::vector<int> nearest_indices;

	int num_above_threshold = 0;

	for(int i=0; i<second_cloud_ptr_voxelized->points.size(); i++)
	{
		if ( kdtree.nearestKSearch (second_cloud_ptr_voxelized->points[i], 1, nearest_indices, nearest_dist_squareds) > 0 )
		{
			float distance;
			if(req.use_intensity_for_depth)
				distance = second_cloud_ptr_voxelized->points[i].intensity - first_cloud_ptr_voxelized->points[nearest_indices[0]].intensity;   //sqrt(nearest_dist_squareds[0]);
			else 
				distance = sqrt(nearest_dist_squareds[0]);
			if(sqrt(nearest_dist_squareds[0]) < req.outlier_distance_threshold)
			{
				pcl::PointXYZI difference_point;

				difference_point.x = second_cloud_ptr_voxelized->points[i].x;
				difference_point.y = second_cloud_ptr_voxelized->points[i].y;
				difference_point.z = second_cloud_ptr_voxelized->points[i].z;
				difference_point.intensity = fabs(distance);
				difference_cloud_ptr->points.push_back(difference_point);

				if(res.min_deviation > distance)
					res.min_deviation = distance;
				if(res.max_deviation < distance)
					res.max_deviation = distance;
				res.mean_deviation += distance;
				res.mean_deviation_abs += fabs(distance);
				valid_points++;
				if(fabs(distance) > req.acceptable_distance_threshold)
					num_above_threshold++;
			}
			else
			{ 
				ROS_WARN_STREAM_THROTTLE(30, "[WallChange] Presumed outlier found in second cloud at coordinates " 
					<< second_cloud_ptr_voxelized->points[i].x << ", " << second_cloud_ptr_voxelized->points[i].y << ", " << second_cloud_ptr_voxelized->points[i].z 
					<< ". This point is being passed over. Nearest target point is " << sqrt(nearest_dist_squareds[0]) << ". So far, " << i - valid_points << " outliers found using outlier distance threshold of " << req.outlier_distance_threshold);
			}
		}
	}
	res.mean_deviation /= valid_points;
	res.mean_deviation_abs /= valid_points;
	res.percent_points_valid = float(valid_points) / float(second_cloud_ptr_voxelized->points.size()) *100;
	res.percent_points_acceptable = float(valid_points-num_above_threshold) / float(valid_points) * 100;
	ROS_INFO_STREAM("[WallChange] Performed estimate of difference between scans. Min difference: " << res.min_deviation << "; Max difference: " << res.max_deviation << "; Mean difference: " << res.mean_deviation << "; Mean difference abs: " << res.mean_deviation_abs);
	ROS_INFO_STREAM("[WallChange]   Valid Points: " << valid_points << "; Total Points: " << second_cloud_ptr_voxelized->points.size() << "; " << res.percent_points_valid << "%% of points are Valid.");
	ROS_INFO_STREAM("[WallChange]   " << num_above_threshold << " points were above the acceptability threshold of " << req.acceptable_distance_threshold << ", which is " << 100-res.percent_points_acceptable << "%% of the total.");

	pcl::toROSMsg(*difference_cloud_ptr, res.difference_cloud);
	res.difference_cloud.header.stamp = ros::Time::now();
	res.difference_cloud.header.frame_id = "/map";

	pcl::toROSMsg(*first_cloud_ptr_voxelized, res.first_cloud_voxelized);
	pcl::toROSMsg(*second_cloud_ptr_voxelized, res.second_cloud_voxelized);
	res.first_cloud_voxelized.header = res.difference_cloud.header;
	res.second_cloud_voxelized.header = res.difference_cloud.header;

	return true;
}

bool WallChangeEstimator::wall_change(wall_features::wall_change::Request &req, wall_features::wall_change::Response &res)
{
	// do some registration stuff first using OpenCV

	ROS_INFO_STREAM("[WallChange] Creating first OpenCV Image, with size " << req.first_image.height << "x" << req.first_image.width << " and depth range " << req.first_image_min << " to " << req.first_image_max);
	cv_bridge::CvImagePtr first_image_ptr; 
	try
	{
		first_image_ptr = cv_bridge::toCvCopy(req.first_image, sensor_msgs::image_encodings::RGB8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR_STREAM("[WallChange] cv_bridge exception: " << e.what());
		return false; 
	}

	ROS_INFO_STREAM("[WallChange] Creating second OpenCV Image, with size " << req.second_image.height << "x" << req.second_image.width << " and depth range " << req.second_image_min << " to " << req.second_image_max);
	cv_bridge::CvImagePtr second_image_ptr; 
	try
	{
		second_image_ptr = cv_bridge::toCvCopy(req.second_image, sensor_msgs::image_encodings::RGB8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR_STREAM("[WallChange] cv_bridge exception: " << e.what());
		return false; 
	}

	if(req.first_image.height != req.second_image.height || req.first_image.width != req.second_image.width)
		ROS_WARN_STREAM_THROTTLE(1,"Warning - input images are of different sizes. First image size: " << req.first_image.height << "x" << req.first_image.width << "; Second image size: " << req.second_image.height << "x" << req.second_image.width);

	int image_hgt = std::min(req.first_image.height, req.second_image.height);
	int image_wdt = std::min(req.first_image.width, req.second_image.width);
	ROS_INFO_STREAM("[WallChange] Creating output image with size " << image_hgt << "x" << image_wdt);

	cv_bridge::CvImagePtr wall_cv(new cv_bridge::CvImage);
	cv::Mat img(image_hgt,image_wdt,CV_8UC3,cv::Scalar(0,0,0));

	float unscaled_image[image_hgt][image_wdt];
	float min_difference = 10000;
	float max_difference = -10000;
	float mean_difference = 0;
	float mean_difference_abs = 0;
	int num_real_pixels = 0;

	for(int i=0; i<image_hgt; i++)
		for(int j=0; j<image_wdt; j++)
		{
			float first_depth = first_image_ptr->image.at<cv::Vec3b>(i,j)[0];
			float second_depth = second_image_ptr->image.at<cv::Vec3b>(i,j)[0];

			if(first_depth != 0.0 && second_depth != 0.0)
			{
				float first_depth_scaled = first_depth/255.0*(req.first_image_max - req.first_image_min) + req.first_image_min;
				float second_depth_scaled = second_depth/255.0*(req.second_image_max - req.second_image_min) + req.second_image_min;

				unscaled_image[i][j] = first_depth_scaled - second_depth_scaled;

				if(unscaled_image[i][j] < min_difference)
					min_difference = unscaled_image[i][j];
				if(unscaled_image[i][j] > max_difference)
					max_difference = unscaled_image[i][j];

				mean_difference += unscaled_image[i][j];
				mean_difference_abs += std::fabs(unscaled_image[i][j]);

				num_real_pixels++;
			}
			else	// If either image has a hole at this point 
			{
				unscaled_image[i][j] = 0;
			}

		}
		// at some point update the following to disclude empty pixels
	mean_difference = float(mean_difference/num_real_pixels);
	mean_difference_abs = float(mean_difference_abs/num_real_pixels);

	ROS_INFO_STREAM("[WallChange] Populated matrix of unscaled image differences. Min: " << min_difference << "; Max: " << max_difference << "; Mean: " << mean_difference << "; ABS Mean: " << mean_difference_abs << "; Num Nonzero Pixels: " << num_real_pixels);

	for(int i=0; i<image_hgt; i++)
		for(int j=0; j<image_wdt; j++)
		{
			if(unscaled_image != 0)
			{
				img.at<cv::Vec3b>(i,j)[0] = (unscaled_image[i][j] - min_difference) / (max_difference-min_difference) * 255;
				img.at<cv::Vec3b>(i,j)[1] = (unscaled_image[i][j] - min_difference) / (max_difference-min_difference) * 255;
				img.at<cv::Vec3b>(i,j)[2] = (unscaled_image[i][j] - min_difference) / (max_difference-min_difference) * 255;
			}
			else 
			{
				img.at<cv::Vec3b>(i,j)[0] = (unscaled_image[i][j] - min_difference) / (max_difference-min_difference) * 255;
				img.at<cv::Vec3b>(i,j)[1] = (unscaled_image[i][j] - min_difference) / (max_difference-min_difference) * 255;
				img.at<cv::Vec3b>(i,j)[2] = (unscaled_image[i][j] - min_difference) / (max_difference-min_difference) * 255;
			}
		}
	ROS_INFO_STREAM("[WallChange] Populated scaled image matrix of differences. Creating ROS object and service outputs.");

	cv_bridge::CvImagePtr difference_image_ptr(new cv_bridge::CvImage); 
	img.copyTo(difference_image_ptr->image);
	difference_image_ptr->toImageMsg(res.difference_image);
	res.difference_image.encoding = sensor_msgs::image_encodings::BGR8;
	res.image_max = max_difference;
	res.image_min = min_difference;
	res.image_mean = mean_difference;
	res.image_mean_abs = mean_difference_abs;

	ROS_INFO_STREAM("[WallChange] Successfully performed wall change analysis. Image created of size " << image_hgt << "x" << image_wdt);
	ROS_INFO_STREAM("[WallChange]    Min difference: " << min_difference << "; Max difference: " << max_difference);
}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "wall_change");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();	

	WallChangeEstimator wall_change;

}