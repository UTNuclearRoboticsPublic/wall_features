
#include <wall_features/rasterizer.h>

template <typename PointType> 
Rasterizer<PointType>::Rasterizer()
{
	ros::NodeHandle nh;

	std::string service_name;
	nh.param<std::string>("rasterizer/service_name", service_name, "rasterizer");
	ros::ServiceServer service_server = nh.advertiseService(service_name, &Rasterizer<PointType>::rasterizer_service, this);

	ros::spin();
}


// For some reason, the Eigen function fromTwoVectors(), which returns a quaternion rotation between two vectors, seems unstable for vectors that are near-parallel
// Writing my own implementation that doesn't suck 
// This finds the normalized quaternion rotation FROM first_vec TO second_vec
Eigen::Quaternion<float> fromTwoVectorsStable(Eigen::Vector3f first_vec, Eigen::Vector3f second_vec)
{
	ROS_DEBUG_STREAM("[Rasterizer]   FromTwoVector function inputs - first vector: " << first_vec.x() << " " << first_vec.y() << " " << first_vec.z() << " second vector: "  << second_vec.x() << " " << second_vec.y() << " " << second_vec.z());
	Eigen::Quaternion<float> identity_quat;
	identity_quat.setIdentity();
	// Check whether input vectors are well-conditioned
	if( !std::fpclassify(first_vec.x()) == FP_NORMAL || !std::fpclassify(first_vec.y()) == FP_NORMAL || !std::fpclassify(first_vec.z()) == FP_NORMAL )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, first input vector is poorly conditioned: " << first_vec.x() << " " << first_vec.y() << " " << first_vec.z() << "; returning identity quaternion.");
		return identity_quat;
	}
	if( !std::fpclassify(second_vec.x()) == FP_NORMAL || !std::fpclassify(second_vec.y()) == FP_NORMAL || !std::fpclassify(second_vec.z()) == FP_NORMAL )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, second input vector is poorly conditioned: " << second_vec.x() << " " << second_vec.y() << " " << second_vec.z() << "; returning identity quaternion.");
		return identity_quat;
	}
	// Check whether norms of input vectors are well-conditioned
	float first_vec_norm = first_vec.norm();
	if( !std::fpclassify(first_vec_norm) == FP_NORMAL || first_vec_norm > pow(10,10) )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, norm of first input vector is poorly conditioned: " << first_vec_norm << "; Returning identity quaternion.");
		return identity_quat;
	}
	float second_vec_norm = second_vec.norm();
	if( !std::fpclassify(second_vec_norm) == FP_NORMAL || second_vec_norm > pow(10,10) )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, norm of second input vector is poorly conditioned: " << second_vec_norm << "; Returning identity quaternion.");
		return identity_quat;
	}

	// Check whether norm of cross product is well-conditioned
	Eigen::Vector3f cross = first_vec.cross(second_vec);
	float norm_of_cross = cross.norm();
	if(fabs(norm_of_cross) > pow(10,30) || fabs(norm_of_cross) < pow(10,-30))
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, cross product norm is ill-posed: " << norm_of_cross << "; returning identity quaternion.");
		return identity_quat;
	}
	Eigen::Vector3f unit_normal = cross/norm_of_cross;

	float angle = asin( norm_of_cross/first_vec_norm/second_vec_norm );

	ROS_DEBUG_STREAM("[Rasterizer]   FromTwoVector function outputs - angle: " << angle << " n: " << unit_normal.x() << " " << unit_normal.y() << " " << unit_normal.z() << " norm_of_cross " << norm_of_cross);
	Eigen::Quaternion<float> output( cos(angle/2), sin(angle/2)*unit_normal.x(), sin(angle/2)*unit_normal.y(), sin(angle/2)*unit_normal.z() );

	return output;
}

template <typename PointType> bool
Rasterizer<PointType>::rasterizer_service(wall_features::rasterizer_service::Request &req, wall_features::rasterizer_service::Response &res)
{
	ROS_INFO_STREAM("[Rasterizer] Received service call. Input cloud is of size " << req.input_cloud.height*req.input_cloud.width);

    // ------------------------------------------------------------------------
	// --------------------------- Remove Outliers ----------------------------
	// ------------------------------------------------------------------------
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
	PCP input_cloud_ptr(new PC);
	PCP outlierless_cloud_ptr(new PC);
	pcl::fromROSMsg(req.input_cloud, *input_cloud_ptr);
	filter.setInputCloud(input_cloud_ptr);
	filter.setRadiusSearch(req.outlier_filter_scale);
	filter.setMinNeighborsInRadius(req.rad_outlier_min_neighbors);
	filter.setKeepOrganized(false);
	// Perform filtering
	filter.filter(*outlierless_cloud_ptr);	
	ROS_INFO_STREAM("[Rasterizer] Outlier filter of size " << req.outlier_filter_scale << " applied to plane cloud. Filtered size is " << outlierless_cloud_ptr->size());
	pcl::toROSMsg(*outlierless_cloud_ptr, res.outlierless_cloud);

	// ------------------------------------------------------------------------
	// ---------------------------- Plane RANSAC ------------------------------
	// ------------------------------------------------------------------------
	// Segmented Clouds
	PCP plane_cloud_ptr( new PC() );
	// Create the segmentation objects
	pcl::SACSegmentation<PointType> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Basic SAC Settings
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	// Segmentation Parameters
	seg.setOptimizeCoefficients (true);
	seg.setMaxIterations (req.max_iterations);
	seg.setDistanceThreshold (req.threshold_distance);
	// Find Plane Inliers
	seg.setInputCloud (outlierless_cloud_ptr);
	seg.segment (*inliers, *coefficients);
	// Check success
	if(inliers->indices.size() == 0) 
	{
		ROS_WARN_STREAM("[PointcloudProcessing]   Could not find any planes in the point cloud.");
		return false;
	}  
	// Extract Inliers to New Cloud
	pcl::ExtractIndices<PointType> extract (true);
	extract.setInputCloud(outlierless_cloud_ptr);
	extract.setIndices(inliers);
	extract.setNegative(false); 	// Remove points given by indices
	// Actually segment out plane pointcloud, and set input to match new, smaller cloud
	extract.filter(*plane_cloud_ptr); 
	pcl::toROSMsg(*plane_cloud_ptr, res.plane_cloud);
	res.plane_cloud.header.stamp = req.input_cloud.header.stamp;
	res.plane_cloud.header.frame_id = req.input_cloud.header.frame_id;
	ROS_INFO_STREAM("[Rasterizer] Plane RANSAC performed. Plane cloud size is " << plane_cloud_ptr->size());

    // ------------------------------------------------------------------------
	// -------------------------- Transform Cloud --------------------------
	// ------------------------------------------------------------------------
	// ----------- Finding Rotation ----------- 
	// Output Clouds
	PCP rotated_plane_ptr(new PC()); 
	// Rotation --> make normal horizontal
	Eigen::Vector3f input_cloud_norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	Eigen::Vector3f input_cloud_norm_horz(coefficients->values[0], coefficients->values[1], 0.0); 
	ROS_DEBUG_STREAM("[Rasterizer] Plane Normal: " << " " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2]);
	//input_cloud_norm_horz.cast<float>();
	Eigen::Quaternion<float> horz_rot;
	horz_rot = fromTwoVectorsStable(input_cloud_norm, input_cloud_norm_horz);
	ROS_DEBUG_STREAM("[Rasterizer] Plane Rotation to Horz (Quat): " << horz_rot.x() << " " << horz_rot.y() << " " << horz_rot.z() << " " << horz_rot.w());
	// Rotation --> make normal in Y+
	//   Either use positive or negative Y axis, depending on whether plane normal is mostly + or - Y
	//   Otherwise, plane will sometimes get flipped 180 degrees for no reason, since for a plane there's no difference between +/- normal
	//   Would be nice if PCL defaulted to, for example, return the normal which pointed away from the origin, or some other convention,
	//     but as of June 2018 they don't - it's just random (from RANSAC)
	float y_axis_direction;
	if(coefficients->values[1] > 0)
		y_axis_direction = 1.0;
	else y_axis_direction = -1.0;
	Eigen::Vector3f y_axis(0.0, y_axis_direction, 0.0);
	Eigen::Quaternion<float> onto_y_rot;
	onto_y_rot = fromTwoVectorsStable(input_cloud_norm_horz, y_axis); 
	ROS_DEBUG_STREAM("[Rasterizer] Plane Rotation to Y (Quat): " << onto_y_rot.x() << " " << onto_y_rot.y() << " " << onto_y_rot.z() << " " << onto_y_rot.w());
	// Build Transform
	Eigen::Quaternion<float> final_rotation = horz_rot * onto_y_rot;
	final_rotation.normalize();
	ROS_DEBUG_STREAM("[Rasterizer] Final Plane Rotation (Quat): " << final_rotation.x() << " " << final_rotation.y() << " " << final_rotation.z() << " " << final_rotation.w());
	geometry_msgs::TransformStamped cloud_rotation; 
	cloud_rotation.header = req.input_cloud.header;
	cloud_rotation.transform.rotation.x = float(final_rotation.x());
	cloud_rotation.transform.rotation.y = float(final_rotation.y());
	cloud_rotation.transform.rotation.z = float(final_rotation.z());
	cloud_rotation.transform.rotation.w = float(final_rotation.w());
	cloud_rotation.transform.translation.x = 0;
	cloud_rotation.transform.translation.y = 0;
	cloud_rotation.transform.translation.z = 0;
	// ----------- Performing Rotation -----------
	tf2::doTransform (res.plane_cloud, res.rotated_cloud, cloud_rotation);  	// transforms input_pc2 into process_message
	res.rotated_cloud.header.stamp = req.input_cloud.header.stamp;
	res.rotated_cloud.header.frame_id = req.input_cloud.header.frame_id;
	pcl::fromROSMsg(res.rotated_cloud, *rotated_plane_ptr);
	ROS_INFO_STREAM("[Rasterizer] Plane cloud rotated. Size is " << res.rotated_cloud.height*res.rotated_cloud.width << "; rotation quaternion coefficients: " << final_rotation.x() << " " << final_rotation.y() << " " << final_rotation.z() << " " << final_rotation.w());

	// ----------- Finding Translation ----------- 
	// Output Clouds
	PCP transformed_plane_ptr(new PC());
	// Find Translation
	float min_x = rotated_plane_ptr->points[0].x;
	float max_x = rotated_plane_ptr->points[0].x;
	float min_z = rotated_plane_ptr->points[0].z;
	float max_z = rotated_plane_ptr->points[0].z;
	float mean_y = 0;
	float min_y = rotated_plane_ptr->points[0].y;
	float max_y = rotated_plane_ptr->points[0].y;
	for(int i=1; i<rotated_plane_ptr->points.size(); i++)
	{
		if(rotated_plane_ptr->points[i].x < min_x)
			min_x = rotated_plane_ptr->points[i].x;
		if(rotated_plane_ptr->points[i].x > max_x)
			max_x = rotated_plane_ptr->points[i].x;
		if(rotated_plane_ptr->points[i].z < min_z)
			min_z = rotated_plane_ptr->points[i].z;
		if(rotated_plane_ptr->points[i].z > max_z)
			max_z = rotated_plane_ptr->points[i].z;
		if(rotated_plane_ptr->points[i].y < min_y)
			min_y = rotated_plane_ptr->points[i].y;
		if(rotated_plane_ptr->points[i].y > max_y)
			max_y = rotated_plane_ptr->points[i].y;
		mean_y += rotated_plane_ptr->points[i].y;
	}
	mean_y /= rotated_plane_ptr->points.size();
	// Build Transform
	geometry_msgs::TransformStamped cloud_translation;
	cloud_translation.transform.rotation.x = 0;
	cloud_translation.transform.rotation.y = 0;
	cloud_translation.transform.rotation.z = 0;
	cloud_translation.transform.rotation.w = 0;
	cloud_translation.transform.translation.x = -min_x;
	cloud_translation.transform.translation.y = -mean_y;
	cloud_translation.transform.translation.z = -max_z;
	// ----------- Performing Translation -----------
	tf2::doTransform (res.rotated_cloud, res.transformed_cloud, cloud_translation);  	// transforms input_pc2 into process_message
	res.transformed_cloud.header.stamp = req.input_cloud.header.stamp;
	res.transformed_cloud.header.frame_id = req.input_cloud.header.frame_id;
	pcl::fromROSMsg(res.transformed_cloud, *transformed_plane_ptr);

	ROS_INFO_STREAM("[Rasterizer] Plane cloud translated. Size is " << res.transformed_cloud.height*res.transformed_cloud.width << "; translation coefficients: " << -min_x << " " << -mean_y << " " << -max_z);

	// ------------------------------------------------------------------------
	// ---------------------------- Voxelize Cloud ----------------------------
	// ------------------------------------------------------------------------
	// Output Clouds
	PCP voxelized_cloud_ptr(new PC());
	// Create the filtering object, set parameters
	pcl::VoxelGrid<PointType> vg;
	vg.setInputCloud(transformed_plane_ptr);
	float wall_depth = .2;  		// Voxel width in Y direction just needs to be arbitrarily large relative to wall depth variation
	vg.setLeafSize(req.pixel_wdt, wall_depth, req.pixel_hgt); 	
	// Apply Filter and return Voxelized Data
	vg.filter(*voxelized_cloud_ptr);
	toROSMsg(*voxelized_cloud_ptr, res.output_cloud);
	res.output_cloud.header.stamp = req.input_cloud.header.stamp;
	res.output_cloud.header.frame_id = req.input_cloud.header.frame_id;
	ROS_INFO_STREAM("[Rasterizer] Voxel filter with leaf size " << req.pixel_wdt << " x " << wall_depth << " x " << req.pixel_hgt << " applied to plane cloud. Voxelized size is " << voxelized_cloud_ptr->size());

	// ------------------------------------------------------------------------
	// ---------------------------- Rasterize Cloud ---------------------------
	// ------------------------------------------------------------------------
	// Image dimensions
	int image_hgt = int( std::ceil( (max_z - min_z) / req.pixel_hgt ) );
	int image_wdt = int( std::ceil( (max_x - min_x) / req.pixel_wdt ) );
	// Create Output Objects
	cv_bridge::CvImagePtr wall_cv(new cv_bridge::CvImage);
	cv::Mat img(image_hgt,image_wdt,CV_8UC3,cv::Scalar(0,0,0));
	// Check which pixels are occupied
	bool occupied[image_hgt][image_wdt];
	for(int i=0; i<image_hgt; i++)
	{
		for(int j=0; j<image_wdt; j++)
		{
			occupied[i][j] = false;
			img.at<cv::Vec3b>(i,j)[2] = 0; 		// R
			img.at<cv::Vec3b>(i,j)[1] = 0;		// G
			img.at<cv::Vec3b>(i,j)[0] = 0;		// B
		}
	}

	// Build Image
	res.image_fill_ratio = 0;
	for(int k=0; k<voxelized_cloud_ptr->points.size(); k++)
	{
		int i = int(floor(-voxelized_cloud_ptr->points[k].z / req.pixel_hgt));
		int j = int(floor(voxelized_cloud_ptr->points[k].x / req.pixel_wdt));

		//img.at<cv::Vec3b>(i,j)[4] = 1;		// Offset
		//img.at<cv::Vec3b>(i,j)[3] = 1;		// I
		float depth_color = (voxelized_cloud_ptr->points[k].y-min_y+mean_y)/(max_y-min_y);
		int colormap = 0;
		if(colormap == 0)
		{
			img.at<cv::Vec3b>(i,j)[2] = floor(depth_color*255); 		// R
			img.at<cv::Vec3b>(i,j)[1] = floor(depth_color*255);		// G
			img.at<cv::Vec3b>(i,j)[0] = floor(depth_color*255);		// B
		}
		if(colormap == 1)
		{
			if(depth_color < 0 || depth_color > 1)
				ROS_ERROR_STREAM("oops " << i << " " << j << " " <<  k << " " << depth_color);
			if(depth_color < 0.5)
				img.at<cv::Vec3b>(i,j)[2] = 0; 							// Red
			else 
				img.at<cv::Vec3b>(i,j)[2] = 2*(depth_color-0.5)*255;
			if(depth_color < 0.5)
				img.at<cv::Vec3b>(i,j)[1] = 2*depth_color*255; 			// Green
			else 
				img.at<cv::Vec3b>(i,j)[1] = 2*(1-depth_color)*255;
			if(depth_color < 0.5)
				img.at<cv::Vec3b>(i,j)[0] = 2*(0.5-depth_color)*255; 	// Blue
			else 
				img.at<cv::Vec3b>(i,j)[0] = 0;
		}
		if(colormap == 2)
		{
			img.at<cv::Vec3b>(i,j)[2] = floor(depth_color*255); 		// R
			img.at<cv::Vec3b>(i,j)[1] = 0;		// G
			img.at<cv::Vec3b>(i,j)[0] = 255;		// B
		}
		// Note that we found another good pixel
		res.image_fill_ratio++;
	}
	res.image_fill_ratio /= (image_wdt*image_hgt);
	img.copyTo(wall_cv->image);
	//wall_cv->encoding = cv_bridge::CV_8UC3; 
	// Output
	wall_cv->toImageMsg(res.output_image);
	res.output_image.encoding = sensor_msgs::image_encodings::BGR8;
	res.image_wdt = image_wdt;
	res.image_hgt = image_hgt;
	ROS_INFO_STREAM("[Rasterizer] Rasterized cloud built. Image size is " << image_wdt << " by " << image_hgt);

}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "rasterizer");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();	

	Rasterizer<pcl::PointXYZ> rasterizer;

}