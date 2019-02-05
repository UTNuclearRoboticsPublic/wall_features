
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

	PCP input_cloud_ptr(new PC);
	pcl::fromROSMsg(req.input_cloud, *input_cloud_ptr);

	// ------------------------------------------------------------------------
	// ----------------------- Initially Voxelize Cloud -----------------------
	// ------------------------------------------------------------------------
	// Output Clouds
	PCP initial_voxelized_cloud_ptr(new PC());
	sensor_msgs::PointCloud2 initial_voxelized_cloud;
	// Create the filtering object, set parameters
	pcl::VoxelGrid<PointType> vg_init;
	vg_init.setInputCloud(input_cloud_ptr);
	vg_init.setLeafSize(req.pixel_wdt/2.5, req.pixel_hgt/2.5, req.pixel_hgt/2.5); 	
	// Apply Filter and return Voxelized Data
	vg_init.filter(*initial_voxelized_cloud_ptr);
	pcl::toROSMsg(*initial_voxelized_cloud_ptr, initial_voxelized_cloud);
	ROS_INFO_STREAM("[Rasterizer] Initial voxel filter applied to cloud. New size is " << initial_voxelized_cloud_ptr->size());

    // ------------------------------------------------------------------------
	// --------------------------- Remove Outliers ----------------------------
	// ------------------------------------------------------------------------
	pcl::RadiusOutlierRemoval<PointType> filter;
	PCP outlierless_cloud_ptr(new PC);
	if(req.outlier_filter)
	{
		filter.setInputCloud(initial_voxelized_cloud_ptr);
		filter.setRadiusSearch(req.outlier_filter_scale);
		filter.setMinNeighborsInRadius(req.rad_outlier_min_neighbors);
		filter.setKeepOrganized(false);
		// Perform filtering
		filter.filter(*outlierless_cloud_ptr);	
		ROS_INFO_STREAM("[Rasterizer] Outlier filter of size " << req.outlier_filter_scale << " applied to plane cloud. Filtered size is " << outlierless_cloud_ptr->size());
	}
	else
		*outlierless_cloud_ptr = *initial_voxelized_cloud_ptr;
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
	// Rotate cloud to be on XZ plane
	res.rotated_cloud = PointcloudUtilities::rotatePlaneToXZ(initial_voxelized_cloud, coefficients->values);
	PCP rotated_plane_ptr(new PC());
	pcl::fromROSMsg(res.rotated_cloud, *rotated_plane_ptr);
	// ----------- Finding Translation ----------- 
	// Output Clouds
	PCP transformed_plane_ptr(new PC());
	// Find Translation
	res.transformed_cloud = PointcloudUtilities::translatePlaneToXZ(res.rotated_cloud);
	float min_x, max_x, min_y, max_y, min_z, max_z, min_intensity, max_intensity;
	PointcloudUtilities::cloudLimits(res.rotated_cloud, &min_x, &max_x, &min_y, &max_y, &min_z, &max_z, &min_intensity, &max_intensity);
	float mean_y = PointcloudUtilities::meanValue(res.rotated_cloud, 'y');
	res.transformed_cloud.header.stamp = req.input_cloud.header.stamp;
	res.transformed_cloud.header.frame_id = req.input_cloud.header.frame_id;
	pcl::fromROSMsg(res.transformed_cloud, *transformed_plane_ptr);

	// ------------------------------------------------------------------------
	// ---------------------------- Voxelize Cloud ----------------------------
	// ------------------------------------------------------------------------
	// Output Clouds
	PCP voxelized_cloud_ptr(new PC());
	// Create the filtering object, set parameters
	pcl::VoxelGrid<PointType> vg;
	vg.setInputCloud(transformed_plane_ptr);
	float wall_depth = 0.2;  		// Voxel width in Y direction just needs to be arbitrarily large relative to wall depth variation
	vg.setLeafSize(req.pixel_wdt, wall_depth, req.pixel_hgt); 	
	// Apply Filter and return Voxelized Data
	vg.filter(*voxelized_cloud_ptr);
	
	// ------------------------------------------------------------------------
	// ------------------------------ Flip Cloud ------------------------------
	// ------------------------------------------------------------------------
	// For some reason, raster images are currently coming out backwards - flip them left/right here
	//for(int i=0; i<voxelized_cloud_ptr->points.size(); i++)
	//{	
	//	voxelized_cloud_ptr->points[i].x *= -1; 
	//}
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
	cv_bridge::CvImagePtr wall_depth_cv(new cv_bridge::CvImage);
	cv_bridge::CvImagePtr wall_intensity_cv(new cv_bridge::CvImage);
	cv::Mat depth_img(image_hgt,image_wdt,CV_8UC1,cv::Scalar(0));
	cv::Mat intensity_img(image_hgt,image_wdt,CV_8UC1,cv::Scalar(0));
	// Check which pixels are occupied
	bool occupied[image_hgt][image_wdt];
	for(int i=0; i<image_hgt; i++)
	{
		for(int j=0; j<image_wdt; j++)
		{
			occupied[i][j] = false;
			depth_img.at<uchar>(i,j) = 0; 		// R
			intensity_img.at<uchar>(i,j) = 0; 		// R
		}
	}

	// Build Image
	res.image_fill_ratio = 0;
	for(int k=0; k<voxelized_cloud_ptr->points.size(); k++)
	{
		int i = int(floor(-voxelized_cloud_ptr->points[k].z / req.pixel_hgt));
		int j = int(floor(voxelized_cloud_ptr->points[k].x / req.pixel_wdt));

		float depth_color = (voxelized_cloud_ptr->points[k].y-min_y+mean_y)/(max_y-min_y);
		float intensity_color = (voxelized_cloud_ptr->points[k].intensity-min_intensity)/(max_intensity-min_intensity);
		depth_img.at<uchar>(i,j) = floor(depth_color*255);
		intensity_img.at<uchar>(i,j) = floor(intensity_color*255);

		// Note that we found another good pixel
		res.image_fill_ratio++;
		occupied[i][j] = true;
	}

	if(!req.fill_holes)
		res.image_fill_ratio /= (image_wdt*image_hgt);
	else
	{
		pcl::search::KdTree<pcl::PointXYZI> tree;
		tree.setInputCloud(voxelized_cloud_ptr);

		for(int i=0; i<image_hgt; i++)
			for(int j=0; j<image_wdt; j++)
			{
				if(occupied[i][j])
					continue;
				pcl::PointXYZI source_point;
				source_point.x = j*req.pixel_wdt;
				source_point.y = 0;
				source_point.z = -i*req.pixel_hgt;
				std::vector<int> nearest_indices;
				std::vector<float> nearest_dist_squareds;
				float depth = 0;
				float intensity = 0;
				float total_inverse_distance = 0;
				if(tree.nearestKSearch(source_point, req.hole_filling_neighbor_count, nearest_indices, nearest_dist_squareds))
				{
					for(int k=0; k<nearest_indices.size(); k++)
					{
						pcl::PointXYZI target_point = voxelized_cloud_ptr->points[nearest_indices[k]];
						float dist = pow(nearest_dist_squareds[k],0.5);
						float depth_color = (target_point.y-min_y+mean_y)/(max_y-min_y);
						depth += depth_color / dist;
						float intensity_color = (target_point.intensity-min_intensity)/(max_intensity-min_intensity);
						intensity += intensity_color / dist;
						total_inverse_distance += 1 / dist;
					}
					depth_img.at<uchar>(i,j) = 255 * depth / total_inverse_distance;
					intensity_img.at<uchar>(i,j) = 255 * intensity / total_inverse_distance;
				}
			}

/*		cv::Mat filled(image_hgt,image_wdt,CV_8UC1,cv::Scalar(0));
		cv::Mat mask = (cv::Mat_<double>(3,3) << 0.125, 0.125, 0.125, 0.125, 0, 0.125, 0.125, 0.125, 0.125);
		for(int i=0; i<image_hgt; i++)
			for(int j=0; j<image_wdt; j++)
			{
				if(occupied[i][j])
					continue;
				float weight_total = 0;
				for(int i_off=-2; i_off<3; i_off++)
					for(int j_off=-2; j_off<3; j_off++)
					{
						if(i_off+i < 0 || i_off+i > image_hgt || j_off+j < 0 || j_off+j > image_hgt)
							continue;
						filled.at<uchar>(i,j) += depth_img.at<uchar>(i+i_off,j+j_off) * mask.at<double>(i_off,j_off);
						weight_total += mask.at<double>(i_off,j_off);
					}
				filled.at<uchar>(i,j) /= weight_total;
			}
		cv::addWeighted( wall_depth_cv->image, 1, filled, 1, 0, wall_depth_cv->image ); */
	} 
	depth_img.copyTo(wall_depth_cv->image);
	intensity_img.copyTo(wall_intensity_cv->image);

	//wall_cv->encoding = cv_bridge::CV_8UC3; 
	// Output
	wall_depth_cv->toImageMsg(res.output_depth_image);
	wall_intensity_cv->toImageMsg(res.output_intensity_image);
	res.output_depth_image.encoding = sensor_msgs::image_encodings::MONO8;
	res.output_intensity_image.encoding = sensor_msgs::image_encodings::MONO8;
	res.image_wdt = image_wdt;
	res.image_hgt = image_hgt;

	res.depth_min = min_y;
	res.depth_max = max_y;
	res.intensity_min = min_intensity;
	res.intensity_max = max_intensity;
	for (int i=0; i < coefficients->values.size(); i++)
		res.plane_coefficients.push_back(coefficients->values[i]);

	ros::Duration(3).sleep();

	ROS_INFO_STREAM("[Rasterizer] Rasterized cloud built. Image size is " << image_wdt << " by " << image_hgt << ", with min and max depth at " << res.depth_min << " and " << res.depth_max);

	ros::Duration(3).sleep();

	return true;
}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "rasterizer");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();	

	Rasterizer<pcl::PointXYZI> rasterizer;

}