
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

template <typename PointType> bool
Rasterizer<PointType>::rasterizer_service(wall_features::rasterizer_srv::Request &req, wall_features::rasterizer_srv::Response &res)
{
	ROS_INFO_STREAM("[Rasterizer] Received service call. Input cloud is of size " << req.input_cloud.height*req.input_cloud.width);
	// Of course, the input cloud should probably always have height=1, since if it's already ordered there's not a lot of point to this program...

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

	// ------------------------------------------------------------------------
	// ---------------------------- Plane RANSAC ------------------------------
	// ------------------------------------------------------------------------
	// Segmented Clouds
	PCP plane_cloud_ptr( new PC() );
	sensor_msgs::PointCloud2 plane_cloud;
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
	pcl::toROSMsg(*plane_cloud_ptr, plane_cloud);
	plane_cloud.header.stamp = req.input_cloud.header.stamp;
	plane_cloud.header.frame_id = req.input_cloud.header.frame_id;
	ROS_INFO_STREAM("[Rasterizer] Plane RANSAC performed. Plane cloud size is " << plane_cloud_ptr->size());

    // ------------------------------------------------------------------------
	// -------------------------- Transform Cloud --------------------------
	// ------------------------------------------------------------------------
	// ----------- Finding Rotation ----------- 
	// Output Clouds
	sensor_msgs::PointCloud2 rotated_plane;
	PCP rotated_plane_ptr(new PC()); 
	// Rotation --> make normal horizontal
	Eigen::Vector3f input_cloud_norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	Eigen::Vector3f input_cloud_norm_horz(coefficients->values[0], coefficients->values[1], 0.0); 
	//input_cloud_norm_horz.cast<float>();
	Eigen::Quaternion<float> horz_rot;
	horz_rot.FromTwoVectors(input_cloud_norm, input_cloud_norm_horz);
	// Rotation --> make normal in Y+
	Eigen::Vector3f y_axis(1.0, 0.0, 0.0);
	//y_axis.cast<float>();
	Eigen::Quaternion<float> onto_y_rot;
	onto_y_rot.FromTwoVectors(input_cloud_norm_horz, y_axis); 
	// Build Transform
	Eigen::Quaternion<float> final_rotation = horz_rot * onto_y_rot;
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
	tf2::doTransform (plane_cloud, rotated_plane, cloud_rotation);  	// transforms input_pc2 into process_message
	pcl::fromROSMsg(rotated_plane, *rotated_plane_ptr);
	//rotated_plane.header.stamp = req.input_cloud.header.stamp;
	//rotated_plane.header.frame_id = req.input_cloud.header.frame_id;
	ROS_INFO_STREAM("[Rasterizer] Plane cloud rotated. Size is " << rotated_plane.height*rotated_plane.width);

	// ----------- Finding Translation ----------- 
	// Output Clouds
	sensor_msgs::PointCloud2 transformed_plane;
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
	tf2::doTransform (rotated_plane, transformed_plane, cloud_translation);  	// transforms input_pc2 into process_message
	//transformed_plane.header.stamp = req.input_cloud.header.stamp;
	//transformed_plane.header.frame_id = req.input_cloud.header.frame_id;
	pcl::fromROSMsg(transformed_plane, *transformed_plane_ptr);
	ROS_INFO_STREAM("[Rasterizer] Plane cloud translated. Size is " << transformed_plane.height*transformed_plane.width);

	// ------------------------------------------------------------------------
	// ---------------------------- Voxelize Cloud ----------------------------
	// ------------------------------------------------------------------------
	// Output Clouds
	PCP voxelized_cloud_ptr(new PC());
	sensor_msgs::PointCloud2 voxelized_cloud;
	// Create the filtering object, set parameters
	pcl::VoxelGrid<PointType> vg;
	vg.setInputCloud(transformed_plane_ptr);
	float wall_depth = .2;  		// Voxel width in Y direction just needs to be arbitrarily large relative to wall depth variation
	vg.setLeafSize(req.pixel_wdt, wall_depth, req.pixel_hgt); 	
	// Apply Filter and return Voxelized Data
	vg.filter(*voxelized_cloud_ptr);
	toROSMsg(*voxelized_cloud_ptr, voxelized_cloud);
	voxelized_cloud.header.stamp = req.input_cloud.header.stamp;
	voxelized_cloud.header.frame_id = req.input_cloud.header.frame_id;
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
		if(depth_color < 0 || depth_color > 1)
			ROS_ERROR_STREAM("oops " << i << " " << j << " " <<  k << " " << depth_color);
		img.at<cv::Vec3b>(i,j)[2] = floor(depth_color*255); 		// R
		img.at<cv::Vec3b>(i,j)[1] = 0;		// G
		img.at<cv::Vec3b>(i,j)[0] = 255;		// B
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

	res.output_cloud = voxelized_cloud;

}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "rasterizer");

	Rasterizer<pcl::PointXYZ> rasterizer;

}