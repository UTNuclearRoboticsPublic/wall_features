
#include "wall_features/wall_damage_estimator_server.h"

WallDamageEstimatorServer::WallDamageEstimatorServer():
	input_cloud_ptr_(new PC()),
	segmentation_input_ptr_(new PC()),
	wall_damage_input_ptr_(new PC()),
	wall_damage_histogram_input_ptr_(new PC()),
	wall_cloud_ptr_(new PC()),
	wall_damage_ptr_(new pcl::PointCloud<pcl::PointWallDamage>()),
	wall_damage_histogram_ptr_(new pcl::PointCloud<pcl::WallDamageHistogram>())
{
	// Create publishers
	input_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/wall_damage/input_cloud", 1, this);
	damage_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/wall_damage/damage_cloud", 1, this);
	damage_histogram_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/wall_damage/damage_histogram_cloud", 1, this);
	// Create service clients and server
	wall_finder_ = nh_.serviceClient<pointcloud_primitive_search::primitive_process>("primitive_search");
	damage_estimator_server_ = nh_.advertiseService("/wall_damage_estimation", &WallDamageEstimatorServer::wallDamageCallback, this);
	// Wait for service inputs 
	ros::spin();
}

bool WallDamageEstimatorServer::resetServer(std::string primitive_search_name)
{
	PrimitiveProcessCreation::createProcesses(&wall_process_, primitive_search_name);
	primitive_pub_.updatePublishers(wall_process_);
}

// *** SERVICE ***
//   Perform actual service execution
//   1) Load data (from client or bag file)
//   2) Perform any voxelization requested
//   3) Locate wall (RANSAC segmentation or client-provided)
//   4) Generate wall damage cloud (DAMAGE_CLOUD)
//   5) Generate features cloud (DAMAGE_HISTOGRAM_CLOUD)
bool WallDamageEstimatorServer::wallDamageCallback(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res)
{
	// Initialze PCL Clouds and Services
	resetServer(req.primitive_search_name);

	// -------------------------------------------------------------------
	// 1) INPUT - get and check size of input cloud 
	if(req.load_from_bags)
		inputFromBag(req);
	// Check minimum cloud size
	ROS_INFO_STREAM("[WallDamageEstimator] Input cloud is of size " << req.input_cloud.height * req.input_cloud.width);
	if(req.input_cloud.height * req.input_cloud.width == 0)
	{
		ROS_ERROR_STREAM("[WallDamageEstimator] Input cloud is too small! Exiting service as failed...");
		return false;
	}
	// Create PCL cloud type
	pcl::fromROSMsg(req.input_cloud, *input_cloud_ptr_);

	// -------------------------------------------------------------------
	// 2) VOXELIZATION - downsample clouds to required density
	//   The size of these clouds determines the size of the outputs from these respective processes
	voxelizeCloud(input_cloud_ptr_, segmentation_input_ptr_, req.segmentation_cloud_voxel_size, "Segmentation Input Cloud");
	voxelizeCloud(input_cloud_ptr_, wall_damage_input_ptr_, req.wall_damage_voxel_size, "Wall Damage Input Cloud");
	voxelizeCloud(input_cloud_ptr_, wall_damage_histogram_input_ptr_, req.wall_damage_histogram_voxel_size, "Wall Damage Histogram Input Cloud");

	// -------------------------------------------------------------------
	// 3) Wall Coefficient Determination
	//   By default this will use planar RANSAC to segment out a plane from the input cloud and determine its coefficients
	//   This information is used later to determine the offset of all points in the wall from their expected positions
	if( !planeSegmentation(res) )
		return false;

	// -------------------------------------------------------------------
	// 4) Generate Wall Damage Cloud
	//   Generate cloud of offsets from expected cloud in position and surface normals
	estimateWallDamage(req, res);

	// -------------------------------------------------------------------
	// 5) Generate Wall Damage Histogram Cloud
	//   Generate cloud of local histograms of offsets from expected cloud in position and surface normals
	estimateWallDamageHistogram(req, res);

	// -------------------------------------------------------------------
	// 6) Publish Outputs
	publishClouds(req, res); 

	return true;
}


bool WallDamageEstimatorServer::inputFromBag(wall_features::wall_damage_service::Request &req)
{
    ROS_INFO_STREAM("[WallDamageEstimator] Loading clouds from bag files, using bag name: " << req.bag_name << " and topic name: " << req.bag_topic << ".");
	// Open Bag
    rosbag::Bag input_bag; 
    input_bag.open(req.bag_name, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics;
    topics.push_back(req.bag_topic);
    rosbag::View view_1(input_bag, rosbag::TopicQuery(topics));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view_1)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            req.input_cloud = *cloud_ptr;
        else
          ROS_ERROR_STREAM("[WallDamageEstimator] Cloud caught for first cloud is null...");
    }
    input_bag.close();

    return true;
}



bool WallDamageEstimatorServer::voxelizeCloud(PCP input_cloud_ptr, PCP output_cloud_ptr, float leaf_size, std::string cloud_name)
{
	pcl::VoxelGrid<PointType> vg_primitive;
	vg_primitive.setInputCloud(input_cloud_ptr);
	vg_primitive.setLeafSize(leaf_size, leaf_size, leaf_size);
	vg_primitive.filter(*output_cloud_ptr); 
	ROS_INFO_STREAM("[WallDamageEstimator] Performed voxelization on cloud " << cloud_name << " - new cloud size is " << output_cloud_ptr->points.size());

	return true;
}



bool WallDamageEstimatorServer::planeSegmentation(wall_features::wall_damage_service::Response &res)
{
	// --------------------------------------------- Find Wall Within Cloud ---------------------------------------------
	pcl::toROSMsg(*segmentation_input_ptr_, wall_process_.request.pointcloud);
	int process_size;

	if(!wall_finder_.call(wall_process_))
	{
		ROS_ERROR_STREAM("[WallDamageEstimator] Failed to perform Primitive_Search on input cloud. Returning service as failed.");
		return false;
	}
	else
	{
		process_size = wall_process_.response.outputs.size();
		wall_cloud_ = wall_process_.response.outputs[process_size-1].task_results[1].task_pointcloud;
		ROS_INFO_STREAM("[WallDamageEstimator] Successfully called PrimitiveSearch on input cloud. Output wall cloud size: " << wall_cloud_.height*wall_cloud_.width);
	}
	pcl::fromROSMsg(wall_cloud_, *wall_cloud_ptr_);

	ROS_INFO_STREAM("coeffs: " << wall_process_.response.outputs[0].task_results[1].primitive_coefficients.size());
	for(int i=0; i<wall_process_.response.outputs[0].task_results[1].primitive_coefficients.size(); i++)
	{
		ROS_INFO_STREAM("[WallDamageEstimator]   Plane Coefficient " << i << ": " << wall_process_.response.outputs[0].task_results[1].primitive_coefficients[i]);
		res.wall_coefficients.push_back(wall_process_.response.outputs[0].task_results[1].primitive_coefficients[i]);
	}

	return true;
}


bool WallDamageEstimatorServer::estimateWallDamage(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res)
{
	float wall_coefficients_array[4];
	for(int i=0; i<res.wall_coefficients.size(); i++)
		wall_coefficients_array[i] = res.wall_coefficients[i];
	point_damage_estimator_.setWallCoefficients(wall_coefficients_array);
	point_damage_estimator_.setKSearch(req.k_search_normals);
	point_damage_estimator_.compute(*wall_damage_input_ptr_, *wall_damage_ptr_);
	ROS_INFO_STREAM("[WallDamageEstimator] Performed pointwise damage estimation - output cloud size is " << wall_damage_ptr_->points.size());
	pcl::toROSMsg(*wall_damage_ptr_, res.wall_damage_cloud);
	res.wall_damage_cloud.header = req.input_cloud.header;

	return true;
}


bool WallDamageEstimatorServer::estimateWallDamageHistogram(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res)
{	
	if(req.automatically_set_bins)
	{
		req.lower_angle_bin_limit = wall_damage_ptr_->points[0].angle_offset;
		req.upper_angle_bin_limit = req.lower_angle_bin_limit;
		req.lower_dist_bin_limit = wall_damage_ptr_->points[0].dist_offset;
		req.upper_dist_bin_limit = req.lower_dist_bin_limit;
		for(int i=0; i<wall_damage_ptr_->size(); i++)
		{
			if(wall_damage_ptr_->points[i].angle_offset < req.lower_angle_bin_limit)
				req.lower_angle_bin_limit = wall_damage_ptr_->points[i].angle_offset;
			if(wall_damage_ptr_->points[i].angle_offset > req.upper_angle_bin_limit)
				req.upper_angle_bin_limit = wall_damage_ptr_->points[i].angle_offset;
			if(wall_damage_ptr_->points[i].dist_offset < req.lower_dist_bin_limit)
				req.lower_dist_bin_limit = wall_damage_ptr_->points[i].dist_offset;
			if(wall_damage_ptr_->points[i].dist_offset > req.upper_dist_bin_limit)
				req.upper_dist_bin_limit = wall_damage_ptr_->points[i].dist_offset;
		}
	}
	ROS_DEBUG_STREAM("[WallDamageEstimator] Bin limits: " << req.lower_angle_bin_limit << " " << req.upper_angle_bin_limit << " " << req.lower_dist_bin_limit << " " << req.upper_dist_bin_limit);
	damage_histogram_estimator_.setBinLimits(req.lower_angle_bin_limit, req.upper_angle_bin_limit, req.lower_dist_bin_limit, req.upper_dist_bin_limit);
	damage_histogram_estimator_.setKSearch(req.k_search_histogram);

	damage_histogram_estimator_.compute(*wall_damage_ptr_, *wall_damage_histogram_input_ptr_, *wall_damage_histogram_ptr_);
	pcl::toROSMsg(*wall_damage_histogram_ptr_, res.wall_damage_histogram_cloud);
	res.wall_damage_histogram_cloud.header = res.wall_damage_cloud.header;
	ROS_INFO_STREAM("[WallDamageEstimator] Performed histogram cloud estimation. Final cloud size is " << wall_damage_histogram_ptr_->points.size());

	return true;
}


bool WallDamageEstimatorServer::publishClouds(wall_features::wall_damage_service::Request &req, wall_features::wall_damage_service::Response &res)
{
	input_cloud_pub_.publish(req.input_cloud);
	damage_cloud_pub_.publish(res.wall_damage_cloud);
	damage_histogram_cloud_pub_.publish(res.wall_damage_histogram_cloud);

	return true;
}


int main (int argc, char **argv)
{ 
  ros::init(argc, argv, "wall_damage_estimator");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); 

  WallDamageEstimatorServer server;
}