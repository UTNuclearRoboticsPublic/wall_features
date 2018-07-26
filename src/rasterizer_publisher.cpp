
#include "wall_features/rasterizer_publisher.h"

RasterizerPublisher::RasterizerPublisher()
{
	publishers_initialized_ = false;
	has_data_ = false;
}

RasterizerPublisher::RasterizerPublisher(std::string name)
{
	initializePublishers(name);
	has_data_ = false;
}

RasterizerPublisher::RasterizerPublisher(wall_features::rasterizer_service srv, std::string name)
{
	populate(srv);
	initializePublishers(name);
}

void RasterizerPublisher::setName(std::string name)
{
	initializePublishers(name);
}

void RasterizerPublisher::initializePublishers(std::string name)
{
	input_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/input", 1, this);
	outlierless_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/no_outliers", 1, this);
	plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/plane", 1, this);
	rotated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/rotated", 1, this);
	translated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/translated", 1, this);
	voxelized_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/voxelized", 1, this);
	output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/output", 1, this);
	image_pub_ = nh_.advertise<sensor_msgs::Image>(name + "/image", 1, this);

	publishers_initialized_ = true;
}

void RasterizerPublisher::populate(wall_features::rasterizer_service srv)
{
	srv_ = srv;
	has_data_ = true;
}

void RasterizerPublisher::publish()
{
	if(!has_data_)
		ROS_ERROR("[RasterizerPublisher] Asked to publish outputs, but this object hasn't been initialized yet with any output data.");
	else if(!publishers_initialized_)
		ROS_ERROR_STREAM("[RasterizerPublisher] Asked to publish outputs, but publishers haven't yet been initialized.");
	else 
	{
		input_pub_.publish(srv_.request.input_cloud);
		outlierless_pub_.publish(srv_.response.outlierless_cloud);
		plane_pub_.publish(srv_.response.plane_cloud);
		rotated_pub_.publish(srv_.response.rotated_cloud);
		translated_pub_.publish(srv_.response.transformed_cloud);
		output_pub_.publish(srv_.response.output_cloud);
		image_pub_.publish(srv_.response.output_image);
	}
}

void RasterizerPublisher::publish(wall_features::rasterizer_service srv)
{
	populate(srv);
	publish();
}