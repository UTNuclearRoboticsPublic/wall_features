
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include "wall_features/rasterizer_service.h"

class RasterizerPublisher
{
public:
	RasterizerPublisher();
	RasterizerPublisher(std::string name);
	RasterizerPublisher(wall_features::rasterizer_service srv, std::string name);
	void setName(std::string name);
	void populate(wall_features::rasterizer_service srv);
	void publish();
	void publish(wall_features::rasterizer_service srv);

private:
	void initializePublishers(std::string name);
	
	ros::NodeHandle nh_;
	wall_features::rasterizer_service srv_;

	ros::Publisher input_pub_;
	ros::Publisher outlierless_pub_;
	ros::Publisher plane_pub_;
	ros::Publisher rotated_pub_;
	ros::Publisher translated_pub_;
	ros::Publisher voxelized_pub_;
	ros::Publisher output_pub_;
	ros::Publisher image_pub_;

	bool has_data_;
	bool publishers_initialized_;
};