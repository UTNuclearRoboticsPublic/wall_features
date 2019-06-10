
#include <wall_features/rasterizer.h>
#include <ros/callback_queue.h>
#include <pcl/filters/crop_box.h>

sensor_msgs::PointCloud2 input_cloud;
bool found_pointcloud;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
	if(!found_pointcloud)
	{
		input_cloud = *pointcloud;
		found_pointcloud = true;
	}
}


int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "rasterizer_client");

	ros::NodeHandle nh;


	ros::Publisher input_cloud_pub;
	ros::Publisher output_cloud_pub;
	ros::Publisher image_pub;
	ros::Publisher blurred_depth_pub;
	ros::Publisher image_int_pub;
	ros::Publisher grad_image_pub;
	ros::Publisher lap_image_pub;
	ros::Publisher normals_image_pub;
	input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rasterizer/input_cloud", 1);
	output_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rasterizer/output_cloud", 1);
	image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/image", 1);
	blurred_depth_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/image_blurred", 1);
	image_int_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/image_int", 1);
	grad_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/gradient", 1);
	lap_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/laplacian", 1);
	normals_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/normals", 1);

	float pixel_width, pixel_height;
	nh.param<float>("/rasterizer/pixel_width", pixel_width, 0.02);
	nh.param<float>("/rasterizer/pixel_height", pixel_height, 0.02);
	bool outlier_filter;
	nh.param<bool>("/rasterizer/outlier_filter", outlier_filter, true);
	float outlier_filter_scale;
	nh.param<float>("/rasterizer/outlier_filter_scale", outlier_filter_scale, 0.06);
	float plane_threshold_distance;
	nh.param<float>("/rasterizer/plane_threshold_distance", plane_threshold_distance, 0.05);
	float wall_threshold_distance;
	nh.param<float>("/rasterizer/wall_threshold_distance", wall_threshold_distance, 0.08);
	int rad_outlier_min_neighbors, max_iterations;
	nh.param<int>("/rasterizer/outlier_neighbors", rad_outlier_min_neighbors, 1);
	nh.param<int>("/rasterizer/max_iterations", max_iterations, 10000);

	bool fill_holes;
	int hole_filling_neighbors;
	float hole_filling_max_dist;
	nh.param<bool>("/rasterizer/fill_holes", fill_holes, false);
	nh.param<int>("/rasterizer/hole_filling_neighbors", hole_filling_neighbors, 3);
	nh.param<float>("/rasterizer/hole_filling_max_dist", hole_filling_max_dist, 0.05);

	bool trim_edges;
	std::vector<float> edge_trimming_vector;
	nh.param<bool>("/rasterizer/trim_edges", trim_edges, false);
	if(trim_edges)
		nh.getParam("/rasterizer/trim_vector", edge_trimming_vector);

	bool load_from_bag;
	nh.param<bool>("/rasterizer/load_from_bag", load_from_bag, true);
	// -------------------------------------------------------------------
	// -------------------------- LOAD FROM BAG --------------------------
	// -------------------------------------------------------------------
	if(load_from_bag)
	{
		// ------------- Bag Names and Topics -------------
		std::string cloud_bag_name, cloud_bag_topic;
		nh.param<std::string>("/rasterizer/bag_name", cloud_bag_name, "/home/conor/ros_data/Tunnel_Scans/Nov_06_2017/fast_arm_fore.bag");
		nh.param<std::string>("/rasterizer/bag_topic", cloud_bag_topic, "/laser_mapper/local_dense_cloud");
		ROS_INFO_STREAM("[RasterizerClient] Loading data from bag files.");

		// ------------- First Bag - CLOUD -------------
		rosbag::Bag cloud_bag; 
		cloud_bag.open(cloud_bag_name, rosbag::bagmode::Read);

		std::vector<std::string> topics;
		topics.push_back(cloud_bag_topic);
		rosbag::View view_cloud(cloud_bag, rosbag::TopicQuery(topics));

		BOOST_FOREACH(rosbag::MessageInstance const m, view_cloud)
	    {
	        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
	        if (cloud_ptr != NULL)
	            input_cloud = *cloud_ptr;
	        else
	        	ROS_ERROR_STREAM("[RasterizerClient] Cloud retrieved from bag is null...");
	    }
	    cloud_bag.close(); 
	}
	else
	{
		ros::NodeHandle sub_handle;
		ros::CallbackQueue input_callback_queue;
		sub_handle.setCallbackQueue(&input_callback_queue);

		std::string pointcloud_topic;
		nh.param<std::string>("/rasterizer/cloud_input_topic", pointcloud_topic, "right_wall");
		ros::Subscriber pointcloud_sub = sub_handle.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1, pointcloud_callback);
		while(ros::ok() && !found_pointcloud)
		{
			input_callback_queue.callAvailable(ros::WallDuration());
			ros::Duration(0.1).sleep();
			ROS_INFO_STREAM("waiting for pointcloud input message...");
		}
	}

	if(trim_edges)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::fromROSMsg(input_cloud, *input_cloud_ptr);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::CropBox<pcl::PointXYZI> crop;
		crop.setInputCloud(input_cloud_ptr);
		// Set dimensions of clipping box:
		Eigen::Vector4f min_point = Eigen::Vector4f(edge_trimming_vector[0], -100, edge_trimming_vector[2], 0);
		Eigen::Vector4f max_point = Eigen::Vector4f(edge_trimming_vector[1], 100, edge_trimming_vector[3], 0);
		crop.setMin(min_point);
		crop.setMax(max_point);
		// Set pose of clipping box: 
		Eigen::Vector3f translation = Eigen::Vector3f(0, 0, 0);
		Eigen::Vector3f rotation = Eigen::Vector3f(0, 0, 0);
		crop.setTranslation(translation);
		crop.setRotation(rotation);

		crop.filter(*cropped_cloud);
		pcl::toROSMsg(*cropped_cloud, input_cloud);
	}

	ros::ServiceClient client = nh.serviceClient<wall_features::rasterizer_service>("rasterizer");
	wall_features::rasterizer_service srv;
	srv.request.input_cloud = input_cloud;
	srv.request.outlier_filter = outlier_filter;
	srv.request.outlier_filter_scale = outlier_filter_scale;
	srv.request.rad_outlier_min_neighbors = rad_outlier_min_neighbors;
	srv.request.max_iterations = max_iterations;
	srv.request.plane_threshold_distance = plane_threshold_distance;
	srv.request.wall_threshold_distance = wall_threshold_distance;
	srv.request.pixel_wdt = pixel_width;
	srv.request.pixel_hgt = pixel_height;
	srv.request.fill_holes = fill_holes;
	srv.request.hole_filling_neighbor_count = hole_filling_neighbors;
	srv.request.hole_filling_max_dist = hole_filling_max_dist;

	// Run Service
	while(ros::ok())
	{
		// Wait a moment to ensure that the service is up...
		ros::Duration(1.0).sleep();
		// Call service
		if( ! client.call(srv) )
			ROS_WARN_STREAM("[RasterizerClient] Rasterizer service call failed - prob not up yet");
		else
		{	
			ROS_INFO_STREAM("[RasterizerClient] Successfully called rasterizer service.");
			ROS_INFO_STREAM("[RasterizerClient]   Cloud Size: " << srv.response.output_cloud.height*srv.response.output_cloud.width);
			ros::Duration(0.5).sleep();
			break;
		}
	}	

	input_cloud_pub.publish(input_cloud);
	output_cloud_pub.publish(srv.response.output_cloud);
	//image_pub.publish(srv.response.output_image);

	cv_bridge::CvImagePtr grayscale_ptr(new cv_bridge::CvImage);
	grayscale_ptr = cv_bridge::toCvCopy(srv.response.output_depth_image, sensor_msgs::image_encodings::BGR8);
	cv::cvtColor(grayscale_ptr->image, grayscale_ptr->image, cv::COLOR_BGR2GRAY);


	cv_bridge::CvImagePtr unsmoothed_depth_ptr(new cv_bridge::CvImage);
	unsmoothed_depth_ptr = cv_bridge::toCvCopy(srv.response.output_depth_image, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr unsmoothed_intensity_ptr(new cv_bridge::CvImage);
	unsmoothed_intensity_ptr = cv_bridge::toCvCopy(srv.response.output_intensity_image, sensor_msgs::image_encodings::BGR8);



	//cv_bridge::CvImagePtr raster_image_ptr(new cv_bridge::CvImage);
	cv_bridge::CvImagePtr gradient_image(new cv_bridge::CvImage);
	sensor_msgs::Image gradient_msg;
	//cv::Mat gradient_mat(res.image_wdt, res.image_hgt, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat gradient_mat;
	int blur_size;
	nh.param<int>("rasterizer/blur_size", blur_size, 10);
	ROS_INFO_STREAM("image size: " << grayscale_ptr->image.rows << "x" << grayscale_ptr->image.cols << " with blur size " << blur_size);
	cv::GaussianBlur(grayscale_ptr->image, grayscale_ptr->image, cv::Size(blur_size,blur_size), 0, 0, cv::BORDER_DEFAULT);
	ROS_INFO_STREAM("made the blur");
	sensor_msgs::Image blurred_depth_msg;
	grayscale_ptr->toImageMsg(blurred_depth_msg);
	blurred_depth_msg.encoding = "mono8";
	blurred_depth_pub.publish(blurred_depth_msg);


	cv::Mat gradient_x, gradient_y;
	cv::Mat abs_gradient_x, abs_gradient_y;
	int ddepth = 3; // cv::CV_16S
	cv::Sobel(grayscale_ptr->image, gradient_x, ddepth, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::Sobel(grayscale_ptr->image, gradient_y, ddepth, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs( gradient_x, abs_gradient_x );
	cv::convertScaleAbs( gradient_y, abs_gradient_y );
	cv::addWeighted( abs_gradient_x, 0.5, abs_gradient_y, 0.5, 0, gradient_image->image );
	gradient_image->image = gradient_image->image*10;
	gradient_image->toImageMsg(gradient_msg);
	gradient_msg.encoding = "mono8"; 
	grad_image_pub.publish(gradient_msg);

	cv_bridge::CvImage::Ptr laplacian(new cv_bridge::CvImage);
	cv_bridge::CvImage::Ptr laplacian_abs(new cv_bridge::CvImage);
	sensor_msgs::Image laplacian_msg;
	cv::Laplacian(grayscale_ptr->image, laplacian->image, ddepth, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(laplacian->image, laplacian_abs->image);
	laplacian_abs->image = laplacian_abs->image*10; 
	laplacian_abs->toImageMsg(laplacian_msg);
	laplacian_msg.encoding = "mono8";
	lap_image_pub.publish(laplacian_msg);

	int flop = 1;
	cv_bridge::CvImage::Ptr normals_ptr(new cv_bridge::CvImage);
	//if(grayscale_ptr->image.type() != CV_32FC1)
	//{
	//	ROS_WARN_STREAM("depth type is " << grayscale_ptr->image.type() << " instead of " << CV_32FC1 << " as it should be. Converting.");
	//	grayscale_ptr->image.convertTo(grayscale_ptr->image, CV_32FC1);
	//}
	cv::GaussianBlur(grayscale_ptr->image, grayscale_ptr->image, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::Mat normals(grayscale_ptr->image.size(),CV_8UC3);
	cv::Mat normal_gray(grayscale_ptr->image.size(), CV_8UC1);
	sensor_msgs::Image normals_msg;
	int min_x = 30;
	int max_x = 0;
	for(int x = 1; x < grayscale_ptr->image.cols - 1; ++x)
	{
	    for(int y = 1; y < grayscale_ptr->image.rows - 1; ++y)
	    {
	        /*double dzdx = (depth(y, x+1) - depth(y, x-1)) / 2.0;
	        double dzdy = (depth(y+1, x) - depth(y-1, x)) / 2.0;
	        Vec3d d = (-dzdx, -dzdy, 1.0);*/
	        //cv::Vec3b t(x,y-1,grayscale_ptr->image.at<uchar>(y-1, x)/*depth(y-1,x)*/);
	        //cv::Vec3b l(x-1,y,grayscale_ptr->image.at<uchar>(y, x-1)/*depth(y,x-1)*/);
	        //cv::Vec3b c(x,y,grayscale_ptr->image.at<uchar>(y, x)/*depth(y,x)*/);	
	        //cv::Vec3b d = (l-c).cross(t-c);
	        //cv::Vec3b n = cv::normalize(d);

	    	//float dzdx = float(grayscale_ptr->image.at<uchar>(x+1,y) - grayscale_ptr->image.at<uchar>(x-1,y)) / 2.0;
	    	//float dzdy = float(grayscale_ptr->image.at<uchar>(x,y+1) - grayscale_ptr->image.at<uchar>(x,y-1)) / 2.0;
	    	//cv::Vec3f d(-dzdx,-dzdy, 1.0f);
	    	//cv::Vec3f n = normalize(d);
	    	//ROS_INFO_STREAM("input" << " " << float(gradient_x.at<uchar>(x,y)) << " " << float(gradient_y.at<uchar>(x,y)));
	        cv::Vec3f normal_color_float;
	        //normal_color_float[0] = float(gradient_x.at<uchar>(y,x));
	        //normal_color_float[1] = float(gradient_y.at<uchar>(y,x));
	        normal_color_float[0] = -(float(grayscale_ptr->image.at<uchar>(y,x+1)) - float(grayscale_ptr->image.at<uchar>(y,x-1))) / 2.0;
	        normal_color_float[1] = -(float(grayscale_ptr->image.at<uchar>(y+1,x)) - float(grayscale_ptr->image.at<uchar>(y-1,x))) / 2.0;
	        normal_color_float[2] = 1.0f;
	        normal_color_float = normalize(normal_color_float);
	        //ROS_INFO_STREAM(normal_color_float[0] << " " << normal_color_float[1] << " " << normal_color_float[2]);
	        if(normal_color_float[0] < 0)
	        	normal_color_float[0] = 0;
	        if(normal_color_float[0] > 255)
	        	normal_color_float[0] = 255;
	        if(normal_color_float[1] < 0)
	        	normal_color_float[1] = 0;
	        if(normal_color_float[1] > 255)
	        	normal_color_float[1] = 255;
	        if(normal_color_float[2] < 0)
	        	normal_color_float[2] = 0;
	        if(normal_color_float[2] > 255)
	        	normal_color_float[2] = 255;
	        normals.at<cv::Vec3b>(y,x)[0] = int(floor(normal_color_float[0]*255));
	        normals.at<cv::Vec3b>(y,x)[1] = int(floor(normal_color_float[1]*255));
	        normals.at<cv::Vec3b>(y,x)[2] = int(floor(normal_color_float[2]*255));

	        if(normal_color_float[2] == 0)
	        	normal_gray.at<uchar>(y,x) = 0;
	        else
	        	normal_gray.at<uchar>(y,x) = int(floor( atan(sqrt( float(pow(normal_color_float[0], 2) + pow(normal_color_float[1], 2)) ) /normal_color_float[2]) *100 ));
	        //ROS_INFO_STREAM(float(normals.at<cv::Vec3b>(x,y)[0]) << " " << float(normals.at<cv::Vec3b>(x,y)[1]) << " " << float(normals.at<cv::Vec3b>(x,y)[2]));

	        //normals.at<cv::Vec3f>(y,x)
	        //ROS_INFO_STREAM(x << " " << y << " " << dzdx << " " << dzdy << " " << d[0] << " " << d[1] << " " << d[2] << " " << n[0] << " " << n[1] << " " << n[2]);
	        //normals.at<cv::Vec3b>(y,x)[1] = n[1]*10;
	        //normals.at<cv::Vec3b>(y,x)[2] = n[2]*10;
	        //normals.at<cv::Vec3d>(y,x) = n;

	        //if(min_x > n[0])
	        //	min_x = n[0];
	        //if(max_x < n[0])
	        //	max_x = n[0];
	    }
	    ROS_INFO_STREAM(x << " " << grayscale_ptr->image.cols);
	}
	//cv::GaussianBlur(normal_gray, normal_gray, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	normal_gray.copyTo(normals_ptr->image);
	normals_ptr->toImageMsg(normals_msg);
	normals_msg.encoding = "mono8";
	normals_image_pub.publish(normals_msg);

	float min_val = 100;
	float max_val = 0;
	for(int i=0; i<srv.response.image_hgt; i++)
	{
		for(int j=0; j<srv.response.image_wdt; j++)
		{
			if(min_val > grayscale_ptr->image.at<uchar>(i,j) && grayscale_ptr->image.at<uchar>(i,j) != 0)
				min_val = grayscale_ptr->image.at<uchar>(i,j);
			if(max_val < grayscale_ptr->image.at<uchar>(i,j))
				max_val = grayscale_ptr->image.at<uchar>(i,j);
		}
	}
	min_val = 20;
	for(int i=0; i<srv.response.image_hgt; i++)
	{
		for(int j=0; j<srv.response.image_wdt; j++)
		{
			grayscale_ptr->image.at<uchar>(i,j) = (grayscale_ptr->image.at<uchar>(i,j)-min_val) * 255 / (max_val-min_val);
		}
	}
	ROS_INFO_STREAM("range: " << min_val << " " << max_val);
	sensor_msgs::Image grayscale_msg;
	grayscale_ptr->toImageMsg(grayscale_msg);
	grayscale_msg.encoding = "mono8";
	image_pub.publish(srv.response.output_depth_image);
	image_int_pub.publish(srv.response.output_intensity_image);

	std::vector<int> low_compression_params;
    low_compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    low_compression_params.push_back(0);


    // Save Output Images
	cv::imwrite("raster_output_depth_low.png", unsmoothed_depth_ptr->image, low_compression_params);
	cv::imwrite("raster_output_intensity_low.png", unsmoothed_intensity_ptr->image, low_compression_params);
	// Save Point Cloud Raster
	rosbag::Bag bag;
	std::string bag_name = "rasterized_cloud.bag";
	bag.open(bag_name, rosbag::bagmode::Write);
	bag.write("rasterized_cloud", ros::Time::now(), srv.response.output_cloud);
	ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name);


	ros::Duration(3).sleep();

}