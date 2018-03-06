
#include <wall_features/rasterizer.h>
#include <ros/callback_queue.h>

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

	bool load_from_bag = false;

	ros::Publisher input_cloud_pub;
	ros::Publisher output_cloud_pub;
	ros::Publisher image_pub;
	ros::Publisher grad_image_pub;
	ros::Publisher lap_image_pub;
	ros::Publisher normals_image_pub;
	input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rasterizer/input_cloud", 1);
	output_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rasterizer/output_cloud", 1);
	image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/image", 1);
	grad_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/gradient", 1);
	lap_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/laplacian", 1);
	normals_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/normals", 1);

	float pixel_width, pixel_height;
	nh.param<float>("/rasterizer/pixel_width", pixel_width, 0.02);
	nh.param<float>("/rasterizer/pixel_height", pixel_height, 0.02);
	float outlier_filter_scale;
	nh.param<float>("/rasterizer/outlier_filter_scale", outlier_filter_scale, 0.06);
	float plane_threshold_distance;
	nh.param<float>("/rasterizer/plane_threshold_distance", plane_threshold_distance, 0.05);
	int rad_outlier_min_neighbors, max_iterations;
	nh.param<int>("/rasterizer/outlier_neighbors", rad_outlier_min_neighbors, 1);
	nh.param<int>("/rasterizer/max_iterations", max_iterations, 10000);

	// -------------------------------------------------------------------
	// -------------------------- LOAD FROM BAG --------------------------
	// -------------------------------------------------------------------
	if(load_from_bag)
	{
		// ------------- Bag Names and Topics -------------
		std::string cloud_bag_name = 	"/home/conor/catkin-ws/data/Tunnel_Scans/Nov_06_2017/fast_arm_fore.bag";
		std::string cloud_bag_topic = 	"/laser_stitcher/local_dense_cloud";
		std::string front_bag_topic = 	"front_camera/image_raw";
		std::string rear_bag_topic = 	"rear_camera/image_raw";
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

	ros::ServiceClient client = nh.serviceClient<wall_features::rasterizer_srv>("rasterizer");
	wall_features::rasterizer_srv srv;
	srv.request.input_cloud = input_cloud;
	srv.request.rad_outlier_min_neighbors = rad_outlier_min_neighbors;
	srv.request.max_iterations = max_iterations;
	srv.request.threshold_distance = plane_threshold_distance;
	srv.request.pixel_wdt = pixel_width;
	srv.request.pixel_hgt = pixel_height;
	srv.request.outlier_filter_scale = outlier_filter_scale;

	// Run Service
	while(ros::ok())
	{
		// Wait a moment to ensure that the service is up...
		ros::Duration(1.0).sleep();
		// Call service
		if( ! client.call(srv) )
			ROS_WARN_STREAM("[RasterizerClient] Painting service call failed - prob not up yet");
		else
		{	
			ROS_INFO_STREAM("[RasterizerClient] Successfully called painting service.");
			ROS_INFO_STREAM("[RasterizerClient]   Cloud Size: " << srv.response.output_cloud.height*srv.response.output_cloud.width);
			ros::Duration(0.5).sleep();
			break;
		}
	}	

	input_cloud_pub.publish(input_cloud);
	output_cloud_pub.publish(srv.response.output_cloud);
	//image_pub.publish(srv.response.output_image);

	cv_bridge::CvImagePtr grayscale_ptr(new cv_bridge::CvImage);
	grayscale_ptr = cv_bridge::toCvCopy(srv.response.output_image, sensor_msgs::image_encodings::BGR8);
	cv::cvtColor(grayscale_ptr->image, grayscale_ptr->image, cv::COLOR_BGR2GRAY);
	grayscale_ptr->image = grayscale_ptr->image*2;
	sensor_msgs::Image grayscale_msg;
	grayscale_ptr->toImageMsg(grayscale_msg);
	grayscale_msg.encoding = "mono8";
	image_pub.publish(grayscale_msg);

	//cv_bridge::CvImagePtr raster_image_ptr(new cv_bridge::CvImage);
	cv_bridge::CvImagePtr gradient_image(new cv_bridge::CvImage);
	sensor_msgs::Image gradient_msg;
	//cv::Mat gradient_mat(res.image_wdt, res.image_hgt, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat gradient_mat;
	cv::GaussianBlur(grayscale_ptr->image, grayscale_ptr->image, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	
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

	ros::Duration(3).sleep();

}