
#include <wall_features/rasterizer.h>
#include <ros/callback_queue.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <Eigen/Dense>

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
	ros::init(argc, argv, "raster_analysis");

	ros::NodeHandle nh;

	bool load_from_bag = false;

	ros::Publisher input_pub;
	ros::Publisher image_pub;
	ros::Publisher grad_image_pub;
	ros::Publisher lap_image_pub;
	ros::Publisher normals_image_pub;
	input_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/input_image", 1);
	image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/image", 1);
	grad_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/gradient", 1);
	lap_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/laplacian", 1);
	normals_image_pub = nh.advertise<sensor_msgs::Image>("/rasterizer/normals", 1);

	// -------------------------------------------------------------------
	// -------------------------- LOAD FROM PNG --------------------------
	// -------------------------------------------------------------------
	cv::Mat input_image = cv::imread("/home/conor/Downloads/wall_scan_intensity.bmp", CV_LOAD_IMAGE_COLOR);

	cv_bridge::CvImagePtr grayscale_ptr(new cv_bridge::CvImage);
	input_image.copyTo(grayscale_ptr->image);
	sensor_msgs::Image input_msg;
	grayscale_ptr->toImageMsg(input_msg);
	input_msg.encoding = sensor_msgs::image_encodings::BGR8;
	ROS_INFO_STREAM("about to publish input... " << input_msg.height << " " << input_msg.width << " " << input_msg.data.size());
	input_msg.header.stamp = ros::Time::now();
	input_msg.header.frame_id = "map";
	input_pub.publish(input_msg);

	//cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    //cv::imshow("Display Image", input_image);

	cv::cvtColor(grayscale_ptr->image, grayscale_ptr->image, cv::COLOR_BGR2GRAY);
	cv::Size s = grayscale_ptr->image.size();

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

	float min_val = grayscale_ptr->image.at<uchar>(0,0);
	float max_val = grayscale_ptr->image.at<uchar>(0,0);
	for(int i=0; i<s.height; i++)
	{
		for(int j=0; j<s.width; j++)
		{
			if(min_val > grayscale_ptr->image.at<uchar>(i,j) && grayscale_ptr->image.at<uchar>(i,j) != 0)
				min_val = grayscale_ptr->image.at<uchar>(i,j);
			if(max_val < grayscale_ptr->image.at<uchar>(i,j))
				max_val = grayscale_ptr->image.at<uchar>(i,j);
		}
	}
	//min_val = 20;
	for(int i=0; i<s.height; i++)
	{
		for(int j=0; j<s.width; j++)
		{
			grayscale_ptr->image.at<uchar>(i,j) = (grayscale_ptr->image.at<uchar>(i,j)-min_val) * 255 / (max_val-min_val);
		}
	}
	ROS_INFO_STREAM("range: " << min_val << " " << max_val << "   size: " << s.height << " " << s.width);
	sensor_msgs::Image grayscale_msg;
	grayscale_ptr->toImageMsg(grayscale_msg);
	grayscale_msg.encoding = "mono8";
	image_pub.publish(grayscale_msg);


	ros::Duration(3).sleep();

}