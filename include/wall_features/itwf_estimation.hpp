

#ifndef IMPL_ITWF_ESTIMATION_
#define IMPL_ITWF_ESTIMATION_

#include "itwf_estimation.h"

/* temp - list of parameters to be determined:
  -- PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  -- PCL_ADD_NORMAL4D; 				// This adds the member normal[3] which can also be accessed using the point (which is float[4])
  float angle_offset_avg; 			// Average offset in angle between local normal vectors of each neighbor point and the expected normal vector of the containing plane primitive definition
  float depth_offset_avg; 			// Average offset in position between each neighbor point and the containing plane primitive definition
  float	histogram[80];
*/
namespace pcl
{ 
// -----------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- ITWF Estimation ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature90>::initializeOutputPoint(pcl::ITWFSignature90 &histogram_point, pcl::PointWallDamage input_point)
	{
		// Initialize Euclidian and Histogram
		histogram_point.x = input_point.x;
		histogram_point.y = input_point.y;
		histogram_point.z = input_point.z;
		for(int i=0; i<histogram_point.descriptorSize(); i++)
			histogram_point.histogram[i] = 0;
		// Initialize Basal Averages
		histogram_point.depth_offset_avg = 0;
		histogram_point.horz_angle_offset_avg = 0;
		histogram_point.second_angle_offset_avg = 0;
		// Initialize Partial Derivative Averages
		histogram_point.depth_offset_deriv_horz_avg = 0;
		histogram_point.depth_offset_deriv_second_avg = 0;
		histogram_point.horz_angle_offset_deriv_horz_avg = 0;
		histogram_point.horz_angle_offset_deriv_second_avg = 0;
		histogram_point.second_angle_offset_deriv_horz_avg = 0;
		histogram_point.second_angle_offset_deriv_second_avg = 0;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature210>::initializeOutputPoint(pcl::ITWFSignature210 &histogram_point, pcl::PointWallDamage input_point)
	{
		// Initialize Euclidian and Histogram
		histogram_point.x = input_point.x;
		histogram_point.y = input_point.y;
		histogram_point.z = input_point.z;
		for(int i=0; i<histogram_point.descriptorSize(); i++)
			histogram_point.histogram[i] = 0;
		// Initialize Basal Averages
		histogram_point.depth_offset_avg = 0;
		histogram_point.horz_angle_offset_avg = 0;
		histogram_point.second_angle_offset_avg = 0;
		histogram_point.r_avg = 0;
		histogram_point.g_avg = 0;
		histogram_point.b_avg = 0;
		histogram_point.intensity_avg = 0;
		// Initialize Partial Derivative Averages
		histogram_point.depth_offset_deriv_horz_avg = 0;
		histogram_point.depth_offset_deriv_second_avg = 0;
		histogram_point.horz_angle_offset_deriv_horz_avg = 0;
		histogram_point.horz_angle_offset_deriv_second_avg = 0;
		histogram_point.second_angle_offset_deriv_horz_avg = 0;
		histogram_point.second_angle_offset_deriv_second_avg = 0;
		histogram_point.intensity_deriv_horz_avg = 0;
		histogram_point.intensity_deriv_second_avg = 0;
		histogram_point.r_deriv_horz_avg = 0;
		histogram_point.r_deriv_second_avg = 0;
		histogram_point.g_deriv_horz_avg = 0;
		histogram_point.g_deriv_second_avg = 0;
		histogram_point.b_deriv_horz_avg = 0;
		histogram_point.b_deriv_second_avg = 0;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature90>::incrementAverages(pcl::ITWFSignature90 &histogram_point, pcl::PointWallDamage neighbor_point)
	{
		histogram_point.depth_offset_avg += neighbor_point.depth_offset;
		histogram_point.horz_angle_offset_avg += neighbor_point.horz_normal_offset;
		histogram_point.second_angle_offset_avg += neighbor_point.second_normal_offset;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature210>::incrementAverages(pcl::ITWFSignature210 &histogram_point, pcl::PointWallDamage neighbor_point)
	{
		histogram_point.depth_offset_avg += neighbor_point.depth_offset;
		histogram_point.horz_angle_offset_avg += neighbor_point.horz_normal_offset;
		histogram_point.second_angle_offset_avg += neighbor_point.second_normal_offset;
		histogram_point.r_avg += neighbor_point.r;
		histogram_point.g_avg += neighbor_point.g;
		histogram_point.b_avg += neighbor_point.b;
		histogram_point.intensity_avg += neighbor_point.intensity;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature90>::normalizeValues(pcl::ITWFSignature90 &histogram_point, int num_points)
	{
		// Normalize Histogram
		for(int i=0; i<histogram_point.descriptorSize(); i++)
			histogram_point.histogram[i] /= num_points;
		// Normalize Basal Averages
		histogram_point.depth_offset_avg /= num_points;
		histogram_point.horz_angle_offset_avg /= num_points;
		histogram_point.second_angle_offset_avg /= num_points;
		// Normalize Partial Derivative Averages
		histogram_point.depth_offset_deriv_horz_avg /= num_points;
		histogram_point.depth_offset_deriv_second_avg /= num_points;
		histogram_point.horz_angle_offset_deriv_horz_avg /= num_points;
		histogram_point.horz_angle_offset_deriv_second_avg /= num_points;
		histogram_point.second_angle_offset_deriv_horz_avg /= num_points;
		histogram_point.second_angle_offset_deriv_second_avg /= num_points;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature210>::normalizeValues(pcl::ITWFSignature210 &histogram_point, int num_points)
	{
		// Normalize Histogram
		for(int i=0; i<histogram_point.descriptorSize(); i++)
			histogram_point.histogram[i] /= num_points;
		// Normalize Basal Averages
		histogram_point.depth_offset_avg /= num_points;
		histogram_point.horz_angle_offset_avg /= num_points;
		histogram_point.second_angle_offset_avg /= num_points;
		histogram_point.r_avg /= num_points;
		histogram_point.g_avg /= num_points;
		histogram_point.g_avg /= num_points;
		histogram_point.intensity_avg /= num_points;
		// Normalize Partial Derivative Averages
		histogram_point.depth_offset_deriv_horz_avg /= num_points;
		histogram_point.depth_offset_deriv_second_avg /= num_points;
		histogram_point.horz_angle_offset_deriv_horz_avg /= num_points;
		histogram_point.horz_angle_offset_deriv_second_avg /= num_points;
		histogram_point.second_angle_offset_deriv_horz_avg /= num_points;
		histogram_point.second_angle_offset_deriv_second_avg /= num_points;
		histogram_point.intensity_deriv_horz_avg /= num_points;
		histogram_point.intensity_deriv_second_avg /= num_points;
		histogram_point.r_deriv_horz_avg /= num_points;
		histogram_point.r_deriv_second_avg /= num_points;
		histogram_point.g_deriv_horz_avg /= num_points;
		histogram_point.g_deriv_second_avg /= num_points;
		histogram_point.b_deriv_horz_avg /= num_points;
		histogram_point.b_deriv_second_avg /= num_points;
	}

	template <typename PointInT, typename PointInterestT, typename PointOutT> void ITWFEstimation<PointInT, PointInterestT, PointOutT>::computeFeature (pcl::PointCloud<PointOutT> &output){ }
	//template <typename PointInT> void setInputCloud(const pcl::PointCloud<PointInT> &input);

	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{
		pcl::PointCloud<PointInterestT> interest_points;
		for(int i=0; i<input.points.size(); i++)
		{
			PointInterestT point;
			point.x = input.points[i].x;
			point.y = input.points[i].y;
			point.z = input.points[i].z;
		}
		this->compute(input, interest_points, output);
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																const pcl::PointCloud<PointInterestT> &interest_points,
																pcl::PointCloud<PointOutT> &output)
	{ 
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  
		// Clear Output Cloud
		output.points.clear();
		// Create XYZ-only copies of Input and Interset Point clouds (to search via KDTree)
		pcl::PointCloud<PointXYZ>::Ptr interest_points_ptr(new pcl::PointCloud<PointXYZ>);
		for(int i=0; i<interest_points.points.size(); i++)
		{
			PointXYZ point;
			point.x = interest_points.points[i].x;
			point.y = interest_points.points[i].y;
			point.z = interest_points.points[i].z;
			interest_points_ptr->points.push_back(point);
		}
		pcl::PointCloud<PointXYZ>::Ptr input_ptr(new pcl::PointCloud<PointXYZ>);
		for(int i=0; i<input.points.size(); i++)
		{
			PointXYZ point;
			point.x = input.points[i].x;
			point.y = input.points[i].y;
			point.z = input.points[i].z;
			input_ptr->points.push_back(point);
		}
		ROS_DEBUG_STREAM("[ITWFEstimation] Built interest points cloud, with size " << input_ptr->points.size());
		pcl::KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(input_ptr);
		ROS_DEBUG_STREAM("[ITWFEstimation] Initialized KdTree object.");
		
		for(int i=0; i<interest_points.points.size(); i++)
		{ 
			PointOutT histogram_point;
			initializeOutputPoint(histogram_point, interest_points.points[i]);

			std::vector<int> nearest_indices;
			std::vector<float> nearest_dist_squareds;

			ROS_DEBUG_STREAM_THROTTLE(1, "[ITWFEstimation] Working on a histogram point with index " << i);

			if ( kdtree.nearestKSearch (interest_points_ptr->points[i], k_, nearest_indices, nearest_dist_squareds) > 0 )
			{
				for (size_t j = 0; j < nearest_indices.size (); ++j)
				{
		  			comparePoints(histogram_point, interest_points.points[i], input.points[nearest_indices[j]]);
		  			incrementAverages(histogram_point, input.points[nearest_indices[j]]);
		  		}
		  		// Normalize all values by the number of neighbor points used
		  		normalizeValues(histogram_point, nearest_indices.size());
			}
			else 
				ROS_ERROR_STREAM_THROTTLE(0.1, "[ITWFEstimation] KdTree Nearest Neighbor search failed! Unable to populate histogram for point " << i << "with XYZ values " << interest_points.points[i].x << " " << interest_points.points[i].y << " " << interest_points.points[i].z << ". This message throttled...");
			output.points.push_back(histogram_point); 
		}  
		ROS_DEBUG_STREAM("[ITWFEstimation] Successfully created histogram cloud of size " << output.points.size());
	} 
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setKSearch(const int k_search)
	{
		k_ = k_search;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setBinLimits(float dist_half_lim, float dist_deriv_half_lim, float minimum_radius)
	{
		dist_half_lim_ = dist_half_lim; 
		dist_deriv_half_lim_ = dist_deriv_half_lim; 
		minimum_radius_ = minimum_radius;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setBinLimits(float dist_half_lim, float dist_deriv_half_lim, float minimum_radius, float intensity_lower_value, float intensity_upper_value)
	{
		dist_half_lim_ = dist_half_lim; 
		dist_deriv_half_lim_ = dist_deriv_half_lim; 
		minimum_radius_ = minimum_radius;
		intensity_lower_value_ = intensity_lower_value;
		intensity_upper_value_ = intensity_upper_value;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setPlaneVectors(Eigen::Vector3f in_plane_horz, Eigen::Vector3f in_plane_second)
	{
		in_plane_horz_ = in_plane_horz; 
		in_plane_second_ = in_plane_second;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setPlaneVectors(float horz_x, float horz_y, float horz_z, float second_x, float second_y, float second_z)
	{
		in_plane_horz_ << horz_x, horz_y, horz_z;
		in_plane_second_ << second_x, second_y, second_z;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setWallCoordinateFrame(std::vector<float> wall_coeffs)
	{
		// Wall Z-axis (Plane Normal)
		Eigen::Vector3f normal_vector;
		for(int i=0; i<3; i++)
			normal_vector[i] = wall_coeffs[i];

		// Wall X-axis (Horizontal)
		if(normal_vector[0] == 0 && normal_vector[1] == 0 && normal_vector[2] != 0)
		{
			in_plane_horz_[0] = 1;
			in_plane_horz_[1] = 0;
			in_plane_horz_[2] = 0;
		}
		else
		{
			Eigen::Vector3f global_z;
			global_z[0] = 0;
			global_z[1] = 0;
			global_z[2] = 1;
			in_plane_horz_ = global_z.cross(normal_vector);
		}

		// Wall Y-axis
		in_plane_second_ = normal_vector.cross(in_plane_horz_);

		ROS_DEBUG_STREAM("[ITFWEstimation] Initialized to the following wall coordinate frame: ");
		ROS_DEBUG_STREAM("                               X: " << in_plane_horz_[0] << " " << in_plane_horz_[1] << " " << in_plane_horz_[2]);
		ROS_DEBUG_STREAM("                               Y: " << in_plane_second_[0] << " " << in_plane_second_[1] << " " << in_plane_second_[2]);
		ROS_DEBUG_STREAM("                               Z: " << normal_vector[0] << " " << normal_vector[1] << " " << normal_vector[2]);
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::findColorDerivatives(const PointInT source_point, const PointInterestT target_point,
																			  float horz_distance, 		   float second_distance,
																			  float &intensity_deriv_horz, float &intensity_deriv_second,
																			  float &r_deriv_horz, 		   float &r_deriv_second,
																			  float &g_deriv_horz, 		   float &g_deriv_second,
																			  float &b_deriv_horz, 		   float &b_deriv_second)
	{
		intensity_deriv_horz =   (source_point.intensity - target_point.intensity) / horz_distance;
		intensity_deriv_second = (source_point.intensity - target_point.intensity) / second_distance;
		r_deriv_horz =   		 (source_point.r - target_point.r) / horz_distance;
		r_deriv_second =   		 (source_point.r - target_point.r) / second_distance;
		g_deriv_horz =   		 (source_point.g - target_point.g) / horz_distance;
		g_deriv_second =   		 (source_point.g - target_point.g) / second_distance;
		b_deriv_horz =   		 (source_point.b - target_point.b) / horz_distance;
		b_deriv_second =   		 (source_point.b - target_point.b) / second_distance;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::comparePoints(PointOutT &point, const PointInT source_point, const PointInterestT target_point)
	{
		// First Spatial Partial Derivatives of Depth Deviations
		float depth_offset_horz, depth_offset_second;
		// First Spatial Partial Derivatives of Normal Deviations
		float horz_normal_offset_diff_horz, second_normal_offset_diff_horz, horz_normal_offset_diff_second, second_normal_offset_diff_second;

		// Distance between points in Horizontal and Secondary Dimensions
		float horz_distance =   (source_point.x - target_point.x)*in_plane_horz_[0] +   (source_point.y - target_point.y)*in_plane_horz_[1] +   (source_point.z - target_point.z)*in_plane_horz_[2];
		float second_distance = (source_point.x - target_point.x)*in_plane_second_[0] + (source_point.y - target_point.y)*in_plane_second_[1] + (source_point.z - target_point.z)*in_plane_second_[2];

		if(horz_distance < 1.0e-20)
			horz_distance = 1.0e-20;
		if(second_distance < 1.0e-20)
			second_distance = 1.0e-20;

		// Partial Derivative of Depth Deviation, with respect to Horizontal Dimension
		depth_offset_horz =   (source_point.depth_offset - target_point.depth_offset) / horz_distance;									// d D / d q1
		// Partial Derivative of Depth Deviation, with respect to Secondary Dimension
		depth_offset_second = (source_point.depth_offset - target_point.depth_offset) / second_distance;									// d D / d q2

		// Partial Derivatives of Horizontal and Secondary Normal Deviations, with respect to Horizontal Dimension
		horz_normal_offset_diff_horz =     (source_point.horz_normal_offset - target_point.horz_normal_offset)     / horz_distance; 	// d N_q1 / d q1
		second_normal_offset_diff_horz =   (source_point.second_normal_offset - target_point.second_normal_offset) / horz_distance;		// d N_q2 / d q1
		// Partial Derivatives of Horizontal and Secondary Normal Deviations, with respect to Secondary Dimension
		horz_normal_offset_diff_second =   (source_point.horz_normal_offset - target_point.horz_normal_offset)     / second_distance; 	// d N_q1 / d q2
		second_normal_offset_diff_second = (source_point.second_normal_offset - target_point.second_normal_offset) / second_distance; 	// d N_q2 / d q2

		float intensity_diff_horz, intensity_diff_second, r_diff_horz, r_diff_second, g_diff_horz, g_diff_second, b_diff_horz, b_diff_second;
		findColorDerivatives(source_point,  		 target_point,
							 horz_distance, 		 second_distance,
							 intensity_diff_horz, 	 intensity_diff_second, 
							 r_diff_horz,  		 	 r_diff_second, 
							 g_diff_horz,  		 	 g_diff_second, 
							 b_diff_horz,  		 	 b_diff_second);

		// Set Up Depth Bins
		// This function is templated on different output histogram sizes
		addHistogramValues(point, target_point.depth_offset, target_point.horz_normal_offset, target_point.second_normal_offset,
			 					  depth_offset_horz, 		horz_normal_offset_diff_horz, 	 second_normal_offset_diff_horz,
			 					  depth_offset_second, 		horz_normal_offset_diff_second,  second_normal_offset_diff_second,
			 					  target_point.intensity, 	intensity_diff_horz, 			 intensity_diff_second,
			 					  target_point.r, 			r_diff_horz, 					 r_diff_second,
			 					  target_point.g, 			g_diff_horz, 					 g_diff_second,
			 					  target_point.b, 			b_diff_horz, 					 b_diff_second  );
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature210>::addHistogramValues(pcl::ITWFSignature210 &point, float depth_offset, float normal_offset_horz, float normal_offset_second, 
																							  float depth_deriv_horz, float horz_normal_deriv_horz, float second_normal_deriv_horz,
																							  float depth_deriv_second, float horz_normal_deriv_second, float second_normal_deriv_second,
																							  float intensity, float intensity_deriv_horz, float intensity_deriv_second,
																							  float r, float r_deriv_horz, float r_deriv_second,
																							  float g, float g_deriv_horz, float g_deriv_second,
																							  float b, float b_deriv_horz, float b_deriv_second  )
	{
		// Increment Depth Bins
		point.histogram[ getBinIndex(depth_offset, -dist_half_lim_, dist_half_lim_, 10) ] ++;
		point.histogram[ getBinIndex(depth_deriv_horz, -dist_deriv_half_lim_, dist_deriv_half_lim_, 10)+10 ] ++;
		point.histogram[ getBinIndex(depth_deriv_second, -dist_deriv_half_lim_, dist_deriv_half_lim_, 10)+20 ] ++;
		// Increment Horizontal Normal Bins
		point.histogram[ getBinIndex(normal_offset_horz, -1, 1, 10)+30 ] ++;
		point.histogram[ getBinIndex(horz_normal_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+40 ] ++;
		point.histogram[ getBinIndex(horz_normal_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+50 ] ++;
		// Increment Secondary Normal Bins
		point.histogram[ getBinIndex(normal_offset_second, -1, 1, 10)+60 ] ++;
		point.histogram[ getBinIndex(second_normal_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+70 ] ++;
		point.histogram[ getBinIndex(second_normal_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+80 ] ++;
		// Increment Intensity Bins
		point.histogram[ getBinIndex(intensity, intensity_lower_value_, intensity_upper_value_, 10)+90 ] ++;
		point.histogram[ getBinIndex(intensity_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+100 ] ++;
		point.histogram[ getBinIndex(intensity_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+110 ] ++;
		// Increment Red Bins
		point.histogram[ getBinIndex(r, 0, 255, 10)+120 ] ++;
		point.histogram[ getBinIndex(r_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+130 ] ++;
		point.histogram[ getBinIndex(r_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+140 ] ++;
		// Increment Green Bins
		point.histogram[ getBinIndex(g, 0, 255, 10)+150 ] ++;
		point.histogram[ getBinIndex(g_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+160 ] ++;
		point.histogram[ getBinIndex(g_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+170 ] ++;
		// Increment Blue Bins
		point.histogram[ getBinIndex(b, 0, 255, 10)+180 ] ++;
		point.histogram[ getBinIndex(b_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+190 ] ++;
		point.histogram[ getBinIndex(b_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+200 ] ++;

		// Increment Average Values
		point.depth_offset_deriv_horz_avg += depth_deriv_horz;
		point.depth_offset_deriv_second_avg += depth_deriv_second;
		point.horz_angle_offset_deriv_horz_avg += horz_normal_deriv_horz;
		point.horz_angle_offset_deriv_second_avg += horz_normal_deriv_second;
		point.second_angle_offset_deriv_horz_avg += second_normal_deriv_horz;
		point.second_angle_offset_deriv_second_avg += second_normal_deriv_second;
		point.intensity_deriv_horz_avg += intensity_deriv_horz;
		point.intensity_deriv_second_avg += intensity_deriv_second;
		point.r_deriv_horz_avg += r_deriv_horz;
		point.r_deriv_second_avg += r_deriv_second;
		point.g_deriv_horz_avg += g_deriv_horz;
		point.g_deriv_second_avg += g_deriv_second;
		point.b_deriv_horz_avg += b_deriv_horz;
		point.b_deriv_second_avg += b_deriv_second;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature90>::addHistogramValues(pcl::ITWFSignature90 &point, float depth_offset, float normal_offset_horz, float normal_offset_second, 
																							  float depth_deriv_horz, float horz_normal_deriv_horz, float second_normal_deriv_horz,
																							  float depth_deriv_second, float horz_normal_deriv_second, float second_normal_deriv_second,
																							  float intensity, float intensity_deriv_horz, float intensity_deriv_second,
																							  float r, float r_deriv_horz, float r_deriv_second,
																							  float g, float g_deriv_horz, float g_deriv_second,
																							  float b, float b_deriv_horz, float b_deriv_second  )
	{
		// Increment Depth Bins
		point.histogram[ getBinIndex(depth_offset, -dist_half_lim_, dist_half_lim_, 10) ] ++;
		point.histogram[ getBinIndex(depth_deriv_horz, -dist_deriv_half_lim_, dist_deriv_half_lim_, 10)+10 ] ++;
		point.histogram[ getBinIndex(depth_deriv_second, -dist_deriv_half_lim_, dist_deriv_half_lim_, 10)+20 ] ++;
		// Increment Horizontal Normal Bins
		point.histogram[ getBinIndex(normal_offset_horz, -1, 1, 10)+30 ] ++;
		point.histogram[ getBinIndex(horz_normal_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+40 ] ++;
		point.histogram[ getBinIndex(horz_normal_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+50 ] ++;
		// Increment Secondary Normal Bins
		point.histogram[ getBinIndex(normal_offset_second, -1, 1, 10)+60 ] ++;
		point.histogram[ getBinIndex(second_normal_deriv_horz, -1/minimum_radius_, 1/minimum_radius_, 10)+70 ] ++;
		point.histogram[ getBinIndex(second_normal_deriv_second, -1/minimum_radius_, 1/minimum_radius_, 10)+80 ] ++;

		// Increment Average Values
		point.depth_offset_deriv_horz_avg += depth_deriv_horz;
		point.depth_offset_deriv_second_avg += depth_deriv_second;
		point.horz_angle_offset_deriv_horz_avg += horz_normal_deriv_horz;
		point.horz_angle_offset_deriv_second_avg += horz_normal_deriv_second;
		point.second_angle_offset_deriv_horz_avg += second_normal_deriv_horz;
		point.second_angle_offset_deriv_second_avg += second_normal_deriv_second;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> int
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::getBinIndex(float target_value, float bin_min, float bin_max, int bin_num)
	{
		float bin_percentile = (target_value - bin_min) / (bin_max - bin_min);
		if(bin_percentile >= 1) 
			return bin_num-1;
		if(bin_percentile < 0)
			return 0;
		return int(floor(bin_percentile*bin_num));
	}

}

#endif // IMPL_ITWF_ESTIMATION_