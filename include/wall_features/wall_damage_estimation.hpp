

#ifndef IMPL_WALL_DAMAGE_ESTIMATION_
#define IMPL_WALL_DAMAGE_ESTIMATION_

#include "wall_damage_estimation.h"

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
// ------------------------------------------------ Pointwise Damage Estimation ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
	// Handle color and intensity for various input types
	// RGB input, 
	template <> void 
	WallDamagePointwiseEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::PointWallDamage>::setColorValues(pcl::PointXYZRGB input_point, pcl::PointWallDamage &damage_point)
	{
		damage_point.r = input_point.r;
		damage_point.g = input_point.g;
		damage_point.b = input_point.b;
		damage_point.intensity = 0;
	}
	template <> void 
	WallDamagePointwiseEstimation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::PointWallDamage>::setColorValues(pcl::PointXYZI input_point, pcl::PointWallDamage &damage_point)
	{
		damage_point.intensity = input_point.intensity;
		damage_point.r = 0;
		damage_point.g = 0;
		damage_point.b = 0;
	}
	template <> void 
	WallDamagePointwiseEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointWallDamage>::setColorValues(pcl::PointXYZ input_point, pcl::PointWallDamage &damage_point)
	{
		damage_point.r = 0;
		damage_point.g = 0;
		damage_point.b = 0;
		damage_point.intensity = 0;
	}

	template <typename PointInT, typename PointNormalT, typename PointOutT> void WallDamagePointwiseEstimation<PointInT, PointNormalT, PointOutT>::computeFeature (pcl::PointCloud<PointOutT> &output) { }
	//template <typename PointInT> void setInputCloud(const pcl::PointCloud<PointInT> &input);

	template <typename PointInT, typename PointNormalT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointNormalT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{ 
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  

		findWallCoordinateFrame();

		// TEMPORARY - this an external input later on...
		pcl::Normal wall_normal;
		wall_normal.normal_x = wall_coeffs_[0];
		wall_normal.normal_y = wall_coeffs_[1];
		wall_normal.normal_z = wall_coeffs_[2];
		float wall_distance = wall_coeffs_[3];

		// ------------------------ Generate Normals from Input ------------------------
		// Note that this obviously only has to be done once per input cloud 
		//   Remove NaNs from input (for safety - had crashes before due to NaN in input)
		std::vector<int> nan_indices;
		PCP nanless_input (new PC());
		pcl::removeNaNFromPointCloud(input, *nanless_input, nan_indices); 
		ROS_DEBUG_STREAM("[WallDamageEstimation] Removed NaNs from input - had " << input.points.size()-nanless_input->points.size() << " NaN entries.");
		//   Generate cloud, initialize KdTree and NormEst objects
		PCNP points_with_normals (new PCN);
		pcl::NormalEstimation<PointInT, PointNormalT> norm_est;
		KDTreePtr tree (new KDTree ());
		norm_est.setSearchMethod (tree);
		norm_est.setKSearch (30); 		// Do this more smartly later, maybe? Multiple kinds of wall_damage_estimation point types depending on K used, or set this as an input?
		ROS_DEBUG_STREAM("[WallDamageEstimation] Initialized norm_est object.");
		//   Run Normal Estimation --> generate Normals
		norm_est.setInputCloud (nanless_input);
		norm_est.compute (*points_with_normals);
		pcl::copyPointCloud(*nanless_input, *points_with_normals);
		for(int i=0; i<nanless_input->points.size(); i++)
		{
			// Update Position
			points_with_normals->points[i].x = nanless_input->points[i].x;
			points_with_normals->points[i].y = nanless_input->points[i].y;
			points_with_normals->points[i].z = nanless_input->points[i].z;
			// Flip Normals to Align with Plane Normal
			float normal_dot_product = points_with_normals->points[i].normal_x * wall_normal.normal_x
									 + points_with_normals->points[i].normal_y * wall_normal.normal_y
									 + points_with_normals->points[i].normal_z * wall_normal.normal_z;
			if(normal_dot_product < 0)
			{
				points_with_normals->points[i].normal_x *= -1;
				points_with_normals->points[i].normal_y *= -1;
				points_with_normals->points[i].normal_z *= -1;
			}
		}
		ROS_DEBUG_STREAM("[WallDamageEstimation] Created XYZNormal cloud from input, with size " << points_with_normals->points.size());

		output.points.clear();
		for(int i=0; i<points_with_normals->points.size(); i++)
		{
			PointOutT point;
			point.x = points_with_normals->points[i].x;
			point.y = points_with_normals->points[i].y;
			point.z = points_with_normals->points[i].z;
			point.normal_x = points_with_normals->points[i].normal_x;
			point.normal_y = points_with_normals->points[i].normal_y;
			point.normal_z = points_with_normals->points[i].normal_z;
			
			setColorValues(nanless_input->points[i], point);

			// Angular offset between point normal and wall normal
			float dot_product = point.normal_x*wall_normal.normal_x + point.normal_y*wall_normal.normal_y + point.normal_z*wall_normal.normal_z;
			float wall_normal_mag = sqrt(pow(wall_normal.normal_x,2) + pow(wall_normal.normal_y,2) + pow(wall_normal.normal_z,2));
			float point_normal_mag = sqrt(pow(point.normal_x,2) + pow(point.normal_y,2) + pow(point.normal_z,2));
			point.angle_offset = acos(dot_product/wall_normal_mag/point_normal_mag);	// Absolute value of the angle difference between plane and point normal (assuming acos -> 0 to pi)
			if(point.angle_offset < 0)
				ROS_WARN_STREAM("oops " << point.angle_offset << " " << dot_product << " " << point.normal_x << " " << point.normal_y << " " << point.normal_z);
			// Move angle range from (0 to pi) to (-pi/2 to pi/2) --> puts 'parallel to wall' (ie angle=0) at middle, not edges  
			if(point.angle_offset > 1.570796)
				point.angle_offset = 3.141593-point.angle_offset;

			// Express the difference of the POINT NORMAL and the WALL NORMAL in the WALL COORDINATE FRAME 
			Eigen::Vector3f point_normal_eigen;
			point_normal_eigen << point.normal_x, point.normal_y, point.normal_z;
			point.horz_normal_offset = wall_x_axis_.dot(point_normal_eigen);
			point.second_normal_offset = wall_y_axis_.dot(point_normal_eigen);

			// Distance between point and plane
			pcl::PointXYZ point_to_wall;			// Vector from the current point to the wall normal-from-origin point
			point_to_wall.x = point.x + wall_normal.normal_x*wall_distance;
			point_to_wall.y = point.y + wall_normal.normal_y*wall_distance;
			point_to_wall.z = point.z + wall_normal.normal_z*wall_distance;
			point.depth_offset = (point_to_wall.x*wall_normal.normal_x + point_to_wall.y*wall_normal.normal_y + point_to_wall.z*wall_normal.normal_z)/wall_normal_mag;

			output.points.push_back(point);
		}
		ROS_DEBUG_STREAM("[WallDamageEstimation] Found normal and angular offsets for all points in input cloud.");
	} 
	template <typename PointInT, typename PointNormalT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointNormalT, PointOutT>::setKSearch(const int k_search)
	{
		k_ = k_search;
	}
	template <typename PointInT, typename PointNormalT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointNormalT, PointOutT>::setWallCoefficients(const float wall_coeffs[4])
	{
		wall_coeffs_[0] = wall_coeffs[0];
		wall_coeffs_[1] = wall_coeffs[1];
		wall_coeffs_[2] = wall_coeffs[2];
		wall_coeffs_[3] = wall_coeffs[3];
	}
	template <typename PointInT, typename PointNormalT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointNormalT, PointOutT>::setViewpoint(const float viewpoint[3])
	{
		viewpoint_[0] = viewpoint[0];
		viewpoint_[1] = viewpoint[1];
		viewpoint_[2] = viewpoint[2];
	}
	template <typename PointInT, typename PointNormalT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointNormalT, PointOutT>::findWallCoordinateFrame()
	{
		// Wall Z-axis (Plane Normal)
		for(int i=0; i<3; i++)
			wall_z_axis_[i] = wall_coeffs_[i];

		// Wall X-axis (Horizontal)
		if(wall_z_axis_[0] == 0 && wall_z_axis_[1] == 0 && wall_z_axis_[2] != 0)
		{
			wall_x_axis_[0] = 1;
			wall_x_axis_[1] = 0;
			wall_x_axis_[2] = 0;
		}
		else
		{
			Eigen::Vector3f global_z;
			global_z[0] = 0;
			global_z[1] = 0;
			global_z[2] = 1;
			wall_x_axis_ = global_z.cross(wall_z_axis_);
		}

		// Wall Y-axis
		wall_y_axis_ = wall_z_axis_.cross(wall_x_axis_);

		ROS_DEBUG_STREAM("[WallDamageEstimation] Wall Coordinate Frame: ");
		ROS_DEBUG_STREAM("   X: " << wall_x_axis_[0] << " " << wall_x_axis_[1] << " " << wall_x_axis_[2]);
		ROS_DEBUG_STREAM("   Y: " << wall_y_axis_[0] << " " << wall_y_axis_[1] << " " << wall_y_axis_[2]);
		ROS_DEBUG_STREAM("   Z: " << wall_z_axis_[0] << " " << wall_z_axis_[1] << " " << wall_z_axis_[2]);
	}













	
// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Histogram Damage Estimation ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
	template <typename PointInT, typename PointInterestT, typename PointOutT> void WallDamageHistogramEstimation<PointInT, PointInterestT, PointOutT>::computeFeature (pcl::PointCloud<PointOutT> &output){ }
	//template <typename PointInT> void setInputCloud(const pcl::PointCloud<PointInT> &input);

	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	WallDamageHistogramEstimation<PointInT, PointInterestT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
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
	WallDamageHistogramEstimation<PointInT, PointInterestT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																const pcl::PointCloud<PointInterestT> &interest_points,
																pcl::PointCloud<PointOutT> &output)
	{
		//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    //ros::console::notifyLoggerLevelsChanged();  

		output.points.clear();
		pcl::PointCloud<PointXYZ>::Ptr interest_points_ptr(new pcl::PointCloud<PointXYZ>);
		interest_points_ptr->points.clear();
		for(int i=0; i<interest_points.points.size(); i++)
		{
			PointXYZ point;
			point.x = interest_points.points[i].x;
			point.y = interest_points.points[i].y;
			point.z = interest_points.points[i].z;
			interest_points_ptr->points.push_back(point);
		}
		pcl::PointCloud<PointXYZ>::Ptr input_ptr(new pcl::PointCloud<PointXYZ>);
		input_ptr->points.clear();
		for(int i=0; i<input.points.size(); i++)
		{
			PointXYZ point;
			point.x = input.points[i].x;
			point.y = input.points[i].y;
			point.z = input.points[i].z;
			input_ptr->points.push_back(point);
		}
		ROS_DEBUG_STREAM("[WallDamageEstimation] Built interest points cloud, with size " << input_ptr->points.size());
		pcl::KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(input_ptr);
		ROS_DEBUG_STREAM("[WallDamageEstimation] Initialized KdTree object.");

		int bins = 80;
		
		for(int i=0; i<interest_points.points.size(); i++)
		{
			PointOutT histogram_point;
			for(int j=0; j<bins; j++)
				histogram_point.histogram[j] = 0;
			histogram_point.angle_offset_avg = 0;
			histogram_point.depth_offset_avg = 0;
			histogram_point.x = interest_points.points[i].x;
			histogram_point.y = interest_points.points[i].y;
			histogram_point.z = interest_points.points[i].z;

			std::vector<int> nearest_indices(k_);
			std::vector<float> nearest_dist_squareds(k_);

			ROS_DEBUG_STREAM_THROTTLE(1, "[WallDamageEstimation] Working on a histogram point with index " << i);

			if ( kdtree.nearestKSearch (interest_points_ptr->points[i], k_, nearest_indices, nearest_dist_squareds) > 0 )
			{
				float avg_x = 0;
				float avg_y = 0;
				float avg_z = 0;
				for (size_t j = 0; j < nearest_indices.size (); ++j)
				{
		  			int hist_ind_angle = floor( bins/2 * (input.points[nearest_indices[j]].angle_offset - angle_min_) / (angle_max_ - angle_min_) );
		  			if(hist_ind_angle <= 0)
		  				hist_ind_angle = 0;
		  			else if(hist_ind_angle > bins)
		  				hist_ind_angle = bins;
		  			histogram_point.histogram[hist_ind_angle]++;
		  			int hist_ind_dist = floor( bins/2 * (input.points[nearest_indices[j]].depth_offset - dist_min_) / (dist_max_ - dist_min_) );
		  			if(hist_ind_dist <= 0)
		  				hist_ind_dist = 0;
		  			else if(hist_ind_dist > bins)
		  				hist_ind_dist = bins;
		  			histogram_point.histogram[hist_ind_dist+bins/2]++;
		  			histogram_point.angle_offset_avg += input.points[nearest_indices[j]].angle_offset;
		  			histogram_point.depth_offset_avg += input.points[nearest_indices[j]].depth_offset;
		  			avg_x += input.points[nearest_indices[j]].x;
		  			avg_y += input.points[nearest_indices[j]].y;
		  			avg_z += input.points[nearest_indices[j]].z;
		  		}
		  		histogram_point.angle_offset_avg /= k_;
		  		histogram_point.depth_offset_avg /= k_;
			}
			else 
				ROS_ERROR_STREAM_THROTTLE(0.1, "[WallDamageEstimation] KdTree Nearest Neighbor search failed! Unable to populate histogram for point " << i << "with XYZ values " << interest_points.points[i].x << " " << interest_points.points[i].y << " " << interest_points.points[i].z << ". This message throttled...");
			output.points.push_back(histogram_point);
		}

		ROS_DEBUG_STREAM("[WallDamageEstimation] Successfully created histogram cloud of size " << output.points.size());
	} 
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	WallDamageHistogramEstimation<PointInT, PointInterestT, PointOutT>::setKSearch(const int k_search)
	{
		k_ = k_search;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	WallDamageHistogramEstimation<PointInT, PointInterestT, PointOutT>::setBinLimits(float angle_min, float angle_max, float dist_min, float dist_max)
	{
		angle_min_ = angle_min; 
		angle_max_ = angle_max; 
		dist_min_ = dist_min; 
		dist_max_ = dist_max; 
	}
}

#endif // IMPL_WALL_DAMAGE_ESTIMATION_