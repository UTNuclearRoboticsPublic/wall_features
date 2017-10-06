

#ifndef IMPL_WALL_DAMAGE_ESTIMATION_
#define IMPL_WALL_DAMAGE_ESTIMATION_

#include "wall_damage_estimation.h"

/* temp - list of parameters to be determined:
  -- PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  -- PCL_ADD_NORMAL4D; 				// This adds the member normal[3] which can also be accessed using the point (which is float[4])
  float angle_offset_avg; 			// Average offset in angle between local normal vectors of each neighbor point and the expected normal vector of the containing plane primitive definition
  float dist_offset_avg; 			// Average offset in position between each neighbor point and the containing plane primitive definition
  float	histogram[80];
*/
namespace pcl
{ 
// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Pointwise Damage Estimation ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
	template <typename PointInT, typename PointOutT> void WallDamagePointwiseEstimation<PointInT, PointOutT>::computeFeature (pcl::PointCloud<PointOutT> &output) { }
	//template <typename PointInT> void setInputCloud(const pcl::PointCloud<PointInT> &input);

	template <typename PointInT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{ 
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  

		// TEMPORARY - this an external input later on...
		pcl::Normal wall_normal;
		wall_normal.normal_x = 0;
		wall_normal.normal_y = 1;
		wall_normal.normal_z = 0;
		float wall_distance = 0.953;

		// ------------------------ Generate Normals from Input ------------------------
		// Note that this obviously only has to be done once per input cloud 
		//   Remove NaNs from input (for safety - had crashes before due to NaN in input)
		std::vector<int> nan_indices;
		pcl::PointCloud<PCLPoint>::Ptr nanless_input (new pcl::PointCloud<PCLPoint>());
		pcl::removeNaNFromPointCloud(input, *nanless_input, nan_indices); 
		ROS_DEBUG_STREAM("[WallDamageEstimation] Removed NaNs from input - had " << input.points.size()-nanless_input->points.size() << " NaN entries.");
		//   Generate cloud, initialize KdTree and NormEst objects
		PCNP points_with_normals (new PCN);
		pcl::NormalEstimation<PCLPoint, pcl::PointNormal> norm_est;
		pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint> ());
		norm_est.setSearchMethod (tree);
		norm_est.setKSearch (30); 		// Do this more smartly later, maybe? Multiple kinds of wall_damage_estimation point types depending on K used, or set this as an input?
		ROS_DEBUG_STREAM("[WallDamageEstimation] Initialized norm_est object.");
		//   Run Normal Estimation --> generate Normals
		norm_est.setInputCloud (nanless_input);
		norm_est.compute (*points_with_normals);
		for(int i=0; i<nanless_input->points.size(); i++)
		{
			points_with_normals->points[i].x = nanless_input->points[i].x;
			points_with_normals->points[i].y = nanless_input->points[i].y;
			points_with_normals->points[i].z = nanless_input->points[i].z;
		}
		ROS_DEBUG_STREAM("[WallDamageEstimation] Created XYZNormal cloud from input, with size " << points_with_normals->points.size());
		//   Remove NaNs from normal cloud (for safety - had crashes before due to NaN in input)
		//pcl::removeNaNFromPointCloud(*points_with_normals, *points_with_normals, nan_indices); 
		//ROS_DEBUG_STREAM("[WallDamageEstimation] Removed NaNs from normal cloud - had " << nan_indices.size() << "NaN entries.");

		output.points.clear();
		for(int i=0; i<points_with_normals->points.size(); i++)
		{
			PointOutT point;
			point.x = points_with_normals->points[i].x;
			point.y = points_with_normals->points[i].y;
			point.z = points_with_normals->points[i].z;
			ROS_DEBUG_STREAM_THROTTLE(0.1, "first point data: " << points_with_normals->points[i].x << " " << points_with_normals->points[i].y << " " << points_with_normals->points[i].x << " " << input.points[i].x << " " << input.points[i].y << " " << input.points[i].z);
			point.normal_x = points_with_normals->points[i].normal_x;
			point.normal_y = points_with_normals->points[i].normal_y;
			point.normal_z = points_with_normals->points[i].normal_z;

			// Angular offset between point normal and wall normal
			float dot_product = point.normal_x*wall_normal.normal_x + point.normal_y*wall_normal.normal_y + point.normal_z*wall_normal.normal_z;
			float wall_normal_mag = sqrt(pow(wall_normal.normal_x,2) + pow(wall_normal.normal_y,2) + pow(wall_normal.normal_z,2));
			float point_normal_mag = sqrt(pow(point.normal_x,2) + pow(point.normal_y,2) + pow(point.normal_z,2));
			point.angle_offset = acos(dot_product/wall_normal_mag/point_normal_mag);	// Absolute value of the angle difference between plane and point normal (assuming acos -> 0 to pi)

			// Distance between point and plane
			pcl::PointXYZ point_to_wall;			// Vector from the current point to the wall normal-from-origin point
			point_to_wall.x = point.x - wall_normal.normal_x*wall_distance;
			point_to_wall.y = point.y - wall_normal.normal_y*wall_distance;
			point_to_wall.z = point.z - wall_normal.normal_z*wall_distance;
			point.dist_offset = (point_to_wall.x*wall_normal.normal_x + point_to_wall.y*wall_normal.normal_y + point_to_wall.z*wall_normal.normal_z)/wall_normal_mag;

			output.points.push_back(point);
		}
		ROS_ERROR_STREAM("[WallDamageEstimation] Found normal and angular offsets for all points in input cloud.");
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
		pcl::PointCloud<pcl::PointXYZRGB> interest_points;
		for(int i=0; i<input.points.size(); i++)
		{
			pcl::PointXYZRGB point;
			point.x = input.points[i].x;
			point.x = input.points[i].y;
			point.x = input.points[i].z;
		}
		this->compute(input, interest_points, output);
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	WallDamageHistogramEstimation<PointInT, PointInterestT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																const pcl::PointCloud<PointInterestT> &interest_points,
																pcl::PointCloud<PointOutT> &output)
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  
		ROS_ERROR_STREAM("entered second features processor");

		output.points.clear();
		pcl::PointCloud<PointXYZ>::Ptr interest_points_ptr(new pcl::PointCloud<PointXYZ>);
		for(int i=0; i<interest_points.points.size(); i++)
		{
			PointXYZ point;
			point.x = interest_points.points[i].x;
			point.y = interest_points.points[i].y;
			point.y = interest_points.points[i].z;
			interest_points_ptr->points.push_back(point);
		}
		ROS_DEBUG_STREAM("[WallDamageEstimation] Built interest points cloud, with size " << interest_points_ptr->points.size());
		pcl::KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(interest_points_ptr);
		ROS_DEBUG_STREAM("[WallDamageEstimation] Initialized KdTree object.");

		//float max_dist_recorded; 						// Normalize distances by maximum? To allow simple binning... but destroys absolute deviation information 
		float min_dist_bin = -0.007; 	// 15cm
		float max_dist_bin = 0.007; 		// 15cm
		float min_angle_bin = 0;
		float max_angle_bin = 3.14159;
		int bins = 80;
		
		float min_angle = 0;
		float max_angle = 0;
		float min_deviation = 0;
		float max_deviation = 0;
		for(int i=0; i<input.points.size(); i++)
		{
			if(input.points[i].angle_offset < min_angle)
				min_angle = input.points[i].angle_offset;
			if(input.points[i].angle_offset > max_angle)
				max_angle = input.points[i].angle_offset;
			if(input.points[i].dist_offset < min_deviation)
				min_deviation = input.points[i].dist_offset;
			if(input.points[i].dist_offset < max_deviation)
				max_deviation = input.points[i].dist_offset;
		}
		ROS_ERROR_STREAM("min max... " << min_angle << " " << max_angle << " " << min_deviation << " " << max_deviation);

		float avg_x = 0;
		float min_x = 0; 
		float max_x = 0;
		for(int i=0; i<input.points.size(); i++)
		{
			avg_x += input.points[i].x;
			if(min_x > input.points[i].x)
				min_x = input.points[i].x;
			if(max_x < input.points[i].x)
				max_x = input.points[i].x;
		}
		ROS_ERROR_STREAM("input variation: " << avg_x << " " << min_x << " " << max_x);

		for(int i=0; i<interest_points.points.size(); i++)
		{
			PointOutT histogram_point;
			for(int j=0; j<bins; j++)
				histogram_point.histogram[j] = 0;
			histogram_point.angle_offset_avg = 0;
			histogram_point.dist_offset_avg = 0;
			histogram_point.x = interest_points.points[i].x;
			histogram_point.y = interest_points.points[i].y;
			histogram_point.z = interest_points.points[i].z;

			int k = 300; 									// Consider switching to R, and also exposing choices for these to external manipulation
			std::vector<int> nearest_indices(k);
			std::vector<float> nearest_dist_squareds(k);

			ROS_DEBUG_STREAM_THROTTLE(1, "[WallDamageEstimation] Working on a histogram point with index " << i);

			if ( kdtree.nearestKSearch (interest_points_ptr->points[i], k, nearest_indices, nearest_dist_squareds) > 0 )
			{
				for (size_t j = 0; j < nearest_indices.size (); ++j)
				{
		  			int hist_ind_angle = floor( bins/2 * (input.points[nearest_indices[j]].angle_offset - min_angle_bin) / (max_angle_bin - min_angle_bin) );
		  			if(hist_ind_angle <= 0)
		  				hist_ind_angle = 0;
		  			else if(hist_ind_angle > bins)
		  				hist_ind_angle = bins;
		  			histogram_point.histogram[hist_ind_angle]++;
		  			int hist_ind_dist = floor( bins/2 * (input.points[nearest_indices[j]].dist_offset - min_dist_bin) / (max_dist_bin - min_dist_bin) );
		  			if(hist_ind_dist <= 0)
		  				hist_ind_dist = 0;
		  			else if(hist_ind_dist > bins)
		  				hist_ind_dist = bins;
		  			histogram_point.histogram[hist_ind_dist+bins/2]++;
					ROS_ERROR_STREAM(input.points[nearest_indices[j]].angle_offset << " " << input.points[nearest_indices[j]].dist_offset << " " << hist_ind_angle << " " << hist_ind_dist << " Current point: " << input.points[nearest_indices[j]].x << " " << input.points[nearest_indices[j]].y << " " << input.points[nearest_indices[j]].z << " Interest Point: " << interest_points_ptr->points[i].x <<  " " << interest_points_ptr->points[i].y <<  " " << interest_points_ptr->points[i].z);
		  			histogram_point.angle_offset_avg += input.points[nearest_indices[j]].angle_offset;
		  			histogram_point.dist_offset_avg += input.points[nearest_indices[j]].dist_offset;
		  		}
			}
			else 
				ROS_ERROR_STREAM("[WallDamageEstimation] KdTree Nearest Neighbor search failed! Unable to populate histogram for point " << i << "with XYZ values " << interest_points.points[i].x << " " << interest_points.points[i].y << " " << interest_points.points[i].z);
			output.points.push_back(histogram_point);
		}

		ROS_DEBUG_STREAM("[WallDamageEstimation] Successfully created histogram cloud of size " << output.points.size());
	} 
}

#endif // IMPL_WALL_DAMAGE_ESTIMATION_