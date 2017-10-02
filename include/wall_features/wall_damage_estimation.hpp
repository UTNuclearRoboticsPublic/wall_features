

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
	template <typename PointInT, typename PointOutT> void 
	WallDamagePointwiseEstimation<PointInT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{
		// TEMPORARY - this an external input later on...
		pcl::Normal wall_normal;
		wall_normal.normal_x = 0;
		wall_normal.normal_y = 1;
		wall_normal.normal_z = 0;
		float wall_distance = 1.5;

		// ------------------------ Generate Normals from Input ------------------------
		// Note that this obviously only has to be done once per input cloud 
		//   Remove NaNs from input (for safety - had crashes before due to NaN in input)
		std::vector<int> nan_indices;
		pcl::PointCloud<PCLPoint>::Ptr nanless_input;// (new pcl::PointCloud<PointInT>());
		pcl::removeNaNFromPointCloud(input, *nanless_input, nan_indices); 
		ROS_DEBUG_STREAM("[WallDamageEstimation] Removed NaNs from input - had " << nan_indices.size() << "NaN entries.");
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
		ROS_DEBUG_STREAM("[WallDamageEstimation] Created XYZNormal cloud from input.");
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
		ROS_DEBUG_STREAM("[WallDamageEstimation] Found normal and angular offsets for all points in input cloud.");
	} 

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Histogram Damage Estimation ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
	template <typename PointInT, typename PointOutT> void 
	WallDamageHistogramEstimation<PointInT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{
		this->compute(input, input, output);
	}
	template <typename PointInT, typename PointOutT> void 
	WallDamageHistogramEstimation<PointInT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																const pcl::PointCloud<PointInT> &interest_points,
																pcl::PointCloud<PointOutT> &output)
	{
		output.points.clear();
		pcl::PointCloud<PointXYZ>::Ptr interest_points_ptr(new pcl::PointCloud<PointXYZ>);
		for(int i=0; i<interest_points.points.size(); i++)
		{
			interest_points_ptr->points[i].x = interest_points.points[i].x;
			interest_points_ptr->points[i].y = interest_points.points[i].y;
			interest_points_ptr->points[i].y = interest_points.points[i].z;
		}
		pcl::KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(interest_points_ptr);
		ROS_DEBUG_STREAM("[WallDamageEstimation] Initialized KdTree object.");

		//float max_dist_recorded; 						// Normalize distances by maximum? To allow simple binning... but destroys absolute deviation information 
		float min_dist_bin = -0.15; 	// 15cm
		float max_dist_bin = 0.15; 		// 15cm
		float min_angle_bin = -3.14159*2;
		float max_angle_bin = 3.14159*2;
		int bins = 80;
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

			int k = 30; 									// Consider switching to R, and also exposing choices for these to external manipulation
			std::vector<int> nearest_indices(k);
			std::vector<float> nearest_dist_squareds(k);

			output.points.push_back(histogram_point);

			if ( kdtree.nearestKSearch (interest_points_ptr->points[i], k, nearest_indices, nearest_dist_squareds) > 0 )
			{
				for (size_t j = 0; j < nearest_indices.size (); ++j)
				{
		  			int hist_ind_angle = floor( bins/2 * (max_angle_bin - min_angle_bin) / input.points[nearest_indices[j]].angle_offset );
		  			histogram_point.histogram[hist_ind_angle]++;
		  			int hist_ind_dist = floor( bins/2 * (max_angle_bin - min_angle_bin) / input.points[nearest_indices[j]].angle_offset );
		  			histogram_point.histogram[hist_ind_dist+bins/2];
		  			histogram_point.angle_offset_avg += input.points[nearest_indices[j]].angle_offset;
		  			histogram_point.dist_offset_avg += input.points[nearest_indices[j]].dist_offset;
		  		}
			}
			else 
				ROS_ERROR_STREAM("[WallDamageEstimation] KdTree Nearest Neighbor search failed! Unable to populate histogram for point " << i << "with XYZ values " << interest_points.points[i].x << " " << interest_points.points[i].y << " " << interest_points.points[i].z);
		}
	} 
}

#endif // IMPL_WALL_DAMAGE_ESTIMATION_