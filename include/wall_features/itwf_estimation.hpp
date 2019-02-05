

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
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature84>::initializeOutputPoint(pcl::ITWFSignature84 &histogram_point, pcl::PointWallDamage input_point)
	{
		// Initialize Euclidian and Histogram
		histogram_point.x = input_point.x;
		histogram_point.y = input_point.y;
		histogram_point.z = input_point.z;
		for(int i=0; i<histogram_point.descriptorSize(); i++)
			histogram_point.histogram[i] = 0;
		for(int i=0; i<histogram_point.descriptorSize()/7; i++)
			histogram_point.bin_count[i] = 0;
		// Initialize Basal Averages
		histogram_point.depth_offset_avg = 0;
		histogram_point.horz_angle_offset_avg = 0;
		histogram_point.second_angle_offset_avg = 0;
		histogram_point.r_avg = 0;
		histogram_point.g_avg = 0;
		histogram_point.b_avg = 0;
		histogram_point.intensity_avg = 0;
		histogram_point.intensity_horz_change_avg = 0;
		histogram_point.intensity_second_change_avg = 0;
	}
	template <> void
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature84>::incrementAverages(pcl::ITWFSignature84 &histogram_point, pcl::PointWallDamage neighbor_point)
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
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature84>::normalizeAverages(pcl::ITWFSignature84 &histogram_point, int num_points)
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
		histogram_point.intensity_horz_change_avg /= num_points;
		histogram_point.intensity_second_change_avg /= num_points;
	}

	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setHistogramScales(float depth_half_lim, float intensity_minimum, float intensity_maximum)
	{
		depth_half_lim_ = depth_half_lim; 
		intensity_minimum_ = intensity_minimum;
		intensity_maximum_ = intensity_maximum;
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
		// Create XYZ-only copies of Input and Interest Point clouds (to search via KDTree)
		pcl::PointCloud<PointXYZ>::Ptr interest_points_xyz_ptr(new pcl::PointCloud<PointXYZ>);
		for(int i=0; i<interest_points.points.size(); i++)
		{
			PointXYZ point;
			point.x = interest_points.points[i].x;
			point.y = interest_points.points[i].y;
			point.z = interest_points.points[i].z;
			interest_points_xyz_ptr->points.push_back(point);
		}
		pcl::PointCloud<PointXYZ>::Ptr input_xyz_ptr(new pcl::PointCloud<PointXYZ>);
		for(int i=0; i<input.points.size(); i++)
		{
			PointXYZ point;
			point.x = input.points[i].x;
			point.y = input.points[i].y;
			point.z = input.points[i].z;
			input_xyz_ptr->points.push_back(point);
		}
		ROS_DEBUG_STREAM("[ITWFEstimation] Built XYZ-only interest point and input clouds, with size " << input_xyz_ptr->points.size() << " and " << interest_points_xyz_ptr->points.size());
		pcl::KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(input_xyz_ptr);
		ROS_DEBUG_STREAM("[ITWFEstimation] Initialized KdTree object.");
		
		int num_empty_bins = 0;

		for(int i=0; i<interest_points.points.size(); i++)
		{ 
			PointOutT histogram_point;
			initializeOutputPoint(histogram_point, interest_points.points[i]);

			std::vector<int> nearest_indices;
			std::vector<float> nearest_dist_squareds;

			ROS_DEBUG_STREAM_THROTTLE(1, "[ITWFEstimation] Working on a histogram point with index " << i);

			if ( kdtree.radiusSearch (interest_points_xyz_ptr->points[i], search_radius_, nearest_indices, nearest_dist_squareds) > 0 )
			{
				for (size_t j = 0; j < nearest_indices.size (); ++j)
				{
		  			incrementHistogram(histogram_point, interest_points.points[i], input.points[nearest_indices[j]]);
		  			incrementAverages(histogram_point, input.points[nearest_indices[j]]);
		  			if(i == 100 || i == 300 || i == 563)
		  				ROS_INFO_STREAM("temp: " << i << " " << j << " " << nearest_indices.size());
		  		}
		  		// Normalize all values by the number of neighbor points used
		  		normalizeAverages(histogram_point, nearest_indices.size());
		  		normalizeHistogram(histogram_point);
		  		incrementEmptyBins(num_empty_bins, histogram_point);
		  		if(i == 100 || i == 300 || i == 563)
		  		{
		  			ROS_ERROR_STREAM("itwf intensity: " << histogram_point.histogram[0+36] << " " << histogram_point.histogram[2+36] << " " << histogram_point.histogram[4+36] << " " << histogram_point.histogram[7+36] << " " << histogram_point.histogram[9+36] << " " << histogram_point.histogram[11+36] << " depth: " 
														  << histogram_point.histogram[0] << " " << histogram_point.histogram[2] << " " << histogram_point.histogram[4] << " " << histogram_point.histogram[7] << " " << histogram_point.histogram[9] << " " << histogram_point.histogram[11] << " horz: " 
														  << histogram_point.histogram[0+12] << " " << histogram_point.histogram[2+12] << " " << histogram_point.histogram[4+12] << " " << histogram_point.histogram[7+12] << " " << histogram_point.histogram[9+12] << " " << histogram_point.histogram[11+12] << " second: " 
														  << histogram_point.histogram[0+24] << " " << histogram_point.histogram[2+24] << " " << histogram_point.histogram[4+24] << " " << histogram_point.histogram[7+24] << " " << histogram_point.histogram[9+24] << " " << histogram_point.histogram[11+24] << " others: "  
														  << histogram_point.histogram[0+48] << " " << histogram_point.histogram[2+48] << " " << histogram_point.histogram[4+60] << " " << histogram_point.histogram[7+60] << " " << histogram_point.histogram[9+72] << " " << histogram_point.histogram[11+72]);  
		  			ROS_ERROR_STREAM("temp " << histogram_point.x << " " << histogram_point.y << " " << histogram_point.z);		
		  		}
			}
			else 
				ROS_ERROR_STREAM_THROTTLE(0.1, "[ITWFEstimation] KdTree Nearest Neighbor search failed! Unable to populate histogram for point " << i << "with XYZ values " << interest_points.points[i].x << " " << interest_points.points[i].y << " " << interest_points.points[i].z << ". This message throttled...");
			output.points.push_back(histogram_point); 
		}  
		ROS_DEBUG_STREAM("[ITWFEstimation] Successfully created histogram cloud of size " << output.points.size() << ". Percent of bins unfilled: " << float(num_empty_bins)/(output.points.size()*output.points[0].descriptorSize()/7)*100 << "%%.");
	} 
	template <typename PointInT, typename PointInterestT, typename PointOutT> void 
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::setSearchRadius(const float search_radius)
	{
		search_radius_ = search_radius;
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
		float wall_normal_mag = sqrt(pow(wall_coeffs[0],2) + pow(wall_coeffs[1],2) + pow(wall_coeffs[2],2));
		for(int i=0; i<3; i++)
			normal_vector[i] = wall_coeffs[i] / wall_normal_mag;

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
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  
		ROS_DEBUG_STREAM("[ITFWEstimation] Initialized to the following wall coordinate frame: ");
		ROS_DEBUG_STREAM("                               X: " << in_plane_horz_[0] << " " << in_plane_horz_[1] << " " << in_plane_horz_[2]);
		ROS_DEBUG_STREAM("                               Y: " << in_plane_second_[0] << " " << in_plane_second_[1] << " " << in_plane_second_[2]);
		ROS_DEBUG_STREAM("                               Z: " << normal_vector[0] << " " << normal_vector[1] << " " << normal_vector[2]);
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::getPlaneVectors(Eigen::Vector3f &in_plane_horz, Eigen::Vector3f &in_plane_second)
	{
		in_plane_horz = in_plane_horz_; 
		in_plane_second = in_plane_second_;
	}
	// Abstractions of this method can be written for arbitrary numbers of concentric rings of equal area...
	// The radius of the ith ring in this series is given by:
	//   r_i = R * sqrt(i/n)
	// where R is the radius of the largest ring and n is the number of rings
	template <> int
	ITWFEstimation<pcl::PointWallDamage, pcl::PointWallDamage, pcl::ITWFSignature84>::radialBinChoice(float radial_distance)
	{
		for(int i=0; i<num_rings_-1; i++)
			if(radial_distance < search_radius_*sqrt(float(i+1)/num_rings_))
				return i;
		return num_rings_-1;
	}
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::incrementHistogram(PointOutT &point, const PointInT source_point, const PointInterestT target_point)
	{
		// (Signed!) Distance between points in Horizontal and Secondary Dimensions 
		float horz_distance =   (source_point.x - target_point.x)*in_plane_horz_[0] +   (source_point.y - target_point.y)*in_plane_horz_[1] +   (source_point.z - target_point.z)*in_plane_horz_[2];
		float second_distance = (source_point.x - target_point.x)*in_plane_second_[0] + (source_point.y - target_point.y)*in_plane_second_[1] + (source_point.z - target_point.z)*in_plane_second_[2];
		float radial_distance = sqrt( pow(horz_distance,2) + pow(second_distance,2) );

		// Skip self-matches from the perspective of the FEATURE HISTOGRAM
		// These will still be used in determining the AVERAGES about each target point
		if(radial_distance == 0)
			return;

		// Choose Spatial Binning
		bool left_hemisphere = (horz_distance > 0);
		bool lower_hemisphere = (second_distance > 0);
		int radial_bin = radialBinChoice(radial_distance);

		int spatial_bin = radial_bin + left_hemisphere*num_rings_ + lower_hemisphere*num_rings_*2;
		// Vote In Histogram
		if(dimension_use_[0])  point.histogram[ spatial_bin ] 							+= target_point.depth_offset - source_point.depth_offset;
		if(dimension_use_[1])  point.histogram[ spatial_bin + num_spatial_bins_ ] 		+= target_point.horz_normal_offset;
		if(dimension_use_[2])  point.histogram[ spatial_bin + num_spatial_bins_*2 ] 	+= target_point.second_normal_offset;
		if(dimension_use_[3])  point.histogram[ spatial_bin + num_spatial_bins_*3 ] 	+= float(target_point.intensity - source_point.intensity);
		if(dimension_use_[4])  point.histogram[ spatial_bin + num_spatial_bins_*4 ] 	+= float(target_point.intensity); //target_point.r - source_point.r;
		if(dimension_use_[5])  point.histogram[ spatial_bin + num_spatial_bins_*5 ] 	+= target_point.g - source_point.g;
		if(dimension_use_[6])  point.histogram[ spatial_bin + num_spatial_bins_*6 ] 	+= target_point.b - source_point.b;
		// Increment Spatial Bin Counts
		point.bin_count[ spatial_bin ] ++;

		ROS_DEBUG_STREAM_THROTTLE(0.05, "itwf stuff - x: " << source_point.x << " " << target_point.x << " y: " << source_point.y << " " << target_point.y << " z: " << source_point.z << " " << target_point.z << 
										" intensity values: " << source_point.intensity << " " << target_point.intensity << 
										" aaand: " << horz_distance << " " << second_distance << " " << radial_distance << " hemis: " << left_hemisphere*1 << " " << lower_hemisphere*1 << 
										" bin_nums: " << num_rings_ << " " << radial_bin << " " << num_spatial_bins_ << " " << spatial_bin << 
										" radius: " << search_radius_ << " bounds: " << intensity_minimum_ << " " << intensity_maximum_ << " " << depth_half_lim_ <<
										" bin count: " << point.bin_count[spatial_bin]); 

		ROS_DEBUG_STREAM_THROTTLE(0.05, "itwf intensity: " << point.histogram[0+36] << " " << point.histogram[2+36] << " " << point.histogram[4+36] << " " << point.histogram[7+36] << " " << point.histogram[9+36] << " " << point.histogram[11+36] << " depth: " 
														  << point.histogram[0] << " " << point.histogram[2] << " " << point.histogram[4] << " " << point.histogram[7] << " " << point.histogram[9] << " " << point.histogram[11] << " horz: " 
														  << point.histogram[0+12] << " " << point.histogram[2+12] << " " << point.histogram[4+12] << " " << point.histogram[7+12] << " " << point.histogram[9+12] << " " << point.histogram[11+12] << " second: " 
														  << point.histogram[0+24] << " " << point.histogram[2+24] << " " << point.histogram[4+24] << " " << point.histogram[7+24] << " " << point.histogram[9+24] << " " << point.histogram[11+24] << " others: "  
														  << point.histogram[0+48] << " " << point.histogram[2+48] << " " << point.histogram[4+60] << " " << point.histogram[7+60] << " " << point.histogram[9+72] << " " << point.histogram[11+72]);  
	}
	
	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::incrementEmptyBins(int &num_empty_bins, PointOutT histogram_point)
	{
		for(int i=0; i<histogram_point.descriptorSize()/7; i++)
			num_empty_bins += (histogram_point.bin_count[i] == 0);
	}

	template <typename PointInT, typename PointInterestT, typename PointOutT> void
	ITWFEstimation<PointInT, PointInterestT, PointOutT>::normalizeHistogram(PointOutT &point)
	{
		// *** Normalize Across Point Densities ***
		// Divide each bin by the number of points in that bin
		for(int spatial_bin_ind=0; spatial_bin_ind<num_spatial_bins_; spatial_bin_ind++) 	// Iterate over spatial bins within a data type
			if( point.bin_count[spatial_bin_ind] > 0 )										// Skip empty bins
				for(int dimension_ind=0; dimension_ind<7; dimension_ind++) 					// Iterate over data types (depth, intensity, etc.)
					point.histogram[ dimension_ind*num_spatial_bins_ + spatial_bin_ind ] /= point.bin_count[spatial_bin_ind];

		// *** Normalize Across Data Types ***
		// Divide each bin by the average value for all spatial bins in that dimension within this point
		std::vector<float> dimension_means;
		for(int dimension_ind=0; dimension_ind<7; dimension_ind++)
		{
			dimension_means.push_back(0);
			for(int spatial_bin_ind=0; spatial_bin_ind<num_spatial_bins_; spatial_bin_ind++)
				dimension_means[dimension_ind] += fabs(point.histogram[ dimension_ind*num_spatial_bins_ + spatial_bin_ind ]);

			dimension_means[dimension_ind] /= num_spatial_bins_;
			
			if(dimension_means[dimension_ind] != 0)					// Skip cases where the average is a 0 (eg dimension was not utilized)
				for(int spatial_bin_ind=0; spatial_bin_ind<num_spatial_bins_; spatial_bin_ind++)
					point.histogram[ dimension_ind*num_spatial_bins_ + spatial_bin_ind ] /= dimension_means[dimension_ind];
		}
		/*

		// *** Normalize Across Parameter Dimensions ***
		//   Depth
		for(int i=0; i<num_spatial_bins_; i++)
			point.histogram[ i ] /= depth_half_lim_;
		//   Intensity Difference
		for(int i=0; i<num_spatial_bins_; i++)
		{
			float unscaled = point.histogram[ i + num_spatial_bins_*3 ];
			point.histogram[ i + num_spatial_bins_*3 ] = (unscaled - intensity_minimum_) / (intensity_maximum_ - intensity_minimum_);
		}
		//   Intensity
		for(int i=0; i<num_spatial_bins_; i++)
		{
			float unscaled = point.histogram[ i + num_spatial_bins_*4 ];
			point.histogram[ i + num_spatial_bins_*4 ] = (unscaled - intensity_minimum_) / (intensity_maximum_ - intensity_minimum_);
		}
		////   Red
		//for(int i=0; i<num_spatial_bins_; i++)
		//	point.histogram[ i + num_spatial_bins_*4 ] /= 255;
		//   Blue
		for(int i=0; i<num_spatial_bins_; i++)
			point.histogram[ i + num_spatial_bins_*5 ] /= 255;
		//   Green
		for(int i=0; i<num_spatial_bins_; i++)
			point.histogram[ i + num_spatial_bins_*6 ] /= 255;
			*/

		// *** Pseudo Gradients ***
		// Intensity
		//   Spatial Bins in Left Hemisphere:  3, 4, 5, 9, 10, 11
		//   Spatial Bins in Lower Hemisphere: 0, 1, 2, 3, 4, 5
		//   Note that this approach assumes all bins get at least some values...
		//  Horizontal
		point.intensity_horz_change_avg -= (point.histogram[num_spatial_bins_*4 + 3] + point.histogram[num_spatial_bins_*4 + 4] + point.histogram[num_spatial_bins_*4 + 5] + point.histogram[num_spatial_bins_*4 + 9] + point.histogram[num_spatial_bins_*4 + 10] + point.histogram[num_spatial_bins_*4 + 11]);
		point.intensity_horz_change_avg += (point.histogram[num_spatial_bins_*4 + 0] + point.histogram[num_spatial_bins_*4 + 1] + point.histogram[num_spatial_bins_*4 + 2] + point.histogram[num_spatial_bins_*4 + 6] + point.histogram[num_spatial_bins_*4 + 7] + point.histogram[num_spatial_bins_*4 + 8]);
		point.intensity_horz_change_avg /= (num_rings_*2);
		//  Off-Horizontal
		point.intensity_second_change_avg -= (point.histogram[num_spatial_bins_*4 + 0] + point.histogram[num_spatial_bins_*4 + 1] + point.histogram[num_spatial_bins_*4 + 2] + point.histogram[num_spatial_bins_*4 + 3] + point.histogram[num_spatial_bins_*4 + 4] + point.histogram[num_spatial_bins_*4 + 5]);
		point.intensity_second_change_avg += (point.histogram[num_spatial_bins_*4 + 6] + point.histogram[num_spatial_bins_*4 + 7] + point.histogram[num_spatial_bins_*4 + 8] + point.histogram[num_spatial_bins_*4 + 9] + point.histogram[num_spatial_bins_*4 + 10] + point.histogram[num_spatial_bins_*4 + 11]);
		point.intensity_second_change_avg /= (num_rings_*2);


		ROS_ERROR_STREAM_THROTTLE(0.02,"outputs of normalized data: \n\t\t\t\t Depth: " << point.histogram[0] << " " << point.histogram[1] << " "  << point.histogram[2] << " "  << point.histogram[3] << " "  << point.histogram[4] << " "  << point.histogram[5] << " "  << point.histogram[6] << " "  << point.histogram[7] << " "  << point.histogram[8] << " "  << point.histogram[9] << " "  << point.histogram[10] << " "  << point.histogram[11] <<
																   "\n\t\t\t\t 1st N: " << point.histogram[12+0] << " " << point.histogram[12+1] << " "  << point.histogram[12+2] << " "  << point.histogram[12+3] << " "  << point.histogram[12+4] << " "  << point.histogram[12+5] << " "  << point.histogram[12+6] << " "  << point.histogram[12+7] << " "  << point.histogram[12+8] << " "  << point.histogram[12+9] << " "  << point.histogram[12+10] << " "  << point.histogram[12+11] <<
																   "\n\t\t\t\t 2nd N: " << point.histogram[24+0] << " " << point.histogram[24+1] << " "  << point.histogram[24+2] << " "  << point.histogram[24+3] << " "  << point.histogram[24+4] << " "  << point.histogram[24+5] << " "  << point.histogram[24+6] << " "  << point.histogram[24+7] << " "  << point.histogram[24+8] << " "  << point.histogram[24+9] << " "  << point.histogram[24+10] << " "  << point.histogram[24+11] <<
																   "\n\t\t\t\t dInt:  " << point.histogram[36+0] << " " << point.histogram[36+1] << " "  << point.histogram[36+2] << " "  << point.histogram[36+3] << " "  << point.histogram[36+4] << " "  << point.histogram[36+5] << " "  << point.histogram[36+6] << " "  << point.histogram[36+7] << " "  << point.histogram[36+8] << " "  << point.histogram[36+9] << " "  << point.histogram[36+10] << " "  << point.histogram[36+11] <<
																   "\n\t\t\t\t Int:   " << point.histogram[48+0] << " " << point.histogram[48+1] << " "  << point.histogram[48+2] << " "  << point.histogram[48+3] << " "  << point.histogram[48+4] << " "  << point.histogram[48+5] << " "  << point.histogram[48+6] << " "  << point.histogram[48+7] << " "  << point.histogram[48+8] << " "  << point.histogram[48+9] << " "  << point.histogram[48+10] << " "  << point.histogram[48+11] <<
																   "\n\t\t\t\t Color: " << point.histogram[60+0] << " " << point.histogram[60+1] << " "  << point.histogram[60+2] << " "  << point.histogram[60+3] << " "  << point.histogram[60+4] << " "  << point.histogram[60+5] << " "  << point.histogram[60+6] << " "  << point.histogram[60+7] << " "  << point.histogram[60+8] << " "  << point.histogram[60+9] << " "  << point.histogram[60+10] << " "  << point.histogram[60+11] );
	}

}

#endif // IMPL_ITWF_ESTIMATION_