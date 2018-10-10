

#ifndef ITWF_ESTIMATION_
#define ITWF_ESTIMATION_

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>

#include <set>

namespace pcl
{

	template <typename PointInT, typename PointInterestT, typename PointOutT>
	class ITWFEstimation: public Feature<PointInT, PointOutT>
	{
    protected:
        using Feature<PointInT, PointOutT>::feature_name_;
        using Feature<PointInT, PointOutT>::getClassName;
        using Feature<PointInT, PointOutT>::indices_;
        using Feature<PointInT, PointOutT>::input_;
        using Feature<PointInT, PointOutT>::surface_;
        using Feature<PointInT, PointOutT>::k_;
        using Feature<PointInT, PointOutT>::search_radius_;
        using Feature<PointInT, PointOutT>::search_parameter_; 
        typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
        typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;
        //int k_;
        float dist_half_lim_, dist_deriv_half_lim_, minimum_radius_, intensity_lower_value_, intensity_upper_value_;     // Bin Limits
        Eigen::Vector3f in_plane_horz_, in_plane_second_;
	public:
		typedef boost::shared_ptr<ITWFEstimation<PointInT,PointInterestT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const ITWFEstimation<PointInT,PointInterestT,PointOutT> > ConstPtr;

		/** \brief Empty constructor. */
		ITWFEstimation () 
		{
			k_ = 30;
            dist_half_lim_ = 0.1;
            dist_deriv_half_lim_ = 0.1;
            minimum_radius_ = 0.05;
            intensity_lower_value_ = 0;
            intensity_upper_value_ = 4000; 
			feature_name_ = "ITWFEstimation";
		};

		/** \brief Empty destructor */
		virtual ~ITWFEstimation () {}
		void compute (const pcl::PointCloud<PointInT> &input, pcl::PointCloud<PointOutT> &output);
		void compute (const pcl::PointCloud<PointInT> &input, const pcl::PointCloud<PointInterestT> &interest_points, pcl::PointCloud<PointOutT> &output);
		void computeFeature (pcl::PointCloud<PointOutT> &output);
		void setInputCloud(const pcl::PointCloud<PointInT> &input);

		virtual inline void 
        setInputCloud (const PointCloudConstPtr &cloud)
        { /*
          input_ = cloud;
          if (use_sensor_origin_)
          {
            vpx_ = input_->sensor_origin_.coeff (0);
            vpy_ = input_->sensor_origin_.coeff (1);
            vpz_ = input_->sensor_origin_.coeff (2);
          } */
        }
        void setKSearch(int k_search);
        void setBinLimits(float dist_half_lim, float dist_deriv_half_lim, float minimum_radius);
        void setBinLimits(float dist_half_lim, float dist_deriv_half_lim, float minimum_radius, float intensity_lower_value, float intensity_upper_value);
        void setPlaneVectors(Eigen::Vector3f in_plane_horz, Eigen::Vector3f in_plane_second);
        void setPlaneVectors(float horz_x, float horz_y, float horz_z, float second_x, float second_y, float second_z);
        void setWallCoordinateFrame(std::vector<float> wall_coeffs);
        void findColorDerivatives(const PointInT source_point, const PointInterestT target_point,
                                    float horz_distance,         float second_distance,
                                    float &intensity_deriv_horz, float &intensity_deriv_second,
                                    float &r_deriv_horz,         float &r_deriv_second,
                                    float &g_deriv_horz,         float &g_deriv_second,
                                    float &b_deriv_horz,         float &b_deriv_second);
        void comparePoints(PointOutT &point, const PointInT source_point, const PointInterestT target_point);
        void addHistogramValues(PointOutT &point, float depth_offset, float normal_offset_horz, float normal_offset_second, 
                                                  float depth_deriv_horz, float horz_normal_deriv_horz, float second_normal_deriv_horz,
                                                  float depth_deriv_second, float horz_normal_deriv_second, float second_normal_deriv_second,
                                                  float intensity, float intensity_deriv_horz, float intensity_deriv_second,
                                                  float r, float r_deriv_horz, float r_deriv_second,
                                                  float g, float g_deriv_horz, float g_deriv_second,
                                                  float b, float b_deriv_horz, float b_deriv_second );    
        void initializeOutputPoint(PointOutT &histogram_point, pcl::PointWallDamage input_point);
        void incrementAverages(PointOutT &histogram_point, pcl::PointWallDamage neighbor_point);
        void normalizeValues(PointOutT &histogram_point, int num_points);
        int getBinIndex(float target_value, float bin_min, float bin_max, int bin_num);
	};





};

#ifdef PCL_NO_PRECOMPILE
#include "wall_features/wall_damage_estimation.hpp"
#endif // PCL_NO_PRECOMPILE

#endif // ITWF_ESTIMATION_