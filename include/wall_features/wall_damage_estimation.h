

#ifndef WALL_DAMAGE_ESTIMATION_
#define WALL_DAMAGE_ESTIMATION_

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

typedef pcl::PointXYZRGB PCLPoint;
typedef pcl::PointCloud<pcl::PointNormal> PCN;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PCNP;

namespace pcl
{

	template <typename PointInT, typename PointOutT>
	class WallDamagePointwiseEstimation//: public Feature<PointInT, PointOutT>
	{
	public:
		typedef boost::shared_ptr<WallDamagePointwiseEstimation<PointInT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const WallDamagePointwiseEstimation<PointInT,PointOutT> > ConstPtr;
		/*using Feature<PointInT, PointOutT>::feature_name_;
        using Feature<PointInT, PointOutT>::getClassName;
        using Feature<PointInT, PointOutT>::indices_;
        using Feature<PointInT, PointOutT>::input_;
        using Feature<PointInT, PointOutT>::surface_;
        using Feature<PointInT, PointOutT>::k_;
        using Feature<PointInT, PointOutT>::search_radius_;
        using Feature<PointInT, PointOutT>::search_parameter_; */
        typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
        typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;
        int k_;
        float wall_coeffs_[4]; 			// Wall Coefficients

		/** \brief Empty constructor. */
		WallDamagePointwiseEstimation () 
		{
			k_ = 30;
			//feature_name_ = "WallDamageEstimation";
		};

		/** \brief Empty destructor */
		virtual ~WallDamagePointwiseEstimation () {}
		void compute (const pcl::PointCloud<PointInT> &input, pcl::PointCloud<PointOutT> &output);
		void computeFeature (pcl::PointCloud<PointOutT> &output);
		
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
        void setWallCoefficients(const float wall_coeffs[4]);
	};




	template <typename PointInT, typename PointInterestT, typename PointOutT>
	class WallDamageHistogramEstimation//: public Feature<PointInT, PointOutT>
	{
	public:
		typedef boost::shared_ptr<WallDamageHistogramEstimation<PointInT,PointInterestT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const WallDamageHistogramEstimation<PointInT,PointInterestT,PointOutT> > ConstPtr;
		/*using Feature<PointInT, PointOutT>::feature_name_;
        using Feature<PointInT, PointOutT>::getClassName;
        using Feature<PointInT, PointOutT>::indices_;
        using Feature<PointInT, PointOutT>::input_;
        using Feature<PointInT, PointOutT>::surface_;
        using Feature<PointInT, PointOutT>::k_;
        using Feature<PointInT, PointOutT>::search_radius_;
        using Feature<PointInT, PointOutT>::search_parameter_; */
        typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
        typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;
        int k_;
        float angle_min_, angle_max_, dist_min_, dist_max_;	 	// Bin Limits

		/** \brief Empty constructor. */
		WallDamageHistogramEstimation () 
		{
			k_ = 30;
			//feature_name_ = "WallDamageEstimation";
		};

		/** \brief Empty destructor */
		virtual ~WallDamageHistogramEstimation () {}
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
        void setBinLimits(float angle_min, float angle_max, float dist_min, float dist_max);
	};





};

#ifdef PCL_NO_PRECOMPILE
#include "wall_features/wall_damage_estimation.hpp"
#endif // PCL_NO_PRECOMPILE

#endif // WALL_DAMAGE_ESTIMATION_