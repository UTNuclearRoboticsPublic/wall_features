

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

#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>

#include <wall_features/point_wall_damage.h>

#include <set>

namespace pcl
{

    template <typename PointInT, typename PointNormalT, typename PointOutT>
    class WallDamagePointwiseEstimation: public FeatureFromNormals<PointInT, PointNormalT, PointOutT>
    {
    public:
        typedef typename pcl::PointCloud<PointInT> PC;
        typedef typename pcl::PointCloud<PointInT>::Ptr PCP;
        typedef typename pcl::PointCloud<PointNormalT> PCN;
        typedef typename pcl::PointCloud<PointNormalT>::Ptr PCNP;

        typedef typename pcl::search::KdTree<PointInT> KDTree;
        typedef typename pcl::search::KdTree<PointInT>::Ptr KDTreePtr;

		typedef boost::shared_ptr<WallDamagePointwiseEstimation<PointInT,PointNormalT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const WallDamagePointwiseEstimation<PointInT,PointNormalT,PointOutT> > ConstPtr;
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
        float wall_coeffs_[4]; 			// Wall Coefficients
        // Coordinate Frame Within Wall
        Eigen::Vector3f wall_x_axis_;    // Horizontal. If entire plane is horizontal, this is set to the global X-axis
        Eigen::Vector3f wall_y_axis_;    // Normal to X and Z
        Eigen::Vector3f wall_z_axis_;    // Wall Normal
        Eigen::Vector3f viewpoint_;      // XYZ coordinates of sensor viewpoint

		/** \brief Empty constructor. */
		WallDamagePointwiseEstimation () 
		{
            feature_name_ = "WallDamagePointwiseEstimation";
			k_ = 30;
            wall_coeffs_[0] = 1;
            wall_coeffs_[1] = 0;
            wall_coeffs_[2] = 0;
            wall_coeffs_[3] = 0;
            viewpoint_[0] = 0;
            viewpoint_[1] = 0;
            viewpoint_[2] = 0;
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
        void setViewpoint(const float viewpoint[3]);
        void findWallCoordinateFrame();
        void setColorValues(PointInT input_point, PointOutT &damage_point);
	};




	template <typename PointInT, typename PointInterestT, typename PointOutT>
	class WallDamageHistogramEstimation: public Feature<PointInT, PointOutT>
	{
	public:
		typedef boost::shared_ptr<WallDamageHistogramEstimation<PointInT,PointInterestT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const WallDamageHistogramEstimation<PointInT,PointInterestT,PointOutT> > ConstPtr;
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
        float angle_min_, angle_max_, dist_min_, dist_max_;	 	// Bin Limits

		/** \brief Empty constructor. */
		WallDamageHistogramEstimation () 
		{
            feature_name_ = "WallDamageHistogramEstimation";
			k_ = 30;
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