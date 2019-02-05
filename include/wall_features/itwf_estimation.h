

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
        using Feature<PointInT, PointOutT>::search_radius_;
        using Feature<PointInT, PointOutT>::search_parameter_; 
        typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
        typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;
        //int k_;
        float depth_half_lim_, intensity_minimum_, intensity_maximum_;     // Feature Limits
        int num_spatial_bins_, num_rings_; 
        Eigen::Vector3f in_plane_horz_, in_plane_second_;
        std::vector<bool> dimension_use_;
	public:
		typedef boost::shared_ptr<ITWFEstimation<PointInT,PointInterestT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const ITWFEstimation<PointInT,PointInterestT,PointOutT> > ConstPtr;

		/** \brief Empty constructor. */
		ITWFEstimation () 
		{
			search_radius_ = 0.05;
            depth_half_lim_ = 0.03;
            intensity_minimum_ = 0;
            intensity_maximum_ = 6000;
			feature_name_ = "ITWFEstimation";

            PointOutT temp;
            num_spatial_bins_ = temp.descriptorSize()/7;
            num_rings_ = num_spatial_bins_/4;
            for(int i=0; i<7; i++)
                dimension_use_.push_back(1);
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
        void setSearchRadius(float search_radius);
        void setDimensionUse(std::vector<bool> dimension_use) { dimension_use_ = dimension_use; };
        // Initializing Data
        void initializeOutputPoint(PointOutT &histogram_point, PointInT input_point);
        // Updating the Histogram
        int radialBinChoice(float radial_distance);
        void incrementHistogram(PointOutT &point, const PointInT source_point, const PointInterestT target_point);   
        void incrementEmptyBins(int &num_empty_bins, PointOutT histogram_point);
        void normalizeHistogram(PointOutT &point);
        // Updating Averages
        void incrementAverages(PointOutT &histogram_point, PointInT neighbor_point);
        void normalizeAverages(PointOutT &histogram_point, int num_points);
        // Set Wall Parameters 
        void setPlaneVectors(Eigen::Vector3f in_plane_horz, Eigen::Vector3f in_plane_second);
        void setPlaneVectors(float horz_x, float horz_y, float horz_z, float second_x, float second_y, float second_z);
        void setWallCoordinateFrame(std::vector<float> wall_coeffs);
        void getPlaneVectors(Eigen::Vector3f &in_plane_horz, Eigen::Vector3f &in_plane_second);
        // Set Histogram Scales
        void setHistogramScales(float depth_half_lim, float intensity_minimum, float intensity_maximum);
	};





};

#ifdef PCL_NO_PRECOMPILE
#include "wall_features/wall_damage_estimation.hpp"
#endif // PCL_NO_PRECOMPILE

#endif // ITWF_ESTIMATION_