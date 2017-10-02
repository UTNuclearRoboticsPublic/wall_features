

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
	class WallDamagePointwiseEstimation: public Feature<PointInT, PointOutT>
	{
	public:
		/** \brief Empty constructor. */
		WallDamagePointwiseEstimation () 
		{
			//feature_name_ = "WallDamageEstimation";
		};

		/** \brief Empty destructor */
		virtual ~WallDamagePointwiseEstimation () {}
		void compute (const pcl::PointCloud<PointInT> &input, pcl::PointCloud<PointOutT> &output);
	};

	template <typename PointInT, typename PointOutT>
	class WallDamageHistogramEstimation: public Feature<PointInT, PointOutT>
	{
	public:
		/** \brief Empty constructor. */
		WallDamageHistogramEstimation () 
		{
			//feature_name_ = "WallDamageEstimation";
		};

		/** \brief Empty destructor */
		virtual ~WallDamageHistogramEstimation () {}
		void compute (const pcl::PointCloud<PointInT> &input, pcl::PointCloud<PointOutT> &output);
		void compute (const pcl::PointCloud<PointInT> &input, const pcl::PointCloud<PointInT> &interest_points, pcl::PointCloud<PointOutT> &output);
	};
};

#endif // WALL_DAMAGE_ESTIMATION_