

#ifndef WALL_DAMAGE_ESTIMATION_
#define WALL_DAMAGE_ESTIMATION_

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

namespace pcl
{
	template <typename PointInT, typename PointOutT>
	class WallDamageEstimation: public Feature<PointInT, PointOutT>
	{
	public:
		/** \brief Empty constructor. */
		WallDamageEstimation () 
		{
			//feature_name_ = "WallDamageEstimation";
		};

		/** \brief Empty destructor */
		virtual ~WallDamageEstimation () {}
		void compute (const pcl::PointCloud<PointInT> &input, pcl::PointCloud<PointOutT> &output);
	};
};

#endif // WALL_DAMAGE_ESTIMATION_