

#ifndef IMPL_WALL_DAMAGE_ESTIMATION_
#define IMPL_WALL_DAMAGE_ESTIMATION_

#include "wall_damage_estimation.h"

namespace pcl
{ 
	template <typename PointInT, typename PointOutT> void 
	WallDamageEstimation<PointInT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{
		for(int i=0; i<input.points.size(); i++)
		{
			output.points[i].x = input.points[i].x;
			output.points[i].y = input.points[i].y;
			output.points[i].z = input.points[i].z;

		}
	} 
}

#endif // IMPL_WALL_DAMAGE_ESTIMATION_x