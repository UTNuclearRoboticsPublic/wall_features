


#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "wall_features/point_wall_damage.h"
#include "wall_features/wall_damage_histogram.h"
#include "wall_features/wall_damage_estimation.h"
#include "wall_features/wall_damage_estimation.hpp"

template class pcl::WallDamagePointwiseEstimation<PCLPoint, PointWallDamage>;
template class pcl::WallDamageHistogramEstimation<PointWallDamage, WallDamageHistogram>;