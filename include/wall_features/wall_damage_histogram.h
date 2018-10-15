
#ifndef WALL_DAMAGE_HISTOGRAM_H_
#define WALL_DAMAGE_HISTOGRAM_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>

namespace pcl {
struct WallDamageHistogram
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_NORMAL4D; 				// This adds the member normal[3] which can also be accessed using the point (which is float[4])
  float	histogram[80];				// 40 points for distance deviation, 40 points for angle deviation - might change this later... 
  float angle_offset_avg;       // Average offset in angle between local normal vectors of each neighbor point and the expected normal vector of the containing plane primitive definition
  float depth_offset_avg;      // Average offset in position between each neighbor point and the containing plane primitive definition
  // Should the above be split into two histograms? Not sure what makes most sense computationally. For now, first half of the indices are distance-based, second half are angle-based 
  static int descriptorSize(){return 80;}
  friend std::ostream& operator << (std::ostream& os, const WallDamageHistogram& p);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
}
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::WallDamageHistogram, 
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[80], histogram, histogram)
                                   (float, angle_offset_avg, angle_offset_avg)
                                   (float, depth_offset_avg, depth_offset_avg)

)
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::WallDamageHistogram& p);


namespace pcl {
template <>
class DefaultPointRepresentation<WallDamageHistogram> : public PointRepresentation<WallDamageHistogram>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 80;
  }

  virtual void
  copyToFloatArray (const WallDamageHistogram &p, float * out) const
  {
    for (int i = 0; i < nr_dimensions_; ++i)
      out[i] = p.histogram[i];
  }
};
}


#endif // WALL_DAMAGE_HISTOGRAM_H_