

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Choosing to encode calculated angle_offset for each point, and retain normal information, even though these are functionally redundant for us
// Choosing NOT to encode gross plane information in each point since it's the same across all of them - just need to make sure to manage this intelligently
//   Note that the angle and dist offsets are only meaningful given a fixed external reference plane! 

namespace pcl {
struct PointWallDamage
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_NORMAL4D;         // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  float angle_offset;         // Offset in angle between local normal vector and expected normal vector of the containing plane primitive definition
  float dist_offset;        // Offset in position between the point and the containing plane primitive definition

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

}
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointWallDamage, 
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, angle_offset, angle_offset)
                                   (float, dist_offset, dist_offset)
)
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::PointWallDamage& p);