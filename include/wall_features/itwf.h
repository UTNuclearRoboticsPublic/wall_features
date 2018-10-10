
#ifndef ITWF_H_
#define ITWF_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>

namespace pcl {
struct ITWFSignature90
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float	histogram[90];				// 40 points for distance deviation, 40 points for angle deviation - might change this later... 
  float horz_angle_offset_avg;       // Average offset in horizontal angle between local normal vectors of all neighbor points and the expected normal vector of the containing plane primitive definition
  float second_angle_offset_avg;       // Average offset in secondary angle between local normal vectors of all neighbor points and the expected normal vector of the containing plane primitive definition
  float dist_offset_avg;      // Average offset in position between all neighbor points and the containing plane primitive definition
  // The average values of the various partial derivatives in X and Y
  float dist_offset_deriv_horz_avg;
  float dist_offset_deriv_second_avg;
  float horz_angle_offset_deriv_horz_avg;
  float horz_angle_offset_deriv_second_avg;
  float second_angle_offset_deriv_horz_avg;
  float second_angle_offset_deriv_second_avg;
  // Should the above be split into two histograms? Not sure what makes most sense computationally. For now, first half of the indices are distance-based, second half are angle-based 
  float depth_half_lim;    // These two variables define the minima and maxima of the depth bins (angles are fixed -90 to 90 degrees)
  float depth_deriv_half_lim;
  float normal_deriv_half_lim;

  static int descriptorSize(){return 90;}
  friend std::ostream& operator << (std::ostream& os, const ITWFSignature90& p);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
}
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ITWFSignature90, 
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[90], histogram, histogram)
                                   (float, horz_angle_offset_avg, horz_angle_offset_avg)
                                   (float, second_angle_offset_avg, second_angle_offset_avg)
                                   (float, dist_offset_avg, dist_offset_avg)
)
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::ITWFSignature90& p);


namespace pcl {
template <>
class DefaultPointRepresentation<ITWFSignature90> : public PointRepresentation<ITWFSignature90>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 90;
  }

  virtual void
  copyToFloatArray (const ITWFSignature90 &p, float * out) const
  {
    for (int i = 0; i < nr_dimensions_; ++i)
      out[i] = p.histogram[i];
  }
};
}


// This implementation incorporates LiDAR return intensity 
namespace pcl {
struct ITWFSignature210
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float histogram[210];        // 40 points for distance deviation, 40 points for angle deviation - might change this later... 
  float dist_offset_avg;      // Average offset in position between all neighbor points and the containing plane primitive definition
  float horz_angle_offset_avg;       // Average offset in horizontal angle between local normal vectors of all neighbor points and the expected normal vector of the containing plane primitive definition
  float second_angle_offset_avg;       // Average offset in secondary angle between local normal vectors of all neighbor points and the expected normal vector of the containing plane primitive definition
  float intensity_avg;
  float r_avg;
  float g_avg;
  float b_avg;
  // The average values of the various partial derivatives in X and Y
  float dist_offset_deriv_horz_avg;
  float dist_offset_deriv_second_avg;
  float horz_angle_offset_deriv_horz_avg;
  float horz_angle_offset_deriv_second_avg;
  float second_angle_offset_deriv_horz_avg;
  float second_angle_offset_deriv_second_avg;
  float intensity_deriv_horz_avg;
  float intensity_deriv_second_avg;
  float r_deriv_horz_avg;
  float r_deriv_second_avg;
  float g_deriv_horz_avg;
  float g_deriv_second_avg;
  float b_deriv_horz_avg;
  float b_deriv_second_avg;
  // Should the above be split into two histograms? Not sure what makes most sense computationally. For now, first half of the indices are distance-based, second half are angle-based 
  float depth_half_lim;    // These two variables define the minima and maxima of the depth bins (angles are fixed -90 to 90 degrees)
  float depth_deriv_half_lim;
  float normal_deriv_half_lim;
  float intensity_deriv_half_lim;
  float r_deriv_half_lim;
  float g_deriv_half_lim;
  float b_deriv_half_lim;

  static int descriptorSize(){return 210;}
  friend std::ostream& operator << (std::ostream& os, const ITWFSignature210& p);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
}
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ITWFSignature210, 
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[210], histogram, histogram)
                                   (float, dist_offset_avg, dist_offset_avg)
                                   (float, horz_angle_offset_avg, horz_angle_offset_avg)
                                   (float, second_angle_offset_avg, second_angle_offset_avg)
                                   (float, intensity_avg, intensity_avg)
                                   (float, r_avg, r_avg)
                                   (float, g_avg, g_avg)
                                   (float, b_avg, b_avg)
                                   (float, intensity_deriv_horz_avg, intensity_deriv_horz_avg)
                                   (float, intensity_deriv_second_avg, intensity_deriv_second_avg)
                                   (float, r_deriv_horz_avg, r_deriv_horz_avg)
                                   (float, r_deriv_second_avg, r_deriv_second_avg)
                                   (float, g_deriv_horz_avg, g_deriv_horz_avg)
                                   (float, g_deriv_second_avg, g_deriv_second_avg)
                                   (float, b_deriv_horz_avg, b_deriv_horz_avg)
                                   (float, b_deriv_second_avg, b_deriv_second_avg)
)
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const pcl::ITWFSignature210& p);


namespace pcl {
template <>
class DefaultPointRepresentation<ITWFSignature210> : public PointRepresentation<ITWFSignature210>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 210;
  }

  virtual void
  copyToFloatArray (const ITWFSignature210 &p, float * out) const
  {
    for (int i = 0; i < nr_dimensions_; ++i)
      out[i] = p.histogram[i];
  }
};
}

#endif // ITWF_H_