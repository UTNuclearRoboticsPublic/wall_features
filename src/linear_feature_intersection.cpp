

// Includes
#include <wall_features/linear_feature_intersection.h>


template <typename PointType>
LinearFeatureIntersections<PointType>::LinearFeatureIntersections()
{
	// do some constructor stuff
}


// Generate a list of feature points within the cloud.
template <typename PointType>
void LinearFeatureIntersections<PointType>::findFeatureIntersections()
{
	
	//while
}


// Build a single line segment struct
template <typename PointType>
void LinearFeatureIntersections<PointType>::generateLineSegment()
{

}



// All points are initally assigned to lines until there are too
//   few points left to build a new line
template <typename PointType>
void LinearFeatureIntersections<PointType>::splitLineSegment()
{

}


// Threshold Line Segment
//   Small lines are discarded if below inlier point count threshold
template <typename PointType>
void LinearFeatureIntersections<PointType>::thresholdLineSegment()
{

}