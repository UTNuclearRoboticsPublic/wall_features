# Table of Contents
1. [About](#about)
2. [Wall Damage Features](#parameter-setup)
3. [Wall Change Estimation](#usage)
4. [Rasterizer](#usage)

## About
This package contains a lot of different utilities which I might end up splitting into separate packages! Right now the main ones are Wall Features, Wall Change Estimation, and the Rasterizer. Each of these is aimed at a different aspect of analyzing the change or damage to planar walls based on LiDAR pointcloud data. The original goal of this package was to enable documentation of the degredation of concrete walls via LiDAR. Details on each of these subpackages are provided in the following sections.

## Wall Damage Features
Several custom pcl feature types were created, both to describe the damage to the wall and to facilitate automatic registration of subsequent wall scans with one another. The following feature types were created:

1. Point_Wall_Damage
- XYZ data describing a point in the wall
- Normal vector data describing the local curvature at that point
- Normal deviation out of or into the expected planar wall surface

Depth deviation from expected planar wall:

<img src=images/depth.png width="600">

Angle deviation from expected planar wall normal:

<img src=images/angle.png width="600">

Normal direction deviation in horizontal direction from expected planar wall normal:

<img src=images/horizontal_normal.png width="600">

Normal direction deviation in secondary direction from expected planar wall normal:

<img src=images/secondary_normal.png width="600">

2. Wall Damage Histogram
- XYZ data describing a point in the wall
- Based on a neighborhood of Point_Wall_Damage points around that point, a histogram of their various normal vector angular deviations from the wall normal
- Based on a neighborhood of Point_Wall_Damage points around that point, a histogram of their various normal depth deviations from the planar wall surface
- The average angular deviation in the histogram
- The average depth deviation in the histogram

3. Is This Wall Flat (ITWF)
- XYZ data describing a point in the wall
- Based on a neighborhood of Point_Wall_Damage points around that, a histogram of their differences from the target point in color, intensity, depth, and normals, as well as the partial derivatives in the horizontal and secondary axes within the plane
- The average values of each of these variables within the local neighborhood for each point

## Wall Change Estimation
This subpackage allows the estimation of change between subsequent scans of a planar wall. TWo different interfaces are provided - one for pointcloud space and one for raster space. 

The pointcloud space method checks each point in the new cloud to see the minimum distance to the nearest point in the old cloud and compiles an 'intensity' cloud based on these distances. The determined differences can be based either on actual absolute distance to the nearest point, or differences in the wall 'depth deviation' values recorded in the respective points (in an XYZI cloud, as a temporary standin for the Point_Wall_Damage structure). 

The raster space method aligns the two raster images on top of one another and checks the depth values in each image against those in the other to determine the deviation at each pixel. It then creates a third raster image of the found differences. 

## Rasterizer
This subpackage allows the downsampling of a planar pointcloud into a raster depth image of the wall. The wall is projected onto the XZ plane and then depth values in Y are taken as the color values of the resulting image. The real-world size of pixels is specified in a yaml file, and the average depth of all the points in each pixel is kept. If no points are found within a pixel, it remains black. 

This avenue was pursued because organized raster images are more efficient for searches and CV analysis than pointclouds, because location is encoded within the matrix structure and so KD Tree searches are not necessary in order to find a point's neighbors. 
