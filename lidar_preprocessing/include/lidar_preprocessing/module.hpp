#ifndef LIDAR_PREPROCESSING__MODULE_HPP_
#define LIDAR_PREPROCESSING__MODULE_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PCLPointCloud2 adaptive_roi(const pcl::PCLPointCloud2 cloud, int velocity);
pcl::PCLPointCloud2 ground_removal(const pcl::PCLPointCloud2 cloud);
pcl::PCLPointCloud2 downsample(const pcl::PCLPointCloud2 cloud);

double get_cos(double x, double y);
double get_tan(double x, double y, double z);
double get_limit(int vel);

#endif //LIDAR_PREPROCESSING__MODULE_HPP_