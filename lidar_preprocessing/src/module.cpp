#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_preprocessing/lidar_preprocessing_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include "velocity_msg/msg/num.hpp"  
#include "xyzit_msg/msg/custom.hpp"

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#define PI 3.14159265359

pcl::PCLPointCloud2 adaptive_roi(const pcl::PCLPointCloud2 cloud, int velocity)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(cloud, *cloud_new);

  //// KDTREE
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundery (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_new, *input);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> idxes;
  std::vector<float> sqr_dists;
  pcl::PointXYZ query(+ 2.5, -5.0, 0.0);

  float times = 1.8;
  float radius = round(velocity * times);
  
  kdtree.setInputCloud(input);
  kdtree.radiusSearch(query, radius, idxes, sqr_dists);
  for (const auto& idx: idxes){
    boundery->points.push_back(cloud_new->points[idx]);
  }

  // ROI - Angle
  pcl::PassThrough<pcl::PointXYZI> pass;

  pass.setInputCloud(boundery);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-13.0, +2.5);
  pass.filter(*boundery);

  for(unsigned int j=0; j<boundery->points.size(); j++)
  {
    double angle_h = get_cos((boundery->points[j].y + 5) , (boundery->points[j].x - 2.5));
    double angle_v = get_tan(boundery->points[j].x, boundery->points[j].y, boundery->points[j].z);
    
    if( angle_h < 4)
    {
        boundery->points[j].x = 0;
        boundery->points[j].y = 0;
        boundery->points[j].z = 0;
    }
    if(angle_h > 150)
    {
        boundery->points[j].x = 0;
        boundery->points[j].y = 0;
        boundery->points[j].z = 0;
    }
    if((boundery->points[j].x - 2.5) > 0)
    {
        boundery->points[j].x = 0;
        boundery->points[j].y = 0;
        boundery->points[j].z = 0;
    }
    if(angle_v > get_limit(velocity))
    {
        boundery->points[j].x = 0;
        boundery->points[j].y = 0;
        boundery->points[j].z = 0;
    }
  }
  pass.setInputCloud(boundery);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.0, 0.0);
  pass.setFilterLimitsNegative(true);
  pass.filter(*boundery);

  pcl::PCLPointCloud2 cloud_ROI;
  pcl::toPCLPointCloud2(*boundery, cloud_ROI);

  return cloud_ROI;
}

pcl::PCLPointCloud2 ground_removal(const pcl::PCLPointCloud2 cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground_removal (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(cloud, *cloud_new);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients (true); // Optional

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud (cloud_new);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZI> extract_indices;
  extract_indices.setInputCloud(cloud_new);
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(false);
  extract_indices.filter(*cloud_ground_removal);

  pcl::PCLPointCloud2 output;
  pcl::toPCLPointCloud2(*cloud_ground_removal, output);

  return output;
}

pcl::PCLPointCloud2 downsample(const pcl::PCLPointCloud2 cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromPCLPointCloud2(cloud, *cloud_new);

  pcl::VoxelGrid<pcl::PointXYZI> vox;

  vox.setInputCloud(cloud_new);
  vox.setLeafSize(0.15, 0.15, 0.15);
  vox.filter(*cloud_new);

  int num_neigbor_points = 10;
  double std_multiplier = 1.0;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud_new);
  sor.setMeanK (num_neigbor_points);
  sor.setStddevMulThresh (std_multiplier);
  sor.filter (*cloud_filtered); 


  pcl::PCLPointCloud2 cloud_out;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_out);
  
  return cloud_out;
}

double get_cos(double x, double y)
{
  double r;
  double theta;

  r = sqrt((x*x)+(y*y));
  theta = acos(x/r)*180/PI;
  return theta;
}     
double get_tan(double x, double y, double z)
{
  double r;
  double theta;

  r = sqrt((x*x)+(y*y));
  theta = atan(z/r)*180/PI;
  return theta;
}
double get_limit(int vel){
  double limit;
  int slow = vel - 60;
  if (slow < 0){
    limit = 22.5;
  }
  else{
    limit = 9.5 - ((0.2)*slow);
  }
  return limit;
}  