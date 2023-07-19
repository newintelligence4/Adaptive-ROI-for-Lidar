#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "lidar_preprocessing/ransac_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

using std::placeholders::_1;
    
RANSAC::RANSAC(const rclcpp::NodeOptions& options) : Node("ransac",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","/os1_cloud_node/points");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_ransac");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 10, std::bind(&RANSAC::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       lidar_preprocessing\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    RANSAC - Ground Removal in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());

}

void RANSAC::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground_removal (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  
  pcl::fromROSMsg(*msg, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients (true); // Optional

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud (cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZI> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(false);
  extract_indices.filter(*cloud_ground_removal);


  pcl::VoxelGrid<pcl::PointXYZI> vox;

  vox.setInputCloud(cloud_ground_removal);
  vox.setLeafSize(0.15, 0.15, 0.15);
  vox.filter(*cloud_ground_removal);

  int num_neigbor_points = 10;
  double std_multiplier = 1.0;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud_ground_removal);
  sor.setMeanK (num_neigbor_points);
  sor.setStddevMulThresh (std_multiplier);
  sor.filter (*cloud_filtered); 


  pcl::PCLPointCloud2 cloud_out;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_out);
    
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_out, output);

  publisher_->publish(output);
}
