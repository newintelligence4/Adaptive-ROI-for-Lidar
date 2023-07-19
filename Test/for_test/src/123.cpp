#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <math.h>
#include "clustering/clustering_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;
typedef pcl::PointXYZ PointT;

void cloud_saver(const std::string& file_name,std::string& path, pcl::PointCloud<PointT>::Ptr cloud_arg){
  pcl::PCDWriter cloud_writer;
  cloud_writer.write<pcl::PointXYZ>(path+std::string(file_name),*cloud_arg, false);
}
Clustering::Clustering(const rclcpp::NodeOptions& options) : Node("clustering",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","/os1_cloud_node/points");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_clustering");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 10, std::bind(&Clustering::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       clustering\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());

}

void Clustering::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Basic Cloud objects
  pcl::PointCloud<PointT>::Ptr cloud        (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr voxel_cloud  (new pcl::PointCloud<PointT>);
  pcl::ModelCoefficients::Ptr  coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr       inliers      (new pcl::PointIndices);

  pcl::ModelCoefficients::Ptr       cylinder_co    (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr            cylinder_in    (new pcl::PointIndices);
  pcl::search::KdTree<PointT>::Ptr  tree           (new pcl::search::KdTree<PointT> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals  (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr      cylinder_cloud (new pcl::PointCloud<PointT> ());

  pcl::fromROSMsg(*msg, *cloud);

  // Normals computation objects
  pcl::NormalEstimation<PointT,pcl::Normal>            normals_estimator;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
  pcl::ExtractIndices<PointT>                          cylinder_indices_extractor;
  pcl::ExtractIndices<pcl::Normal>                     cylinder_indices_extractor_temp;

  // Voxel filter applying
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.05,0.05,0.05);
  voxel_filter.filter(*voxel_cloud);


  // Performing estimation of normals
  normals_estimator.setSearchMethod(tree);
  normals_estimator.setInputCloud(voxel_cloud);
  normals_estimator.setKSearch(30);
  normals_estimator.compute(*cloud_normals);

  // Parameters for segmentation
  cylinder_segmentor.setOptimizeCoefficients(true);
	cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
	cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
	cylinder_segmentor.setNormalDistanceWeight(0.5);
	cylinder_segmentor.setMaxIterations(10000);
	cylinder_segmentor.setDistanceThreshold(0.05);
	cylinder_segmentor.setRadiusLimits(0.1, 0.4);

  int l=0;
  std::string path="/home/skyautonet/temp/src/for_test/clouds/";
  while(true){


  // Appplying segmentation
  cylinder_segmentor.setInputCloud(voxel_cloud);
  cylinder_segmentor.setInputNormals(cloud_normals);
  cylinder_segmentor.segment(*cylinder_in,*cylinder_co);

  // extracting indices
  cylinder_indices_extractor.setInputCloud(voxel_cloud);
  cylinder_indices_extractor.setIndices(cylinder_in);
  cylinder_indices_extractor.setNegative(false);
  cylinder_indices_extractor.filter(*cylinder_cloud);

    // if(!cylinder_cloud->points.empty()){
    //     std::stringstream ss ;ss<< "ex_cylinder_"<<l<<".pcd";
    //     std::cout<<"Cloud Contains " <<cylinder_cloud->points.size()<<std::endl;
    //     if(cylinder_cloud->points.size() > 90){
    //         cloud_saver(ss.str(),path,cylinder_cloud);
    //         l++;
    //     }

    //     cylinder_indices_extractor.setNegative(true);
    //     cylinder_indices_extractor.filter(*voxel_cloud);

    //     // processing normals
    //     cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
    //     cylinder_indices_extractor_temp.setIndices(cylinder_in);
    //     cylinder_indices_extractor_temp.setNegative(true);
    //     cylinder_indices_extractor_temp.filter(*cloud_normals);

    // }
  }
  pcl::PCLPointCloud2 cloud_out;
  pcl::toPCLPointCloud2(*cylinder_cloud, cloud_out);

  sensor_msgs::msg::PointCloud2 output;

  pcl_conversions::fromPCL(cloud_out,output);  

  output.header.frame_id = msg->header.frame_id;
  output.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisher_->publish(output);
}
