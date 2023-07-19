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
#include "clustering/depth_cluster.h"   



using std::placeholders::_1;
typedef pcl::PointXYZ PointT;
DepthCluster depthCluster(2, 0.2, 16, 20);
void cloud_saver(const std::string& file_name,std::string& path, pcl::PointCloud<PointT>::Ptr cloud_arg){
  pcl::PCDWriter cloud_writer;
  cloud_writer.write<pcl::PointXYZ>(path+std::string(file_name),*cloud_arg, false);
}
Clustering::Clustering(const rclcpp::NodeOptions& options) : Node("clustering",options) 
{
      
  //declare_parameter<std::string>("topic_pointcloud_in","bf_lidar/point_cloud_ransac");
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
  laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *laserCloudIn);

  depthCluster.setInputCloud(laserCloudIn);
  vector<vector<int>> clustersIndex = depthCluster.getClustersIndex();
  for (auto cluster_vec:clustersIndex) {
      int intensity = rand()%255;
      for (int i = 0; i < cluster_vec.size(); ++i) {
          laserCloudIn->points[cluster_vec[i]].intensity = intensity;
      }
  }
  auto ground_index = depthCluster.getGroundCloudIndices();
  int intensity = 100;
  for (int j : ground_index) {
      laserCloudIn->points[j].intensity = intensity;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*laserCloudIn, ground_index, *ground_points);

  sensor_msgs::msg::PointCloud2 laserCloudTemp;
  //pcl::toROSMsg(*ground_points, laserCloudTemp);
  //laserCloudTemp.header.stamp = msg->header.stamp;
  //laserCloudTemp.header.frame_id = msg->header.frame_id;
  //ground_pub.publish(laserCloudTemp);

  auto cluster_indices = depthCluster.getMergedClustersIndex();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*laserCloudIn, cluster_indices, *cluster_points);
  pcl::toROSMsg(*cluster_points, laserCloudTemp);
  laserCloudTemp.header.stamp = msg->header.stamp;
  laserCloudTemp.header.frame_id = msg->header.frame_id;
  //point_pub.publish(laserCloudTemp);


  // Publish to ROS2 network
  publisher_->publish(laserCloudTemp);
}
