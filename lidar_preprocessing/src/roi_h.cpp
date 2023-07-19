#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "lidar_preprocessing/roi_h_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <vector>
#include <pcl/common/transforms.h>
#include "velocity_msg/msg/num.hpp"  
#include "xyzit_msg/msg/custom.hpp"

using std::placeholders::_1;

#define PI 3.14159265359

pcl::PCLPointCloud2 adaptive_roi(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int velocity);
double get_cos(double x, double y);
double get_tan(double x, double y, double z);
double get_limit(int vel);
float theta_r = 270*M_PI/180;

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
int speed = 0;
ROI_Horizon::ROI_Horizon(const rclcpp::NodeOptions& options) : Node("roi_h",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","/os1_cloud_node/points");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_roi_h");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 10, std::bind(&ROI_Horizon::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       roi_h\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    ROI_Horizon in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());


  declare_parameter<std::string>("topic_velocity_in","topic");
  param_topic_velocity_in = get_parameter("topic_velocity_in").as_string();
  subscription_v = this->create_subscription<velocity_msg::msg::Num>(          
  param_topic_velocity_in, 10, std::bind(&ROI_Horizon::velocity_callback, this, _1));

}

void ROI_Horizon::velocity_callback(const velocity_msg::msg::Num::SharedPtr vel)
{
  speed = vel->num;
}

void ROI_Horizon::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  //RCLCPP_INFO(this->get_logger(), "speed: '%d'", speed);
  pcl::PCLPointCloud2 cloud_ROI = adaptive_roi(msg, speed);
  xyzit_msg::msg::Custom output;

  pcl_conversions::fromPCL(cloud_ROI,output.point);  
  //std::cerr << "Filtered: "<< cloud_ROI.width * cloud_ROI.height << " data points ("<< pcl::getFieldsList(cloud_ROI)<<")."<<std::endl;
  output.point.header.frame_id = msg->header.frame_id;
  output.stamp = msg->header.stamp;

  publisher_->publish(output.point);
}

pcl::PCLPointCloud2 adaptive_roi(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int velocity)
{
  std::cerr << "Speed: "<< velocity << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  /////// Transform Matrix
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  transform_1 (0,0) = std::cos (theta_r);
  transform_1 (0,1) = -sin(theta_r);
  transform_1 (1,0) = sin (theta_r);
  transform_1 (1,1) = std::cos (theta_r);

  pcl::transformPointCloud (*cloud, *cloud, transform_1); 
  //std::cerr << "Original: "<< cloud->width * cloud->height << " data points ("<< pcl::getFieldsList(*cloud)<<")."<<std::endl;                       

  //// KDTREE
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundery (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *input);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> idxes;
  std::vector<float> sqr_dists;
  pcl::PointXYZ query(+ 2.5, -5.0, 0.0);

  float times = 1.8;
  float radius = round(velocity * times);
  
  kdtree.setInputCloud(input);
  kdtree.radiusSearch(query, radius, idxes, sqr_dists);
  for (const auto& idx: idxes){
    boundery->points.push_back(cloud->points[idx]);
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