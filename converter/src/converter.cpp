#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "converter/converter_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
std::string output_type;
std::string lidar_topic = "/point_cloud"; 


Converter::Converter(const rclcpp::NodeOptions& options) : Node("converter",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","/os1_cloud_node/points");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_converter");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, std::bind(&Converter::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       converter\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());
}
// VLP-16 
int N_SCAN = 16;
int Horizon_SCAN = 1800;    

static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

void Converter::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {

        PointXYZIRT new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = 0; 

        //16 ring. The range of index is 0~15. Up to Down.
        float ang_bottom = 15.0+0.1;
        float ang_res_y = 2;
        float verticalAngle = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) * 180 / M_PI;
        float rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        
        new_point.ring = int(rowIdn);
        new_point.time = (point_id / N_SCAN)*0.1/Horizon_SCAN ;

        pc_new->points.push_back(new_point);
    }

    pc_new->is_dense = true;
    // publish
    sensor_msgs::msg::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*pc_new, pc_new_msg);

    pc_new_msg.header = msg->header;
    pc_new_msg.header.frame_id = "/os1_lidar";
    pc_new_msg.header.stamp = msg->header.stamp;

    RCLCPP_INFO(this->get_logger(), "Time is %ld", pc_new_msg.header.stamp);
    
    publisher_->publish(pc_new_msg);
}
