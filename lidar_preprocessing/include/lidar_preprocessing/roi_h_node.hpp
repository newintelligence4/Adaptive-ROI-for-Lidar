#ifndef ROI_H__ROI_H_NODE_HPP_
#define ROI_H__ROI_H_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "velocity_msg/msg/num.hpp"  

class ROI_Horizon : public rclcpp::Node
{
  public:

    explicit ROI_Horizon(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ROI_Horizon() {};

  protected:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void velocity_callback(const velocity_msg::msg::Num::SharedPtr vel);

    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    rclcpp::Subscription<velocity_msg::msg::Num>::SharedPtr subscription_v;
    std::string param_topic_velocity_in;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out;
    
};
#endif //ROI_H__ROI_H_NODE_HPP_
