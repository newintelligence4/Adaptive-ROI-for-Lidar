// Copyright (c) 2022 Jonas Mahler

// This file is part of ground_removal.

// ground_removal is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// ground_removal is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with ground_removal. If not, see <https://www.gnu.org/licenses/>. 

#ifndef GROUND_REMOVAL__GROUND_REMOVAL_NODE_HPP_
#define GROUND_REMOVAL__GROUND_REMOVAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_msgs/msg/model_coefficients.hpp"

/**
 * @class ground_removal::RANSAC
 * @brief Receives Pointcloud2 message from lidar sensor and filter its points with an optional pcl filter.
 * 
 */
class RANSAC : public rclcpp::Node
{
  public:
    
    /**
     * @brief A constructor for ground_removal::RANSAC class
     * @param options Additional options to control creation of the node.
     */
    explicit RANSAC(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief A destructor for ground_removal::RANSAC class
     */
    ~RANSAC() {};

  protected:
    /**
     * @brief Use a no filter of pcl library
     * @param msg Pointcloud2 message receveived from the ros2 node
     * @return -
     * @details Omit pointcloud filtering in this example
     */
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out;

    //rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr publisher_;
    //std::string param_topic_pointcloud_out;
    
};

#endif //GROUND_REMOVAL__GROUND_REMOVAL_NODE_HPP_