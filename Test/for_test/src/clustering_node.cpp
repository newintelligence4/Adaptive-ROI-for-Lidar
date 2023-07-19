// Copyright (c) 2022 Jonas Mahler

// This file is part of clustering.

// clustering is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// clustering is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with Foobar. If not, see <https://www.gnu.org/licenses/>. 

#include <clustering/clustering_node.hpp>
#include <time.h>

int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  clock_t start = clock();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clustering>());
  clock_t end = clock();
  std::cerr << "Time: "<< (double)(end-start)/ CLOCKS_PER_SEC << std::endl; 
  rclcpp::shutdown();
  return 0;
}