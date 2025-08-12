/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

/**
 * \file rslidar_component.cpp
 * Component wrapper implementation for RSLidar
 */

#include "rslidar_component.hpp"
#include "utility/yaml_reader.hpp"
#include <rs_driver/macro/version.hpp>

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#endif

using namespace robosense::lidar;

namespace robosense
{
namespace lidar
{

RSLidarComponent::RSLidarComponent(const rclcpp::NodeOptions & options)
: Node("rslidar_component", options), should_stop_(false)
{
  RCLCPP_INFO(this->get_logger(), "RoboSense-LiDAR-Driver Component V%d.%d.%d", 
              RSLIDAR_VERSION_MAJOR, RSLIDAR_VERSION_MINOR, RSLIDAR_VERSION_PATCH);
  
  // Get config path parameter
  config_path_ = this->declare_parameter<std::string>("config_path", "");
  
  if (config_path_.empty()) {
    // Default config path relative to package
    config_path_ = std::string(PROJECT_PATH) + "/config/config.yaml";
  }
  
  RCLCPP_INFO(this->get_logger(), "Using config file: %s", config_path_.c_str());
  
  initialize();
}

RSLidarComponent::~RSLidarComponent()
{
  should_stop_ = true;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  if (node_manager_) {
    node_manager_->stop();
  }
}

void RSLidarComponent::initialize()
{
  try {
    YAML::Node config = YAML::LoadFile(config_path_);
    node_manager_ = std::make_shared<NodeManager>();
    node_manager_->init(config);
    node_manager_->start();
    
    // Start worker thread to monitor the driver
    worker_thread_ = std::thread(&RSLidarComponent::run, this);
    
    RCLCPP_INFO(this->get_logger(), "RSLidar component initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize RSLidar driver: %s", e.what());
    throw;
  }
}

void RSLidarComponent::run()
{
  while (!should_stop_ && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace lidar
} // namespace robosense

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLidarComponent)
