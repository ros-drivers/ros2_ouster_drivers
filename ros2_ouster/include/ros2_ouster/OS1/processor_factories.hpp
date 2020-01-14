// Copyright 2020, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_OUSTER__OS1__PROCESSOR_FACTORIES_HPP_
#define ROS2_OUSTER__OS1__PROCESSOR_FACTORIES_HPP_

#include <string>
#include <map>
#include <utility>

#include "ros2_ouster/conversions.hpp"
#include "ros2_ouster/OS1/processors/image_processor.hpp"
#include "ros2_ouster/OS1/processors/imu_processor.hpp"
#include "ros2_ouster/OS1/processors/pointcloud_processor.hpp"
#include "ros2_ouster/OS1/processors/scan_processor.hpp"

namespace ros2_ouster
{

/**
 * @brief Factory method to get a pointer to a processor
 * to create the image (range, intensity, noise) interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_ouster::DataProcessorInterface * createImageProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_ouster::Metadata & mdata,
  const std::string & frame)
{
  return new OS1::ImageProcessor(node, mdata, frame);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the pointcloud (PointXYZ be default) interface
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_ouster::DataProcessorInterface * createPointcloudProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_ouster::Metadata & mdata,
  const std::string & frame)
{
  return new OS1::PointcloudProcessor(node, mdata, frame);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the IMU interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_ouster::DataProcessorInterface * createIMUProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_ouster::Metadata & mdata,
  const std::string & frame)
{
  return new OS1::IMUProcessor(node, mdata, frame);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the scan interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_ouster::DataProcessorInterface * createScanProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_ouster::Metadata & mdata,
  const std::string & frame)
{
  return new OS1::ScanProcessor(node, mdata, frame);
}

inline std::multimap<ClientState, DataProcessorInterface *> createProcessors(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_ouster::Metadata & mdata,
  const std::string & imu_frame,
  const std::string & laser_frame)
{
  std::multimap<ClientState, DataProcessorInterface *> data_processors;

  data_processors.insert(std::pair<ClientState, DataProcessorInterface *>(
      ClientState::IMU_DATA, createIMUProcessor(
        node, mdata, imu_frame)));
  data_processors.insert(std::pair<ClientState, DataProcessorInterface *>(
      ClientState::LIDAR_DATA, createPointcloudProcessor(
        node, mdata, laser_frame)));
  data_processors.insert(std::pair<ClientState, DataProcessorInterface *>(
      ClientState::LIDAR_DATA, createImageProcessor(
        node, mdata, laser_frame)));
  data_processors.insert(std::pair<ClientState, DataProcessorInterface *>(
      ClientState::LIDAR_DATA, createScanProcessor(
        node, mdata, laser_frame)));

  return data_processors;
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__OS1__PROCESSOR_FACTORIES_HPP_
