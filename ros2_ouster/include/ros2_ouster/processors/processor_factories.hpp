// Copyright 2021, Steve Macenski
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

#ifndef ROS2_OUSTER__PROCESSORS__PROCESSOR_FACTORIES_HPP_
#define ROS2_OUSTER__PROCESSORS__PROCESSOR_FACTORIES_HPP_

#include <cstdint>
#include <string>
#include <map>
#include <utility>
#include <memory>

#include "image_processor.hpp"
#include "imu_processor.hpp"
#include "pointcloud_processor.hpp"
#include "rclcpp/qos.hpp"
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/conversions.hpp"
#include "ros2_ouster/processors/scan_processor.hpp"
#include "ros2_ouster/string_utils.hpp"

namespace ros2_ouster
{

constexpr std::uint32_t PROC_IMG = (1 << 0);
constexpr std::uint32_t PROC_PCL = (1 << 1);
constexpr std::uint32_t PROC_IMU = (1 << 2);
constexpr std::uint32_t PROC_SCAN = (1 << 3);

constexpr std::uint32_t DEFAULT_PROC_MASK =
  PROC_IMG | PROC_PCL | PROC_IMU | PROC_SCAN;

/**
 * Transforms a data processor mask-like-string into a mask value
 *
 * We define a mask-like-string to be a pipe-separated list of
 * data processor suffixes. For example, all of the following
 * are valid:
 *
 * IMG|PCL|IMU|SCAN
 * IMG|PCL
 * PCL
 *
 * @param[in] mask_str The string to convert into a mask
 * @return The mask obtained from the parsed input string.
 */
inline std::uint32_t
toProcMask(const std::string & mask_str)
{
  std::uint32_t mask = 0x0;
  auto tokens = ros2_ouster::split(mask_str, '|');

  for (auto & token : tokens) {
    if (token == "IMG") {
      mask |= ros2_ouster::PROC_IMG;
    } else if (token == "PCL") {
      mask |= ros2_ouster::PROC_PCL;
    } else if (token == "IMU") {
      mask |= ros2_ouster::PROC_IMU;
    } else if (token == "SCAN") {
      mask |= ros2_ouster::PROC_SCAN;
    }
  }

  return mask;
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the image (range, intensity, noise) interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline std::unique_ptr<ros2_ouster::DataProcessorInterface> createImageProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ouster::sensor::sensor_info & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos,
  const ouster::sensor::packet_format & pf,
  std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator)
{
  return std::make_unique<sensor::ImageProcessor>(
    node, mdata, frame, qos, pf,
    fullRotationAccumulator);
//  return new sensor::ImageProcessor(node, mdata, frame, qos, pf);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the pointcloud (PointXYZ be default) interface
 * @return Raw pointer to a data processor interface to use
 */
inline std::unique_ptr<ros2_ouster::DataProcessorInterface> createPointcloudProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ouster::sensor::sensor_info & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos,
  const ouster::sensor::packet_format & pf,
  std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator)
{
  return std::make_unique<sensor::PointcloudProcessor>(
    node, mdata, frame, qos, pf, fullRotationAccumulator);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the IMU interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline std::unique_ptr<ros2_ouster::DataProcessorInterface> createIMUProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ouster::sensor::sensor_info & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos,
  const ouster::sensor::packet_format & pf)
{
  return std::make_unique<sensor::IMUProcessor>(node, mdata, frame, qos, pf);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the scan interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline std::unique_ptr<ros2_ouster::DataProcessorInterface> createScanProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ouster::sensor::sensor_info & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos,
  const ouster::sensor::packet_format & pf,
  std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator)
{
  return std::make_unique<sensor::ScanProcessor>(
    node, mdata, frame, qos, pf,
    fullRotationAccumulator);
}

inline std::multimap<ouster::sensor::client_state,
  std::unique_ptr<ros2_ouster::DataProcessorInterface>> createProcessors(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ouster::sensor::sensor_info & mdata,
  const std::string & imu_frame,
  const std::string & laser_frame,
  const rclcpp::QoS & qos,
  const ouster::sensor::packet_format & pf,
  std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator,
  std::uint32_t mask = ros2_ouster::DEFAULT_PROC_MASK)
{
  std::multimap<ouster::sensor::client_state,
    std::unique_ptr<ros2_ouster::DataProcessorInterface>> data_processors;

  if ((mask & ros2_ouster::PROC_IMG) == ros2_ouster::PROC_IMG) {
    data_processors.insert(
      std::pair<ouster::sensor::client_state, std::unique_ptr<ros2_ouster::DataProcessorInterface>>(
        ouster::sensor::client_state::LIDAR_DATA, createImageProcessor(
          node, mdata, laser_frame, qos, pf,
          fullRotationAccumulator)));
  }

  if ((mask & ros2_ouster::PROC_PCL) == ros2_ouster::PROC_PCL) {
    data_processors.insert(
      std::pair<ouster::sensor::client_state, std::unique_ptr<ros2_ouster::DataProcessorInterface>>(
        ouster::sensor::client_state::LIDAR_DATA, createPointcloudProcessor(
          node, mdata, laser_frame, qos, pf,
          fullRotationAccumulator)));
  }

  if ((mask & ros2_ouster::PROC_IMU) == ros2_ouster::PROC_IMU) {
    data_processors.insert(
      std::pair<ouster::sensor::client_state, std::unique_ptr<ros2_ouster::DataProcessorInterface>>(
        ouster::sensor::client_state::IMU_DATA, createIMUProcessor(
          node, mdata, imu_frame, qos, pf)));
  }

  if ((mask & ros2_ouster::PROC_SCAN) == ros2_ouster::PROC_SCAN) {
    data_processors.insert(
      std::pair<ouster::sensor::client_state, std::unique_ptr<ros2_ouster::DataProcessorInterface>>(
        ouster::sensor::client_state::LIDAR_DATA, createScanProcessor(
          node, mdata, laser_frame, qos, pf,
          fullRotationAccumulator)));
  }

  return data_processors;
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__PROCESSORS__PROCESSOR_FACTORIES_HPP_
