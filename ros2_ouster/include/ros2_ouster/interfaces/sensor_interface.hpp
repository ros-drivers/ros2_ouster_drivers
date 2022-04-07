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

#ifndef ROS2_OUSTER__INTERFACES__SENSOR_INTERFACE_HPP_
#define ROS2_OUSTER__INTERFACES__SENSOR_INTERFACE_HPP_

#include <memory>

#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/interfaces/configuration.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/client/client.h"

namespace ros2_ouster
{
/**
 * @class ros2_ouster::SensorInterface
 * @brief An interface for lidars units
 */
class SensorInterface
{
public:
  using SharedPtr = std::shared_ptr<SensorInterface>;
  using Ptr = std::unique_ptr<SensorInterface>;

  /**
   * @brief A sensor interface constructor
   */
  SensorInterface() {}

  /**
   * @brief A sensor interface destructor
   */
  virtual ~SensorInterface() = default;

  // copy
  SensorInterface(const SensorInterface &) = delete;
  SensorInterface & operator=(const SensorInterface &) = delete;

  // move
  SensorInterface(SensorInterface &&) = default;
  SensorInterface & operator=(SensorInterface &&) = default;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   * @param node pointer to the driver node, which provides access to ROS params
   */
  virtual void reset(
    ros2_ouster::Configuration & config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   * @param node pointer to the driver node, which provides access to ROS params
   */
  virtual void configure(
    ros2_ouster::Configuration & config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  virtual ouster::sensor::client_state get() = 0;

  /**
   * @brief reading a lidar packet
   * @param state of the sensor
   * @param buf pointer to a buffer to hold the packet data. Must hold getPacketFormat().lidar_packet_size bytes.
   * @return true if a packet was recieved, false otherwise
   */
  virtual bool readLidarPacket(const ouster::sensor::client_state & state, uint8_t * buf) = 0;

  /**
   * @brief reading an imu packet
   * @param state of the sensor
   * @param buf pointer to a buffer to hold the packet data. Must hold getPacketFormat().imu_packet_size bytes.
   * @return true if a packet was recieved, false otherwise
   */
  virtual bool readImuPacket(const ouster::sensor::client_state & state, uint8_t * buf) = 0;

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  virtual ros2_ouster::Metadata getMetadata() = 0;

  /**
   * @brief Get lidar sensor's packet format
   * @return packet format struct
   */
  virtual ouster::sensor::packet_format getPacketFormat() = 0;
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__SENSOR_INTERFACE_HPP_
