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

#ifndef ROS2_OUSTER__SENSOR_HPP_
#define ROS2_OUSTER__SENSOR_HPP_

#include <memory>
#include <vector>
#include <string>

#include "ros2_ouster/processors/processor_factories.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/ros2_utils.hpp"

namespace sensor
{

class Sensor : public ros2_ouster::SensorInterface
{
public:
  Sensor();

  ~Sensor() override;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   * @param node pointer to the driver node, which provides access to ROS params
   */
  void reset(
    ros2_ouster::Configuration & config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   * @param node pointer to the driver node, which provides access to ROS params
   */
  void configure(
    ros2_ouster::Configuration & config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  ros2_ouster::Metadata getMetadata() override;

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  ouster::sensor::client_state get() override;

  /**
   * @brief reading a lidar packet
   * @param state of the sensor
   * @param buf pointer to a buffer to hold the packet data. Must hold getPacketFormat().lidar_packet_size bytes.
   * @return true if a packet was recieved, false otherwise
   */
  bool readLidarPacket(const ouster::sensor::client_state & state, uint8_t * buf) override;

  /**
   * @brief reading an imu packet
   * @param state of the sensor
   * @param buf pointer to a buffer to hold the packet data. Must hold getPacketFormat().imu_packet_size bytes.
   * @return true if a packet was recieved, false otherwise
   */
  bool readImuPacket(const ouster::sensor::client_state & state, uint8_t * buf) override;

  /**
   * @brief Sets the metadata class variable
   * @param lidar_port
   * @param imu_port
   */
  void setMetadata(
    int lidar_port, int imu_port,
    const std::string & timestamp_mode);

  /**
   * @brief Get lidar sensor's packet format
   * @return packet format struct
   */
  ouster::sensor::packet_format getPacketFormat() override;

private:
  std::shared_ptr<ouster::sensor::client> _ouster_client;
  ros2_ouster::Metadata _metadata{};
};

}  // namespace sensor

#endif  // ROS2_OUSTER__SENSOR_HPP_
