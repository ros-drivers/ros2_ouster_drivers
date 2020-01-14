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

#ifndef ROS2_OUSTER__INTERFACES__SENSOR_INTERFACE_HPP_
#define ROS2_OUSTER__INTERFACES__SENSOR_INTERFACE_HPP_

#include <memory>

#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/interfaces/configuration.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"

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
  virtual ~SensorInterface() {}

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  virtual void reset(const ros2_ouster::Configuration & config) {}

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  virtual void configure(const ros2_ouster::Configuration & config) {}

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  virtual ros2_ouster::ClientState get() {}

  /**
   * @brief reading the packet corresponding to the sensor state
   * @param state of the sensor
   * @return the packet of data
   */
  virtual uint8_t * readPacket(const ros2_ouster::ClientState & state) {}

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  virtual ros2_ouster::Metadata getMetadata() {}
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__SENSOR_INTERFACE_HPP_
