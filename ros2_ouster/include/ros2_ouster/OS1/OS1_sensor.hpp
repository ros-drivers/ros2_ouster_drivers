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

#ifndef ROS2_OUSTER__OS1__OS1_SENSOR_HPP_
#define ROS2_OUSTER__OS1__OS1_SENSOR_HPP_

#include <memory>
#include <vector>

#include "ros2_ouster/OS1/processor_factories.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/OS1/OS1.hpp"

namespace OS1
{

class OS1Sensor : public ros2_ouster::SensorInterface
{
public:
  OS1Sensor();

  ~OS1Sensor() override;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  void reset(const ros2_ouster::Configuration & config) override;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  void configure(const ros2_ouster::Configuration & config) override;

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  ros2_ouster::Metadata getMetadata() override;

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  ros2_ouster::ClientState get() override;

  /**
   * @brief reading the packet corresponding to the sensor state
   * @param state of the sensor
   * @return the packet of data
   */
  uint8_t * readPacket(const ros2_ouster::ClientState & state) override;

private:
  std::shared_ptr<client> _ouster_client;
  std::vector<uint8_t> _lidar_packet;
  std::vector<uint8_t> _imu_packet;
};

}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__OS1_SENSOR_HPP_
