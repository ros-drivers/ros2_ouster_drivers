// Copyright 2021, Matthew Young (Trimble Inc)
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

#ifndef ROS2_OUSTER__SENSOR_TINS_HPP_
#define ROS2_OUSTER__SENSOR_TINS_HPP_

#include <memory>
#include <vector>
#include "ros2_ouster/processors/processor_factories.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/client/client.h"

namespace sensor
{

class SensorTins : public ros2_ouster::SensorInterface
{
  public:

    /**
     * @brief Default constructor
     */
    SensorTins();

    /**
     * @brief Default destructor 
     */
    ~SensorTins();

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
    ouster::sensor::client_state get() override;

    /**
     * @brief reading a lidar packet
     * @param state of the sensor
     * @return the packet of data
     */
    uint8_t * readLidarPacket(
      const ouster::sensor::client_state & state) override;

    /**
     * @brief reading an imu packet
     * @param state of the sensor
     * @return the packet of data
     */
    uint8_t * readImuPacket(
      const ouster::sensor::client_state & state) override;

    /**
     * @brief Sets the metadata class variable
     * @param lidar_port
     * @param imu_port
     */
    void setMetadata(
      int lidar_port, 
      int imu_port,
      const std::string & timestamp_mode);

    /**
     * @brief Get lidar sensor's packet format
     * @return packet format struct
     */
    ouster::sensor::packet_format getPacketFormat() override;

    /**
     * @brief Load metadata from a file. 
     * @details Some important notes about this function: This populates an 
     *          ouster::sensor::sensor_info object, but the more commonly used 
     *          ros2_ouster::Metadata object has some additional parameters
     *          that must be sourced elsewhere. Also note that the underlying 
     *          Ouster library is inconsistent in what parameters must be 
     *          provided in the JSON file vs what will be silently set to a 
     *          default value if not provided.
     * @param filepath_to_read A fully qualified filepath to a yaml file 
     *        containing the parameters to load
     * @param sensor_info The metadata to load data into. 
     */
    void loadSensorInfoFromJsonFile(
      const std::string filepath_to_read,
      ouster::sensor::sensor_info& sensor_info);

  private:

    std::shared_ptr<ouster::sensor::client> _ouster_client;
    std::vector<uint8_t> _lidar_packet;
    std::vector<uint8_t> _imu_packet;
    ros2_ouster::Metadata _metadata{};
};

}  // namespace sensor

#endif  // ROS2_OUSTER__SENSOR_TINS_HPP_