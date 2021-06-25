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

#include <string>
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/sensor_tins.hpp"


namespace sensor
{

  SensorTins::SensorTins() : SensorInterface() {}

  SensorTins::~SensorTins()
  {
    _ouster_client.reset();
    _lidar_packet.clear();
    _imu_packet.clear();
  }

  void SensorTins::reset(const ros2_ouster::Configuration & config)
  {
    _ouster_client.reset();
    configure(config);
  }

  void SensorTins::configure(const ros2_ouster::Configuration & config)
  {
    if (!ouster::sensor::lidar_mode_of_string(config.lidar_mode)) 
    {
      throw ros2_ouster::OusterDriverException(
        "Invalid lidar mode: " + config.lidar_mode);
      exit(-1);
    }

    if (!ouster::sensor::timestamp_mode_of_string(config.timestamp_mode)) 
    {
      throw ros2_ouster::OusterDriverException(
        "Invalid timestamp mode: " + config.timestamp_mode);
      exit(-1);
    }

    // Read metadata from file
    loadSensorInfoFromJsonFile(
        config.metadata_filepath,
        _metadata);  

    // loadSensorInfoFromJsonFile actually returns a sensor_info object, so 
    // fill in the params specific to the ros2_ouster::Metadata object, that 
    // aren't normally supplied in the metadata file.
    _metadata.imu_port = config.imu_port;
    _metadata.lidar_port = config.lidar_port;
    _metadata.timestamp_mode = config.timestamp_mode;

    // Fill anything missing with defaults and resize the packet containers
    ros2_ouster::populate_missing_metadata_defaults(_metadata);
    _lidar_packet.resize(getPacketFormat().lidar_packet_size + 1);
    _imu_packet.resize(getPacketFormat().imu_packet_size + 1);

    // Create and initialize the Tins sniffer object

  }

  ouster::sensor::client_state SensorTins::get()
  {
    
  }

  uint8_t * SensorTins::readLidarPacket(const ouster::sensor::client_state & state)
  {
    if (state == ouster::sensor::client_state::LIDAR_DATA)
    {
      return _lidar_packet.data();
    }
    else
    {
      return nullptr;
    }
  }

  uint8_t * SensorTins::readImuPacket(const ouster::sensor::client_state & state)
  {
    if (state == ouster::sensor::client_state::IMU_DATA)
    {
      return _imu_packet.data();
    }
    else
    {
      return nullptr;
    }
  }

  ros2_ouster::Metadata SensorTins::getMetadata()
  {
    return _metadata;
  }

  ouster::sensor::packet_format SensorTins::getPacketFormat()
  {
    return ouster::sensor::get_format(getMetadata());
  }

  void SensorTins::loadSensorInfoFromJsonFile(
    std::string filepath_to_read,
    ouster::sensor::sensor_info& sensor_info)
  {
    try
    {
      sensor_info = ouster::sensor::metadata_from_json(filepath_to_read);
    }
    catch(const std::exception& e)
    {
      throw ros2_ouster::OusterDriverException(
        "Failed to read metadata from file: " + filepath_to_read + 
        " with exception ");
      exit(-1);
    }    
  }

} // namespace sensor