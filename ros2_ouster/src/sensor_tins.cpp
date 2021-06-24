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

  }

  void SensorTins::configure(const ros2_ouster::Configuration & config)
  {

  }

  ouster::sensor::client_state SensorTins::get()
  {
    
  }

  uint8_t * SensorTins::readLidarPacket(const ouster::sensor::client_state & state)
  {

  }

  uint8_t * SensorTins::readImuPacket(const ouster::sensor::client_state & state)
  {
    
  }

  void SensorTins::setMetadata(
    int lidar_port, int imu_port,
    const std::string & timestamp_mode)
  {
    
  }

  ros2_ouster::Metadata SensorTins::getMetadata()
  {
    return _metadata;
  }

  ouster::sensor::packet_format SensorTins::getPacketFormat()
  {
    return ouster::sensor::get_format(getMetadata());
  }

} // namespace sensor