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

#include <string>

#include "ros2_ouster/conversions.hpp"
#include "ros2_ouster/OS1/OS1_sensor.hpp"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/client/client.h"

namespace OS1
{

OS1Sensor::OS1Sensor()
: SensorInterface() {}

OS1Sensor::~OS1Sensor()
{
  _ouster_client.reset();
  _lidar_packet.clear();
  _imu_packet.clear();
}

void OS1Sensor::reset(const ros2_ouster::Configuration & config)
{
  _ouster_client.reset();
  configure(config);
}

void OS1Sensor::configure(const ros2_ouster::Configuration & config)
{
  if (!ouster::sensor::lidar_mode_of_string(config.lidar_mode)) {
    throw ros2_ouster::OusterDriverException(
            std::string("Invalid lidar mode %s!", config.lidar_mode.c_str()));
    exit(-1);
  }

  if (!ouster::sensor::timestamp_mode_of_string(config.timestamp_mode)) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Invalid timestamp mode %s!", config.timestamp_mode.c_str()));
    exit(-1);
  }

  _ouster_client = ouster::sensor::init_client(
    config.lidar_ip,
    config.computer_ip,
    ouster::sensor::lidar_mode_of_string(config.lidar_mode),
    ouster::sensor::timestamp_mode_of_string(config.timestamp_mode),
    config.lidar_port,
    config.imu_port);

  if (!_ouster_client) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to create connection to lidar."));
  }

  _pf = &ouster::sensor::get_format(getMetadata());
  _lidar_packet.resize(_pf->lidar_packet_size + 1);
  _imu_packet.resize(_pf->imu_packet_size + 1);
}

ouster::sensor::client_state OS1Sensor::get()
{
  const ouster::sensor::client_state state = ouster::sensor::poll_client(*_ouster_client);

  if (state & ouster::sensor::client_state::EXIT) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned exit!"));
  } else if (state & ouster::sensor::client_state::CLIENT_ERROR) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned error!"));
  }

  return state;
}

uint8_t * OS1Sensor::readPacket(const ouster::sensor::client_state & state)
{
  // todo : Manage case when we receive lidar + imu data
  if (state & ouster::sensor::client_state::LIDAR_DATA) {
    if (ouster::sensor::read_lidar_packet(*_ouster_client, _lidar_packet.data(), *_pf)) {
      return _lidar_packet.data();
    }
  } else if (state & ouster::sensor::client_state::IMU_DATA) {
    if (read_imu_packet(*_ouster_client, _imu_packet.data(), *_pf)) {
      return _imu_packet.data();
    }
  }
  return nullptr;
}

ouster::sensor::sensor_info OS1Sensor::getMetadata()
{
//  if (_ouster_client) {
    return ouster::sensor::parse_metadata(ouster::sensor::get_metadata(*_ouster_client));
//  } else {
//    return {"UNKNOWN", "UNKNOWN", "UNNKOWN", "UNNKOWN", "UNKNOWN",
//      {}, {}, {}, {}, 7503, 7502};
//  }
}

}  // namespace OS1
