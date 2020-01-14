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

namespace OS1
{

OS1Sensor::OS1Sensor()
: SensorInterface()
{
  _lidar_packet.resize(lidar_packet_bytes + 1);
  _imu_packet.resize(imu_packet_bytes + 1);
}

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
  if (!OS1::lidar_mode_of_string(config.lidar_mode)) {
    throw ros2_ouster::OusterDriverException(
            std::string("Invalid lidar mode %s!", config.lidar_mode.c_str()));
    exit(-1);
  }

  _ouster_client = OS1::init_client(
    config.lidar_ip, config.computer_ip,
    OS1::lidar_mode_of_string(config.lidar_mode),
    config.lidar_port, config.imu_port);

  if (!_ouster_client) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to create connection to lidar."));
  }
}

ros2_ouster::ClientState OS1Sensor::get()
{
  const ros2_ouster::ClientState state = OS1::poll_client(*_ouster_client);

  if (state == ros2_ouster::ClientState::EXIT) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to get valid sensor data "
            "information from lidar, returned exit!"));
  } else if (state == ros2_ouster::ClientState::ERROR) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to get valid sensor data "
            "information from lidar, returned error!"));
  }

  return state;
}

uint8_t * OS1Sensor::readPacket(const ros2_ouster::ClientState & state)
{
  switch (state) {
    case ros2_ouster::ClientState::LIDAR_DATA:
      if (read_lidar_packet(*_ouster_client, _lidar_packet.data())) {
        return _lidar_packet.data();
      } else {
        return nullptr;
      }
    case ros2_ouster::ClientState::IMU_DATA:
      if (read_imu_packet(*_ouster_client, _imu_packet.data())) {
        return _imu_packet.data();
      } else {
        return nullptr;
      }
  }
}

ros2_ouster::Metadata OS1Sensor::getMetadata()
{
  if (_ouster_client) {
    return OS1::parse_metadata(OS1::get_metadata(*_ouster_client));
  } else {
    return {"UNKNOWN", "UNKNOWN", "UNNKOWN", "UNNKOWN",
      {}, {}, {}, {}, 7503, 7502};
  }
}

}  // namespace OS1
