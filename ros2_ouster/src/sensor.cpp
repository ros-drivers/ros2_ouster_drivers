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

#include <string>
#include <fstream>
#include <sstream>
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/sensor.hpp"

namespace sensor
{

Sensor::Sensor()
: SensorInterface() {}

Sensor::~Sensor()
{
  _ouster_client.reset();
}

void Sensor::reset(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  _ouster_client.reset();
  configure(config, node);
}

void Sensor::configure(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(
    node->get_logger(), 
    "Configuring Ouster driver node.");

  // Check the validity of some of the retrieved parameters
  if (!ouster::sensor::lidar_mode_of_string(config.lidar_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invalid lidar mode: " + config.lidar_mode);
    exit(-1);
  }

  if (!ouster::sensor::timestamp_mode_of_string(config.timestamp_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invalid timestamp mode: " + config.timestamp_mode);
    exit(-1);
  }

  // Report to the user whether automatic address detection is being used, and 
  // what the source / destination IPs are
  RCLCPP_INFO(
    node->get_logger(),
    "Connecting to sensor at %s.", config.lidar_ip.c_str());
  if (config.computer_ip == "") {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to computer using automatic address detection");
  }  else {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to %s.", config.computer_ip.c_str());
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

  setMetadata(config.lidar_port, config.imu_port, config.timestamp_mode);
}

ouster::sensor::client_state Sensor::get()
{
  const ouster::sensor::client_state state = ouster::sensor::poll_client(*_ouster_client);

  if (state == ouster::sensor::client_state::EXIT) {
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

bool Sensor::readLidarPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state & ouster::sensor::client_state::LIDAR_DATA &&
    ouster::sensor::read_lidar_packet(
      *_ouster_client, buf,
      this->getPacketFormat()))
  {
    return true;
  }
  return false;
}

bool Sensor::readImuPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state & ouster::sensor::client_state::IMU_DATA &&
    ouster::sensor::read_imu_packet(
      *_ouster_client, buf,
      this->getPacketFormat()))
  {
    return true;
  }
  return false;
}

void Sensor::setMetadata(
  int lidar_port, int imu_port,
  const std::string & timestamp_mode)
{
  if (_ouster_client) {
    _metadata = ros2_ouster::Metadata(
      ouster::sensor::parse_metadata(
        ouster::sensor::get_metadata(*_ouster_client)),
      imu_port, lidar_port, timestamp_mode);
  }
  ros2_ouster::populate_missing_metadata_defaults(_metadata);
}

ros2_ouster::Metadata Sensor::getMetadata()
{
  return _metadata;
}

ouster::sensor::packet_format Sensor::getPacketFormat()
{
  return ouster::sensor::get_format(getMetadata());
}

}  // namespace sensor
