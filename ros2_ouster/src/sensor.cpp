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

  //TODO(force-reinit): currently always forcing reinit, rethink this later?
  reactivate_sensor(true);

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
  if (config.computer_ip.empty()) {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to computer using automatic address detection");
  }  else {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to %s.", config.computer_ip.c_str());
  }

  _ouster_client = configure_and_initialize_sensor(config);

  if (!_ouster_client) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to create connection to lidar."));
  }

  setMetadata(config.lidar_port, config.imu_port, config.timestamp_mode);
}

bool Sensor::shouldReset(const ouster::sensor::client_state & state, const uint8_t * packet)
{
  return (state == ouster::sensor::client_state::LIDAR_DATA) &&
         is_non_legacy_lidar_profile(getMetadata()) &&
         init_id_changed(getPacketFormat(), packet);
}

std::shared_ptr<ouster::sensor::client> Sensor::configure_and_initialize_sensor(
        const ros2_ouster::Configuration &config)
{
  ouster::sensor::sensor_config sensor_config{};
  sensor_config.udp_dest = config.computer_ip;
  sensor_config.udp_port_imu = config.imu_port;
  sensor_config.udp_port_lidar = config.lidar_port;
  sensor_config.ld_mode = ouster::sensor::lidar_mode_of_string(config.lidar_mode);
  sensor_config.ts_mode = ouster::sensor::timestamp_mode_of_string(config.timestamp_mode);
  sensor_config.udp_profile_lidar =
          ouster::sensor::udp_profile_lidar_of_string(config.lidar_udp_profile);

  uint8_t config_flags = compose_config_flags(sensor_config);
  if (!set_config(config.lidar_ip, sensor_config, config_flags)) {
    throw std::runtime_error("Error connecting to sensor " + config.lidar_ip);
  }

  std::cout << "Sensor " << config.lidar_ip
            << " configured successfully, initializing client" << std::endl;

  _ouster_client = ouster::sensor::init_client(
          config.lidar_ip,
          config.computer_ip,
          ouster::sensor::lidar_mode_of_string(config.lidar_mode),
          ouster::sensor::timestamp_mode_of_string(config.timestamp_mode),
          config.lidar_port,
          config.imu_port);
  return ouster::sensor::init_client(config.lidar_ip, sensor_config.udp_dest.value(),
                          sensor_config.ld_mode.value(),
                          sensor_config.ts_mode.value(),
                          sensor_config.udp_port_lidar.value(),
                          sensor_config.udp_port_imu.value());
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
  ros2_ouster::populate_missing_metadata_defaults(_metadata, ouster::sensor::MODE_UNSPEC);
}

ros2_ouster::Metadata Sensor::getMetadata()
{
  return _metadata;
}

ouster::sensor::packet_format Sensor::getPacketFormat()
{
  return ouster::sensor::get_format(getMetadata());
}

uint8_t Sensor::compose_config_flags(const ouster::sensor::sensor_config &config)
{
  uint8_t config_flags = 0;
  if (config.udp_dest) {
    std::cout << "Will send UDP data to " << config.udp_dest.value()
              << std::endl;
  }
  else {
    std::cout << "Will use automatic UDP destination" << std::endl;
    config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
  }

  if (force_sensor_reinit) {
    force_sensor_reinit = false;
    std::cout << "Forcing sensor to reinitialize" << std::endl;
    config_flags |= ouster::sensor::CONFIG_FORCE_REINIT;
  }

  return config_flags;
}

// param init_id_reset is overriden to true when force_reinit is true
void Sensor::reset_sensor(bool force_reinit, bool init_id_reset) {
//  if (!sensor_connection_active) {
//    RCLCPP_WARN(get_logger(),
//                "sensor reset is invoked but sensor connection is not "
//                "active, ignoring call!");
//    return;
//  }

  force_sensor_reinit = force_reinit;
  reset_last_init_id = force_reinit || init_id_reset;
//  auto request_transitions = std::vector<uint8_t>{
//          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
//          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
//          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
//          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
//  execute_transitions_sequence(request_transitions, 0);
}

// TODO: need to notify dependent node(s) of the update
void Sensor::reactivate_sensor(bool init_id_reset) {
//  if (!sensor_connection_active) {
//     This may indicate that we are in the process of re-activation
//    RCLCPP_WARN(get_logger(),
//                "sensor reactivate is invoked but sensor connection is "
//                "not active, ignoring call!");
//    return;
//  }

  reset_last_init_id = init_id_reset;
//  update_config_and_metadata();
//  publish_metadata();
//  save_metadata();
//  auto request_transitions = std::vector<uint8_t>{
//          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
//          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
//  execute_transitions_sequence(request_transitions, 0);
}

}  // namespace sensor
