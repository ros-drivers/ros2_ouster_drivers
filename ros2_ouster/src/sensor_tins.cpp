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
#include <memory>
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/sensor_tins.hpp"


namespace sensor
{

SensorTins::SensorTins()
: SensorInterface() {}

SensorTins::~SensorTins()
{
  _ouster_client.reset();
  _lidar_packet.clear();
  _imu_packet.clear();
}

void SensorTins::reset(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  _ouster_client.reset();
  configure(config, node);
}

void SensorTins::configure(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(
    node->get_logger(), 
    "Configuring Tins-based Ouster driver node.");

  // Declare parameters specific to the SensorTins implementation
  ros2_ouster::declare_parameter_if_not_declared(
    node, "ethernet_device", rclcpp::ParameterValue("no_ethernet_device_configured"));
  ros2_ouster::declare_parameter_if_not_declared(
    node, "metadata_filepath", rclcpp::ParameterValue("no_filepath_specified"));

  // Get parameters specific to the SensorTins implementation
  try {
    config.ethernet_device = node->get_parameter("ethernet_device").as_string();
    config.metadata_filepath = node->get_parameter("metadata_filepath").as_string();
  } catch (...) {
    throw ros2_ouster::OusterDriverException(
            "TinsDriver failed to retrieve the ethernet_device or metadata_filepath parameter");
    exit(-1);
  }

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

  RCLCPP_INFO(
    node->get_logger(),
    "Looking for packets from sensor IPv4 address %s to destination %s.",
    config.lidar_ip.c_str(),
    config.computer_ip.c_str());

  // The driver config is saved internally b/c some parameters are needed for
  // the Tins sniffer
  _driver_config = config;

  // Read sensor metadata from file
  loadSensorInfoFromJsonFile(
    _driver_config.metadata_filepath,
    _metadata);

  // loadSensorInfoFromJsonFile actually returns a sensor_info object, so
  // fill in the params specific to the ros2_ouster::Metadata object, that
  // aren't normally supplied in the metadata file.
  _metadata.imu_port = _driver_config.imu_port;
  _metadata.lidar_port = _driver_config.lidar_port;
  _metadata.timestamp_mode = _driver_config.timestamp_mode;

  // Fill anything missing with defaults and resize the packet containers
  ros2_ouster::populate_missing_metadata_defaults(_metadata);
  _lidar_packet.resize(getPacketFormat().lidar_packet_size + 1);
  _imu_packet.resize(getPacketFormat().imu_packet_size + 1);

  // Create and initialize the Tins sniffer object
  try {
    initializeSniffer(_driver_config.ethernet_device);
  } catch (const std::exception & e) {
    throw ros2_ouster::OusterDriverException(
            "Failed to configure Tins sniffer. Check that the entered ethernet device " +
            _driver_config.ethernet_device + " is valid.");
    exit(-1);
  }
}

ouster::sensor::client_state SensorTins::get()
{
  // Start a sniff loop to look for a valid LiDAR or IMU packet
  _tins_sniffer_pointer->sniff_loop(
    std::bind(
      &SensorTins::sniffOnePacket,
      this,
      std::placeholders::_1));

  // Return the state, as best can be inferred from the data
  return _inferred_state;
}

bool SensorTins::readLidarPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state == ouster::sensor::client_state::LIDAR_DATA) {
    std::memcpy(buf, _lidar_packet.data(), this->getPacketFormat().lidar_packet_size);
    return true;
  } else {
    return false;
  }
}

bool SensorTins::readImuPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state == ouster::sensor::client_state::IMU_DATA) {
    std::memcpy(buf, _imu_packet.data(), this->getPacketFormat().imu_packet_size);
    return true;
  } else {
    return false;
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
  ouster::sensor::sensor_info & sensor_info)
{
  if (filepath_to_read == "") {
    throw ros2_ouster::OusterDriverException(
            "Metadata filepath is empty! The Tins driver needs a valid metadata file!");
    exit(-1);
  }

  try {
    sensor_info = ouster::sensor::metadata_from_json(filepath_to_read);
  } catch (const std::exception & e) {
    throw ros2_ouster::OusterDriverException(
            "Failed to read metadata from file: " + filepath_to_read +
            " with exception " + e.what());
    exit(-1);
  }
}

void SensorTins::initializeSniffer(const std::string eth_device)
{
  _sniffer_config.set_promisc_mode(true);
  _sniffer_config.set_immediate_mode(true);

  // Filter out all packets not from the sensor
  std::string filter_string = "ip src " + _driver_config.lidar_ip;
  _sniffer_config.set_filter(filter_string);

  _tins_sniffer_pointer = std::make_unique<Tins::Sniffer>(eth_device, _sniffer_config);
}

bool SensorTins::sniffOnePacket(Tins::Packet & packet)
{
  auto & pdu_to_process = *packet.pdu();

  // Reassemble the packet if it's fragmented
  if (_tins_ipv4_reassembler.process(pdu_to_process) !=
    Tins::IPv4Reassembler::FRAGMENTED)
  {
    // Reject the packet if it's not a IP>UDP>RawPDU packet
    const Tins::IP * ip = pdu_to_process.find_pdu<Tins::IP>();
    if (!ip) {
      return true;
    }
    const Tins::UDP * udp = pdu_to_process.find_pdu<Tins::UDP>();
    if (!udp) {
      return true;
    }
    const Tins::RawPDU * raw = pdu_to_process.find_pdu<Tins::RawPDU>();
    if (!raw) {
      return true;
    }

    const Tins::RawPDU::payload_type & payload = raw->payload();

    // If a LiDAR packet...
    if ((ip->dst_addr().to_string() == _driver_config.computer_ip) &&
      (payload.size() == getPacketFormat().lidar_packet_size) &&
      (udp->dport() == _driver_config.lidar_port))
    {
      _inferred_state = ouster::sensor::client_state::LIDAR_DATA;
      _lidar_packet = payload;
      return false;
    }

    // If an IMU packet...
    if ((ip->dst_addr().to_string() == _driver_config.computer_ip) &&
      (payload.size() == getPacketFormat().imu_packet_size) &&
      (udp->dport() == _driver_config.imu_port))
    {
      _inferred_state = ouster::sensor::client_state::IMU_DATA;
      _imu_packet = payload;
      return false;
    }
  }

  // The packet is valid but from neither the IMU or LiDAR. Return true to
  // keep sniffing.
  return true;
}

}  // namespace sensor
