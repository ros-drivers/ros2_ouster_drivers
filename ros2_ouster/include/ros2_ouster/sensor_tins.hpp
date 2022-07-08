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

// Libtins includes
#include <tins/packet.h>
#include <tins/rawpdu.h>
#include <tins/sniffer.h>
#include <tins/tins.h>
#include <tins/udp.h>
#include <tins/ip_reassembler.h>

#include <memory>
#include <vector>
#include <string>
#include "ros2_ouster/processors/processor_factories.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/ros2_utils.hpp"

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
   * @param node pointer to the driver node, which provides access to ROS params
   */
  void reset(
    ros2_ouster::Configuration & config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   * @param node pointer to the driver node, which provides access to ROS params
   */
  void configure(
    ros2_ouster::Configuration & config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

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
   * @param buf pointer to a buffer to hold the packet data. Must hold getPacketFormat().lidar_packet_size bytes.
   * @return true if a packet was recieved, false otherwise
   */
  bool readLidarPacket(
    const ouster::sensor::client_state & state, uint8_t * buf) override;

  /**
   * @brief reading an imu packet
   * @param state of the sensor
   * @param buf pointer to a buffer to hold the packet data. Must hold getPacketFormat().imu_packet_size bytes.
   * @return true if a packet was recieved, false otherwise
   */
  bool readImuPacket(
    const ouster::sensor::client_state & state, uint8_t * buf) override;

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
   * @param[in] filepath_to_read A fully qualified filepath to a yaml file
   *            containing the parameters to load
   * @param[out] sensor_info The metadata to load data into.
   */
  void loadSensorInfoFromJsonFile(
    const std::string filepath_to_read,
    ouster::sensor::sensor_info & sensor_info);

  /**
   * @brief Initializes the internal Tins::Sniffer object.
   * @param[in] eth_device The name of the ethernet device for the sniffer to
   *            listen to. This is typically a device like "eth0", "eth1" or
   *            "eno0"
   */
  void initializeSniffer(const std::string eth_device);

  /**
   * @brief sniff for one packet and one packet only. If a valid LiDAR or IMU
   *        packet is found, it will set the internal _replay_state to
   *        client_state::LIDAR_DATA or client_state::IMU_DATA
   * @details This function is intended to only be used as a callback for the
   *          sniffer->sniff_loop function
   * @param[in] packet The input packet from the tins sniffer to parse.
   * @returns False if the packet is an Ouster LiDAR or IMU packet, in which
   *          case we don't want to continue sniffing, and want sniff_loop to
   *          stop. True if the packets aren't good and we need to continue
   *          looking.
   */
  bool sniffOnePacket(Tins::Packet & packet);

private:
  /**
   * the client is here because it needs to be, but is not actually used
   * in this implementation of the SensorInterface class.
   */
  std::shared_ptr<ouster::sensor::client> _ouster_client;

  /** Internal storage for LiDAR data */
  std::vector<uint8_t> _lidar_packet;

  /** Internal storage for IMU data */
  std::vector<uint8_t> _imu_packet;

  /** Metadata for the sensor */
  ros2_ouster::Metadata _metadata{};

  /** Driver configuration */
  ros2_ouster::Configuration _driver_config;

  /**
   * The inferred state of the LiDAR, determined by the most recently
   * received packet instead of requested from the real LiDAR
   */
  ouster::sensor::client_state _inferred_state;

  /** Tins sniffer object for listening to packets */
  std::unique_ptr<Tins::Sniffer> _tins_sniffer_pointer;

  /** Tins reassembler for reconstructing fragmented packets */
  Tins::IPv4Reassembler _tins_ipv4_reassembler;

  /** Configuration for _tins_sniffer_pointer */
  Tins::SnifferConfiguration _sniffer_config;
};

}  // namespace sensor

#endif  // ROS2_OUSTER__SENSOR_TINS_HPP_
