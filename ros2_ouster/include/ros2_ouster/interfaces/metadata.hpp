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

#ifndef ROS2_OUSTER__INTERFACES__METADATA_HPP_
#define ROS2_OUSTER__INTERFACES__METADATA_HPP_

#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "ros2_ouster/client/types.h"
#include "ros2_ouster/client/version.h"
#include "ros2_ouster/client/client.h"

namespace ros2_ouster
{
/**
 * @brief metadata about Ouster lidar sensor, inherited from client sensor_info.
 */
struct Metadata : ouster::sensor::sensor_info
{
  Metadata()
  {
    name = "UNKNOWN";
    sn = "UNKNOWN";
    fw_rev = "UNKNOWN";
    mode = ouster::sensor::lidar_mode::MODE_UNSPEC;
    prod_line = "UNKNOWN";
    format = {};
    beam_azimuth_angles = {};
    beam_altitude_angles = {};
    lidar_origin_to_beam_origin_mm = 0.;
    beam_to_lidar_transform = {};
    imu_to_sensor_transform = {};
    lidar_to_sensor_transform = {};
    extrinsic = {};
    init_id = 0;
    udp_port_lidar = 0;
    udp_port_imu = 0;
    timestamp_mode = "UNKNOWN";
  }
  Metadata(
    const ouster::sensor::sensor_info & info, int _imu_port,
    int _lidar_port, std::string _timestamp_mode)
      : timestamp_mode(std::move(_timestamp_mode))
  {
    name = info.name;
    sn = info.sn;
    fw_rev = info.fw_rev;
    mode = info.mode;
    prod_line = info.prod_line;
    format = info.format;
    beam_azimuth_angles = info.beam_azimuth_angles;
    beam_altitude_angles = info.beam_altitude_angles;
    lidar_origin_to_beam_origin_mm = info.lidar_origin_to_beam_origin_mm;
    beam_to_lidar_transform = info.beam_to_lidar_transform;
    imu_to_sensor_transform = info.imu_to_sensor_transform;
    lidar_to_sensor_transform = info.lidar_to_sensor_transform;
    extrinsic = info.extrinsic;
    udp_port_lidar = _lidar_port;
    udp_port_imu = _imu_port;
  }

  std::string timestamp_mode;
};


/**
 * @brief fill in values that could not be parsed from metadata.
 */
inline void populate_missing_metadata_defaults(ouster::sensor::sensor_info & info,
                                               ouster::sensor::lidar_mode specified_lidar_mode)
{
  if (info.name.empty()) {
    info.name = "UNKNOWN";
  }

  if (info.sn.empty()) {
    info.sn = "UNKNOWN";
  }

  ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
  if (v == ouster::util::invalid_version) {
    std::cerr << "Unknown sensor firmware version; output may not be reliable"
              << std::endl;
  }
  else if (v < ouster::sensor::min_version) {
    std::cerr << "Firmware < " << to_string(ouster::sensor::min_version).c_str()
              << " not supported; output may not be reliable" << std::endl;
  }

  if (!info.mode) {
    std::cerr << "Lidar mode not found in metadata; output may not be reliable"
              << std::endl;
    info.mode = specified_lidar_mode;
  }

  if (info.prod_line.empty()) {
    info.prod_line = "UNKNOWN";
  }

  if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
    std::cerr << "Beam angles not found in metadata; using design values"
              << std::endl;
    info.beam_azimuth_angles = ouster::sensor::gen1_azimuth_angles;
    info.beam_altitude_angles = ouster::sensor::gen1_altitude_angles;
  }
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__METADATA_HPP_
