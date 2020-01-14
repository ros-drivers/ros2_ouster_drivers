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

#ifndef ROS2_OUSTER__INTERFACES__METADATA_HPP_
#define ROS2_OUSTER__INTERFACES__METADATA_HPP_

#include <vector>
#include <string>

namespace ros2_ouster
{

/**
 * @brief client response on current state
 */
enum ClientState
{
  TIMEOUT = 0,
  ERROR = 1,
  LIDAR_DATA = 2,
  IMU_DATA = 4,
  EXIT = 8
};

/**
 * @brief metadata about Ouster lidar sensor
 */
struct Metadata
{
  std::string hostname;
  std::string sn;
  std::string fw_rev;
  std::string mode;
  std::vector<double> beam_azimuth_angles;
  std::vector<double> beam_altitude_angles;
  std::vector<double> imu_to_sensor_transform;
  std::vector<double> lidar_to_sensor_transform;
  int imu_port;
  int lidar_port;
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__METADATA_HPP_
