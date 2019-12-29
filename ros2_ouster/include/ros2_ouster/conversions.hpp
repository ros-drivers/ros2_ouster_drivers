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

#ifndef ROS2_OUSTER__CONVERSIONS_HPP_
#define ROS2_OUSTER__CONVERSIONS_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ouster_msgs/msg/metadata.hpp"
#include "ros2_ouster/OS1/OS1.hpp"

namespace ros2_ouster
{
/**
 * @brief A sensor interface constructor
 */
inline ouster_msgs::msg::Metadata toMsg(const ros2_ouster::Metadata & mdata)
{
  ouster_msgs::msg::Metadata msg;
  msg.hostname = mdata.hostname;
  msg.lidar_mode = mdata.mode;
  msg.beam_azimuth_angles = mdata.beam_azimuth_angles;
  msg.beam_altitude_angles = mdata.beam_altitude_angles;
  msg.imu_to_sensor_transform = mdata.imu_to_sensor_transform;
  msg.lidar_to_sensor_transform = mdata.lidar_to_sensor_transform;
  msg.serial_no = mdata.sn;
  msg.firmware_rev = mdata.fw_rev;
  msg.imu_port = mdata.imu_port;
  msg.lidar_port = mdata.lidar_port;
  return msg;
};

inline geometry_msgs::msg::TransformStamped toMsg(
    const std::vector<double> & mat, const std::string & frame,
    const std::string & child_frame, const rclcpp::Time & time)
{
  assert(mat.size() == 16);

  tf2::Transform tf;

  tf.setOrigin({mat[3] / 1e3, mat[7] / 1e3, mat[11] / 1e3});
  tf.setBasis({mat[0], mat[1], mat[2], mat[4], mat[5],
    mat[6], mat[8], mat[9], mat[10]});

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame;
  msg.child_frame_id = child_frame;
  msg.transform = tf2::toMsg(tf);

  return msg;
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__CONVERSIONS_HPP_
