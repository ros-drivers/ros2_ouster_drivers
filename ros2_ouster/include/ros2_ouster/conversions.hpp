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

#ifndef ROS2_OUSTER__CONVERSIONS_HPP_
#define ROS2_OUSTER__CONVERSIONS_HPP_

#include <string>
#include <vector>
#include <algorithm>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros2_ouster/point_os.hpp"
#include "ros2_ouster/image_os.hpp"
#include "ros2_ouster/scan_os.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ouster_msgs/msg/metadata.hpp"

#include "ros2_ouster/OS1/OS1.hpp"
#include "ros2_ouster/OS1/OS1_packet.hpp"

namespace ros2_ouster
{

/**
 * @brief Convert ClientState to string
 */
inline std::string toString(const ClientState & state)
{
  switch (state) {
    case TIMEOUT:
      return std::string("timeout");
    case ERROR:
      return std::string("error");
    case EXIT:
      return std::string("exit");
    case IMU_DATA:
      return std::string("lidar data");
    case LIDAR_DATA:
      return std::string("imu data");
    default:
      return std::string("unknown");
  }
}

/**
 * @brief Convert metadata to message format
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
}

/**
 * @brief Convert transformation to message format
 */
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

/**
 * @brief Convert IMU to message format
 */
inline sensor_msgs::msg::Imu toMsg(
  const uint8_t * buf,
  const std::string & frame)
{
  const double standard_g = 9.80665;
  sensor_msgs::msg::Imu m;
  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 1;

  rclcpp::Time t(OS1::imu_gyro_ts(buf));
  m.header.stamp = t;
  m.header.frame_id = frame;

  m.linear_acceleration.x = OS1::imu_la_x(buf) * standard_g;
  m.linear_acceleration.y = OS1::imu_la_y(buf) * standard_g;
  m.linear_acceleration.z = OS1::imu_la_z(buf) * standard_g;

  m.angular_velocity.x = OS1::imu_av_x(buf) * M_PI / 180.0;
  m.angular_velocity.y = OS1::imu_av_y(buf) * M_PI / 180.0;
  m.angular_velocity.z = OS1::imu_av_z(buf) * M_PI / 180.0;

  for (int i = 0; i < 9; i++) {
    m.orientation_covariance[i] = -1;
    m.angular_velocity_covariance[i] = 0;
    m.linear_acceleration_covariance[i] = 0;
  }
  for (int i = 0; i < 9; i += 4) {
    m.linear_acceleration_covariance[i] = 0.01;
    m.angular_velocity_covariance[i] = 6e-4;
  }

  return m;
}

/**
 * @brief Convert Pointcloud to message format
 */
inline sensor_msgs::msg::PointCloud2 toMsg(
  const pcl::PointCloud<point_os::PointOS> & cloud,
  std::chrono::nanoseconds timestamp,
  const std::string & frame)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame;
  rclcpp::Time t(timestamp.count());
  msg.header.stamp = t;
  return msg;
}

/**
 * @brief Convert Scan to message format
 */
inline sensor_msgs::msg::LaserScan toMsg(
  const std::vector<scan_os::ScanOS> & scans,
  std::chrono::nanoseconds timestamp,
  const std::string & frame,
  const ros2_ouster::Metadata & mdata,
  const uint8_t ring_to_use)
{
  sensor_msgs::msg::LaserScan msg;
  rclcpp::Time t(timestamp.count());
  msg.header.stamp = t;
  msg.header.frame_id = frame;
  msg.angle_min = -M_PI;
  msg.angle_max = M_PI;
  msg.range_min = 0.1;
  msg.range_max = 120.0;

  double resolution, rate;
  if (mdata.mode == "512x10") {
    resolution = 512.0;
    rate = 10.0;
  } else if (mdata.mode == "512x20") {
    resolution = 512.0;
    rate = 20.0;
  } else if (mdata.mode == "1024x10") {
    resolution = 1024.0;
    rate = 10.0;
  } else if (mdata.mode == "1024x20") {
    resolution = 1024.0;
    rate = 20.0;
  } else if (mdata.mode == "2048x10") {
    resolution = 2048.0;
    rate = 10.0;
  } else {
    std::cout << "Error: Could not determine lidar mode!" << std::endl;
    resolution = 512.0;
    rate = 10.0;
  }

  msg.scan_time = 1.0 / rate;
  msg.time_increment = 1.0 / rate / resolution;
  msg.angle_increment = 2 * M_PI / resolution;

  for (uint i = 0; i != scans.size(); i++) {
    if (scans[i].ring == ring_to_use) {
      msg.ranges.push_back(scans[i].range * 5e-3);
      msg.intensities.push_back(std::min(scans[i].intensity, 255.0f));
    }
  }

  return msg;
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__CONVERSIONS_HPP_
