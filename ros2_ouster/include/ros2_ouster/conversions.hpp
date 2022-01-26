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

#ifndef ROS2_OUSTER__CONVERSIONS_HPP_
#define ROS2_OUSTER__CONVERSIONS_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros2_ouster/client/point.h"
#include "ros2_ouster/interfaces/metadata.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ouster_msgs/msg/metadata.hpp"

#include "ros2_ouster/client/client.h"

namespace ros2_ouster
{
/**
 * @brief Convert ClientState to string
 */
inline std::string toString(const ouster::sensor::client_state & state)
{
  std::ostringstream output;

  if (state & ouster::sensor::client_state::TIMEOUT) {
    output << "timeout ";
  }
  if (state & ouster::sensor::client_state::CLIENT_ERROR) {
    output << "error ";
  }
  if (state & ouster::sensor::client_state::EXIT) {
    output << "exit ";
  }
  if (state & ouster::sensor::client_state::IMU_DATA) {
    output << "imu data ";
  }
  if (state & ouster::sensor::client_state::LIDAR_DATA) {
    output << "lidar data ";
  }
  if (output.str().empty()) {
    output << "unknown ";
  }

  return output.str();
}

/**
 * @brief Converts a 4dMatrix into a Vector
 */
static std::vector<double> toVector(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> & mat)
{
  return std::vector<double>(mat.data(), mat.data() + mat.rows() * mat.cols());
}

/**
 * @brief Convert metadata to message format
 */
inline ouster_msgs::msg::Metadata toMsg(const ros2_ouster::Metadata & mdata)
{
  ouster_msgs::msg::Metadata msg;
  msg.hostname = mdata.name;
  msg.lidar_mode = ouster::sensor::to_string(mdata.mode);
  msg.timestamp_mode = mdata.timestamp_mode;
  msg.beam_azimuth_angles = mdata.beam_azimuth_angles;
  msg.beam_altitude_angles = mdata.beam_altitude_angles;
  msg.imu_to_sensor_transform = ros2_ouster::toVector(mdata.imu_to_sensor_transform);
  msg.lidar_to_sensor_transform = ros2_ouster::toVector(mdata.lidar_to_sensor_transform);
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
  const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> & mat, const std::string & frame,
  const std::string & child_frame, const rclcpp::Time & time)
{
  assert(mat.size() == 16);

  tf2::Transform tf;

  tf.setOrigin({mat(3) / 1e3, mat(7) / 1e3, mat(11) / 1e3});
  tf.setBasis(
    {
      mat(0), mat(1), mat(2),
      mat(4), mat(5), mat(6),
      mat(8), mat(9), mat(10)
    });

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame;
  msg.child_frame_id = child_frame;
  msg.transform = tf2::toMsg(tf);

  return msg;
}

/**
 * @brief Parse an imu packet message into a ROS imu message
 * @param pm packet message populated by read_imu_packet
 * @param frame the frame to set in the resulting ROS message
 * @return ROS sensor message with fields populated from the packet
 */
inline sensor_msgs::msg::Imu toMsg(
  const uint8_t * buf,
  const std::string & frame,
  const ouster::sensor::packet_format & pf,
  const uint64_t override_ts = 0)
{
  const double standard_g = 9.80665;
  sensor_msgs::msg::Imu m;
  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 1;

  m.header.stamp = override_ts == 0 ? rclcpp::Time(pf.imu_gyro_ts(buf)) : rclcpp::Time(override_ts);
  m.header.frame_id = frame;

  m.linear_acceleration.x = pf.imu_la_x(buf) * standard_g;
  m.linear_acceleration.y = pf.imu_la_y(buf) * standard_g;
  m.linear_acceleration.z = pf.imu_la_z(buf) * standard_g;

  m.angular_velocity.x = pf.imu_av_x(buf) * M_PI / 180.0;
  m.angular_velocity.y = pf.imu_av_y(buf) * M_PI / 180.0;
  m.angular_velocity.z = pf.imu_av_z(buf) * M_PI / 180.0;

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
 * @brief Convert Pointcloud to ROS message format
 *
 * This function assumes the input `cloud` has its data in column major order
 * but shaped improperly according to the LiDAR resulting in tiled columns
 * across the rows of the in-memory data buffer. This data format is described
 * more closely here:
 * https://github.com/SteveMacenski/ros2_ouster_drivers/issues/52
 *
 * This function converts the PCL type to a ROS type and the resulting ROS type
 * will reorder the data in memory to be in row-major order consistent to the
 * rows x cols shape of the LiDAR receiver array.
 *
 * @param[in] cloud A PCL PointCloud containing Ouster point data as described
 *                  above.
 * @param[in] timestamp The timestamp to put on the ROS message header
 * @param[in] frame The TF coordinate frame identifier to put on the ROS
 *                  message header.
 *
 * @return A ROS `PointCloud2` message of LiDAR data whose memory buffer is
 *         row-major ordered consistent to the shape of the LiDAR array.
 */
inline sensor_msgs::msg::PointCloud2 toMsg(
  const pcl::PointCloud<ouster_ros::Point> & cloud,
  const std::chrono::nanoseconds timestamp,
  const std::string & frame,
  const uint64_t override_ts)
{
  sensor_msgs::msg::PointCloud2 msg{};
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = override_ts == 0 ? rclcpp::Time(timestamp.count()) : rclcpp::Time(override_ts);
  return msg;
}

/**
 * @brief Convert cloud to scan message format
 */
inline sensor_msgs::msg::LaserScan toMsg(
  const ouster::LidarScan ls,
  const std::chrono::nanoseconds timestamp,
  const std::string & frame,
  const ouster::sensor::sensor_info & mdata,
  const uint8_t ring_to_use,
  const uint64_t override_ts)
{
  sensor_msgs::msg::LaserScan msg;
  rclcpp::Time t(timestamp.count());
  msg.header.stamp = override_ts == 0 ? t : rclcpp::Time(override_ts);
  msg.header.frame_id = frame;
  msg.angle_min = -M_PI;
  msg.angle_max = M_PI;
  msg.range_min = 0.1;
  msg.range_max = 120.0;

  msg.scan_time = 1.0 / ouster::sensor::frequency_of_lidar_mode(mdata.mode);
  msg.time_increment = 1.0 / ouster::sensor::frequency_of_lidar_mode(mdata.mode) /
    ouster::sensor::n_cols_of_lidar_mode(mdata.mode);
  msg.angle_increment = 2 * M_PI / ouster::sensor::n_cols_of_lidar_mode(mdata.mode);

  // Fix #90 (PR #92) - The iterator is in the loop condition to prevent overflow by
  // decrementing unsigned variable 'i' bellow 0. This happened when ring 0 was selected
  // due to the condition being reduced to i >= 0
  for (size_t i = ls.w * ring_to_use + ls.w; i-- > ls.w * ring_to_use;) {
    msg.ranges.push_back(
      static_cast<float>((ls.field(ouster::LidarScan::RANGE)(i) * ouster::sensor::range_unit))
    );
    msg.intensities.push_back(
      static_cast<float>((ls.field(ouster::LidarScan::INTENSITY)(i)))
    );
  }

  return msg;
}

/**
* Populate a PCL point cloud from a LidarScan
* @param xyz_lut lookup table from sensor beam angles (see lidar_scan.h)
* @param scan_ts scan start used to caluclate relative timestamps for points
* @param ls input lidar data
* @param cloud output pcl pointcloud to populate
*/
static void toCloud(
  const ouster::XYZLut & xyz_lut,
  ouster::LidarScan::ts_t scan_ts,
  const ouster::LidarScan & ls, pcl::PointCloud<ouster_ros::Point> & cloud)
{
  cloud.resize(ls.w * ls.h);
  auto points = ouster::cartesian(ls, xyz_lut);

  for (auto u = 0; u < ls.h; u++) {
    for (auto v = 0; v < ls.w; v++) {
      const auto xyz = points.row(u * ls.w + v);
      const auto pix = ls.data.row(u * ls.w + v);
      const auto ts = (ls.header(v).timestamp - scan_ts).count();
      cloud(v, u) = ouster_ros::Point{
        {{static_cast<float>(xyz(0)),
          static_cast<float>(xyz(1)),
          static_cast<float>(xyz(2)), 1.0f}},
        static_cast<float>(pix(ouster::LidarScan::INTENSITY)),
        static_cast<uint32_t>(ts),
        static_cast<uint16_t>(pix(ouster::LidarScan::REFLECTIVITY)),
        static_cast<uint8_t>(u),
        static_cast<uint16_t>(pix(ouster::LidarScan::AMBIENT)),
        static_cast<uint32_t>(pix(ouster::LidarScan::RANGE))};
    }
  }
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__CONVERSIONS_HPP_
