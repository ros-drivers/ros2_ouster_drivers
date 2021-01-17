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

#include <cstdint>
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

#include "ros2_ouster/OS1/OS1_packet.hpp"
#include "ros2_ouster/client/client.h"

namespace ros2_ouster
{

/**
 * @brief Run-time big endian check
 *
 * Run-time initialize a constant determining if the underlying machine is big
 * endian. This is needed to properly stamp the `PointCloud2` ROS message
 * (below).
 */
static const bool IS_BIGENDIAN = []()
  {
    std::uint16_t dummy = 0x1;
    std::uint8_t * dummy_ptr = reinterpret_cast<std::uint8_t *>(&dummy);
    return dummy_ptr[0] == 0x1 ? false : true;
  } ();

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
  if (output.str() == "") {
    output << "unknown ";
  }

  return output.str();
}

/**
 * @brief Convert metadata to message format
 */
inline ouster_msgs::msg::Metadata toMsg(const ouster::sensor::sensor_info & mdata)
{
  ouster_msgs::msg::Metadata msg;
  msg.hostname = mdata.name;
  msg.lidar_mode = mdata.mode;
//  msg.timestamp_mode = mdata.timestamp_mode; //todo: Find how to get timestamp_mode
  msg.beam_azimuth_angles = mdata.beam_azimuth_angles;
  msg.beam_altitude_angles = mdata.beam_altitude_angles;
  msg.imu_to_sensor_transform = std::vector<double>(mdata.imu_to_sensor_transform.data(), mdata.imu_to_sensor_transform.data() + mdata.imu_to_sensor_transform.rows() * mdata.imu_to_sensor_transform.cols());
  msg.lidar_to_sensor_transform = std::vector<double>(mdata.lidar_to_sensor_transform.data(), mdata.lidar_to_sensor_transform.data() + mdata.lidar_to_sensor_transform.rows() * mdata.lidar_to_sensor_transform.cols());
  msg.serial_no = mdata.sn;
  msg.firmware_rev = mdata.fw_rev;
//  msg.imu_port = mdata.imu_port; //todo: Find how to get
//  msg.lidar_port = mdata.lidar_port; //todo: Find how to get
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

  tf.setOrigin({*(mat.data()+3) / 1e3, *(mat.data()+7) / 1e3, *(mat.data()+11) / 1e3});
  tf.setBasis({
    *(mat.data()+0), *(mat.data()+1), *(mat.data()+2), *(mat.data()+4), *(mat.data()+5),
    *(mat.data()+6), *(mat.data()+8), *(mat.data()+9), *(mat.data()+10)
  });

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
  const std::string & frame,
  uint64_t override_ts = 0)
{
  const double standard_g = 9.80665;
  sensor_msgs::msg::Imu m;
  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 1;

  m.header.stamp = override_ts == 0 ?
    rclcpp::Time(OS1::imu_gyro_ts(buf)) : rclcpp::Time(override_ts);
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
  const pcl::PointCloud<point_os::PointOS> & cloud,
  std::chrono::nanoseconds timestamp,
  const std::string & frame)
{
  std::size_t pt_size = sizeof(point_os::PointOS);
  std::size_t data_size = pt_size * cloud.points.size();

  pcl::PCLPointCloud2 cloud2;
  cloud2.height = cloud.height;
  cloud2.width = cloud.width;
  cloud2.fields.clear();
  pcl::for_each_type<typename pcl::traits::fieldList<point_os::PointOS>::type>(
    pcl::detail::FieldAdder<point_os::PointOS>(cloud2.fields));
  cloud2.header = cloud.header;
  cloud2.point_step = pt_size;
  cloud2.row_step = static_cast<std::uint32_t>(pt_size * cloud2.width);
  cloud2.is_dense = cloud.is_dense;
  cloud2.is_bigendian = ros2_ouster::IS_BIGENDIAN;

  cloud2.data.resize(data_size);
  if (data_size) {
    // column-major to row-major conversion
    for (int i = 0; i < cloud.width; ++i) {
      for (int j = 0; j < cloud.height; ++j) {
        std::memcpy(
          &cloud2.data[(j * cloud.width + i) * pt_size],
          &cloud.points[i * cloud.height + j],
          pt_size);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl_conversions::moveFromPCL(cloud2, msg);
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
  const ouster::sensor::sensor_info & mdata,
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
  if (mdata.mode == ouster::sensor::lidar_mode::MODE_512x10) {
    resolution = 512.0;
    rate = 10.0;
  } else if (mdata.mode == ouster::sensor::lidar_mode::MODE_512x20) {
    resolution = 512.0;
    rate = 20.0;
  } else if (mdata.mode == ouster::sensor::lidar_mode::MODE_1024x10) {
    resolution = 1024.0;
    rate = 10.0;
  } else if (mdata.mode == ouster::sensor::lidar_mode::MODE_1024x20) {
    resolution = 1024.0;
    rate = 20.0;
  } else if (mdata.mode == ouster::sensor::lidar_mode::MODE_2048x10) {
    resolution = 2048.0;
    rate = 10.0;
  } else {
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
