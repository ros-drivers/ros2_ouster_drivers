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

#ifndef ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
#define ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/qos.hpp"

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/client/lidar_scan.h"
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/client/ouster_ros/point.h"

using Cloud = pcl::PointCloud<ouster_ros::Point>;

namespace OS1
{
/**
 * @class OS1::PointcloudProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Pointclouds in the
 * driver in ROS2.
 */
class PointcloudProcessor : public ros2_ouster::DataProcessorInterface
{
public:
  /**
   * @brief A constructor for OS1::PointcloudProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  PointcloudProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ouster::sensor::sensor_info & mdata,
    const std::string & frame,
    const rclcpp::QoS & qos,
    const ouster::sensor::packet_format& pf)
  : DataProcessorInterface(), _node(node), _frame(frame), _pf(pf)
  {
    _height = mdata.format.pixels_per_column;
    _width = mdata.format.columns_per_frame;
    _batch = new ouster::ScanBatcher(_width, _pf);
    _xyz_lut = ouster::make_xyz_lut(mdata);
    _ls = ouster::LidarScan{_width, _height};
    _cloud = new Cloud{_width, _height};
    _pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points", qos);
  }

  /**
   * Populate a PCL point cloud from a LidarScan
   * @param xyz_lut lookup table from sensor beam angles (see lidar_scan.h)
   * @param scan_ts scan start used to caluclate relative timestamps for points
   * @param ls input lidar data
   * @param cloud output pcl pointcloud to populate
   */
  static void scan_to_cloud(const ouster::XYZLut& xyz_lut,
                     ouster::LidarScan::ts_t scan_ts,
                     const ouster::LidarScan& ls, Cloud& cloud) {
    cloud.resize(ls.w * ls.h);
    auto points = ouster::cartesian(ls, xyz_lut);

    for (auto u = 0; u < ls.h; u++) {
      for (auto v = 0; v < ls.w; v++) {
        const auto xyz = points.row(u * ls.w + v);
        const auto pix = ls.data.row(u * ls.w + v);
        const auto ts = (ls.header(v).timestamp - scan_ts).count();
        cloud(v, u) = ouster_ros::Point{
            {{static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
              static_cast<float>(xyz(2)), 1.0f}},
            static_cast<float>(pix(ouster::LidarScan::INTENSITY)),
            static_cast<uint32_t>(ts),
            static_cast<uint16_t>(pix(ouster::LidarScan::REFLECTIVITY)),
            static_cast<uint8_t>(u),
            static_cast<uint16_t>(pix(ouster::LidarScan::AMBIENT)),
            static_cast<uint32_t>(pix(ouster::LidarScan::REFLECTIVITY))};
      }
    }
  }

  void lidar_handler(const uint8_t* data) {
    if (_batch->operator()(data, _ls)) {
      auto h = std::find_if(
          _ls.headers.begin(), _ls.headers.end(), [](const auto& h) {
            return h.timestamp != std::chrono::nanoseconds{0};
          });
      if (h != _ls.headers.end()) {
        scan_to_cloud(_xyz_lut, h->timestamp, _ls, *_cloud);
        _pub->publish(ros2_ouster::toMsg(*_cloud, h->timestamp, _frame));
      }
    }
  };

  /**
   * @brief A destructor clearing memory allocated
   */
  ~PointcloudProcessor()
  {
    _pub.reset();
    delete(_batch);
    delete(_cloud);
  }

  /**
   * @brief Process method to create pointcloud
   * @param data the packet data
   */
  bool process(uint8_t * data, uint64_t override_ts) override
  {
    lidar_handler(data);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _pub->on_deactivate();
  }

private:
  ouster::ScanBatcher* _batch;
  ouster::LidarScan _ls;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
  Cloud* _cloud;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  ouster::XYZLut _xyz_lut;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
  const ouster::sensor::packet_format& _pf;
};

}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
