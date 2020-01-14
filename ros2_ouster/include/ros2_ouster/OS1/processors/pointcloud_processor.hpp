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

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/OS1/OS1.hpp"
#include "ros2_ouster/OS1/OS1_util.hpp"

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
    const ros2_ouster::Metadata & mdata,
    const std::string & frame)
  : DataProcessorInterface(), _node(node), _frame(frame)
  {
    _height = OS1::pixels_per_column;
    _width = OS1::n_cols_of_lidar_mode(
      OS1::lidar_mode_of_string(mdata.mode));
    _xyz_lut = OS1::make_xyz_lut(_width, _height, mdata.beam_azimuth_angles,
        mdata.beam_altitude_angles);
    _cloud = std::make_shared<pcl::PointCloud<point_os::PointOS>>(_width, _height);
    _pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS());

    _batch_and_publish =
      OS1::batch_to_iter<pcl::PointCloud<point_os::PointOS>::iterator>(
      _xyz_lut, _width, _height, {}, &point_os::PointOS::make,
      [&](uint64_t scan_ts) mutable
      {
        if (_pub->get_subscription_count() > 0 && _pub->is_activated()) {
          sensor_msgs::msg::PointCloud2 msg = ros2_ouster::toMsg(*_cloud,
          std::chrono::nanoseconds(scan_ts), _frame);
          _pub->publish(msg);
        }
      });
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~PointcloudProcessor()
  {
    _pub.reset();
  }

  /**
   * @brief Process method to create pointcloud
   * @param data the packet data
   */
  bool process(uint8_t * data) override
  {
    pcl::PointCloud<point_os::PointOS>::iterator it = _cloud->begin();
    _batch_and_publish(data, it);
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
  std::function<void(const uint8_t *, pcl::PointCloud<point_os::PointOS>::iterator)>
  _batch_and_publish;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
  std::shared_ptr<pcl::PointCloud<point_os::PointOS>> _cloud;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  std::vector<double> _xyz_lut;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
};

}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
