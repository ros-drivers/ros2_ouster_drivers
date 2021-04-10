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

#ifndef ROS2_OUSTER__PROCESSORS__SCAN_PROCESSOR_HPP_
#define ROS2_OUSTER__PROCESSORS__SCAN_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/conversions.hpp"
#include "ros2_ouster/full_rotation_accumulator.hpp"

using Cloud = pcl::PointCloud<ouster_ros::Point>;

namespace sensor
{
/**
 * @class sensor::ScanProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Scans in the
 * driver in ROS2.
 */
class ScanProcessor : public ros2_ouster::DataProcessorInterface
{
public:
  /**
   * @brief A constructor for sensor::ScanProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  ScanProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ouster::sensor::sensor_info & mdata,
    const std::string & frame,
    const rclcpp::QoS & qos,
    const ouster::sensor::packet_format & pf,
    std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator)
  : DataProcessorInterface(), _node(node), _frame(frame), _pf(pf)
  {
    _fullRotationAccumulator = fullRotationAccumulator;
    _mdata = mdata;
    _pub = _node->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

    double zero_angle = 9999.0;
    _ring = 0;
    for (uint i = 0; i != _mdata.beam_altitude_angles.size(); i++) {
      if (fabs(_mdata.beam_altitude_angles[i]) < zero_angle) {
        _ring = static_cast<uint8_t>(i);
        zero_angle = fabs(_mdata.beam_altitude_angles[i]);
      }
    }
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~ScanProcessor()
  {
    _pub.reset();
  }

  /**
   * @brief Process method to create scan
   * @param data the packet data
   */
  bool process(const uint8_t * data, const uint64_t override_ts) override
  {
    if (!_fullRotationAccumulator->isBatchReady()) {
      return true;
    }

    _pub->publish(
      ros2_ouster::toMsg(
        *_fullRotationAccumulator->getLidarScan(),
        _fullRotationAccumulator->getTimestamp(),
        _frame, _mdata, _ring, override_ts));
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
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr _pub;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  ouster::sensor::sensor_info _mdata;
  std::string _frame;
  uint8_t _ring;
  ouster::sensor::packet_format _pf;
  std::shared_ptr<sensor::FullRotationAccumulator> _fullRotationAccumulator;
};

}  // namespace sensor

#endif  // ROS2_OUSTER__PROCESSORS__SCAN_PROCESSOR_HPP_
