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

#ifndef ROS2_OUSTER__PROCESSORS__IMAGE_PROCESSOR_HPP_
#define ROS2_OUSTER__PROCESSORS__IMAGE_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <algorithm>
#include <limits>

#include "rclcpp/qos.hpp"

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/client/viz/autoexposure.h"
#include "ros2_ouster/client/viz/beam_uniformity.h"
#include "ros2_ouster/full_rotation_accumulator.hpp"

using Cloud = pcl::PointCloud<ouster_ros::Point>;
namespace viz = ouster::viz;

namespace sensor
{
/**
 * @class sensor::ImageProcessor
 * @brief A data processor interface implementation of a processor
 * for creating range, intensity, and noise images in the
 * driver in ROS2.
 */
class ImageProcessor : public ros2_ouster::DataProcessorInterface
{
public:
  /**
   * @brief A constructor for sensor::ImageProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  ImageProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ouster::sensor::sensor_info & mdata,
    const std::string & frame,
    const rclcpp::QoS & qos,
    const ouster::sensor::packet_format & pf,
    std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator)
  : DataProcessorInterface(), _node(node), _frame(frame), _pf(pf)
  {
    _fullRotationAccumulator = fullRotationAccumulator;
    _height = mdata.format.pixels_per_column;
    _width = mdata.format.columns_per_frame;
    _px_offset = mdata.format.pixel_shift_by_row;
    _ls = ouster::LidarScan{_width, _height};

    _range_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "range_image", qos);
    _intensity_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "intensity_image", qos);
    _ambient_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "ambient_image", qos);
  }

  /**
  * @brief A destructor clearing memory allocated
  */
  ~ImageProcessor()
  {
    _range_image_pub.reset();
    _ambient_image_pub.reset();
    _intensity_image_pub.reset();
  }

  void generate_images(const std::chrono::nanoseconds timestamp, const uint64_t override_ts)
  {
    _range_image.width = _width;
    _range_image.height = _height;
    _range_image.step = _width;
    _range_image.encoding = "mono8";
    _range_image.header.frame_id = _frame;
    _range_image.data.resize(
      _width * _height * _bit_depth /
      (8 * sizeof(*_range_image.data.data())));
    _range_image.header.stamp = override_ts == 0 ? rclcpp::Time(timestamp.count()) : rclcpp::Time(
      override_ts);

    _ambient_image.width = _width;
    _ambient_image.height = _height;
    _ambient_image.step = _width;
    _ambient_image.header.frame_id = _frame;
    _ambient_image.encoding = "mono8";
    _ambient_image.data.resize(
      _width * _height * _bit_depth /
      (8 * sizeof(*_ambient_image.data.data())));
    _ambient_image.header.stamp = override_ts == 0 ? rclcpp::Time(timestamp.count()) : rclcpp::Time(
      override_ts);

    _intensity_image.width = _width;
    _intensity_image.height = _height;
    _intensity_image.step = _width;
    _intensity_image.header.frame_id = _frame;
    _intensity_image.encoding = "mono8";
    _intensity_image.data.resize(
      _width * _height * _bit_depth /
      (8 * sizeof(*_intensity_image.data.data())));
    _intensity_image.header.stamp = override_ts ==
      0 ? rclcpp::Time(timestamp.count()) : rclcpp::Time(override_ts);

    using im_t = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::RowMajor>;
    im_t ambient_image_eigen(_height, _width);
    im_t intensity_image_eigen(_height, _width);

    for (size_t u = 0; u < _height; u++) {
      for (size_t v = 0; v < _width; v++) {
        const size_t vv = (v + _width - _px_offset[u]) % _width;
        const size_t index = u * _width + vv;

        if (_ls.field(ouster::LidarScan::RANGE)(index) == 0) {
          reinterpret_cast<uint8_t *>(
            _range_image.data.data())[u * _width + v] = 0;
        } else {
          reinterpret_cast<uint8_t *>(
            _range_image.data.data())[u * _width + v] =
            _pixel_value_max -
            std::min(
            std::round(_ls.field(ouster::LidarScan::RANGE)(index) * _range_multiplier),
            static_cast<double>(_pixel_value_max));
        }
        ambient_image_eigen(u, v) = _ls.field(ouster::LidarScan::AMBIENT)(index);
        intensity_image_eigen(u, v) = _ls.field(ouster::LidarScan::INTENSITY)(index);
      }
    }

    _ambient_buc.correct(ambient_image_eigen);
    _ambient_ae(
      Eigen::Map<Eigen::ArrayXd>(ambient_image_eigen.data(), _width * _height));
    _intensity_ae(
      Eigen::Map<Eigen::ArrayXd>(intensity_image_eigen.data(), _width * _height));
    ambient_image_eigen = ambient_image_eigen.sqrt();
    intensity_image_eigen = intensity_image_eigen.sqrt();
    for (size_t u = 0; u < _height; u++) {
      for (size_t v = 0; v < _width; v++) {
        reinterpret_cast<uint8_t *>(
          _ambient_image.data.data())[u * _width + v] =
          ambient_image_eigen(u, v) * _pixel_value_max;
        reinterpret_cast<uint8_t *>(
          _intensity_image.data.data())[u * _width + v] =
          intensity_image_eigen(u, v) * _pixel_value_max;
      }
    }
    _range_image_pub->publish(_range_image);
    _ambient_image_pub->publish(_ambient_image);
    _intensity_image_pub->publish(_intensity_image);
  }

  /**
   * @brief Process method to create images
   * @param data the packet data
   */
  bool process(const uint8_t * data, const uint64_t override_ts) override
  {
    if (!_fullRotationAccumulator->isBatchReady()) {
      return true;
    }

    _ls = *_fullRotationAccumulator->getLidarScan();
    generate_images(_fullRotationAccumulator->getTimestamp(), override_ts);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _intensity_image_pub->on_activate();
    _ambient_image_pub->on_activate();
    _range_image_pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _intensity_image_pub->on_deactivate();
    _ambient_image_pub->on_deactivate();
    _range_image_pub->on_deactivate();
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _range_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _ambient_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _intensity_image_pub;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  sensor_msgs::msg::Image _range_image;
  sensor_msgs::msg::Image _ambient_image;
  sensor_msgs::msg::Image _intensity_image;
  std::vector<int> _px_offset;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
  size_t _bit_depth = 8 * sizeof(uint8_t);
  const ouster::sensor::packet_format & _pf;
  const size_t _pixel_value_max = std::numeric_limits<uint8_t>::max();
  // assuming 200 m range typical
  double _range_multiplier = ouster::sensor::range_unit * (1.0 / 200.0);
  viz::AutoExposure _ambient_ae, _intensity_ae;
  viz::BeamUniformityCorrector _ambient_buc;
  ouster::LidarScan _ls;
  std::shared_ptr<sensor::FullRotationAccumulator> _fullRotationAccumulator;
};

}  // namespace sensor

#endif  // ROS2_OUSTER__PROCESSORS__IMAGE_PROCESSOR_HPP_
