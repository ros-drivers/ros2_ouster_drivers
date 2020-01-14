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

#ifndef ROS2_OUSTER__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_
#define ROS2_OUSTER__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <algorithm>

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/OS1/OS1_util.hpp"

namespace OS1
{
/**
 * @class OS1::ImageProcessor
 * @brief A data processor interface implementation of a processor
 * for creating range, intensity, and noise images in the
 * driver in ROS2.
 */
class ImageProcessor : public ros2_ouster::DataProcessorInterface
{
public:
  typedef std::vector<image_os::ImageOS> OSImage;
  typedef OSImage::iterator OSImageIt;

  /**
   * @brief A constructor for OS1::ImageProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  ImageProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ros2_ouster::Metadata & mdata,
    const std::string & frame)
  : DataProcessorInterface(), _node(node), _frame(frame)
  {
    _height = OS1::pixels_per_column;
    _width = OS1::n_cols_of_lidar_mode(
      OS1::lidar_mode_of_string(mdata.mode));
    _px_offset = OS1::get_px_offset(_width);
    _xyz_lut = OS1::make_xyz_lut(_width, _height, mdata.beam_azimuth_angles,
        mdata.beam_altitude_angles);

    _range_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "range_image", rclcpp::SensorDataQoS());
    _intensity_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "intensity_image", rclcpp::SensorDataQoS());
    _noise_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "noise_image", rclcpp::SensorDataQoS());
    _reflectivity_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "reflectivity_image", rclcpp::SensorDataQoS());

    _range_image.width = _width;
    _range_image.height = _height;
    _range_image.step = _width;
    _range_image.encoding = "mono8";
    _range_image.header.frame_id = _frame;
    _range_image.data.resize(_width * _height);

    _intensity_image.width = _width;
    _intensity_image.height = _height;
    _intensity_image.step = _width;
    _intensity_image.encoding = "mono8";
    _intensity_image.header.frame_id = _frame;
    _intensity_image.data.resize(_width * _height);

    _noise_image.width = _width;
    _noise_image.height = _height;
    _noise_image.step = _width;
    _noise_image.encoding = "mono8";
    _noise_image.header.frame_id = _frame;
    _noise_image.data.resize(_width * _height);

    _reflectivity_image.width = _width;
    _reflectivity_image.height = _height;
    _reflectivity_image.step = _width;
    _reflectivity_image.encoding = "mono8";
    _reflectivity_image.header.frame_id = _frame;
    _reflectivity_image.data.resize(_width * _height);

    _information_image.resize(_width * _height);

    _batch_and_publish =
      OS1::batch_to_iter<OSImageIt>(
      _xyz_lut, _width, _height, {}, &image_os::ImageOS::make,
      [&](uint64_t scan_ts) mutable
      {
        rclcpp::Time t(scan_ts);
        _range_image.header.stamp = t;
        _noise_image.header.stamp = t;
        _intensity_image.header.stamp = t;
        _reflectivity_image.header.stamp = t;

        OSImageIt it;
        for (uint u = 0; u != _height; u++) {
          for (uint v = 0; v != _width; v++) {
            const size_t vv = (v + _px_offset[u]) % _width;
            const size_t index = vv * _height + u;
            image_os::ImageOS & px = _information_image[index];

            const uint & idx = u * _width + v;
            if (px.range == 0) {
              _range_image.data[idx] = 0;
            } else {
              _range_image.data[idx] = 255 - std::min(std::round(px.range * 5e-3), 255.0);
            }
            _noise_image.data[idx] = std::min(px.noise, static_cast<uint16_t>(255));
            _intensity_image.data[idx] = std::min(px.intensity, 255.0f);
            _reflectivity_image.data[idx] = std::min(px.reflectivity, static_cast<uint16_t>(255));
          }
        }

        if (_range_image_pub->get_subscription_count() > 0 &&
        _range_image_pub->is_activated())
        {
          _range_image_pub->publish(_range_image);
        }

        if (_noise_image_pub->get_subscription_count() > 0 &&
        _noise_image_pub->is_activated())
        {
          _noise_image_pub->publish(_noise_image);
        }

        if (_intensity_image_pub->get_subscription_count() > 0 &&
        _intensity_image_pub->is_activated())
        {
          _intensity_image_pub->publish(_intensity_image);
        }

        if (_reflectivity_image_pub->get_subscription_count() > 0 &&
        _reflectivity_image_pub->is_activated())
        {
          _reflectivity_image_pub->publish(_reflectivity_image);
        }
      });
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~ImageProcessor()
  {
    _reflectivity_image_pub.reset();
    _intensity_image_pub.reset();
    _noise_image_pub.reset();
    _range_image_pub.reset();
  }

  /**
   * @brief Process method to create images
   * @param data the packet data
   */
  bool process(uint8_t * data) override
  {
    OSImageIt it = _information_image.begin();
    _batch_and_publish(data, it);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _reflectivity_image_pub->on_activate();
    _intensity_image_pub->on_activate();
    _range_image_pub->on_activate();
    _noise_image_pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _reflectivity_image_pub->on_deactivate();
    _intensity_image_pub->on_deactivate();
    _range_image_pub->on_deactivate();
    _noise_image_pub->on_deactivate();
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _reflectivity_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _intensity_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _range_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _noise_image_pub;
  std::function<void(const uint8_t *, OSImageIt)> _batch_and_publish;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  sensor_msgs::msg::Image _reflectivity_image;
  sensor_msgs::msg::Image _intensity_image;
  sensor_msgs::msg::Image _range_image;
  sensor_msgs::msg::Image _noise_image;
  std::vector<double> _xyz_lut;
  std::vector<int> _px_offset;
  OSImage _information_image;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
};

}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_
