// Copyright 2020
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

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "ros2_ouster/image_os.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

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
    uint32_t _height = OS1::pixels_per_column;
    uint32_t _width = OS1::n_cols_of_lidar_mode(
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

    _range_image = std::make_unique<sensor_msgs::msg::Image>();
    _range_image->width = _width;
    _range_image->height = _height;
    _range_image->step = _width;
    _range_image->encoding = "mono8";
    _range_image->header.frame_id = _frame;
    _range_image->data.resize(_width * _height);

    _intensity_image = std::make_unique<sensor_msgs::msg::Image>();
    _intensity_image->width = _width;
    _intensity_image->height = _height;
    _intensity_image->step = _width;
    _intensity_image->encoding = "mono8";
    _intensity_image->header.frame_id = _frame;
    _intensity_image->data.resize(_width * _height);

    _noise_image = std::make_unique<sensor_msgs::msg::Image>();
    _noise_image->width = _width;
    _noise_image->height = _height;
    _noise_image->step = _width;
    _noise_image->encoding = "mono8";
    _noise_image->header.frame_id = _frame;
    _noise_image->data.resize(_width * _height);

    _reflectivity_image = std::make_unique<sensor_msgs::msg::Image>();
    _reflectivity_image->width = _width;
    _reflectivity_image->height = _height;
    _reflectivity_image->step = _width;
    _reflectivity_image->encoding = "mono8";
    _reflectivity_image->header.frame_id = _frame;
    _reflectivity_image->data.resize(_width * _height);

    _information_image.resize(_width * _height);

    _batch_and_publish =
      OS1::batch_to_iter<std::vector<image_os::ImageOS>::iterator>(
      _xyz_lut, _width, _height, {}, &image_os::ImageOS::make,
      [&](uint64_t scan_ts) mutable
      {
        rclcpp::Time t(scan_ts);
        _range_image->header.stamp = t;
        _noise_image->header.stamp = t;
        _intensity_image->header.stamp = t;
        _reflectivity_image->header.stamp = t;

        std::vector<image_os::ImageOS>::iterator it;
        for (uint i = 0; i != _information_image.size(); i++) {
          _range_image->data[i] = _information_image[i].range;
          _noise_image->data[i] = _information_image[i].noise;
          _intensity_image->data[i] = _information_image[i].intensity;
          _reflectivity_image->data[i] = _information_image[i].reflectivity;
        }

        if (_range_image_pub->get_subscription_count() > 0 &&
        _range_image_pub->is_activated())
        {
          std::cout << "PUBLISHING" << std::endl;
          _range_image_pub->publish(std::move(_range_image));
        }

        if (_noise_image_pub->get_subscription_count() > 0 &&
        _noise_image_pub->is_activated())
        {
          _noise_image_pub->publish(std::move(_noise_image));
        }

        if (_intensity_image_pub->get_subscription_count() > 0 &&
        _intensity_image_pub->is_activated())
        {
          _intensity_image_pub->publish(std::move(_intensity_image));
        }

        if (_reflectivity_image_pub->get_subscription_count() > 0 &&
        _reflectivity_image_pub->is_activated())
        {
          _reflectivity_image_pub->publish(std::move(_reflectivity_image));
        }
      });
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~ImageProcessor()
  {
    _noise_image_pub.reset();
    _intensity_image_pub.reset();
    _range_image_pub.reset();
    _reflectivity_image_pub.reset();
  }

  /**
   * @brief Process method to create images
   * @param data the packet data
   */
  bool process(uint8_t * data) override
  {
    std::vector<image_os::ImageOS>::iterator it = _information_image.begin();
    _batch_and_publish(data, it);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _range_image_pub->on_activate();
    _intensity_image_pub->on_activate();
    _noise_image_pub->on_activate();
    _reflectivity_image_pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _range_image_pub->on_deactivate();
    _intensity_image_pub->on_deactivate();
    _noise_image_pub->on_deactivate();
    _reflectivity_image_pub->on_deactivate();
  }

private:
  std::vector<image_os::ImageOS> _information_image;
  sensor_msgs::msg::Image::UniquePtr _range_image;
  sensor_msgs::msg::Image::UniquePtr _noise_image;
  sensor_msgs::msg::Image::UniquePtr _intensity_image;
  sensor_msgs::msg::Image::UniquePtr _reflectivity_image;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _range_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _intensity_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _noise_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _reflectivity_image_pub;
  std::function<void(const uint8_t *, std::vector<image_os::ImageOS>::iterator)> _batch_and_publish;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  uint32_t _height;
  uint32_t _width;
  std::vector<double> _xyz_lut;
  std::vector<int> _px_offset;
  std::string _frame;
};

}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_
