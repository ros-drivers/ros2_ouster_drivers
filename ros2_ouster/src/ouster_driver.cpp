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

#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include "ros2_ouster/driver_types.hpp"

namespace ros2_ouster
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std::chrono_literals;

// TODO(stevemacenski):
// main method in a good designed way
// readme

template<typename SensorT>
OusterDriver<SensorT>::OusterDriver(const rclcpp::NodeOptions & options)
: LifecycleInterface("OusterDriver", options)
{
  this->declare_parameter("lidar_ip");
  this->declare_parameter("computer_ip");
  this->declare_parameter("imu_port", 7503);
  this->declare_parameter("lidar_port", 7502);
  this->declare_parameter("lidar_mode", std::string("512x10"));
  this->declare_parameter("sensor_frame", std::string("laser_sensor_frame"));
  this->declare_parameter("laser_frame", std::string("laser_data_frame"));
  this->declare_parameter("imu_frame", std::string("imu_data_frame"));
}

template<typename SensorT>
OusterDriver<SensorT>::~OusterDriver()
{
}

template<typename SensorT>
void OusterDriver<SensorT>::onConfigure()
{
  ros2_ouster::Configuration lidar_config;
  try {
    lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
    lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(),
      "Failed to get lidar or IMU IP address or "
      "hostname. An IP address for both are required!");
    exit(-1);
  }

  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = get_parameter("lidar_mode").as_string();

  RCLCPP_INFO(this->get_logger(),
    "Connecting to sensor at %s.", lidar_config.lidar_ip.c_str());
  RCLCPP_INFO(this->get_logger(),
    "Broadcasting data from sensor to %s.", lidar_config.computer_ip.c_str());

  _range_im_pub = this->create_publisher<sensor_msgs::msg::Image>(
    "range_image", rclcpp::SensorDataQoS());
  _intensity_im_pub = this->create_publisher<sensor_msgs::msg::Image>(
    "intensity_image", rclcpp::SensorDataQoS());
  _noise_im_pub = this->create_publisher<sensor_msgs::msg::Image>(
    "noise_image", rclcpp::SensorDataQoS());
  _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS());
  _pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS());
  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "reset", std::bind(&OusterDriver::resetService, this, _1, _2, _3));
  _metadata_srv = this->create_service<ouster_msgs::srv::GetMetadata>(
    "get_metadata", std::bind(&OusterDriver::getMetadata, this, _1, _2, _3));

  _sensor = std::make_shared<SensorT>();
  _sensor->configure(lidar_config);

  // TODO(stevemacenski): will this work with lifecycle publisher?
  _tf_b = std::make_unique<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  broadcastStaticTransforms();
}

template<typename SensorT>
void OusterDriver<SensorT>::onActivate()
{
  _range_im_pub->on_activate();
  _intensity_im_pub->on_activate();
  _noise_im_pub->on_activate();
  _pc_pub->on_activate();
  _imu_pub->on_activate();

  // speed of the Ouster lidars is 1280 hz
  _process_timer = create_wall_timer(781250ns,
      std::bind(&OusterDriver::processData, this));
}

template<typename SensorT>
void OusterDriver<SensorT>::onError()
{
}

template<typename SensorT>
void OusterDriver<SensorT>::onDeactivate()
{
  _range_im_pub->on_deactivate();
  _intensity_im_pub->on_deactivate();
  _noise_im_pub->on_deactivate();
  _pc_pub->on_deactivate();
  _imu_pub->on_deactivate();
  _process_timer.reset();
}

template<typename SensorT>
void OusterDriver<SensorT>::onCleanup()
{
  _range_im_pub.reset();
  _intensity_im_pub.reset();
  _noise_im_pub.reset();
  _pc_pub.reset();
  _imu_pub.reset();
  _sensor.reset();
  _tf_b.reset();
}

template<typename SensorT>
void OusterDriver<SensorT>::onShutdown()
{
}

template<typename SensorT>
void OusterDriver<SensorT>::broadcastStaticTransforms()
{
  std::string laser_sensor_frame = get_parameter("laser_sensor_frame").as_string();
  std::string laser_data_frame = get_parameter("laser_data_frame").as_string();
  std::string imu_data_frame = get_parameter("imu_data_frame").as_string();
  if (_tf_b) {
    ros2_ouster::Metadata mdata = _sensor->getMetadata();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(toMsg(mdata.imu_to_sensor_transform,
      laser_sensor_frame, imu_data_frame, this->now()));
    transforms.push_back(toMsg(mdata.lidar_to_sensor_transform,
      laser_sensor_frame, laser_data_frame, this->now()));
    _tf_b->sendTransform(transforms);
  }
}

template<typename SensorT>
void OusterDriver<SensorT>::processData()
{
  // TODO(stevemacenski) this full method
  // if (auto state = _sensor->poll() && _sensor->isValid(state))
  // {
  // if (auto data = _sensor_client->get(state)) // data type will be a pkt processed
  // {
  // data_processors[state]->handle(data);  // take and buffer / convert / publish
  //         I'd really like to not have the publishers in this.... maybe a functor?
  // }
  // }
}

template<typename SensorT>
void OusterDriver<SensorT>::resetService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  if (!this->isActive()) {
    return;
  }

  ros2_ouster::Configuration lidar_config;
  lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
  lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = get_parameter("lidar_mode").as_string();
  _sensor->reset(lidar_config);
}

template<typename SensorT>
void OusterDriver<SensorT>::getMetadata(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ouster_msgs::srv::GetMetadata::Request> request,
  std::shared_ptr<ouster_msgs::srv::GetMetadata::Response> response)
{
  if (!this->isActive()) {
    return;
  }
  response->metadata = toMsg(_sensor->getMetadata());
}

}  // namespace ros2_ouster
