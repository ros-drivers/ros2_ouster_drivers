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

#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <utility>

#include "rclcpp/qos.hpp"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/driver_types.hpp"

namespace ros2_ouster
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

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

  _laser_sensor_frame = get_parameter("sensor_frame").as_string();
  _laser_data_frame = get_parameter("laser_frame").as_string();
  _imu_data_frame = get_parameter("imu_frame").as_string();
  _use_system_default_qos = get_parameter("use_system_default_qos").as_bool();

  RCLCPP_INFO(this->get_logger(),
    "Connecting to sensor at %s.", lidar_config.lidar_ip.c_str());
  RCLCPP_INFO(this->get_logger(),
    "Broadcasting data from sensor to %s.", lidar_config.computer_ip.c_str());

  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&OusterDriver::resetService, this, _1, _2, _3));
  _metadata_srv = this->create_service<ouster_msgs::srv::GetMetadata>(
    "~/get_metadata", std::bind(&OusterDriver::getMetadata, this, _1, _2, _3));

  _sensor = std::make_shared<SensorT>();

  try {
    _sensor->configure(lidar_config);
  } catch (const OusterDriverException & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception thrown: (%s)", e.what());
    exit(-1);
  }

  ros2_ouster::Metadata mdata = _sensor->getMetadata();

  if (_use_system_default_qos) {
    RCLCPP_INFO(
      this->get_logger(), "Using system defaults QoS for sensor data");
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), mdata, _imu_data_frame, _laser_data_frame,
      rclcpp::SystemDefaultsQoS());
  } else {
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), mdata, _imu_data_frame, _laser_data_frame,
      rclcpp::SensorDataQoS());
  }

  _tf_b = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    shared_from_this());
  broadcastStaticTransforms(mdata);
}

template<typename SensorT>
void OusterDriver<SensorT>::onActivate()
{
  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onActivate();
  }

  // speed of the Ouster lidars is 1280 hz
  _process_timer = this->create_wall_timer(781250ns,
      std::bind(&OusterDriver<SensorT>::processData, this));
}

template<typename SensorT>
void OusterDriver<SensorT>::onError()
{
}

template<typename SensorT>
void OusterDriver<SensorT>::onDeactivate()
{
  _process_timer->cancel();
  _process_timer.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onDeactivate();
  }
}

template<typename SensorT>
void OusterDriver<SensorT>::onCleanup()
{
  _data_processors.clear();
  _tf_b.reset();
  _reset_srv.reset();
  _metadata_srv.reset();
}

template<typename SensorT>
void OusterDriver<SensorT>::onShutdown()
{
  _process_timer->cancel();
  _process_timer.reset();
  _tf_b.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    delete it->second;
  }
  _data_processors.clear();
}

template<typename SensorT>
void OusterDriver<SensorT>::broadcastStaticTransforms(
  const ros2_ouster::Metadata & mdata)
{
  if (_tf_b) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(toMsg(mdata.imu_to_sensor_transform,
      _laser_sensor_frame, _imu_data_frame, this->now()));
    transforms.push_back(toMsg(mdata.lidar_to_sensor_transform,
      _laser_sensor_frame, _laser_data_frame, this->now()));
    _tf_b->sendTransform(transforms);
  }
}

template<typename SensorT>
void OusterDriver<SensorT>::processData()
{
  try {
    ClientState state = _sensor->get();
    RCLCPP_DEBUG(this->get_logger(),
      "Packet with state: %s",
      ros2_ouster::toString(state).c_str());

    uint8_t * packet_data = _sensor->readPacket(state);

    if (packet_data) {
      std::pair<DataProcessorMapIt, DataProcessorMapIt> key_its;
      key_its = _data_processors.equal_range(state);
      for (DataProcessorMapIt it = key_its.first; it != key_its.second; it++) {
        it->second->process(packet_data);
      }
    }
  } catch (const OusterDriverException & e) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to process packet with exception %s.", e.what());
  }
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
