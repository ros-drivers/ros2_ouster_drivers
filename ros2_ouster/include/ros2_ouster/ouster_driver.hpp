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

#ifndef ROS2_OUSTER__OUSTER_DRIVER_HPP_
#define ROS2_OUSTER__OUSTER_DRIVER_HPP_

#include <memory>
#include <map>
#include <string>

#include "ros2_ouster/conversions.hpp"

#include "ros2_ouster/interfaces/lifecycle_interface.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "ouster_msgs/srv/get_metadata.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

#include "ros2_ouster/interfaces/configuration.hpp"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/full_rotation_accumulator.hpp"

namespace ros2_ouster
{

class SensorInterface;

/**
 * @class ros2_ouster::OusterDriver
 * @brief A lifecycle interface implementation of a Ouster OS-1 Lidar
 * driver in ROS2.
 */
class OusterDriver : public lifecycle_interface::LifecycleInterface
{
public:
  using DataProcessorMap = std::multimap<ouster::sensor::client_state,
      std::unique_ptr<ros2_ouster::DataProcessorInterface>>;
  using DataProcessorMapIt = DataProcessorMap::iterator;

  /**
   * @brief A constructor for ros2_ouster::OusterDriver
   * @param options Node options for lifecycle node interfaces
   */
  OusterDriver(
    std::unique_ptr<SensorInterface> sensor,
    const rclcpp::NodeOptions & options);

  /**
   * @brief A destructor for ros2_ouster::OusterDriver
   */
  ~OusterDriver();

  /**
   * @brief lifecycle node's implementation of configure step
   * which will configure ROS interfaces and allocate resources
   */
  void onConfigure() override;

  /**
   * @brief lifecycle node's implementation of activate step
   * which will activate ROS interfaces and start processing information
   */
  void onActivate() override;

  /**
   * @brief lifecycle node's implementation of deactivate step
   * which will deactivate ROS interfaces and stop processing information
   */
  void onDeactivate() override;

  /**
   * @brief lifecycle node's implementation of error step
   * which will handle errors in the lifecycle node system
   */
  void onError() override;

  /**
   * @brief lifecycle node's implementation of shutdown step
   * which will shut down the lifecycle node
   */
  void onShutdown() override;

  /**
   * @brief lifecycle node's implementation of cleanup step
   * which will deallocate resources
   */
  void onCleanup() override;

private:
  /**
  * @brief Timer callback to process the UDP socket
  */
  void processData();

  /**
   * @brief Create TF2 frames for the lidar sensor
   */
  void broadcastStaticTransforms(const ouster::sensor::sensor_info & mdata);

  /**
  * @brief service callback to reset the lidar
  * @param request_header Header of rmw request
  * @param request Shared ptr of the Empty request
  * @param response Shared ptr of the Empty response
  */
  void resetService(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /**
  * @brief service callback to get metadata from lidar
  * @param request_header Header of rmw request
  * @param request Shared ptr of the GetMetadata request
  * @param response Shared ptr of the GetMetadata response
  */
  void getMetadata(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<ouster_msgs::srv::GetMetadata::Request> request,
    std::shared_ptr<ouster_msgs::srv::GetMetadata::Response> response);

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_srv;
  rclcpp::Service<ouster_msgs::srv::GetMetadata>::SharedPtr _metadata_srv;

  std::unique_ptr<SensorInterface> _sensor;
  std::multimap<ouster::sensor::client_state,
    std::unique_ptr<ros2_ouster::DataProcessorInterface>> _data_processors;
  rclcpp::TimerBase::SharedPtr _process_timer;

  std::shared_ptr<sensor::FullRotationAccumulator> _full_rotation_accumulator;

  std::string _laser_sensor_frame, _laser_data_frame, _imu_data_frame;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _tf_b;

  bool _use_system_default_qos;
  bool _use_ros_time;

  std::uint32_t _proc_mask;

  uint8_t * _lidar_packet_data;
  uint8_t * _imu_packet_data;
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__OUSTER_DRIVER_HPP_
