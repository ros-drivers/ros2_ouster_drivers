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

#ifndef ROS2_OUSTER__DRIVER_TYPES_HPP_
#define ROS2_OUSTER__DRIVER_TYPES_HPP_

#include <string>
#include <vector>

#include "ros2_ouster/ouster_driver.hpp"
#include "ros2_ouster/OS1/OS1_sensor.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_ouster
{

template class ros2_ouster::OusterDriver<OS1::OS1Sensor>;
using OS1Driver = ros2_ouster::OusterDriver<OS1::OS1Sensor>;

}  // namespace ros2_ouster

// RCLCPP_COMPONENTS_REGISTER_NODE(ros2_ouster::OS1Driver)

#endif  // ROS2_OUSTER__DRIVER_TYPES_HPP_
