// Copyright (c) 2019 Intel Corporation
//
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

#ifndef ROS2_OUSTER__ROS2_UTILS_HPP_
#define ROS2_OUSTER__ROS2_UTILS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace ros2_ouster
{

/**
 * @brief Declares static ROS2 parameter and sets it to a given value if it was
 *        not already declared.
 * @details This utility function is borrowed from the navigation2 package.
 *
 * @param[in] node_ptr A node in which given parameter to be declared
 * @param[in] param_name The name of parameter
 */
template<typename NodeTypePtr>
void declare_parameter_if_not_declared(
  NodeTypePtr node_ptr,
  const std::string & param_name)
{
  if (!node_ptr->has_parameter(param_name)) {
    node_ptr->declare_parameter(param_name);
  }
}


/**
 * @brief Declares static ROS2 parameter and sets it to a given value if it was
 *        not already declared.
 * @details This utility function is borrowed from the navigation2 package.
 *
 * @param[in] node_ptr A node in which given parameter to be declared
 * @param[in] param_name The name of parameter
 * @param[in] default_value Parameter value to initialize with
 * @param[in] parameter_descriptor Parameter descriptor (optional)
 */
template<typename NodeTypePtr>
void declare_parameter_if_not_declared(
  NodeTypePtr node_ptr,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node_ptr->has_parameter(param_name)) {
    node_ptr->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__ROS2_UTILS_HPP_
