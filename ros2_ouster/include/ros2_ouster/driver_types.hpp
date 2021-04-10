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

#ifndef ROS2_OUSTER__DRIVER_TYPES_HPP_
#define ROS2_OUSTER__DRIVER_TYPES_HPP_

#include "ros2_ouster/ouster_driver.hpp"

namespace ros2_ouster
{

class Driver : public OusterDriver
{
public:
  explicit Driver(rclcpp::NodeOptions options);
};

}  // namespace ros2_ouster


#endif  // ROS2_OUSTER__DRIVER_TYPES_HPP_
