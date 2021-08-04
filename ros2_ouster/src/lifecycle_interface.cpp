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

#include <string>

#include "ros2_ouster/interfaces/lifecycle_interface.hpp"

namespace lifecycle_interface
{

LifecycleInterface::LifecycleInterface(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(name, options), is_active(false)
{
}

CallbackReturn LifecycleInterface::on_configure(const rclcpp_lifecycle::State & state)
{
  onConfigure();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Activating Ouster driver node.");
  onActivate();
  is_active = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Ouster driver node.");
  onDeactivate();
  is_active = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(this->get_logger(), "Handing error in Ouster driver node.");
  onError();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Ouster driver node.");
  onShutdown();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Ouster driver node.");
  onCleanup();
  return CallbackReturn::SUCCESS;
}

}  // namespace lifecycle_interface
