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

#ifndef ROS2_OUSTER__INTERFACES__LIFECYCLE_INTERFACE_HPP_
#define ROS2_OUSTER__INTERFACES__LIFECYCLE_INTERFACE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lifecycle_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class lifecycle_interface::LifecycleInterface
 * @brief An implementation of the rclcpp lifecycle node in the styling of
 * this codebase. Also abstracts out logging requirements and rclcpp_lifecycle
 * specific types.
 */
class LifecycleInterface : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief A constructor for lifecycle_interface::LifecycleInterface
   * @param name Name of node
   * @param options Node options for lifecycle node interfaces
   */
  LifecycleInterface(const std::string & name, const rclcpp::NodeOptions & options);

  /**
   * @brief Configure class members
   * @param state Reference to LifeCycle node state
   * @return SUCCESS
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate class members
   * @param state Reference to LifeCycle node state
   * @return SUCCESS
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate class members
   * @param state Reference to LifeCycle node state
   * @return SUCCESS
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in error lifecycle state to handle failure safely
   * @param state Reference to LifeCycle node state
   * @return SUCCESS
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown lifecycle state to exit node's lifecycle
   * @param state Reference to LifeCycle node state
   * @return SUCCESS
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in cleanup lifecycle state to clean up node's resources
   * @param state Reference to LifeCycle node state
   * @return SUCCESS
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;


  /**
   * @brief lifecycle node's implementation of configure step
   */
  virtual void onConfigure() = 0;

  /**
   * @brief lifecycle node's implementation of activate step
   */
  virtual void onActivate() = 0;

  /**
   * @brief lifecycle node's implementation of deactivate step
   */
  virtual void onDeactivate() = 0;

  /**
   * @brief lifecycle node's implementation of error step
   */
  virtual void onError() = 0;

  /**
   * @brief lifecycle node's implementation of shutdown step
   */
  virtual void onShutdown() = 0;

  /**
   * @brief lifecycle node's implementation of cleanup step
   */
  virtual void onCleanup() = 0;

  /**
   * @brief Get active state of lifecycle node
   * @return if the lifecycle node is currently active
   */
  inline bool isActive()
  {
    return is_active;
  }

private:
  bool is_active;
};

}  // namespace lifecycle_interface

#endif  // ROS2_OUSTER__INTERFACES__LIFECYCLE_INTERFACE_HPP_
