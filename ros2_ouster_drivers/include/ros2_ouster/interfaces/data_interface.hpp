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

#ifndef ROS2_OUSTER__DATA_INTERFACE_HPP_
#define ROS2_OUSTER__DATA_INTERFACE_HPP_

#include <memory>
#include <string>

#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/interfaces/configuration.hpp"

namespace ros2_ouster
{
/**
 * @class ros2_ouster::DataInterface
 * @brief An interface for data types coming from lidar
 */
template <typename DataT>  //TODO ???
class DataInterface
{
  DataInterface() {};

  void onNewData(data); // here can put into struct, or call other methods to buffer

  // should this also own the data publisher and decide to publish on new data?
    // the buffered pointcloud will trigger a publish when its done
    // unless I change the code to wait until called?
    // I dont necessarily like the idea of making this interface ros-y
  // should this know about the sensor itself its using? OS1/2/3/...?

private:
  std::string pkg_buf; // a data type will preallocate this at run time
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__DATA_INTERFACE_HPP_
