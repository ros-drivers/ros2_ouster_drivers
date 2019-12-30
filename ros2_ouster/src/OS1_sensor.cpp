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

#include "ros2_ouster/OS1/OS1_sensor.hpp"

namespace OS1
{

OS1Sensor::OS1Sensor()
: SensorInterface()
{
}

void OS1Sensor::reset(const ros2_ouster::Configuration & config)
{
  _ouster_client.reset();
  configure(config);
}

void OS1Sensor::configure(const ros2_ouster::Configuration & config)
{
  if (!OS1::lidar_mode_of_string(config.lidar_mode)) {
    throw ("Invalid lidar mode %s!", config.lidar_mode.c_str());
    exit(-1);
  }

  _ouster_client = OS1::init_client(
    config.lidar_ip, config.computer_ip,
    OS1::lidar_mode_of_string(config.lidar_mode),
    config.lidar_port, config.imu_port);

  if (!_ouster_client) {
    throw ("Failed to create connection to lidar.");
  }
}

ros2_ouster::Metadata OS1Sensor::getMetadata()
{
  if (_ouster_client) {
    return OS1::parse_metadata(OS1::get_metadata(*_ouster_client));
  } else {
    return {"UNKNOWN", "UNKNOWN", "UNNKOWN", "UNNKOWN",
      {}, {}, {}, {}, 7503, 7502};
  }
}

}  // namespace OS1
