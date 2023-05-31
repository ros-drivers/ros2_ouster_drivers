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

#ifndef ROS2_OUSTER__CLIENT__IMPL_HTTP_HPP_
#define ROS2_OUSTER__CLIENT__IMPL_HTTP_HPP_

#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <json/json.h>

#include "ros2_ouster/client/interfaces/client_interface.hpp"
#include "ros2_ouster/client/impl/curl_client.hpp"

namespace ouster
{
namespace sensor
{
namespace impl
{

/**
 * An implementation of the sensor http interface
 */
class HttpImpl : public util::ClientInterface
{
  public:
  /**
     * Constructs an http interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
  explicit HttpImpl(const std::string & hostname)
      : http_client(std::make_unique<util::CurlClient>("http://" + hostname))
  {}

  /**
     * Deconstruct the sensor http interface.
     */
  ~HttpImpl() override = default;

  /**
     * Queries the sensor metadata.
     *
     * @return returns a Json object of the sensor metadata.
     */
  [[nodiscard]] inline Json::Value metadata() const override
  {
    return get_json("api/v1/sensor/metadata");
  }

  /**
     * Queries the sensor_info.
     *
     * @return returns a Json object representing the sensor_info.
     */
  [[nodiscard]] inline Json::Value sensor_info() const override
  {
    return get_json("api/v1/sensor/metadata/sensor_info");
  }

  /**
     * Queries active/staged configuration on the sensor
     *
     * @param[in] active if true retrieve active, otherwise get staged configs.
     *
     * @return a string represnting the active or staged config
     */
  [[nodiscard]] inline std::string get_config_params(bool active) const override
  {
    auto config_type = active ? "active" : "staged";
    return get(std::string("api/v1/sensor/cmd/get_config_param?args=") +
               config_type);
  }

  /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     */
  inline void set_config_param(const std::string & key,
                               const std::string & value) const override
  {
    auto encoded_value = http_client->encode(value);// encode config params
    auto url = "api/v1/sensor/cmd/set_config_param?args=" + key + "+" +
               encoded_value;
    execute(url, "\"set_config_param\"");
  }

  /**
     * Retrieves the active configuration on the sensor
     */
  [[nodiscard]] inline Json::Value active_config_params() const override
  {
    return get_json("api/v1/sensor/cmd/get_config_param?args=active");
  }

  /**
     * Retrieves the staged configuration on the sensor
     */
  [[nodiscard]] inline Json::Value staged_config_params() const override
  {
    return get_json("api/v1/sensor/cmd/get_config_param?args=staged");
  }

  /**
     * Enables automatic assignment of udp destination ports.
     */
  inline void set_udp_dest_auto() const override
  {
    execute("api/v1/sensor/cmd/set_udp_dest_auto", "{}");
  }

  /**
     * Retrieves beam intrinsics of the sensor.
     */
  [[nodiscard]] inline Json::Value beam_intrinsics() const override
  {
    return get_json("api/v1/sensor/metadata/beam_intrinsics");
  }

  /**
     * Retrieves imu intrinsics of the sensor.
     */
  [[nodiscard]] inline Json::Value imu_intrinsics() const override
  {
    return get_json("api/v1/sensor/metadata/imu_intrinsics");
  }

  /**
     * Retrieves lidar intrinsics of the sensor.
     */
  [[nodiscard]] inline Json::Value lidar_intrinsics() const override
  {
    return get_json("api/v1/sensor/metadata/lidar_intrinsics");
  }

  /**
     * Retrieves lidar data format.
     */
  [[nodiscard]] inline Json::Value lidar_data_format() const override
  {
    return get_json("api/v1/sensor/metadata/lidar_data_format");
  }

  /**
     * Gets the calibaration status of the sensor.
     */
  [[nodiscard]] inline Json::Value calibration_status() const override
  {
    return get_json("api/v1/sensor/metadata/calibration_status");
  }

  /**
     * Restarts the sensor applying all staged configurations.
     */
  inline void reinitialize() const override
  {
    execute("api/v1/sensor/cmd/reinitialize", "{}");
  }

  /**
     * Persist active configuration parameters to the sensor.
     */
  inline void save_config_params() const override
  {
    execute("api/v1/sensor/cmd/save_config_params", "{}");
  }

  private:
  [[nodiscard]] inline std::string get(const std::string & url) const
  {
    return http_client->get(url);
  }

  [[nodiscard]] Json::Value get_json(const std::string & url) const
  {
    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root;
    auto result = get(url);
    if (!reader->parse(result.c_str(), result.c_str() + result.size(), &root,
                       nullptr))
      throw std::runtime_error("SensorHttpImp::get_json failed! url: " + url);
    return root;
  }

  void execute(const std::string & url, const std::string & validation) const
  {
    auto result = get(url);
    if (result != validation)
      throw std::runtime_error("SensorHttpImp::execute failed! url: " + url +
                               " returned [" + result + "], expected [" +
                               validation + "]");
  }

  std::unique_ptr<util::CurlClient> http_client;
};

}// namespace impl
}// namespace sensor
}// namespace ouster

#endif// ROS2_OUSTER__CLIENT__IMPL_HTTP_HPP_
