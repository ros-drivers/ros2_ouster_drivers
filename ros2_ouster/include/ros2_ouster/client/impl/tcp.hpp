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

#ifndef ROS2_OUSTER__CLIENT__IMPL_TCP_HPP_
#define ROS2_OUSTER__CLIENT__IMPL_TCP_HPP_

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

namespace ouster
{
namespace sensor
{
namespace impl
{

class TcpImpl : public util::ClientInterface
{
  public:
  // timeout for reading from a TCP socket during config
  static constexpr int RCVTIMEOUT_SEC = 10;
  // maximum size to handle during recv
  static constexpr size_t MAX_RESULT_LENGTH = 16 * 1024;

  /**
     * Constructs an tcp interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
  explicit TcpImpl(const std::string &hostname)
      : socket_handle(cfg_socket(hostname.c_str())),
        read_buf(std::unique_ptr<char[]>{new char[MAX_RESULT_LENGTH + 1]})
  {}

  /**
     * Deconstruct the sensor tcp interface.
     */
  ~TcpImpl() override { close(socket_handle); }
  /**
     * Queries the sensor metadata.
     *
     * @return returns a Json object of the sensor metadata.
     */
  [[nodiscard]] inline Json::Value metadata() const override
  {
    Json::Value root;
    root["sensor_info"] = sensor_info();
    root["beam_intrinsics"] = beam_intrinsics();
    root["imu_intrinsics"] = imu_intrinsics();
    root["lidar_intrinsics"] = lidar_intrinsics();
    root["lidar_data_format"] = lidar_data_format();
    root["calibration_status"] = calibration_status();
    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    auto res = get_config_params(true);
    Json::Value node;
    auto parse_success = reader->parse(res.c_str(), res.c_str() + res.size(),
                                       &node, nullptr);
    root["config_params"] = parse_success ? node : res;
    return root;
  }

  /**
     * Queries the sensor_info.
     *
     * @return returns a Json object representing the sensor_info.
     */
  [[nodiscard]] inline Json::Value sensor_info() const override
  {
    return tcp_cmd_json({"get_sensor_info"});
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
    return tcp_cmd({"get_config_param", config_type});
  }

  inline static std::string rtrim(const std::string &s)
  {
    return s.substr(
            0, std::distance(
                       s.begin(),
                       std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
                         return !isspace(ch);
                       }).base()));
  }

  /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     */
  inline void set_config_param(const std::string &key,
                               const std::string &value) const override
  {
    tcp_cmd_with_validation({"set_config_param", key, rtrim(value)},
                            "set_config_param");
  }

  /**
     * Retrieves the active configuration on the sensor
     */
  [[nodiscard]] inline Json::Value active_config_params() const override
  {
    return tcp_cmd_json({"get_config_param", "active"});
  }

  /**
     * Retrieves the staged configuration on the sensor
     */
  [[nodiscard]] inline Json::Value staged_config_params() const override
  {
    return tcp_cmd_json({"get_config_param", "staged"});
  }

  /**
     * Enables automatic assignment of udp destination ports.
     */
  inline void set_udp_dest_auto() const override
  {
    tcp_cmd_with_validation({"set_udp_dest_auto"}, "set_udp_dest_auto");
  }

  /**
     * Retrieves beam intrinsics of the sensor.
     */
  [[nodiscard]] inline Json::Value beam_intrinsics() const override
  {
    return tcp_cmd_json({"get_beam_intrinsics"});
  }

  /**
     * Retrieves imu intrinsics of the sensor.
     */
  [[nodiscard]] inline Json::Value imu_intrinsics() const override
  {
    return tcp_cmd_json({"get_imu_intrinsics"});
  }

  /**
     * Retrieves lidar intrinsics of the sensor.
     */
  [[nodiscard]] inline Json::Value lidar_intrinsics() const override
  {
    return tcp_cmd_json({"get_lidar_intrinsics"});
  }

  /**
     * Retrieves lidar data format.
     */
  [[nodiscard]] inline Json::Value lidar_data_format() const override
  {
    return tcp_cmd_json({"get_lidar_data_format"}, false);
  }

  /**
     * Gets the calibaration status of the sensor.
     */
  [[nodiscard]] inline Json::Value calibration_status() const override
  {
    return tcp_cmd_json({"get_calibration_status"}, false);
  }

  /**
     * Restarts the sensor applying all staged configurations.
     */
  inline void reinitialize() const override
  {
    // reinitialize to make all staged parameters effective
    tcp_cmd_with_validation({"reinitialize"}, "reinitialize");
  }

  /**
     * Persist active configuration parameters to the sensor.
     */
  inline void save_config_params() const override
  {
    tcp_cmd_with_validation({"write_config_txt"}, "write_config_txt");
  }

  private:
  /**
 * Configure socket
 * @param addr address of socket to reconfigure
 * @return int The socket ID number
 */
  inline static int cfg_socket(const char *addr)
  {
    struct addrinfo hints {
    }, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    // try to parse as numeric address first: avoids spurious errors from
    // DNS lookup when not using a hostname (and should be faster)
    hints.ai_flags = AI_NUMERICHOST;
    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
      hints.ai_flags = 0;
      ret = getaddrinfo(addr, "7501", &hints, &info_start);
      if (ret != 0) {
        std::cout << "cfg getaddrinfo(): {}" << gai_strerror(ret) << std::endl;
        return -1;
      }
    }

    if (info_start == nullptr) {
      std::cerr << "cfg getaddrinfo(): empty result" << std::endl;
      return -1;
    }

    int sock_fd;
    for (ai = info_start; ai != nullptr; ai = ai->ai_next) {
      sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
      if (sock_fd < 0) {
        std::cerr << "cfg socket(): " << std::strerror(errno) << std::endl;
        continue;
      }

      if (connect(sock_fd, ai->ai_addr,
                  static_cast<socklen_t>(ai->ai_addrlen)) == -1) {
        close(sock_fd);
        continue;
      }

      struct timeval tv {};
      tv.tv_sec = RCVTIMEOUT_SEC;
      tv.tv_usec = 0;
      if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv,
                     sizeof tv)) {
        std::cout << "cfg set_rcvtimeout(): {}" << std::strerror(errno)
                  << std::endl;
        close(sock_fd);
        continue;
      }

      break;
    }

    freeaddrinfo(info_start);
    if (ai == nullptr) { return -1; }

    return sock_fd;
  }

  /**
 * Conduct a specific TCP socket command
 * @param cmd_tokens command tokens
 * @return result string of packet data
 */
  [[nodiscard]] inline std::string
  tcp_cmd(const std::vector<std::string> &cmd_tokens) const
  {
    std::stringstream ss;
    for (const auto &token: cmd_tokens) { ss << token << " "; }
    ss << "\n";
    std::string cmd = ss.str();

    ssize_t len = send(socket_handle, cmd.c_str(), cmd.length(), 0);
    if (len != static_cast<ssize_t>(cmd.length())) {
      throw std::runtime_error("tcp_cmd socket::send failed");
    }

    // need to synchronize with server by reading response
    std::stringstream read_ss;
    do {
      len = recv(socket_handle, read_buf.get(), MAX_RESULT_LENGTH, 0);
      if (len < 0) {
        throw std::runtime_error{"tcp_cmd recv(): " +
                                 std::string(std::strerror(errno))};
      }
      read_buf.get()[len] = '\0';
      read_ss << read_buf.get();
    } while (len > 0 && read_buf.get()[len - 1] != '\n');

    auto res = read_ss.str();
    res.erase(res.find_last_not_of(" \r\n\t") + 1);

    return res;
  }

  [[nodiscard]] inline Json::Value
  tcp_cmd_json(const std::vector<std::string> &cmd_tokens,
               bool exception_on_parse_errors = true) const
  {
    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root;
    auto result = tcp_cmd(cmd_tokens);
    auto success = reader->parse(result.c_str(), result.c_str() + result.size(),
                                 &root, nullptr);
    if (success) return root;
    if (!exception_on_parse_errors) return result;

    throw std::runtime_error("SensorTcp::tcp_cmd_json failed for " +
                             cmd_tokens[0] +
                             " command. returned json string [" + result +
                             "] couldn't be parsed [");
  }

  inline void
  tcp_cmd_with_validation(const std::vector<std::string> &cmd_tokens,
                          const std::string &validation) const
  {
    auto result = tcp_cmd(cmd_tokens);
    if (result != validation) {
      throw std::runtime_error("SensorTcp::tcp_cmd failed: " + cmd_tokens[0] +
                               " command returned [" + result +
                               "], expected [" + validation + "]");
    }
  }

  int socket_handle;
  std::unique_ptr<char[]> read_buf;
};

}// namespace impl
}// namespace sensor
}// namespace ouster

#endif// ROS2_OUSTER__CLIENT__IMPL_TCP_HPP_
