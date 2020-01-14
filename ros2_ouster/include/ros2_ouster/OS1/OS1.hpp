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

#ifndef ROS2_OUSTER__OS1__OS1_HPP_
#define ROS2_OUSTER__OS1__OS1_HPP_

#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "arpa/inet.h"
#include "sys/socket.h"
#include "sys/types.h"

#include "jsoncpp/json/json.h"
#include "ros2_ouster/OS1/OS1_packet.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"

namespace OS1
{

using ns = std::chrono::nanoseconds;

struct client
{
  ~client()
  {
    close(lidar_fd);
    close(imu_fd);
  }

  int lidar_fd{7502};
  int imu_fd{7503};
  Json::Value meta;
};

enum lidar_mode
{
  MODE_512x10 = 1,
  MODE_512x20,
  MODE_1024x10,
  MODE_1024x20,
  MODE_2048x10
};

struct version
{
  int16_t major;
  int16_t minor;
  int16_t patch;
};

const std::array<std::pair<lidar_mode, std::string>, 5> lidar_mode_strings = {
  {{MODE_512x10, "512x10"},
    {MODE_512x20, "512x20"},
    {MODE_1024x10, "1024x10"},
    {MODE_1024x20, "1024x20"},
    {MODE_2048x10, "2048x10"}}};

const size_t lidar_packet_bytes = 12608;
const size_t imu_packet_bytes = 48;

const version invalid_version = {0, 0, 0};

/**
 * Minimum supported version
 */
const OS1::version min_version = {1, 9, 0};

inline bool operator==(const version & u, const version & v)
{
  return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

inline bool operator<(const version & u, const version & v)
{
  return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
         (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

inline bool operator<=(const version & u, const version & v)
{
  return u < v || u == v;
}

/**
 * Get string representation of a version
 * @param version
 * @return string representation of the version
 */
inline std::string to_string(version v)
{
  if (v == invalid_version) {
    return "UNKNOWN";
  }

  std::stringstream ss{};
  ss << "v" << v.major << "." << v.minor << "." << v.patch;
  return ss.str();
}

/**
 * Get lidar mode from string
 * @param string
 * @return lidar mode corresponding to the string, or invalid_version on error
 */
inline version version_of_string(const std::string & s)
{
  std::istringstream is{s};
  char c1, c2, c3;
  version v;

  is >> c1 >> v.major >> c2 >> v.minor >> c3 >> v.patch;

  if (is && is.eof() && c1 == 'v' && c2 == '.' && c3 == '.' && v.major >= 0 &&
    v.minor >= 0 && v.patch >= 0)
  {
    return v;
  } else {
    return invalid_version;
  }
}

/**
 * Get string representation of a lidar mode
 * @param lidar_mode
 * @return string representation of the lidar mode, or "UNKNOWN"
 */
inline std::string to_string(lidar_mode mode)
{
  auto end = lidar_mode_strings.end();
  auto res = std::find_if(lidar_mode_strings.begin(), end,
      [&](const std::pair<lidar_mode, std::string> & p) {
        return p.first == mode;
      });

  return res == end ? "UNKNOWN" : res->second;
}

/**
 * Get lidar mode from string
 * @param string
 * @return lidar mode corresponding to the string, or 0 on error
 */
inline lidar_mode lidar_mode_of_string(const std::string & s)
{
  auto end = lidar_mode_strings.end();
  auto res = std::find_if(lidar_mode_strings.begin(), end,
      [&](const std::pair<lidar_mode, std::string> & p) {
        return p.second == s;
      });

  return res == end ? lidar_mode(0) : res->first;
}

/**
 * Get number of columns in a scan for a lidar mode
 * @param lidar_mode
 * @return number of columns per rotation for the mode
 */
inline int n_cols_of_lidar_mode(lidar_mode mode)
{
  switch (mode) {
    case MODE_512x10:
    case MODE_512x20:
      return 512;
    case MODE_1024x10:
    case MODE_1024x20:
      return 1024;
    case MODE_2048x10:
      return 2048;
    default:
      throw std::invalid_argument{"n_cols_of_lidar_mode"};
  }
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param port port on which the sensor will receive lidar data
 * @return int of socket address
 */
inline int udp_data_socket(int port)
{
  struct addrinfo hints, * info_start, * ai;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET6;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  auto port_s = std::to_string(port);

  int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
  if (ret != 0) {
    std::cerr << "getaddrinfo(): " << gai_strerror(ret) << std::endl;
    return -1;
  }
  if (info_start == NULL) {
    std::cerr << "getaddrinfo: empty result" << std::endl;
    return -1;
  }

  int sock_fd;
  for (ai = info_start; ai != NULL; ai = ai->ai_next) {
    sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock_fd < 0) {
      std::cerr << "udp socket(): " << std::strerror(errno) << std::endl;
      continue;
    }

    if (bind(sock_fd, ai->ai_addr, ai->ai_addrlen) < 0) {
      close(sock_fd);
      std::cerr << "udp bind(): " << std::strerror(errno) << std::endl;
      continue;
    }

    break;
  }

  freeaddrinfo(info_start);
  if (ai == NULL) {
    close(sock_fd);
    return -1;
  }

  if (fcntl(sock_fd, F_SETFL, fcntl(sock_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
    std::cerr << "udp fcntl(): " << std::strerror(errno) << std::endl;
    return -1;
  }

  return sock_fd;
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
inline std::shared_ptr<client> init_client(int lidar_port = 7502, int imu_port = 7503)
{
  auto cli = std::make_shared<client>();

  int lidar_fd = udp_data_socket(lidar_port);
  int imu_fd = udp_data_socket(imu_port);
  cli->lidar_fd = lidar_fd;
  cli->imu_fd = imu_fd;
  return cli;
}

/**
 * Update json object values
 * @param dst Destination json value
 * @param src Source json value
 */
inline void update_json_obj(Json::Value & dst, const Json::Value & src)
{
  for (const auto & key : src.getMemberNames()) {
    dst[key] = src[key];
  }
}

/**
 * Conduct a specific TCP socket command
 * @param sock_fd Socket for connections iwth data
 * @param cmd_tokens command tokens
 * @param res result string of packet data
 * @return bool If valid tcp command
 */
inline bool do_tcp_cmd(int sock_fd, const std::vector<std::string> & cmd_tokens, std::string & res)
{
  const size_t max_res_len = 16 * 1024;
  auto read_buf = std::unique_ptr<char[]>{new char[max_res_len + 1]};

  std::stringstream ss;
  for (const auto & token : cmd_tokens) {
    ss << token << " ";
  }
  ss << "\n";
  std::string cmd = ss.str();

  ssize_t len = write(sock_fd, cmd.c_str(), cmd.length());
  if (len != (ssize_t)cmd.length()) {
    return false;
  }

  // need to synchronize with server by reading response
  std::stringstream read_ss;
  do {
    len = read(sock_fd, read_buf.get(), max_res_len);
    if (len < 0) {
      return false;
    }
    read_buf.get()[len] = '\0';
    read_ss << read_buf.get();
  } while (len > 0 && read_buf.get()[len - 1] != '\n');

  res = read_ss.str();
  res.erase(res.find_last_not_of(" \r\n\t") + 1);

  return true;
}

/**
 * Configure socket
 * @param addr address of socket to reconfigure
 * @return int The socket ID number
 */
inline int cfg_socket(const char * addr)
{
  struct addrinfo hints, * info_start, * ai;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  int ret = getaddrinfo(addr, "7501", &hints, &info_start);
  if (ret != 0) {
    std::cerr << "getaddrinfo: " << gai_strerror(ret) << std::endl;
    return -1;
  }
  if (info_start == NULL) {
    std::cerr << "getaddrinfo: empty result" << std::endl;
    return -1;
  }

  int sock_fd;
  for (ai = info_start; ai != NULL; ai = ai->ai_next) {
    sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock_fd < 0) {
      std::cerr << "socket: " << std::strerror(errno) << std::endl;
      continue;
    }

    if (connect(sock_fd, ai->ai_addr, ai->ai_addrlen) == -1) {
      close(sock_fd);
      continue;
    }

    break;
  }

  freeaddrinfo(info_start);
  if (ai == NULL) {
    return -1;
  }

  return sock_fd;
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param hostname hostname or ip of the sensor
 * @param udp_dest_host hostname or ip where the sensor should send data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
inline std::shared_ptr<client> init_client(
  const std::string & hostname,
  const std::string & udp_dest_host, lidar_mode mode = MODE_1024x10,
  int lidar_port = 7502,
  int imu_port = 7503)
{
  auto cli = init_client(lidar_port, imu_port);

  int sock_fd = cfg_socket(hostname.c_str());

  Json::CharReaderBuilder builder{};
  auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
  Json::Value root{};
  std::string errors{};

  if (sock_fd < 0) {
    return std::shared_ptr<client>();
  }

  std::string res;
  bool success = true;

  success &=
    do_tcp_cmd(sock_fd, {"set_config_param", "udp_ip", udp_dest_host}, res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(sock_fd, {"set_config_param", "udp_port_lidar",
        std::to_string(lidar_port)}, res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(
    sock_fd, {"set_config_param", "udp_port_imu", std::to_string(imu_port)},
    res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(
    sock_fd, {"set_config_param", "lidar_mode", to_string(mode)}, res);
  success &= res == "set_config_param";

  success &= do_tcp_cmd(sock_fd, {"get_sensor_info"}, res);
  success &= reader->parse(res.c_str(), res.c_str() + res.size(), &cli->meta,
      &errors);

  success &= do_tcp_cmd(sock_fd, {"get_beam_intrinsics"}, res);
  success &=
    reader->parse(res.c_str(), res.c_str() + res.size(), &root, &errors);
  update_json_obj(cli->meta, root);

  success &= do_tcp_cmd(sock_fd, {"get_imu_intrinsics"}, res);
  success &=
    reader->parse(res.c_str(), res.c_str() + res.size(), &root, &errors);
  update_json_obj(cli->meta, root);

  success &= do_tcp_cmd(sock_fd, {"get_lidar_intrinsics"}, res);
  success &=
    reader->parse(res.c_str(), res.c_str() + res.size(), &root, &errors);
  update_json_obj(cli->meta, root);

  success &= do_tcp_cmd(sock_fd, {"reinitialize"}, res);
  success &= res == "reinitialize";

  close(sock_fd);

  // merge extra info into metadata
  cli->meta["hostname"] = hostname;
  cli->meta["lidar_mode"] = to_string(mode);
  cli->meta["imu_port"] = imu_port;
  cli->meta["lidar_port"] = lidar_port;

  return success ? cli : std::shared_ptr<client>();
}


/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec seconds to block while waiting for data
 * @return ClientState s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read
 */
inline ros2_ouster::ClientState poll_client(const client & c, const int timeout_sec = 1)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(c.lidar_fd, &rfds);
  FD_SET(c.imu_fd, &rfds);

  timeval tv;
  tv.tv_sec = timeout_sec;
  tv.tv_usec = 0;

  int max_fd = std::max(c.lidar_fd, c.imu_fd);

  int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);

  if (retval == -1 && errno == EINTR) {
    return ros2_ouster::ClientState::EXIT;
  } else if (retval == -1) {
    std::cerr << "select: " << std::strerror(errno) << std::endl;
    return ros2_ouster::ClientState::ERROR;
  } else if (retval) {
    if (FD_ISSET(c.lidar_fd, &rfds)) {
      return ros2_ouster::ClientState::LIDAR_DATA;
    }
    if (FD_ISSET(c.imu_fd, &rfds)) {
      return ros2_ouster::ClientState::IMU_DATA;
    }
  }
  return ros2_ouster::ClientState::TIMEOUT;
}

/**
 * Read lidar data from the sensor. Will not block.
 * @param fd Socket connection for sensor data
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @param len length of packet
 * @return true if a lidar packet was successfully read
 */
static bool recv_fixed(int fd, void * buf, size_t len)
{
  ssize_t n = recvfrom(fd, buf, len + 1, 0, NULL, NULL);
  if (n == (ssize_t)len) {
    return true;
  } else if (n == -1) {
    std::cerr << "recvfrom: " << std::strerror(errno) << std::endl;
  } else {
    std::cerr << "Unexpected udp packet length: " << n << std::endl;
  }
  return false;
}

/**
 * Read lidar data from the sensor. Will not block.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @return true if a lidar packet was successfully read
 */
inline bool read_lidar_packet(const client & cli, uint8_t * buf)
{
  return recv_fixed(cli.lidar_fd, buf, lidar_packet_bytes);
}

/**
 * Read imu data from the sensor. Will not block.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes
 * @return true if an imu packet was successfully read
 */
inline bool read_imu_packet(const client & cli, uint8_t * buf)
{
  return recv_fixed(cli.imu_fd, buf, imu_packet_bytes);
}

/**
 * Get metadata text blob from the sensor
 * @param cli client returned by init_client associated with the connection
 * @return a text blob of metadata parseable into a sensor_info struct
 */
inline std::string get_metadata(const client & cli)
{
  Json::StreamWriterBuilder builder;
  builder["enableYAMLCompatibility"] = true;
  builder["precision"] = 6;
  builder["indentation"] = "    ";
  return Json::writeString(builder, cli.meta);
}

/**
 * Parse metadata text blob from the sensor into a sensor_info struct. String
 * and vector fields will have size 0 if the parameter cannot be found or
 * parsed,
 * while lidar_mode will be set to 0 (invalid).
 * @throw runtime_error if the text is not valid json
 * @param metadata a text blob returned by get_metadata above
 * @return a sensor_info struct populated with a subset of the metadata
 */
inline ros2_ouster::Metadata parse_metadata(const std::string & meta)
{
  Json::Value root{};
  Json::CharReaderBuilder builder{};
  std::string errors{};
  std::stringstream ss{meta};

  if (meta.size()) {
    if (!Json::parseFromStream(builder, ss, &root, &errors)) {
      throw std::runtime_error{errors.c_str()};
    }
  }

  ros2_ouster::Metadata info = {"UNKNOWN", "UNKNOWN", "UNNKOWN", "UNNKOWN",
    {}, {}, {}, {}, 7503, 7502};
  info.hostname = root["hostname"].asString();
  info.sn = root["prod_sn"].asString();
  info.fw_rev = root["build_rev"].asString();

  info.mode = root["lidar_mode"].asString();
  info.lidar_port = root["lidar_port"].asInt();
  info.imu_port = root["lidar_port"].asInt();

  for (const auto & v : root["beam_altitude_angles"]) {
    info.beam_altitude_angles.push_back(v.asDouble());
  }
  if (info.beam_altitude_angles.size() != OS1::pixels_per_column) {
    info.beam_altitude_angles = {};
  }

  for (const auto & v : root["beam_azimuth_angles"]) {
    info.beam_azimuth_angles.push_back(v.asDouble());
  }
  if (info.beam_azimuth_angles.size() != OS1::pixels_per_column) {
    info.beam_azimuth_angles = {};
  }

  for (const auto & v : root["imu_to_sensor_transform"]) {
    info.imu_to_sensor_transform.push_back(v.asDouble());
  }
  if (info.imu_to_sensor_transform.size() != 16) {
    info.imu_to_sensor_transform = {};
  }

  for (const auto & v : root["lidar_to_sensor_transform"]) {
    info.lidar_to_sensor_transform.push_back(v.asDouble());
  }
  if (info.lidar_to_sensor_transform.size() != 16) {
    info.lidar_to_sensor_transform = {};
  }

  return info;
}

}  // namespace OS1

#endif  // ROS2_OUSTER__OS1__OS1_HPP_
