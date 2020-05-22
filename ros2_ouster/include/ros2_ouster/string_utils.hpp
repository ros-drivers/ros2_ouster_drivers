// Copyright 2020, Box Robotics, Inc.
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

#ifndef ROS2_OUSTER__STRING_UTILS_HPP_
#define ROS2_OUSTER__STRING_UTILS_HPP_

#include <sstream>
#include <string>
#include <vector>

namespace ros2_ouster
{

/**
 * Trims whitespace from the left side of a string
 *
 * @param str The string to trim
 * @param white Whitespace characters to erase
 *
 * @return A reference to the trimmed string
 */
inline std::string &
ltrim(std::string & str, const std::string & white = "\t\n\v\f\r ")
{
  str.erase(0, str.find_first_not_of(white));
  return str;
}

/**
 * Trims whitespace from the right side of a string
 *
 * @param str The string to trim
 * @param white Whitespace characters to erase
 *
 * @return A reference to the trimmed string
 */
inline std::string &
rtrim(std::string & str, const std::string & white = "\t\n\v\f\r ")
{
  str.erase(str.find_last_not_of(white) + 1);
  return str;
}

/**
 * Trims whitespace from the front and back of a string
 *
 * @param str The string to trim
 * @param white Whitespace characters to erase
 *
 * @return A reference to the trimmed string
 */
inline std::string &
trim(std::string & str, const std::string & white = "\t\n\v\f\r ")
{
  return ros2_ouster::ltrim(ros2_ouster::rtrim(str, white), white);
}

/**
 * Tokenizes a string based on a char delimeter
 *
 * @param[in] in The string to tokenize
 * @param[in] delim The char delimeter to split the string on
 *
 * @return A vector of tokens
 */
inline std::vector<std::string>
split(const std::string & in, char delim)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream ss(in);

  while (std::getline(ss, token, delim)) {
    tokens.push_back(token);
  }

  return tokens;
}

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__STRING_UTILS_HPP_
