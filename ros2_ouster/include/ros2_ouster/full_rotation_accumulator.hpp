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

#ifndef ROS2_OUSTER__FULL_ROTATION_ACCUMULATOR_HPP_
#define ROS2_OUSTER__FULL_ROTATION_ACCUMULATOR_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_ouster/exception.hpp"

namespace sensor
{
/**
 * @class sensor::FullRotationAccumulator
 * @brief The FullRotationAccumulator creates LidarScan for the processors from
 * packet data.
 */
class FullRotationAccumulator
{
public:
  FullRotationAccumulator(
    const ouster::sensor::sensor_info & mdata,
    const ouster::sensor::packet_format & pf)
  : _batchReady(false), _pf(pf), _packets_accumulated(0)
  {
    _batch = std::make_unique<ouster::ScanBatcher>(mdata.format.columns_per_frame, _pf);
    _ls = std::make_shared<ouster::LidarScan>(
      ouster::LidarScan{mdata.format.columns_per_frame,
        mdata.format.pixels_per_column});
  }

  /**
   * @brief Returns true if the lidarscan is ready
   */
  bool isBatchReady()
  {
    return _batchReady;
  }

  /**
   * @brief Returns the ready lidarscan. If the lidarscan is not ready it will
   * throw an exception
   */
  std::shared_ptr<ouster::LidarScan> getLidarScan()
  {
    if (!_batchReady) {
      throw ros2_ouster::OusterDriverException("Full rotation not accumulated.");
    }

    return _ls;
  }

  /**
   * @brief Returns the ready timestamp. If the timestamp is not ready it will
   * throw an exception
   */
  std::chrono::nanoseconds getTimestamp()
  {
    if (!_batchReady) {
      throw ros2_ouster::OusterDriverException("Full rotation not accumulated.");
    }

    return _timestamp;
  }

  /**
   * @brief Takes packet data to batch it into a lidarscan
   */
  void accumulate(const uint8_t * data, uint64_t override_ts)
  {
    if (_batchReady) {
      _batchReady = false;
      _packets_accumulated = 0;
    }

    _packets_accumulated++;

    if (_batch->operator()(data, *_ls)) {
      auto h = std::find_if(
        _ls->headers.begin(), _ls->headers.end(), [](const auto & h) {
          return h.timestamp != std::chrono::nanoseconds{0};
        });
      if (h != _ls->headers.end()) {
        _timestamp = h->timestamp;
      }
      _batchReady = true;
    }
  }

  uint64_t getPacketsAccumulated()
  {
    return _packets_accumulated;
  }

private:
  bool _batchReady;
  std::chrono::nanoseconds _timestamp;
  std::unique_ptr<ouster::ScanBatcher> _batch;
  std::shared_ptr<ouster::LidarScan> _ls;
  ouster::sensor::packet_format _pf;

  uint64_t _packets_accumulated = 0;
};

}  // namespace sensor

#endif  // ROS2_OUSTER__FULL_ROTATION_ACCUMULATOR_HPP_
