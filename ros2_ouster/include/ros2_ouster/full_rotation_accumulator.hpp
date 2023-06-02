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
    ouster::sensor::packet_format  pf)
  : _batchReady(false), _pf(std::move(pf)), _packets_accumulated(0)
  {
    _batch = std::make_unique<ouster::ScanBatcher>(mdata.format.columns_per_frame, _pf);
    _ls = std::make_shared<ouster::LidarScan>(
      ouster::LidarScan{mdata.format.columns_per_frame,
        mdata.format.pixels_per_column});

    _compute_scan_ts = [this](const auto& ts_v) {
      return compute_scan_ts_0(ts_v);
    };
  }

  /**
   * @brief Returns true if the lidarscan is ready
   */
  bool isBatchReady() const
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
      _timestamp = std::chrono::nanoseconds(_compute_scan_ts(_ls->timestamp()));
      _batchReady = true;
    }
  }

  uint64_t getPacketsAccumulated() const
  {
    return _packets_accumulated;
  }

private:
    template <typename T, typename UnaryPredicate>
    int find_if_reverse(const Eigen::Array<T, -1, 1>& array,
                        UnaryPredicate predicate) {
    auto p = array.data() + array.size() - 1;
    do {
      if (predicate(*p)) return p - array.data();
    } while (p-- != array.data());
    return -1;
    }

    template <typename T>
    uint64_t ulround(T value) {
    T rounded_value = std::round(value);
    if (rounded_value < 0) return 0ULL;
    if (rounded_value > ULLONG_MAX) return ULLONG_MAX;
    return static_cast<uint64_t>(rounded_value);
    }

    static uint64_t linear_interpolate(int x0, uint64_t y0, int x1, uint64_t y1, int x) {
    uint64_t min_v, max_v;
    double sign;
    if (y1 > y0) {
      min_v = y0;
      max_v = y1;
      sign = +1;
    } else {
      min_v = y1;
      max_v = y0;
      sign = -1;
    }
    return y0 + (x - x0) * sign * (max_v - min_v) / (x1 - x0);
    }

    uint64_t impute_value(int last_scan_last_nonzero_idx,
                          uint64_t last_scan_last_nonzero_value,
                          int curr_scan_first_nonzero_idx,
                          uint64_t curr_scan_first_nonzero_value,
                          int scan_width) {
    assert(scan_width + curr_scan_first_nonzero_idx >
           last_scan_last_nonzero_idx);
    double interpolated_value = linear_interpolate(
            last_scan_last_nonzero_idx, last_scan_last_nonzero_value,
            scan_width + curr_scan_first_nonzero_idx,
            curr_scan_first_nonzero_value, scan_width);
    return ulround(interpolated_value);
    }

    uint64_t extrapolate_value(int curr_scan_first_nonzero_idx,
                               uint64_t curr_scan_first_nonzero_value) {
    double extrapolated_value =
            curr_scan_first_nonzero_value -
            _scan_col_ts_spacing_ns * curr_scan_first_nonzero_idx;
    return ulround(extrapolated_value);
    }

    // compute_scan_ts_0 for first scan
    uint64_t compute_scan_ts_0(
            const ouster::LidarScan::Header<uint64_t>& ts_v) {
    auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                            [](uint64_t h) { return h != 0; });
    assert(idx != ts_v.data() + ts_v.size());  // should never happen
    int curr_scan_first_nonzero_idx = idx - ts_v.data();
    uint64_t curr_scan_first_nonzero_value = *idx;

    uint64_t scan_ns =
            curr_scan_first_nonzero_idx == 0
                    ? curr_scan_first_nonzero_value
                    : extrapolate_value(curr_scan_first_nonzero_idx,
                                        curr_scan_first_nonzero_value);

    _last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
    assert(_last_scan_last_nonzero_idx >= 0);  // should never happen
    _last_scan_last_nonzero_value = ts_v(_last_scan_last_nonzero_idx);
    _compute_scan_ts = [this](const auto& ts_v) {
      return compute_scan_ts_n(ts_v);
    };
    return scan_ns;
    }

    // compute_scan_ts_n applied to all subsequent scans except first one
    uint64_t compute_scan_ts_n(
            const ouster::LidarScan::Header<uint64_t>& ts_v) {
    auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                            [](uint64_t h) { return h != 0; });
    assert(idx != ts_v.data() + ts_v.size());  // should never happen
    int curr_scan_first_nonzero_idx = idx - ts_v.data();
    uint64_t curr_scan_first_nonzero_value = *idx;

    uint64_t scan_ns = curr_scan_first_nonzero_idx == 0
                               ? curr_scan_first_nonzero_value
                               : impute_value(_last_scan_last_nonzero_idx,
                                              _last_scan_last_nonzero_value,
                                              curr_scan_first_nonzero_idx,
                                              curr_scan_first_nonzero_value,
                                              static_cast<int>(ts_v.size()));

    _last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
    assert(_last_scan_last_nonzero_idx >= 0);  // should never happen
    _last_scan_last_nonzero_value = ts_v(_last_scan_last_nonzero_idx);
    return scan_ns;
    }

  bool _batchReady;
  std::chrono::nanoseconds _timestamp{};
  std::unique_ptr<ouster::ScanBatcher> _batch;
  std::shared_ptr<ouster::LidarScan> _ls;
  ouster::sensor::packet_format _pf;

  uint64_t _packets_accumulated = 0;

  std::function<uint64_t(const ouster::LidarScan::Header<uint64_t>&)>
          _compute_scan_ts;
  int _last_scan_last_nonzero_idx = -1;
  uint64_t _last_scan_last_nonzero_value = 0;
  double _scan_col_ts_spacing_ns{};  // interval or spacing between columns of a
                                // scan
};

}  // namespace sensor

#endif  // ROS2_OUSTER__FULL_ROTATION_ACCUMULATOR_HPP_
