// Copyright 2022, Mirko Kugelmeier
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

#ifndef ROS2_OUSTER__RINGBUFFER_HPP_
#define ROS2_OUSTER__RINGBUFFER_HPP_

#include <atomic>
#include <memory>

namespace ros2_ouster
{

/**
 * @class ros2_ouster::RingBuffer
 * @brief A simple circular buffer implementation used for queued processing of incoming lidar / imu data
 */
class RingBuffer
{
public:
  /**
   * Create a new RingBuffer instance
   * @param element_size The size (in bytes) of each element
   * @param num_elements Number of maximum elements in the buffer.
   */
  RingBuffer(std::size_t element_size, std::size_t num_elements)
    : _element_size(element_size),
      _num_elements(num_elements),
      _head(0),
      _tail(0),
      _buf(new uint8_t[element_size * _num_elements])
  {}

  /**
   * @brief Check whether there is any data to be read from the buffer at head()
   * @return true if there are no elements in the buffer
   */
  bool empty()
  {
    return _head == _tail;
  }

  /**
   * @brief Check if the ringbuffer is full
   * If the buffer is full, adding new data to it will overwrite or corrupt data at the head() position.
   * This function is only safe to call in the producing thread, i.e. no calls to push() may occur simultaneously
   * @return true if the buffer is full
   */
  bool full()
  {
    // Make sure we have a consistent value for the head index here.
    // This makes sure we do not return 'false positives' when the buffer is not actually
    // full due to a race on _head between the conditions.
    bool head = _head;

    // This ringbuffer implementation lets both head and tail loop twice around the actual number of
    // elements in the buffer to encode the difference between the empty() and full() states. The
    // buffer is full if head and tail refer to the same element but on different 'iterations'
    // of the loop
    return ((_tail * 2) % _num_elements) == head && head != _tail;
  }

  /// @return A pointer to the current element to read from
  uint8_t * head()
  {
    return &_buf[(_head % _num_elements) * _element_size];
  }

  /// @return A pointer to the current element to write to
  uint8_t * tail()
  {
    return &_buf[(_tail % _num_elements) * _element_size];
  }

  /// @brief Remove the current head element from the queue
  void pop()
  {
    _head = (_head + 1) % (_num_elements * 2);
  }

  /// @brief Add a new element (with data already filled at the location pointed to by tail())
  /// to the buffer
  void push()
  {
    _tail = (_tail + 1) % (_num_elements * 2);
  }

protected:
  // The size of each individual element in bytes and the max number of elements in the buffer
  const std::size_t _element_size;
  const std::size_t _num_elements;

  // Since the ringbuffer is used across threads that produce / consume the data, we use
  // std::atomic for head and tail to prevent re-ordering of assignments to these variables
  std::atomic<std::size_t> _head;
  std::atomic<std::size_t> _tail;

  // Defining the backing buffer as a unique_ptr gives us our dtor and correct copy/move
  // semantics for free
  std::unique_ptr<uint8_t[]> _buf;
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__RINGBUFFER_HPP_
