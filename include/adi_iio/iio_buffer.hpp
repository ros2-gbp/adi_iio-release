// Copyright 2025 Analog Devices, Inc.
//
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

#ifndef ADI_IIO__IIO_BUFFER_HPP_
#define ADI_IIO__IIO_BUFFER_HPP_

#include <iio.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "adi_iio/iio_node.hpp"

#define BUFFER_QOS_QUEUE_SIZE 10

#define BUFFER_READ_SUFFIX ""
#define BUFFER_WRITE_SUFFIX ""

class IIOBuffer
{
public:
  IIOBuffer(std::shared_ptr<IIONode> nh, std::string device_path);
  ~IIOBuffer();

  bool createIIOBuffer(std::string & message, bool output = false, bool cyclic = false);
  void destroyIIOBuffer();
  bool refill(std::string & message);
  bool push(std::string & message, std_msgs::msg::Int32MultiArray & data);

  void enableTopic(std::string topic_name, double loopRate);
  void disableTopic();
  void publishingLoop();

  int32_t samples_count() {return m_samples_count;}
  std::string device_path() {return m_device_path;}
  std::vector<std::string> channels() {return m_channels;}
  iio_buffer * buffer() {return m_buffer;}
  bool topic_enabled() {return m_topic_enabled;}
  std_msgs::msg::Int32MultiArray & data()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data;
  }

  void set_samples_count(int32_t samples_count) {m_samples_count = samples_count;}
  void set_device_path(std::string device_path) {m_device_path = device_path;}
  void set_channels(std::vector<std::string> channels) {m_channels = channels;}

private:
  std::shared_ptr<IIONode> m_nh;
  std::string m_device_path;
  std::vector<std::string> m_channels;
  int32_t m_samples_count;
  iio_buffer * m_buffer;

  std_msgs::msg::Int32MultiArray m_data;

  bool m_topic_enabled;
  std::string m_topic_name;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr m_pub;
  std::thread m_th;
  bool m_canceled;
  bool m_stopThread;
  std::mutex m_mutex;
  std::shared_ptr<rclcpp::Rate> m_loopRate;
};

#endif  // ADI_IIO__IIO_BUFFER_HPP_
