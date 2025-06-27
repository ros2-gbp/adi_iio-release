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

#ifndef ADI_IIO__IIO_ATTR_TOPIC_HPP_
#define ADI_IIO__IIO_ATTR_TOPIC_HPP_

#include <iio.h>

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class IIONode;
class AttrPubSub;

class UpdateCallback
{
public:
  virtual ~UpdateCallback() {}
  virtual void update(std::string msg) = 0;
};

class IIOAttrTopic : public UpdateCallback
{
public:
  typedef enum
  {
    TYPE_STRING,
    TYPE_INT,
    TYPE_DOUBLE,
    TYPE_BOOL
  } topicType_t;

  IIOAttrTopic(
    std::shared_ptr<IIONode> nh, std::string topicName, std::string attrPath,
    topicType_t type, double loopRate);
  virtual ~IIOAttrTopic();

  void publishingLoop();
  void update(std::string msg) override;

private:
  std::mutex m_mutex;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<IIONode> m_nh;
  std::string m_attrPath;
  std::string m_topicName;
  topicType_t m_topicType;
  std::unique_ptr<AttrPubSub> m_pub;

  std::thread m_th;
  std::string last_val;
  bool m_stopThread;
  rclcpp::Rate m_loopRate;
};

#endif  // ADI_IIO__IIO_ATTR_TOPIC_HPP_
