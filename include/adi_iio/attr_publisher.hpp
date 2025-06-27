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

#ifndef ADI_IIO__ATTR_PUBLISHER_HPP_
#define ADI_IIO__ATTR_PUBLISHER_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "adi_iio/iio_node.hpp"

#define ATTR_QOS_QUEUE_SIZE 10

#define ATTR_READ_SUFFIX "/read"
#define ATTR_WRITE_SUFFIX "/write"

class AttrPubSub
{
public:
  virtual ~AttrPubSub() {}
  virtual void publish(std::string msg) = 0;
};


// these could be templates ?

class StringPubSub : public AttrPubSub
{
public:
  StringPubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic);
  virtual ~StringPubSub();
  void publish(std::string msg) override;
  void update(const std_msgs::msg::String & msg);

private:
  std::shared_ptr<IIONode> m_nh;
  UpdateCallback * m_updateCallback;
  std::string m_topic;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub;
};

class Int32PubSub : public AttrPubSub
{
public:
  Int32PubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic);
  virtual ~Int32PubSub();
  void publish(std::string msg) override;
  void update(const std_msgs::msg::Int32 & msg);

private:
  std::shared_ptr<IIONode> m_nh;
  UpdateCallback * m_updateCallback;
  std::string m_topic;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_sub;
};

class BoolPubSub : public AttrPubSub
{
public:
  BoolPubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic);
  virtual ~BoolPubSub();
  void publish(std::string msg) override;
  void update(const std_msgs::msg::Bool & msg);

private:
  std::shared_ptr<IIONode> m_nh;
  UpdateCallback * m_updateCallback;
  std::string m_topic;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub;
};


class Float32PubSub : public AttrPubSub
{
public:
  Float32PubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic);
  virtual ~Float32PubSub();
  void publish(std::string msg) override;
  void update(const std_msgs::msg::Float32 & msg);

private:
  std::shared_ptr<IIONode> m_nh;
  UpdateCallback * m_updateCallback;
  std::string m_topic;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_sub;
};

#endif  // ADI_IIO__ATTR_PUBLISHER_HPP_
