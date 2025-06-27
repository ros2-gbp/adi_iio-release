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

#include "adi_iio/iio_attr_topic.hpp"
#include "adi_iio/attr_publisher.hpp"

using std::placeholders::_1;

StringPubSub::StringPubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic)
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Created String publisher ");
  m_topic = topic;
  m_nh = nh;
  m_updateCallback = up;
  m_pub = m_nh->create_publisher<std_msgs::msg::String>(
    topic + ATTR_READ_SUFFIX,
    ATTR_QOS_QUEUE_SIZE);
  m_sub = m_nh->create_subscription<std_msgs::msg::String>(
    topic + ATTR_WRITE_SUFFIX, ATTR_QOS_QUEUE_SIZE, std::bind(&StringPubSub::update, this, _1));
}

StringPubSub::~StringPubSub()
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Deleted String publisher ");
  // shared ptr lifetime should delete this publisher automatically
}

void StringPubSub::publish(std::string msg)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("adi_iio_node"), "Publishing msg %s on topic %s",
    msg.c_str(), m_topic.c_str());
  auto message = std_msgs::msg::String();
  message.data = msg;
  m_pub->publish(message);
}

void StringPubSub::update(const std_msgs::msg::String & msg)
{
  m_updateCallback->update(msg.data);
}

Int32PubSub::Int32PubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic)
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Created Int32 publisher ");
  m_topic = topic;
  m_nh = nh;
  m_updateCallback = up;
  m_pub =
    m_nh->create_publisher<std_msgs::msg::Int32>(topic + ATTR_READ_SUFFIX, ATTR_QOS_QUEUE_SIZE);
  m_sub = m_nh->create_subscription<std_msgs::msg::Int32>(
    topic + ATTR_WRITE_SUFFIX, ATTR_QOS_QUEUE_SIZE, std::bind(&Int32PubSub::update, this, _1));
}

Int32PubSub::~Int32PubSub()
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Deleted Int32 publisher ");
  // shared ptr lifetime should delete this publisher automatically
}

void Int32PubSub::publish(std::string msg)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("adi_iio_node"), "Publishing msg %s on topic %s",
    msg.c_str(), m_topic.c_str());
  auto message = std_msgs::msg::Int32();
  int32_t data = stoi(msg);
  message.data = data;

  m_pub->publish(message);
}

void Int32PubSub::update(const std_msgs::msg::Int32 & msg)
{
  std::string strData = std::to_string(msg.data);
  m_updateCallback->update(strData);
}


BoolPubSub::BoolPubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic)
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Created Bool publisher ");
  m_topic = topic;
  m_nh = nh;
  m_updateCallback = up;
  m_pub =
    m_nh->create_publisher<std_msgs::msg::Bool>(topic + ATTR_READ_SUFFIX, ATTR_QOS_QUEUE_SIZE);
  m_sub = m_nh->create_subscription<std_msgs::msg::Bool>(
    topic + ATTR_WRITE_SUFFIX, ATTR_QOS_QUEUE_SIZE, std::bind(&BoolPubSub::update, this, _1));
}

BoolPubSub::~BoolPubSub()
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Deleted Bool publisher ");
  // shared ptr lifetime should delete this publisher automatically
}

void BoolPubSub::publish(std::string msg)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("adi_iio_node"), "Publishing msg %s on topic %s",
    msg.c_str(), m_topic.c_str());
  auto message = std_msgs::msg::Bool();
  bool data = stoi(msg);
  message.data = data;

  m_pub->publish(message);
}

void BoolPubSub::update(const std_msgs::msg::Bool & msg)
{
  std::string strData = std::to_string(msg.data);
  m_updateCallback->update(strData);
}


Float32PubSub::Float32PubSub(std::shared_ptr<IIONode> nh, UpdateCallback * up, std::string topic)
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Created Float32 publisher ");
  m_topic = topic;
  m_nh = nh;
  m_updateCallback = up;
  m_pub = m_nh->create_publisher<std_msgs::msg::Float32>(
    topic + ATTR_READ_SUFFIX,
    ATTR_QOS_QUEUE_SIZE);
  m_sub = m_nh->create_subscription<std_msgs::msg::Float32>(
    topic + ATTR_WRITE_SUFFIX, ATTR_QOS_QUEUE_SIZE, std::bind(&Float32PubSub::update, this, _1));
}

Float32PubSub::~Float32PubSub()
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Deleted Float32 publisher ");
  // shared ptr lifetime should delete this publisher automatically
}

void Float32PubSub::publish(std::string msg)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("adi_iio_node"), "Publishing msg %s on topic %s",
    msg.c_str(), m_topic.c_str());
  auto message = std_msgs::msg::Float32();
  bool data = stoi(msg);
  message.data = data;

  m_pub->publish(message);
}

void Float32PubSub::update(const std_msgs::msg::Float32 & msg)
{
  std::string strData = std::to_string(msg.data);
  m_updateCallback->update(strData);
}
