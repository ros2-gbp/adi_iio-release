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

#ifndef ADI_IIO__IIO_NODE_HPP_
#define ADI_IIO__IIO_NODE_HPP_

#include <iio.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "adi_iio/srv/attr_read_string.hpp"
#include "adi_iio/srv/attr_write_string.hpp"
#include "adi_iio/srv/attr_enable_topic.hpp"
#include "adi_iio/srv/attr_disable_topic.hpp"
#include "adi_iio/srv/buffer_create.hpp"
#include "adi_iio/srv/buffer_destroy.hpp"
#include "adi_iio/srv/buffer_refill.hpp"
#include "adi_iio/srv/buffer_read.hpp"
#include "adi_iio/srv/buffer_write.hpp"
#include "adi_iio/srv/buffer_enable_topic.hpp"
#include "adi_iio/srv/buffer_disable_topic.hpp"
#include "adi_iio/srv/list_devices.hpp"
#include "adi_iio/srv/list_channels.hpp"
#include "adi_iio/srv/list_attributes.hpp"
#include "adi_iio/srv/scan_context.hpp"

class IIOAttrTopic;
class IIOBuffer;

class IIONode : public rclcpp::Node
{
public:
  IIONode();
  virtual ~IIONode();

  void initBuffers();

  bool rwAttrPath(
    std::string path, std::string & result, bool write = false,
    std::string value = "");

  // service handlers
  void attrReadSrv(
    const std::shared_ptr<adi_iio::srv::AttrReadString::Request> request,
    std::shared_ptr<adi_iio::srv::AttrReadString::Response> response);

  void attrWriteSrv(
    const std::shared_ptr<adi_iio::srv::AttrWriteString::Request> request,
    std::shared_ptr<adi_iio::srv::AttrWriteString::Response> response);

  void attrEnableTopicSrv(
    const std::shared_ptr<adi_iio::srv::AttrEnableTopic::Request> request,
    std::shared_ptr<adi_iio::srv::AttrEnableTopic::Response> response);

  void attrDisableTopicSrv(
    const std::shared_ptr<adi_iio::srv::AttrDisableTopic::Request> request,
    std::shared_ptr<adi_iio::srv::AttrDisableTopic::Response> response);

  void buffRefillSrv(
    const std::shared_ptr<adi_iio::srv::BufferRefill::Request> request,
    std::shared_ptr<adi_iio::srv::BufferRefill::Response> response);

  void buffReadSrv(
    const std::shared_ptr<adi_iio::srv::BufferRead::Request> request,
    std::shared_ptr<adi_iio::srv::BufferRead::Response> response);

  void buffWriteSrv(
    const std::shared_ptr<adi_iio::srv::BufferWrite::Request> request,
    std::shared_ptr<adi_iio::srv::BufferWrite::Response> response);

  void buffCreateSrv(
    const std::shared_ptr<adi_iio::srv::BufferCreate::Request> request,
    std::shared_ptr<adi_iio::srv::BufferCreate::Response> response);

  void buffDestroySrv(
    const std::shared_ptr<adi_iio::srv::BufferDestroy::Request> request,
    std::shared_ptr<adi_iio::srv::BufferDestroy::Response> response);

  void buffEnableTopicSrv(
    const std::shared_ptr<adi_iio::srv::BufferEnableTopic::Request> request,
    std::shared_ptr<adi_iio::srv::BufferEnableTopic::Response> response);

  void buffDisableTopicSrv(
    const std::shared_ptr<adi_iio::srv::BufferDisableTopic::Request> request,
    std::shared_ptr<adi_iio::srv::BufferDisableTopic::Response> response);

  void listDevicesSrv(
    const std::shared_ptr<adi_iio::srv::ListDevices::Request> request,
    std::shared_ptr<adi_iio::srv::ListDevices::Response> response);

  void listChannelsSrv(
    const std::shared_ptr<adi_iio::srv::ListChannels::Request> request,
    std::shared_ptr<adi_iio::srv::ListChannels::Response> response);

  void listAttributesSrv(
    const std::shared_ptr<adi_iio::srv::ListAttributes::Request> request,
    std::shared_ptr<adi_iio::srv::ListAttributes::Response> response);

  void scanContextSrv(
    const std::shared_ptr<adi_iio::srv::ScanContext::Request> request,
    std::shared_ptr<adi_iio::srv::ScanContext::Response> response
  );

  // getters
  std::string uri();
  bool initialized();
  iio_context * ctx();

protected:
  std::vector<iio_device *> getDevices(iio_context * ctx);
  std::vector<iio_channel *> getChannels(iio_device * device);

  template<typename T>
  void setErrorResponse(T & response, const std::string & message)
  {
    RCLCPP_ERROR(rclcpp::get_logger("adi_iio_node"), "%s", message.c_str());
    response->success = false;
    response->message = message;
  }

  template<typename T>
  void setWarningResponse(T & response, const std::string & message)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("adi_iio_node"), "Warning: %s", message.c_str());
    response->success = false;
    response->message = message;
  }

  template<typename T>
  void setSuccessResponse(T & response, const std::string & message)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("adi_iio_node"), "Success: %s", message.c_str());
    response->success = true;
    response->message = message;
  }

private:
  bool m_initialized;
  std::string m_uri;
  int32_t m_timeout;
  iio_context * m_ctx;

  std::map<std::string, std::shared_ptr<IIOAttrTopic>> m_attrTopicMap;
  std::map<std::string, std::shared_ptr<IIOBuffer>> m_bufferMap;
};

#endif  // ADI_IIO__IIO_NODE_HPP_
