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

#include "adi_iio/iio_node.hpp"
#include "adi_iio/iio_path.hpp"
#include "adi_iio/iio_attr_topic.hpp"
#include "adi_iio/iio_buffer.hpp"
#include <memory>

using namespace std::chrono_literals;

#define MAX_ATTR_SIZE 4095

IIONode::IIONode()
: Node("adi_iio_node")
{
  m_initialized = false;
  this->declare_parameter<std::string>("uri", "local:");
  this->declare_parameter<int32_t>("timeout", 0);
  m_uri = this->get_parameter("uri").as_string();
  m_timeout = this->get_parameter("timeout").as_int();

  m_ctx = iio_create_context_from_uri(m_uri.c_str());

  if (m_ctx != nullptr) {
    RCLCPP_INFO(
      rclcpp::get_logger("adi_iio_node"),
      "creating context %p from uri %s",
      (void *)m_ctx, m_uri.c_str());
    m_initialized = true;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("adi_iio_node"),
      "cannot create context from uri %s", m_uri.c_str());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"),
    "setting timeout to %d", m_timeout);
  iio_context_set_timeout(m_ctx, m_timeout);
}

void IIONode::initBuffers()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing buffers...");

  auto devices_ptr = getDevices(ctx());
  for (iio_device * dev : devices_ptr) {
    const char * dev_name_cstr = iio_device_get_name(dev);

    if (dev_name_cstr == nullptr) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "device name is null, skipping buffer initialization");
      continue;
    }

    std::string dev_name = std::string(dev_name_cstr);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Parsing channels of device: %s", dev_name.c_str());

    auto channels_ptr = getChannels(dev);
    for (iio_channel * chn : channels_ptr) {
      if (iio_channel_is_scan_element(chn)) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Inserting %s into bufferMap", dev_name.c_str());
        m_bufferMap.insert(
          {dev_name, std::make_shared<IIOBuffer>(
              std::dynamic_pointer_cast<IIONode>(shared_from_this()), dev_name)});
        break;
      }
    }
  }
}

IIONode::~IIONode()
{
}

bool IIONode::initialized()
{
  return m_initialized;
}

bool IIONode::rwAttrPath(std::string path, std::string & result, bool write, std::string value)
{
  bool ret = false;
  IIOPath iio_path(path);

  iio_device * dev = nullptr;
  iio_channel * ch = nullptr;

  char * val;
  char attr_val[MAX_ATTR_SIZE];
  int ret1;

  if (iio_path.isValid(CONTEXT_ATTR)) {
    val =
      const_cast<char *>(iio_context_get_attr_value(
        m_ctx,
        iio_path.getContextAttrSegment().c_str()));
    if (val) {
      ret = true;
      result = val;
      RCLCPP_DEBUG(
        rclcpp::get_logger("adi_iio_node"),
        "read context attribute \"%s\" with value \"%s\"",
        iio_path.getContextAttrSegment().c_str(), val);
    } else {
      ret = false;
      result = "";
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"),
        "could not find context attribute \"%s\"", iio_path.getContextAttrSegment().c_str());
    }

    if (write) {
      ret = false;
      RCLCPP_WARN(rclcpp::get_logger("adi_iio_node"), "context attributes cannot be written");
    }
  } else if (iio_path.isValid(DEVICE_ATTR)) {
    dev = iio_context_find_device(m_ctx, iio_path.getDeviceSegment().c_str());
    if (!dev) {
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"), "could not find device \"%s\"",
        iio_path.getDeviceSegment().c_str());
      return ret;
    }

    if (write) {
      ret1 = iio_device_attr_write(dev, iio_path.getDeviceAttrSegment().c_str(), value.c_str());
      if (ret1 <= 0) {
        ret1 = iio_device_debug_attr_write(
          dev, iio_path.getDeviceAttrSegment().c_str(), value.c_str());
      }
      if (ret1 > 0) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("adi_iio_node"),
          "wrote device attribute \"%s\" from device \"%s\" with value \"%s\"",
          iio_path.getDeviceAttrSegment().c_str(), iio_path.getDeviceSegment().c_str(),
          value.c_str());
      } else {
        result = strerror(-ret1);
        ret = false;
        RCLCPP_WARN(
          rclcpp::get_logger("adi_iio_node"),
          "could not write attribute \"%s\" in device \"%s\" with value \"%s\" - errno %d - %s",
          iio_path.getDeviceAttrSegment().c_str(), iio_path.getDeviceSegment().c_str(),
          value.c_str(), ret1, result.c_str());
        return ret;
      }
    }

    ret1 = iio_device_attr_read(
      dev, iio_path.getDeviceAttrSegment().c_str(), attr_val, MAX_ATTR_SIZE);
    if (ret1 <= 0) {
      ret1 = iio_device_debug_attr_read(
        dev, iio_path.getDeviceAttrSegment().c_str(), attr_val, MAX_ATTR_SIZE
      );
    }

    if (ret1 > 0) {
      result = attr_val;
      ret = true;
      RCLCPP_DEBUG(
        rclcpp::get_logger("adi_iio_node"),
        "read device attribute \"%s\" from device \"%s\" with value \"%s\"",
        iio_path.getDeviceAttrSegment().c_str(), iio_path.getDeviceSegment().c_str(), attr_val);
    } else {
      result = strerror(-ret1);
      ret = false;
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"),
        "could not read attribute \"%s\" in device \"%s\" - errno %d - %s",
        iio_path.getDeviceAttrSegment().c_str(),
        iio_path.getDeviceSegment().c_str(), ret1, result.c_str());
    }
  } else if (iio_path.isValid(CHANNEL_ATTR)) {
    dev = iio_context_find_device(m_ctx, iio_path.getDeviceSegment().c_str());
    if (!dev) {
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"), "could not find device \"%s\"",
        iio_path.getDeviceSegment().c_str());
      return ret;
    }

    if (iio_path.hasExtendedChannelFormat()) {
      auto [is_output, chn_name] = iio_path.getExtendedChannelSegment();
      ch = iio_device_find_channel(dev, chn_name.c_str(), is_output);
    } else {
      ch = iio_device_find_channel(dev, iio_path.getChannelSegment().c_str(), false);
      if (!ch) {
        ch = iio_device_find_channel(dev, iio_path.getChannelSegment().c_str(), true);
      }
    }

    if (!ch) {
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"), "could not find channel \"%s\"",
        iio_path.getChannelSegment().c_str());
      return ret;
    }

    if (write) {
      ret1 = iio_channel_attr_write(ch, iio_path.getChannelAttrSegment().c_str(), value.c_str());
      if (ret1 > 0) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("adi_iio_node"),
          "wrote channel attribute \"%s\" from channel \"%s\" device \"%s\" with value \"%s\"",
          iio_path.getChannelAttrSegment().c_str(),
          iio_path.getChannelSegment().c_str(), iio_path.getDeviceSegment().c_str(), value.c_str());
      } else {
        result = strerror(-ret1);
        ret = false;
        RCLCPP_WARN(
          rclcpp::get_logger(
            "adi_iio_node"),
          "could not write attribute \"%s\" from channel \"%s\" device \"%s\" "
          "with value \"%s\"- errno %d - %s",
          iio_path.getChannelAttrSegment().c_str(),
          iio_path.getChannelSegment().c_str(),
          iio_path.getDeviceSegment().c_str(),
          value.c_str(),
          ret1,
          result.c_str());
        return ret;
      }
    }

    ret1 = iio_channel_attr_read(
      ch, iio_path.getChannelAttrSegment().c_str(), attr_val, MAX_ATTR_SIZE);
    if (ret1 > 0) {
      result = attr_val;
      ret = true;
      RCLCPP_DEBUG(
        rclcpp::get_logger("adi_iio_node"),
        "read channel attribute \"%s\" from channel \"%s\" device \"%s\" with value \"%s\"",
        iio_path.getChannelAttrSegment().c_str(), iio_path.getChannelSegment().c_str(),
        iio_path.getDeviceSegment().c_str(), attr_val);
    } else {
      result = strerror(-ret1);
      ret = false;
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"),
        "could not read attribute \"%s\" from channel \"%s\" device \"%s\" - errno %d - %s",
        iio_path.getChannelAttrSegment().c_str(), iio_path.getChannelSegment().c_str(),
        iio_path.getDeviceSegment().c_str(), ret1, result.c_str());
    }
  } else {
    result = "Service requires a valid attr_path";
    ret = false;
  }

  return ret;
}

void IIONode::attrReadSrv(
  const std::shared_ptr<adi_iio::srv::AttrReadString::Request> request,   // CHANGE
  std::shared_ptr<adi_iio::srv::AttrReadString::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"), "Service request /AttrReadString : %s",
    request->attr_path.c_str());
  std::string result;
  response->success = rwAttrPath(request->attr_path, result);
  response->message = result;
}

void IIONode::attrWriteSrv(
  const std::shared_ptr<adi_iio::srv::AttrWriteString::Request> request,   // CHANGE
  std::shared_ptr<adi_iio::srv::AttrWriteString::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"), "Service request /AttrWriteString : %s value %s",
    request->attr_path.c_str(), request->value.c_str());
  std::string result;
  response->success = rwAttrPath(request->attr_path, result, true, request->value);
  response->message = result;
}

void IIONode::attrEnableTopicSrv(
  const std::shared_ptr<adi_iio::srv::AttrEnableTopic::Request> request,
  std::shared_ptr<adi_iio::srv::AttrEnableTopic::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"),
    "Service request /AttrEnableTopic %s with type %d with loop_rate %f Hz",
    request->attr_path.c_str(), request->type, request->loop_rate);

  std::string message;
  response->success = rwAttrPath(request->attr_path, message);
  response->message = message;

  std::string local_topic_name = IIOPath::toTopicName(request->topic_name);
  if (local_topic_name == "") {
    local_topic_name = IIOPath::toTopicName(request->attr_path);
  }

  if (response->success) {
    if (m_attrTopicMap.find(local_topic_name) != m_attrTopicMap.end()) {
      m_attrTopicMap.erase(local_topic_name);
    }

    m_attrTopicMap.insert(
      {local_topic_name,
        std::make_shared<IIOAttrTopic>(
          std::dynamic_pointer_cast<IIONode>(shared_from_this()), local_topic_name,
          request->attr_path, static_cast<IIOAttrTopic::topicType_t>(request->type),
          request->loop_rate)});

    setSuccessResponse(response, "Success");
  } else {
    setErrorResponse(response, message);
  }
}

void IIONode::attrDisableTopicSrv(
  const std::shared_ptr<adi_iio::srv::AttrDisableTopic::Request> request,
  std::shared_ptr<adi_iio::srv::AttrDisableTopic::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"), "Service request /AttrDisableTopic %s ",
    request->topic_name.c_str());

  std::string message;

  std::string local_topic_name = request->topic_name;
  local_topic_name = IIOPath::toTopicName(request->topic_name);  // for compatibility with enable

  if (m_attrTopicMap.find(local_topic_name) != m_attrTopicMap.end()) {
    m_attrTopicMap.erase(local_topic_name);
    message = "Success";
    setSuccessResponse(response, message);
  } else {
    message = "Topic not found";
    setErrorResponse(response, message);
  }
}

void IIONode::buffRefillSrv(
  const std::shared_ptr<adi_iio::srv::BufferRefill::Request> request,
  std::shared_ptr<adi_iio::srv::BufferRefill::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"), "Service request /BufferRill %s",
    request->device_path.c_str());

  std::string message;
  bool success = true;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    message = request->device_path + " is not a valid device path";
    setErrorResponse(response, message);
    return;
  }

  if (m_bufferMap.find(path.getDeviceSegment()) == m_bufferMap.end()) {
    message = "Buffer not found";
    setErrorResponse(response, message);
    return;
  }

  if (!m_bufferMap[path.getDeviceSegment()]->topic_enabled()) {
    success = m_bufferMap[path.getDeviceSegment()]->refill(message);
  }

  if (!success) {
    setErrorResponse(response, message);
    return;
  }

  setSuccessResponse(response, message);
  response->buffer = m_bufferMap[path.getDeviceSegment()]->data();
}

void IIONode::buffReadSrv(
  const std::shared_ptr<adi_iio::srv::BufferRead::Request> request,
  std::shared_ptr<adi_iio::srv::BufferRead::Response> response)
{
  std::string channels;
  for (auto & channel : request->channels) {
    channels += channel + " ";
  }

  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"), "Service request /BufferRead %s - %s - %d samples",
    request->device_path.c_str(), channels.c_str(), request->samples_count);

  std::string message;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    message = request->device_path + " is not a valid device path";
    setErrorResponse(response, message);
    return;
  }

  std::shared_ptr<IIOBuffer> buffer;
  if (m_bufferMap.find(path.getDeviceSegment()) == m_bufferMap.end()) {
    message = "Device or buffer not found.";
    setErrorResponse(response, message);
    return;
  }

  buffer = m_bufferMap[path.getDeviceSegment()];
  buffer->destroyIIOBuffer();
  buffer->set_samples_count(request->samples_count);
  buffer->set_channels(request->channels);

  bool success = buffer->createIIOBuffer(message);
  if (!success) {
    setErrorResponse(response, message);
    return;
  }

  success = buffer->refill(message);
  if (!success) {
    setErrorResponse(response, message);
    return;
  }

  setSuccessResponse(response, message);
  response->buffer = buffer->data();
}

void IIONode::buffWriteSrv(
  const std::shared_ptr<adi_iio::srv::BufferWrite::Request> request,
  std::shared_ptr<adi_iio::srv::BufferWrite::Response> response)
{
  std::string channels;
  for (auto & channel : request->channels) {
    channels += channel + " ";
  }

  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"), "Service request /BufferWrite %s - %s - %d samples",
    request->device_path.c_str(), channels.c_str(), request->buffer.layout.dim[0].size);

  std::string message;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    message = request->device_path + " is not a valid device path";
    setErrorResponse(response, message);
    return;
  }

  std::shared_ptr<IIOBuffer> buffer;
  if (m_bufferMap.find(path.getDeviceSegment()) == m_bufferMap.end()) {
    message = "Device or buffer not found.";
    setErrorResponse(response, message);
    return;
  }

  buffer = m_bufferMap[path.getDeviceSegment()];
  buffer->destroyIIOBuffer();
  buffer->set_samples_count(request->buffer.layout.dim[0].size);
  buffer->set_channels(request->channels);

  bool success = buffer->createIIOBuffer(message, true, request->cyclic);
  if (!success) {
    setErrorResponse(response, message);
    return;
  }

  success = buffer->push(message, request->buffer);
  if (!success) {
    setErrorResponse(response, message);
    return;
  }
  setSuccessResponse(response, message);
}

void IIONode::buffCreateSrv(
  const std::shared_ptr<adi_iio::srv::BufferCreate::Request> request,
  std::shared_ptr<adi_iio::srv::BufferCreate::Response> response)
{
  std::string channels;
  for (auto & channel : request->channels) {
    channels += channel + " ";
  }

  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"),
    "Service request /BufferCreate %s - %s - %d samples",
    request->device_path.c_str(), channels.c_str(), request->samples_count);

  std::string message;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    message = request->device_path + " is not a valid device path";
    setErrorResponse(response, message);
    return;
  }

  std::shared_ptr<IIOBuffer> buffer;
  if (m_bufferMap.find(path.getDeviceSegment()) == m_bufferMap.end()) {
    message = "Buffer not found";
    setWarningResponse(response, message);
    return;
  }

  buffer = m_bufferMap[path.getDeviceSegment()];
  buffer->destroyIIOBuffer();
  buffer->set_samples_count(request->samples_count);
  buffer->set_channels(request->channels);

  bool success = buffer->createIIOBuffer(message, false, false);
  if (!success) {
    setErrorResponse(response, message);
    return;
  }

  setSuccessResponse(response, message);
  response->layout = {buffer->data().layout};
}

void IIONode::buffDestroySrv(
  const std::shared_ptr<adi_iio::srv::BufferDestroy::Request> request,
  std::shared_ptr<adi_iio::srv::BufferDestroy::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"),
    "Service request /BufferDestroy %s", request->device_path.c_str());

  std::string msg;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    msg = request->device_path + " is not a valid device path";
    setErrorResponse(response, msg);
    return;
  }

  if (m_bufferMap.find(path.getDeviceSegment()) != m_bufferMap.end()) {
    if (m_bufferMap[path.getDeviceSegment()]->buffer()) {
      m_bufferMap[path.getDeviceSegment()]->destroyIIOBuffer();
      msg = "Success";
      setSuccessResponse(response, msg);
      return;
    }
  }

  msg = "Buffer not found";
  setWarningResponse(response, msg);
}

void IIONode::buffEnableTopicSrv(
  const std::shared_ptr<adi_iio::srv::BufferEnableTopic::Request> request,
  std::shared_ptr<adi_iio::srv::BufferEnableTopic::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"),
    "Service request /BufferEnableTopic %s", request->device_path.c_str());

  std::string msg;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    msg = request->device_path + " is not a valid device path";
    setErrorResponse(response, msg);
    return;
  }

  if (m_bufferMap.find(path.getDeviceSegment()) != m_bufferMap.end()) {
    m_bufferMap[path.getDeviceSegment()]->enableTopic(request->topic_name, request->loop_rate);
    msg = "Success";
    setSuccessResponse(response, msg);
  } else {
    msg = "Buffer not found";
    setWarningResponse(response, msg);
  }
}

void IIONode::buffDisableTopicSrv(
  const std::shared_ptr<adi_iio::srv::BufferDisableTopic::Request> request,
  std::shared_ptr<adi_iio::srv::BufferDisableTopic::Response> response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("adi_iio_node"),
    "Service request /BufferDisableTopic %s", request->device_path.c_str());

  std::string msg;

  IIOPath path(request->device_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    msg = request->device_path + " is not a valid device path";
    setErrorResponse(response, msg);
    return;
  }

  if (m_bufferMap.find(path.getDeviceSegment()) != m_bufferMap.end()) {
    m_bufferMap[path.getDeviceSegment()]->disableTopic();
    msg = "Success";
    setSuccessResponse(response, msg);
  } else {
    msg = "Buffer not found";
    setWarningResponse(response, msg);
  }
}

void IIONode::listDevicesSrv(
  const std::shared_ptr<adi_iio::srv::ListDevices::Request> request,
  std::shared_ptr<adi_iio::srv::ListDevices::Response> response)
{
  (void)request;  // unused
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Service request /ListDevices");
  std::string msg;

  IIOPath path("");
  std::vector<std::string> data;

  auto devices_ptr = getDevices(ctx());
  for (iio_device * dev : devices_ptr) {
    const char * dev_name_cstr = iio_device_get_name(dev);
    if (dev_name_cstr == nullptr) {
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"),
        "device name is null, skipping device");
      continue;
    }
    auto dev_name = std::string(dev_name_cstr);
    auto dev_path = path.append(dev_name);
    data.push_back(dev_path);
  }

  msg = "Found " + std::to_string(data.size()) + " devices";
  setSuccessResponse(response, msg);
  response->data = {data};
}

void IIONode::listChannelsSrv(
  const std::shared_ptr<adi_iio::srv::ListChannels::Request> request,
  std::shared_ptr<adi_iio::srv::ListChannels::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Service request /ListChannels");
  std::string msg;

  IIOPath path(request->iio_path);
  if (!path.isValid(IIOPathType::DEVICE)) {
    msg = "Invalid path: " + request->iio_path;
    setWarningResponse(response, msg);
    return;
  }

  auto dev_name = path.getDeviceSegment();
  iio_device * dev = iio_context_find_device(ctx(), dev_name.c_str());
  if (dev == nullptr) {
    msg = "Could not find device: " + dev_name;
    setWarningResponse(response, msg);
    return;
  }

  std::vector<std::string> data;

  auto channels_ptr = getChannels(dev);
  for (iio_channel * chn : channels_ptr) {
    auto chn_name = std::string(iio_channel_get_id(chn));
    auto is_output = iio_channel_is_output(chn);
    auto chn_path = IIOPath::toExtendedChannelSegment(is_output, chn_name);
    data.push_back(path.append(chn_path));
  }

  msg = "Found " + std::to_string(data.size()) + " channels in device: " + dev_name;
  setSuccessResponse(response, msg);
  response->data = {data};
}

void IIONode::listAttributesSrv(
  const std::shared_ptr<adi_iio::srv::ListAttributes::Request> request,
  std::shared_ptr<adi_iio::srv::ListAttributes::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Service request /ListAttributes");
  std::string msg;
  std::vector<std::string> data;

  IIOPath path(request->iio_path);
  if (path.isValid(IIOPathType::CONTEXT)) {
    int ret{};
    std::string err_str;

    unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx());
    for (unsigned int i = 0; i < nb_ctx_attrs; i++) {
      const char * key, * value;

      ret = iio_context_get_attr(ctx(), i, &key, &value);
      if (ret == 0) {
        data.push_back(path.append(std::string(key)));
      } else {
        err_str = strerror(-ret);
        RCLCPP_WARN(
          rclcpp::get_logger(
            "adi_iio_node"),
          "Unable to read IIO context attribute: %s", err_str.c_str());
      }
    }
    msg = "Found " + std::to_string(data.size()) + " attributes";
  } else if (path.isValid(IIOPathType::DEVICE)) {
    auto dev_name = path.getDeviceSegment();
    iio_device * dev = iio_context_find_device(ctx(), dev_name.c_str());
    if (dev == nullptr) {
      msg = "Could not find device: " + dev_name;
      setWarningResponse(response, msg);
      return;
    }

    unsigned int nb_attrs = iio_device_get_attrs_count(dev);
    for (unsigned int i = 0; i < nb_attrs; i++) {
      std::string attr_key = iio_device_get_attr(dev, i);
      data.push_back(path.append(attr_key));
    }
    msg = "Found " + std::to_string(data.size()) + " attributes in device: " + dev_name;
  } else if (path.isValid(IIOPathType::CHANNEL)) {
    auto dev_name = path.getDeviceSegment();
    iio_device * dev = iio_context_find_device(ctx(), dev_name.c_str());
    if (dev == nullptr) {
      msg = "Could not find device: " + dev_name;
      setWarningResponse(response, msg);
      return;
    }

    iio_channel * chn;
    if (path.hasExtendedChannelFormat()) {
      auto [is_output, chn_name] = path.getExtendedChannelSegment();
      chn = iio_device_find_channel(dev, chn_name.c_str(), is_output);
    } else {
      chn = iio_device_find_channel(dev, path.getChannelSegment().c_str(), false);
      if (chn == nullptr) {
        chn = iio_device_find_channel(dev, path.getChannelSegment().c_str(), true);
      }
    }
    if (chn == nullptr) {
      msg = "Could not find channel: " + path.getChannelSegment();
      setWarningResponse(response, msg);
      return;
    }
    unsigned int nb_attrs = iio_channel_get_attrs_count(chn);
    for (unsigned int i = 0; i < nb_attrs; i++) {
      std::string attr_key = iio_channel_get_attr(chn, i);
      data.push_back(path.append(attr_key));
    }
    msg = "Found " + std::to_string(data.size()) + " attributes in channel: " +
      path.getChannelSegment();
  } else {
    msg = "Invalid path: " + request->iio_path;
    setErrorResponse(response, msg);
    return;
  }

  setSuccessResponse(response, msg);
  response->data = {data};
}

void IIONode::scanContextSrv(
  const std::shared_ptr<adi_iio::srv::ScanContext::Request> request,
  std::shared_ptr<adi_iio::srv::ScanContext::Response> response)
{
  (void)request;  // unused
  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "Service request /ScanContext");
  std::string msg{"Found: "};
  std::vector<std::string> devices;
  std::vector<std::string> channels;
  std::vector<std::string> context_attrs;
  std::vector<std::string> device_attrs;
  std::vector<std::string> channel_attrs;

  // Handle context attributes
  IIOPath ctx_path("");
  int ret{};
  std::string err_str;
  unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx());
  for (unsigned int i = 0; i < nb_ctx_attrs; i++) {
    const char * key, * value;
    ret = iio_context_get_attr(ctx(), i, &key, &value);
    if (ret == 0) {
      context_attrs.push_back(ctx_path.append(std::string(key)));
    } else {
      err_str = strerror(-ret);
      RCLCPP_WARN(
        rclcpp::get_logger(
          "adi_iio_node"),
        "Unable to read IIO context attribute: %s", err_str.c_str());
    }
  }

  // Handle devices
  auto devices_ptr = getDevices(ctx());
  for (iio_device * dev : devices_ptr) {
    const char * dev_name_cstr = iio_device_get_name(dev);
    if (dev_name_cstr == nullptr) {
      RCLCPP_WARN(
        rclcpp::get_logger("adi_iio_node"),
        "device name is null, skipping device");
      continue;
    }
    auto dev_name = std::string(dev_name_cstr);
    auto dev_path = IIOPath(ctx_path.append(dev_name));

    devices.push_back(dev_path.basePath());

    // Handle device attributes
    unsigned int nb_attrs = iio_device_get_attrs_count(dev);
    for (unsigned int i = 0; i < nb_attrs; i++) {
      std::string attr_key = iio_device_get_attr(dev, i);
      device_attrs.push_back(dev_path.append(attr_key));
    }

    // Handle device channels
    auto channels_ptr = getChannels(dev);
    for (iio_channel * chn : channels_ptr) {
      auto chn_name = std::string(iio_channel_get_id(chn));
      auto is_output = iio_channel_is_output(chn);
      auto chn_segment = IIOPath::toExtendedChannelSegment(is_output, chn_name);
      auto chn_path = IIOPath(dev_path.append(chn_segment));

      channels.push_back(chn_path.basePath());

      // Handle channel attributes
      unsigned int nb_attrs = iio_channel_get_attrs_count(chn);
      for (unsigned int i = 0; i < nb_attrs; i++) {
        std::string attr_key = iio_channel_get_attr(chn, i);
        channel_attrs.push_back(chn_path.append(attr_key));
      }
    }
  }
  msg += "Context attributes: " + std::to_string(context_attrs.size()) + "; ";
  msg += "Devices: " + std::to_string(devices.size()) + "; ";
  msg += "Channels: " + std::to_string(channels.size()) + "; ";
  msg += "Device attributes: " + std::to_string(device_attrs.size()) + "; ";
  msg += "Channel attributes: " + std::to_string(channel_attrs.size()) + "; ";

  setSuccessResponse(response, msg);
  response->devices = {devices};
  response->channels = {channels};
  response->context_attrs = {context_attrs};
  response->device_attrs = {device_attrs};
  response->channel_attrs = {channel_attrs};
}

std::vector<iio_device *> IIONode::getDevices(iio_context * ctx)
{
  std::vector<iio_device *> devices;
  unsigned int devices_count = iio_context_get_devices_count(ctx);
  for (unsigned int i = 0; i < devices_count; i++) {
    iio_device * dev = iio_context_get_device(ctx, i);
    devices.push_back(dev);
  }
  return devices;
}

std::vector<iio_channel *> IIONode::getChannels(iio_device * device)
{
  std::vector<iio_channel *> channels;
  unsigned int channels_count = iio_device_get_channels_count(device);
  for (unsigned int i = 0; i < channels_count; i++) {
    iio_channel * ch = iio_device_get_channel(device, i);
    channels.push_back(ch);
  }
  return channels;
}

std::string IIONode::uri()
{
  return m_uri;
}

iio_context * IIONode::ctx()
{
  return m_ctx;
}
