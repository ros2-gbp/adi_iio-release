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

#include <rclcpp/rclcpp.hpp>
#include "adi_iio/srv/attr_read_string.hpp"
#include "adi_iio/srv/attr_write_string.hpp"
#include "adi_iio/iio_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<IIONode> node = std::make_shared<IIONode>();
  rclcpp::executors::MultiThreadedExecutor executor;

  if (!node->initialized()) {
    RCLCPP_FATAL(rclcpp::get_logger("adi_iio_node"), "Node initialization failed.");
    return EXIT_FAILURE;     // Fail if the node isn't properly initialized
  }

  node->initBuffers();

  auto cb_group_attr_read_string = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::AttrReadString>::SharedPtr attrReadSrv =
    node->create_service<adi_iio::srv::AttrReadString>(
    std::string(node->get_name()) + "/AttrReadString",
    std::bind(&IIONode::attrReadSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_attr_read_string);

  auto cb_group_attr_write_string = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::AttrWriteString>::SharedPtr attrWriteSrv =
    node->create_service<adi_iio::srv::AttrWriteString>(
    std::string(node->get_name()) + "/AttrWriteString",
    std::bind(&IIONode::attrWriteSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_attr_write_string);

  auto cb_group_attr_enable_topic = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::AttrEnableTopic>::SharedPtr attrEnableTopicSrv =
    node->create_service<adi_iio::srv::AttrEnableTopic>(
    std::string(node->get_name()) + "/AttrEnableTopic",
    std::bind(&IIONode::attrEnableTopicSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_attr_enable_topic);

  auto cb_group_attr_disable_topic = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::AttrDisableTopic>::SharedPtr attrDisableTopicSrv =
    node->create_service<adi_iio::srv::AttrDisableTopic>(
    std::string(node->get_name()) + "/AttrDisableTopic",
    std::bind(&IIONode::attrDisableTopicSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_attr_disable_topic);

  auto cb_group_buffer_read = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferRead>::SharedPtr buffReadSrv =
    node->create_service<adi_iio::srv::BufferRead>(
    std::string(node->get_name()) + "/BufferRead",
    std::bind(&IIONode::buffReadSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_read);

  auto cb_group_buffer_write = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferWrite>::SharedPtr buffWriteSrv =
    node->create_service<adi_iio::srv::BufferWrite>(
    std::string(node->get_name()) + "/BufferWrite",
    std::bind(&IIONode::buffWriteSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_write);

  auto cb_group_buffer_create = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferCreate>::SharedPtr buffCreateSrv =
    node->create_service<adi_iio::srv::BufferCreate>(
    std::string(node->get_name()) + "/BufferCreate",
    std::bind(&IIONode::buffCreateSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_create);

  auto cb_group_buffer_destroy = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferDestroy>::SharedPtr buffDestroySrv =
    node->create_service<adi_iio::srv::BufferDestroy>(
    std::string(node->get_name()) + "/BufferDestroy",
    std::bind(&IIONode::buffDestroySrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_destroy);

  auto cb_group_buffer_refill = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferRefill>::SharedPtr buffRefillSrv =
    node->create_service<adi_iio::srv::BufferRefill>(
    std::string(node->get_name()) + "/BufferRefill",
    std::bind(&IIONode::buffRefillSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_refill);

  auto cb_group_buffer_enable_topic = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferEnableTopic>::SharedPtr buffEnableTopicSrv =
    node->create_service<adi_iio::srv::BufferEnableTopic>(
    std::string(node->get_name()) + "/BufferEnableTopic",
    std::bind(&IIONode::buffEnableTopicSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_enable_topic);

  auto cb_group_buffer_disable_topic = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::BufferDisableTopic>::SharedPtr buffDisableTopicSrv =
    node->create_service<adi_iio::srv::BufferDisableTopic>(
    std::string(node->get_name()) + "/BufferDisableTopic",
    std::bind(&IIONode::buffDisableTopicSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_buffer_disable_topic);

  auto cb_group_list_devices = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::ListDevices>::SharedPtr listDevicesSrv =
    node->create_service<adi_iio::srv::ListDevices>(
    std::string(node->get_name()) + "/ListDevices",
    std::bind(&IIONode::listDevicesSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_list_devices);

  auto cb_group_list_channels = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::ListChannels>::SharedPtr listChannelsSrv =
    node->create_service<adi_iio::srv::ListChannels>(
    std::string(node->get_name()) + "/ListChannels",
    std::bind(&IIONode::listChannelsSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_list_channels);

  auto cb_group_list_attributes = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::ListAttributes>::SharedPtr listAttributesSrv =
    node->create_service<adi_iio::srv::ListAttributes>(
    std::string(node->get_name()) + "/ListAttributes",
    std::bind(&IIONode::listAttributesSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_list_attributes);

  auto cb_group_scan_context = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Service<adi_iio::srv::ScanContext>::SharedPtr scanContextSrv =
    node->create_service<adi_iio::srv::ScanContext>(
    std::string(node->get_name()) + "/ScanContext",
    std::bind(&IIONode::scanContextSrv, node, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_scan_context);

  RCLCPP_INFO(rclcpp::get_logger("adi_iio_node"), "IIO Node");

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
