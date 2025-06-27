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

#include "adi_iio/iio_path.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>


IIOPath::IIOPath(std::string path, char delimiter)
{
  m_base_path = path;
  m_delimiter = delimiter;

  m_segments = split(path);
}

std::string IIOPath::basePath() const
{
  return m_base_path;
}

std::string IIOPath::getContextAttrSegment() const
{
  return m_segments[0];
}

std::string IIOPath::getDeviceSegment() const
{
  return m_segments[0];
}

std::string IIOPath::getDeviceAttrSegment() const
{
  return m_segments[1];
}

std::string IIOPath::getChannelSegment() const
{
  return m_segments[1];
}

std::pair<bool, std::string> IIOPath::getExtendedChannelSegment() const
{
  auto chn_segment = getChannelSegment();

  std::string prefix = chn_segment.substr(0, chn_segment.find('_'));
  std::string chn_name = chn_segment.substr(chn_segment.find('_') + 1);

  bool output = prefix == "output";
  return {output, chn_name};
}

std::pair<bool, std::string> IIOPath::getExtendedChannelSegment(std::string chn_segment)
{
  std::string prefix = chn_segment.substr(0, chn_segment.find('_'));
  std::string chn_name = chn_segment.substr(chn_segment.find('_') + 1);

  bool output = prefix == "output";
  return {output, chn_name};
}

bool IIOPath::hasExtendedChannelFormat() const
{
  auto chn_segment = getChannelSegment();
  bool hasPrefix = chn_segment.find("input_") == 0 || chn_segment.find("output_") == 0;
  return hasPrefix;
}

bool IIOPath::hasExtendedChannelFormat(std::string chn_segment)
{
  bool hasPrefix = chn_segment.find("input_") == 0 || chn_segment.find("output_") == 0;
  return hasPrefix;
}

std::string IIOPath::getChannelAttrSegment() const
{
  return m_segments[2];
}

std::string IIOPath::append(const std::string & suffix)
{
  if (m_base_path.empty()) {
    return suffix;
  }
  return m_base_path + m_delimiter + suffix;
}

bool IIOPath::isValid(const IIOPathType type)
{
  switch (type) {
    case CONTEXT:
      return m_segments.size() == 0 && m_base_path.empty();
    case CONTEXT_ATTR:
      return m_segments.size() == 1 && !getContextAttrSegment().empty();
    case DEVICE:
      return m_segments.size() == 1 && !getDeviceSegment().empty();
    case DEVICE_ATTR:
      return m_segments.size() == 2 && !getDeviceAttrSegment().empty();
    case CHANNEL:
      return m_segments.size() == 2 && !getExtendedChannelSegment().second.empty();
    case CHANNEL_ATTR:
      return m_segments.size() == 3 && !getChannelAttrSegment().empty();
    default:
      RCLCPP_WARN(rclcpp::get_logger("adi_iio_node"), "Invalid path type");
      return false;
  }
}

std::string IIOPath::toExtendedChannelSegment(bool output, std::string chn_name)
{
  std::string prefix = output ? "output_" : "input_";
  return prefix + chn_name;
}

std::string IIOPath::toTopicName(std::string path)
{
  std::replace(path.begin(), path.end(), '-', '_');
  if (!path.empty() && path.front() != '/') {
    path = '/' + path;
  }
  return path;
}

std::vector<std::string> IIOPath::split(const std::string & str)
{
  std::vector<std::string> segments;
  std::stringstream ss(str);
  std::string segment;

  while (std::getline(ss, segment, m_delimiter)) {
    segments.push_back(segment);
  }
  return segments;
}
