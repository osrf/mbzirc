/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <mbzirc_seed/ReadRfRange.hh>

namespace mbzirc_seed
{

ReadRfRange::ReadRfRange(const rclcpp::NodeOptions & options)
: rclcpp::Node("read_rf_range", options)
{
  rf_range_sub_ = this->create_subscription<ros_ign_interfaces::msg::ParamVec>(
      "range", rclcpp::QoS(10),
      std::bind(&ReadRfRange::onRangeMessage, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(&ReadRfRange::onTimer, this));
}

void ReadRfRange::onTimer()
{
  std::stringstream ss;
  {
    std::lock_guard<std::mutex> lock(ranges_mutex_);
    if (ranges_.size()) {
      ss << "Range Readings: \n";
    }

    rclcpp::Time t = this->now();
    for (auto entry: ranges_)
    {
      auto last_seen = t - entry.second.last_seen; 
      
      ss << "[platform: "
      << entry.second.platform
      << ", range: "
      << entry.second.range
      << ", rssi: "
      << entry.second.rssi
      << ", last_seen: "
      << last_seen.seconds()
      << "]\n";

    }
  }
  if (!ss.str().empty()) {
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  }
}

void ReadRfRange::onRangeMessage(const ros_ign_interfaces::msg::ParamVec & msg)
{
  struct Entry
  {
    std::string platform;
    double range;
    double rssi;
  };

  std::unordered_map<int, Entry> entries;

  auto nEntries = msg.params.size() / 3;

  for (auto param: msg.params) {
   int curIdx = 0;
    for (int ii = 0; ii < nEntries; ++ii) {
      if(param.name.find(std::string("param_" + std::to_string(ii))) != std::string::npos) {
        curIdx = ii;
        break;
      }
    }
    std::string entryName = "param_" + std::to_string(curIdx);

    if (entries.count(curIdx) == 0)
    {
      entries[curIdx] = Entry();
    }

    if (param.name.find("model") != std::string::npos) {
      entries[curIdx].platform = param.value.string_value;
    } else if (param.name.find("rssi") != std::string::npos) {
      entries[curIdx].rssi = param.value.double_value;
    } else if (param.name.find("range") != std::string::npos) {
      entries[curIdx].range = param.value.double_value;
    }
  }

  std::lock_guard<std::mutex> lock(ranges_mutex_);
  for (auto entry: entries)
  {
    this->ranges_[entry.second.platform].platform = entry.second.platform;
    this->ranges_[entry.second.platform].range = entry.second.range;
    this->ranges_[entry.second.platform].rssi = entry.second.rssi;
    this->ranges_[entry.second.platform].last_seen = rclcpp::Time(msg.header.stamp);
  }
}

}  // namespace mbzirc_seed

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mbzirc_seed::ReadRfRange)
