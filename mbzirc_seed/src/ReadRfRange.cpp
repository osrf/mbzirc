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
  std::string log = ss.str();
  if (!log.empty()) {
    RCLCPP_INFO_STREAM(this->get_logger(), log);
  }
}

void ReadRfRange::onRangeMessage(const ros_ign_interfaces::msg::ParamVec & msg)
{
  // Temporary location of parameter information.
  struct Entry
  {
    // Platform name
    std::string platform;
    // Range to platform
    double range;
    // RSSI
    double rssi;
  };

  std::unordered_map<int, Entry> entries;

  // Since we are looking for (model, rssi, range) tuples, 
  // it is expected that the number of entries is a multiple of 3.
  auto nEntries = msg.params.size() / 3;

  // The parameters coming from the simulator are stored in a nested map.
  // The bridge from the simulator to ROS 2 flattens this map by appending
  // a prefix to each group of parameters "param_<NUM>".
  // The logic below un-flattens this into one entry per competitor platform.
  // 
  // Since we are trying to update an entire model at once, we temporarily store
  // the values in the "Entry" structure, to then be copied over once all parameters are parsed.
  for (auto param: msg.params)
  {
   int curIdx = 0;
    for (int ii = 0; ii < nEntries; ++ii) {
      // Find the index of this parameter entry.
      if(param.name.find(std::string("param_" + std::to_string(ii))) != std::string::npos) {
        curIdx = ii;
        break;
      }
    }

    // Add entry to the map if it wasn't already found
    if (entries.count(curIdx) == 0) {
      entries[curIdx] = Entry();
    }

    // Locate the three entries that come from the simulated sensor
    // (model, rssi, range)
    if (param.name.find("model") != std::string::npos) {
      entries[curIdx].platform = param.value.string_value;
    } else if (param.name.find("rssi") != std::string::npos) {
      entries[curIdx].rssi = param.value.double_value;
    } else if (param.name.find("range") != std::string::npos) {
      entries[curIdx].range = param.value.double_value;
    }
  }

  // Copy the temporary entries into the final location.
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
