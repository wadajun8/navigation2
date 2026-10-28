// Copyright (c) 2026 Junya Wada
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

#ifndef NAV2_RVIZ_PLUGINS__WAYPOINT_IO_HPP_
#define NAV2_RVIZ_PLUGINS__WAYPOINT_IO_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_rviz_plugins
{

// Waypoint file format used by the Nav2 panel ("Save WPs" / "Load WPs"):
//
// waypoints:
//   waypoint0:
//     pose: [x, y, z]
//     orientation: [w, x, y, z]

inline std::string waypointsToYaml(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "waypoints";
  out << YAML::BeginMap;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto & p = poses[i].pose;
    out << YAML::Key << "waypoint" + std::to_string(i);
    out << YAML::BeginMap;
    out << YAML::Key << "pose";
    out << YAML::Value << std::vector<double>{p.position.x, p.position.y, p.position.z};
    out << YAML::Key << "orientation";
    out << YAML::Value << std::vector<double>{
      p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  out << YAML::EndMap;
  return out.c_str();
}

// Throws YAML::Exception when the file does not follow the format above.
inline std::vector<geometry_msgs::msg::PoseStamped> waypointsFromYaml(
  const YAML::Node & root, const std::string & frame_id)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto & entry : root["waypoints"]) {
    const auto position = entry.second["pose"].as<std::vector<double>>();
    const auto orientation = entry.second["orientation"].as<std::vector<double>>();
    if (position.size() != 3 || orientation.size() != 4) {
      throw YAML::Exception(entry.second.Mark(), "pose needs 3 values and orientation needs 4");
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];
    msg.pose.orientation.w = orientation[0];
    msg.pose.orientation.x = orientation[1];
    msg.pose.orientation.y = orientation[2];
    msg.pose.orientation.z = orientation[3];
    poses.push_back(msg);
  }
  return poses;
}

inline std::string withYamlExtension(const std::string & file)
{
  const std::string ext = ".yaml";
  if (file.size() >= ext.size() && file.compare(file.size() - ext.size(), ext.size(), ext) == 0) {
    return file;
  }
  return file + ext;
}

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__WAYPOINT_IO_HPP_
