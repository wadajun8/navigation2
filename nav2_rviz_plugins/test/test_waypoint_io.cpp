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

#include <gtest/gtest.h>

#include <vector>

#include "nav2_rviz_plugins/waypoint_io.hpp"

using geometry_msgs::msg::PoseStamped;
using nav2_rviz_plugins::waypointsFromYaml;
using nav2_rviz_plugins::waypointsToYaml;
using nav2_rviz_plugins::withYamlExtension;

namespace
{
PoseStamped makePose(double x, double y, double qz, double qw)
{
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = qz;
  p.pose.orientation.w = qw;
  return p;
}
}  // namespace

TEST(WaypointIo, RoundTripKeepsOrderPositionAndOrientation)
{
  const std::vector<PoseStamped> saved = {
    makePose(1.0, 2.0, 0.0, 1.0),
    makePose(-3.5, 0.25, 0.70710678, 0.70710678),
    makePose(4.0, -1.0, 1.0, 0.0),
  };

  const auto loaded = waypointsFromYaml(YAML::Load(waypointsToYaml(saved)), "map");

  ASSERT_EQ(loaded.size(), saved.size());
  for (size_t i = 0; i < saved.size(); ++i) {
    EXPECT_EQ(loaded[i].header.frame_id, "map");
    EXPECT_DOUBLE_EQ(loaded[i].pose.position.x, saved[i].pose.position.x);
    EXPECT_DOUBLE_EQ(loaded[i].pose.position.y, saved[i].pose.position.y);
    EXPECT_DOUBLE_EQ(loaded[i].pose.orientation.x, saved[i].pose.orientation.x);
    EXPECT_DOUBLE_EQ(loaded[i].pose.orientation.y, saved[i].pose.orientation.y);
    EXPECT_DOUBLE_EQ(loaded[i].pose.orientation.z, saved[i].pose.orientation.z);
    EXPECT_DOUBLE_EQ(loaded[i].pose.orientation.w, saved[i].pose.orientation.w);
  }
}

TEST(WaypointIo, LoadsFilesSavedByThePanel)
{
  // Same layout as config/waypoint/*.yaml in raspicat_navigation (orientation is w, x, y, z)
  const auto loaded = waypointsFromYaml(
    YAML::Load(
      "waypoints:\n"
      "  waypoint0:\n"
      "    pose: [317.85, 181.56, 0]\n"
      "    orientation: [0.2316, 0, 0, 0.9728]\n"
      "  waypoint1:\n"
      "    pose: [320.0, 185.0, 0]\n"
      "    orientation: [1, 0, 0, 0]\n"),
    "map");

  ASSERT_EQ(loaded.size(), 2u);
  EXPECT_DOUBLE_EQ(loaded[0].pose.position.x, 317.85);
  EXPECT_DOUBLE_EQ(loaded[0].pose.orientation.w, 0.2316);
  EXPECT_DOUBLE_EQ(loaded[0].pose.orientation.z, 0.9728);
  // Facing +x must stay a valid quaternion (it used to become all zeros)
  EXPECT_DOUBLE_EQ(loaded[1].pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(loaded[1].pose.orientation.z, 0.0);
}

TEST(WaypointIo, RejectsMalformedWaypoint)
{
  EXPECT_THROW(
    waypointsFromYaml(
      YAML::Load("waypoints:\n  waypoint0:\n    pose: [1, 2]\n    orientation: [1, 0, 0, 0]\n"),
      "map"),
    YAML::Exception);
}

TEST(WaypointIo, AddsYamlExtensionOnlyWhenMissing)
{
  EXPECT_EQ(withYamlExtension("/tmp/wp"), "/tmp/wp.yaml");
  EXPECT_EQ(withYamlExtension("/tmp/wp.yaml"), "/tmp/wp.yaml");
}
