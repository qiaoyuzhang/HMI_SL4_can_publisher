#pragma once

#include <gflags/gflags.h>

DECLARE_string(ifname0);
DECLARE_bool(use_threat_obstacle_topic);
DECLARE_string(perception_obstacle_topic);
DECLARE_string(threat_obstacle_topic);
DECLARE_string(lane_path_topic);
DECLARE_string(odom_topic);
DECLARE_string(steering_report_topic);
DECLARE_string(dbw_enable_topic);
DECLARE_string(planning_trajectory_topic);
DECLARE_string(turn_signal_cmd_topic);
DECLARE_string(longitudinal_report_topic);
DECLARE_string(speed_unit);
