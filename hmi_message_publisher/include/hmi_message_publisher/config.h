#pragma once

#include <gflags/gflags.h>

DECLARE_bool(HMI_SL4_use_threat_obstacle_topic);
DECLARE_string(HMI_SL4_perception_obstacle_topic);
DECLARE_string(HMI_SL4_threat_obstacle_topic);
DECLARE_string(HMI_SL4_lane_path_topic);
DECLARE_string(HMI_SL4_odom_topic);
DECLARE_string(HMI_SL4_steering_report_topic);
DECLARE_string(HMI_SL4_dbw_enable_topic);
DECLARE_string(HMI_SL4_planning_trajectory_topic);
DECLARE_string(HMI_SL4_turn_signal_cmd_topic);
DECLARE_int32(HMI_SL4_can_index);
