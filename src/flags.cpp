#include "hmi_message_publisher/flags.h"

DEFINE_bool(use_threat_obstacle_topic, false, "");
DEFINE_string(perception_obstacle_topic, "", "");
DEFINE_string(threat_obstacle_topic, "", "");
DEFINE_string(lane_path_topic, "", "");
DEFINE_string(odom_topic, "", "");
DEFINE_string(steering_report_topic, "", "");
DEFINE_string(dbw_enable_topic, "", "");
DEFINE_string(planning_trajectory_topic, "", "");
DEFINE_string(turn_signal_cmd_topic, "", "");
DEFINE_string(longitudinal_report_topic, "", "");
DEFINE_string(speed_unit, "", "");
