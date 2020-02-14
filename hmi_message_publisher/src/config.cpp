#include "hmi_message_publisher/config.h"

DEFINE_bool(HMI_SL4_use_threat_obstacle_topic,true,"");
DEFINE_string(HMI_SL4_perception_obstacle_topic,"/perception/obstacles", "");
DEFINE_string(HMI_SL4_threat_obstacle_topic,"/threat_assessment/obstacles","");
DEFINE_string(HMI_SL4_lane_path_topic,  "/perception/lane_path","");
DEFINE_string(HMI_SL4_odom_topic,"/navsat/odom","");
DEFINE_string(HMI_SL4_steering_report_topic, "/vehicle/steering_report","");
DEFINE_string(HMI_SL4_dbw_enable_topic, "/vehicle/dbw_enabled","");
DEFINE_string(HMI_SL4_planning_trajectory_topic, "/planning/trajectory","");
DEFINE_string(HMI_SL4_turn_signal_cmd_topic, "/vehicle/turn_signal_cmd","");
DEFINE_int32(HMI_SL4_can_index, 0, "can index");
