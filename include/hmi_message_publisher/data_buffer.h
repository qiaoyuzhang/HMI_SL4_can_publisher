#pragma once
#include "math/pose.h"
#include "hmi_message_publisher/vehicle_status_general_info_msg.h"
#include "hmi_message_publisher/obstacle_general_info_msg.h"
#include "hmi_message_publisher/lane_general_info_msg.h"
#include "hmi_message_publisher/planning_general_info_msg.h"
#include <vector>
#include <std_msgs/String.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <std_msgs/Bool.h>
#include <dbw_mkz_msgs/TurnSignalCmd.h>
#include <nav_msgs/Odometry.h>

namespace HMI{
namespace SL4{
namespace hmi_message_publisher{

struct DataBuffer{
    hmi_message::VehicleStatusGeneralInfo vehicle_status_general_info;
    std::vector<hmi_message::ObstacleGeneralInfo> obstacle_general_info_vec;
    hmi_message::LaneGeneralInfo lane_general_info;
    hmi_message::PlanningGeneralInfo planning_general_info;
    bool active_AEB = false;
    drive::common::math::Pose pose;
};

}
}
}
