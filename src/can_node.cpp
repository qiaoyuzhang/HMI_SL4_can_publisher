#include "hmi_message_publisher/can_node.h"
#include <glog/logging.h>

namespace HMI{
namespace SL4{
namespace hmi_message_publisher{

bool CanNode::use_threat_obstacle_topic(true);
std::string CanNode::perception_obstacle_topic("");
std::string CanNode::threat_obstacle_topic("");
std::string CanNode::lane_path_topic("");
std::string CanNode::odom_topic("");
std::string CanNode::steering_report_topic("");
std::string CanNode::dbw_enable_topic("");
std::string CanNode::planning_trajectory_topic("");
std::string CanNode::turn_signal_cmd_topic("");
std::string CanNode::longitudinal_report_topic("");
    
CanNode::CanNode(const int index)
    : can_(index) {
        init();
    }

void CanNode::init(){
    DLOG(INFO) << "CanNode::init";
    // set timer to write message to can every 0.1 s
    timer = node_.createTimer(ros::Duration(0.1), &CanNode::writeDataToCan, (CanNode*)this);
    
    lane_sub_ = node_.subscribe(lane_path_topic,
                                10,
                                &MessageHandler::handleLanePath,
                                (MessageHandler*)this);  // 10 hz
    if (use_threat_obstacle_topic) {
        obstacle_sub_ = node_.subscribe(threat_obstacle_topic,
                                        10,
                                        &MessageHandler::handleObstacles,
                                        (MessageHandler*)this);  // 10 hz
    } else {
        obstacle_sub_ = node_.subscribe(perception_obstacle_topic,
                                        10,
                                        &MessageHandler::handleObstacles,
                                        (MessageHandler*)this);  // 10 hz
    }

    odom_sub_ = node_.subscribe(odom_topic,
                                10,
                                &MessageHandler::handleOdom,
                                (MessageHandler*)this);  // 10 hz

    steering_report_sub_ = node_.subscribe(steering_report_topic,
                                           1,
                                           &MessageHandler::handleSteeringReport,
                                           (MessageHandler*)this);  // 100 hz
    turn_signal_cmd_sub_ = node_.subscribe(turn_signal_cmd_topic,
                                           10,
                                           &MessageHandler::handleTurnSignalCmd,
                                           (MessageHandler*)this);
    dbw_enabled_sub_ = node_.subscribe(dbw_enable_topic,
                                       1,
                                       &MessageHandler::handleDbwEnabledUpdate,
                                       (MessageHandler*)this);  // ad hoc
    planning_trajectory_sub_ = node_.subscribe(planning_trajectory_topic,
                                               10,
                                               &MessageHandler::handlePlanningTrajectory,
                                               (MessageHandler*)this);  // 10 hz
    longitudinal_report_sub_ = node_.subscribe(longitudinal_report_topic,
                                               1,
                                               &MessageHandler::handleLongitudinalReport,
                                               (MessageHandler*)this);
}
    
void CanNode::Run(){
        DLOG(INFO) << "CanNode::Run";
        can_.Start();
        ros::spin();
        can_.Stop();
}

void CanNode::writeDataToCan(const ros::TimerEvent& event){
    DLOG(INFO) << "write data to Can";     

    can_.WriteToCan(hmi_message::VehicleStatusGeneralInfoMsgId, _data_buffer.vehicle_status_general_info.getVectorData());
    can_.WriteToCan(hmi_message::ObstacleGeneralInfoMsgId, _data_buffer.obstacle_general_info.getVectorData());
    ros::Duration(0, 1).sleep();
    for(const auto& obstacle : _data_buffer.obstacle_extenged_info_vec){
        can_.WriteToCan(hmi_message::ObstacleExtendedInfoMsgId, obstacle.getVectorData());
        ros::Duration(0, 1).sleep();
    }
    can_.WriteToCan(hmi_message::LaneGeneralInfoMsgId, _data_buffer.lane_general_info.getVectorData());
    ros::Duration(0, 1).sleep();
    can_.WriteToCan(hmi_message::PlanningGeneralInfoMsgId, _data_buffer.planning_general_info.getVectorData());
}


}
}
}

