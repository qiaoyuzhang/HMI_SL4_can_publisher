#include "hmi_message_publisher/can_node.h"
#include <glog/logging.h>

namespace HMI{
namespace SL4{
namespace hmi_message_publisher{
    
CanNode::CanNode()
    : node_(), can_(FLAGS_HMI_SL4_can_index) {
        init();
    }

void CanNode::init(){
    DLOG(INFO) << "CanNode::init";
    // set timer to write message to can every 0.1 s
    timer = node_.createTimer(ros::Duration(0.1), &CanNode::writeDataToCan, (CanNode*)this);
    
    lane_sub_ = node_.subscribe(FLAGS_HMI_SL4_perception_obstacle_topic,
                                10,
                                &MessageHandler::handleLanePath,
                                (MessageHandler*)this);  // 10 hz
    if (FLAGS_HMI_SL4_use_threat_obstacle_topic) {
        obstacle_sub_ = node_.subscribe(FLAGS_HMI_SL4_threat_obstacle_topic,
                                        10,
                                        &MessageHandler::handleObstacles,
                                        (MessageHandler*)this);  // 10 hz
    } else {
        obstacle_sub_ = node_.subscribe(FLAGS_HMI_SL4_perception_obstacle_topic,
                                        10,
                                        &MessageHandler::handleObstacles,
                                        (MessageHandler*)this);  // 10 hz
    }

    odom_sub_ = node_.subscribe(FLAGS_HMI_SL4_odom_topic,
                                10,
                                &MessageHandler::handleOdom,
                                (MessageHandler*)this);  // 10 hz

    steering_report_sub_ = node_.subscribe(FLAGS_HMI_SL4_steering_report_topic,
                                           1,
                                           &MessageHandler::handleSteeringReport,
                                           (MessageHandler*)this);  // 100 hz
    turn_signal_cmd_sub_ = node_.subscribe(FLAGS_HMI_SL4_turn_signal_cmd_topic,
                                           10,
                                           &MessageHandler::handleTurnSignalCmd,
                                           (MessageHandler*)this);
    dbw_enabled_sub_ = node_.subscribe(FLAGS_HMI_SL4_dbw_enable_topic,
                                       1,
                                       &MessageHandler::handleDbwEnabledUpdate,
                                       (MessageHandler*)this);  // ad hoc
    planning_trajectory_sub_ = node_.subscribe(FLAGS_HMI_SL4_planning_trajectory_topic,
                                               10,
                                               &MessageHandler::handlePlanningTrajectory,
                                               (MessageHandler*)this);  // 10 hz
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
     for(const auto& obstacle : _data_buffer.obstacle_general_info_vec){
        can_.WriteToCan(hmi_message::ObstacleGeneralInfoMsgId, obstacle.getVectorData());
     }
     can_.WriteToCan(hmi_message::LaneGeneralInfoMsgId, _data_buffer.lane_general_info.getVectorData());
     can_.WriteToCan(hmi_message::PlanningGeneralInfoMsgId, _data_buffer.planning_general_info.getVectorData());
}


}
}
}

