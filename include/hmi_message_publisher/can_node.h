#pragma once

#include <map>
#include <string>
#include "hmi_message_publisher/message_handler.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "can_common/can_interface.h"

namespace HMI {
namespace SL4{
namespace hmi_message_publisher{

class CanNode : public MessageHandler{
  public:
    CanNode(const int index); 
    void Run();

    static bool &UseThreatObstacleTopic() {return use_threat_obstacle_topic;};
    static std::string &PerceptionObstacleTopic() {return perception_obstacle_topic;};
    static std::string &ThreatObstacleTopic() {return threat_obstacle_topic;};
    static std::string &LanePathTopic() {return lane_path_topic;};
    static std::string &OdomTopic() {return odom_topic;};
    static std::string &SteeringReportTopic() {return steering_report_topic;};
    static std::string &DbwEnableTopic() {return dbw_enable_topic;};
    static std::string &PlanningTrajectoryTopic() {return planning_trajectory_topic;};
    static std::string &TurnSignalCmdTopic() {return turn_signal_cmd_topic;};
    static std::string &LongitudinalReportTopic() {return longitudinal_report_topic;};

  private:
    void init();
    
    ros::NodeHandle node_;

    ros::Subscriber lane_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber steering_report_sub_;
    ros::Subscriber turn_signal_cmd_sub_;
    ros::Subscriber dbw_enabled_sub_;
    ros::Subscriber planning_trajectory_sub_;
    ros::Subscriber longitudinal_report_sub_;

    void writeDataToCan(const ros::TimerEvent&);
    drive::common::CanInterface can_;
    ros::Timer timer;

    static bool use_threat_obstacle_topic;
    static std::string perception_obstacle_topic;
    static std::string threat_obstacle_topic;
    static std::string lane_path_topic;
    static std::string odom_topic;
    static std::string steering_report_topic;
    static std::string dbw_enable_topic;
    static std::string planning_trajectory_topic;
    static std::string turn_signal_cmd_topic;
    static std::string longitudinal_report_topic;
};
}
}
}
