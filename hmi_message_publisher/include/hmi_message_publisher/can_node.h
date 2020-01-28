#pragma once

#include <map>
#include <string>
#include "hmi_message_publisher/message_handler.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "hmi_message_publisher/config.h"
#include "can_common/can_interface.h"

namespace HMI {
namespace SL4{
namespace hmi_message_publisher{

class CanNode : public MessageHandler{

  public:
    CanNode(); 
    void init();
    void Run();
  private:
    ros::NodeHandle node_;

    ros::Subscriber lane_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber steering_report_sub_;
    ros::Subscriber turn_signal_cmd_sub_;
    ros::Subscriber dbw_enabled_sub_;
    ros::Subscriber planning_trajectory_sub_;

    void writeDataToCan(const ros::TimerEvent&);
    drive::common::CanInterface can_;
    ros::Timer timer;
};
}
}
}
