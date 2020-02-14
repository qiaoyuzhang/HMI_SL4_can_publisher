#include "hmi_message_publisher/can_node.h"
#include <glog/logging.h>
using namespace HMI::SL4::hmi_message_publisher;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "hmi_message_publisher_can_node");
    ros::NodeHandle nh("~");

    auto use_threat_obstacle_topic = nh.param("use_threat_obstacle_topic", true);
    auto perception_obstacle_topic = nh.param("perception_obstacle_topic", std::string{"/perception/obstacles"});
    auto threat_obstacle_topic = nh.param("threat_obstacle_topic", std::string{"/threat_assessment/obstacles"});
    auto lane_path_topic = nh.param("lane_path_topic", std::string{"/perception/lane_path"});
    auto odom_topic = nh.param("odom_topic", std::string{"/navsat/odom"});
    auto steering_report_topic = nh.param("steering_report_topic", std::string{"/vehicle/steering_report"});
    auto dbw_enable_topic = nh.param("dbw_enable_topic", std::string{"/vehicle/dbw_enabled"});
    auto planning_trajectory_topic = nh.param("planning_trajectory_topic", std::string{"/planning/trajectory"});
    auto turn_signal_cmd_topic = nh.param("turn_signal_cmd_topic", std::string{"/vehicle/turn_signal_cmd"});
    auto can_id = nh.param("ifname0", std::string{"can0"});

    CanNode can_node(0);
    can_node.Run();
}
