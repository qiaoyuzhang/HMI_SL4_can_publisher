#include "hmi_message_publisher/can_node.h"
#include <glog/logging.h>
using namespace HMI::SL4::hmi_message_publisher;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "hmi_message_publisher_can_node");
    ros::NodeHandle nh("~");

    auto use_threat_obstacle_topic = nh.getParam("use_threat_obstacle_topic", CanNode::UseThreatObstacleTopic());
    auto perception_obstacle_topic = nh.getParam("perception_obstacle_topic", CanNode::PerceptionObstacleTopic());
    auto threat_obstacle_topic = nh.getParam("threat_obstacle_topic", CanNode::ThreatObstacleTopic());
    auto lane_path_topic = nh.getParam("lane_path_topic", CanNode::LanePathTopic());
    auto odom_topic = nh.getParam("odom_topic", CanNode::OdomTopic());
    auto steering_report_topic = nh.getParam("steering_report_topic", CanNode::SteeringReportTopic());
    auto dbw_enable_topic = nh.getParam("dbw_enable_topic", CanNode::DbwEnableTopic());
    auto planning_trajectory_topic = nh.getParam("planning_trajectory_topic", CanNode::PlanningTrajectoryTopic());
    auto turn_signal_cmd_topic = nh.getParam("turn_signal_cmd_topic", CanNode::TurnSignalCmdTopic());

    CanNode can_node(0);
    can_node.Run();
}
