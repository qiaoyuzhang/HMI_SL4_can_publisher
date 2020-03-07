#include "hmi_message_publisher/can_node.h"
#include "base/program.h"
#include "plusmap/common/plusmap_utils.h"
#include "hmi_message_publisher/flags.h"
#include <glog/logging.h>
using namespace HMI::SL4::hmi_message_publisher;
using namespace drive::common::base;

static const std::string node_name = "hmi_message_can_publisher_node";
class HmiMessagePublisherProgram: public ROSProgram {
 public:
    HmiMessagePublisherProgram() : ROSProgram(node_name){};

    bool init() override {

        CanNode::UseThreatObstacleTopic() = FLAGS_use_threat_obstacle_topic;
        CanNode::PerceptionObstacleTopic() = FLAGS_perception_obstacle_topic;
        CanNode::ThreatObstacleTopic() = FLAGS_threat_obstacle_topic;
        CanNode::LanePathTopic() = FLAGS_lane_path_topic;
        CanNode::OdomTopic() = FLAGS_odom_topic;
        CanNode::SteeringReportTopic() = FLAGS_steering_report_topic;
        CanNode::DbwEnableTopic() = FLAGS_dbw_enable_topic;
        CanNode::PlanningTrajectoryTopic() = FLAGS_planning_trajectory_topic;
        CanNode::TurnSignalCmdTopic() = FLAGS_turn_signal_cmd_topic;
        CanNode::LongitudinalReportTopic() = FLAGS_longitudinal_report_topic;
        CanNode::SpeedUnit() = FLAGS_speed_unit;

        return true;
    }

    void go() override {

        CanNode can_node(0);
        can_node.Run();
    }
};

int main(int argc, char** argv)
{
    HmiMessagePublisherProgram p;
    return p.run(argc, argv);
}
