#include "hmi_message_publisher/message_handler.h"
#include "common/lane_detection.pb.h"
#include "common/obstacle_detection.pb.h"
#include "common/planning_trajectory.pb.h"
#include "plusmap/common/plusmap_utils.h"
#include "math/vec2d.h"
#include <glog/logging.h>
#include <cmath>

using namespace drive::common::perception;
using namespace drive::common::planning;
using namespace drive::common::math;
using namespace drive::plusmap;

namespace HMI{
namespace SL4{
namespace hmi_message_publisher{

std::string MessageHandler::_speed_unit("mph");

    MessageHandler::MessageHandler():
    _hmi_ob_id_cache(hmi_message::ObstacleExtendedInfo_ob_id.max){}

    void MessageHandler::handleSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg){
        DLOG(INFO) << "handleSteeringReport";
        DLOG(INFO) << "speed : " << ConvertSpeedUnitFromMps(msg->speed) << " " << _speed_unit << ", steering_angle: " << msg->steering_wheel_angle;
        _data_buffer.vehicle_status_general_info.setSpeed(ConvertSpeedUnitFromMps(msg->speed));
        _data_buffer.vehicle_status_general_info.setSteeringAngle(msg->steering_wheel_angle * 57.296);

    }

    void MessageHandler::handleDbwEnabledUpdate(const std_msgs::Bool::ConstPtr& msg){
        DLOG(INFO) << "handleDbwEnabledUpdate";
        DLOG(INFO) << "dbw_enable: " << msg->data;
        if (msg->data){
             _data_buffer.vehicle_status_general_info.setEngageStatus(hmi_message::VehicleEngageStatus::ENGAGED);
        } else {
            _data_buffer.vehicle_status_general_info.setEngageStatus(hmi_message::VehicleEngageStatus::READY);
        }
    }

    void MessageHandler::handleCheckStatus(const std_msgs::String::ConstPtr& msg){
        DLOG(INFO) << "handleCheckStatus";
        std::string check_status_str = msg->data;
        // if the vehicle is engaged then keep status in engaged
        if (_data_buffer.vehicle_status_general_info.getEngageStatus() == hmi_message::VehicleEngageStatus::ENGAGED){
            DLOG(INFO) << "ENGAGED";
            return;
        }

        if(check_status_str.find("\"sick\": false") != std::string::npos){
            DLOG(INFO) << "NOT READY";
            _data_buffer.vehicle_status_general_info.setEngageStatus(hmi_message::VehicleEngageStatus::NOT_READY);
        }else{
            DLOG(INFO) << "READY";
            _data_buffer.vehicle_status_general_info.setEngageStatus(hmi_message::VehicleEngageStatus::READY);
        }

    }

    void MessageHandler::handleTurnSignalCmd(const dbw_mkz_msgs::TurnSignalCmd::ConstPtr& msg){
        DLOG(INFO) << "handleTurnSignalCmd";
        DLOG(INFO) << "turn left: " << (msg->cmd.value == dbw_mkz_msgs::TurnSignal::LEFT);
        DLOG(INFO) << "turn right: " << (msg->cmd.value == dbw_mkz_msgs::TurnSignal::RIGHT);
        _data_buffer.planning_general_info.setLeftLaneChangeCmd(msg->cmd.value == dbw_mkz_msgs::TurnSignal::LEFT);
        _data_buffer.planning_general_info.setRightLaneChangeCmd(msg->cmd.value == dbw_mkz_msgs::TurnSignal::RIGHT);
    }

    void MessageHandler::handleLanePath(const std_msgs::String::ConstPtr& msg){
        DLOG(INFO) << "handleLanePath";
        LaneDetection lane_detection;
        if (!lane_detection.ParseFromString(msg->data)) {
            DLOG(ERROR) << "failed to parse lane detection message";
            return;
        }
        DLOG(INFO) << "left lane exist: " << (lane_detection.ego_left_lane_id() != -1);
        DLOG(INFO) << "right lane exist: " << (lane_detection.ego_right_lane_id() != -1);

        _data_buffer.lane_general_info.setLeftLaneExist(lane_detection.ego_left_lane_id() != -1);
        _data_buffer.lane_general_info.setRightLaneExist(lane_detection.ego_right_lane_id() != -1);
    }

    void MessageHandler::handlePlanningTrajectory(const std_msgs::String::ConstPtr& msg){
        DLOG(INFO) << "handlePlanningTrajectory";
        PlanningTrajectory planning_trajectory;
        if (!planning_trajectory.ParseFromString(msg->data)) {
            LOG(ERROR) << "failed to parse planning trajectory message";
            return;
        }

        auto trajecotry_type = planning_trajectory.trajectory_type();
        if (trajecotry_type == drive::common::planning::PlanningTrajectory::AEB){
            DLOG(INFO) << "Trajectory type: AEB";
            _data_buffer.active_AEB = true;
        }
        else{
            _data_buffer.active_AEB = false;
        }

        if(_data_buffer.planning_general_info.getLeftLaneChangeCmd()) {
            if (trajecotry_type ==  drive::common::planning::PlanningTrajectory::LANE_CHANGE_LEFT){
                DLOG(INFO) << "Trajectory type: allow left lane change";
                _data_buffer.planning_general_info.setAllowLaneChange(true);
            }
            else{
                DLOG(INFO) << "Trajectory type: not allow left lane change";
                _data_buffer.planning_general_info.setAllowLaneChange(false);
            }
        }

        if(_data_buffer.planning_general_info.getRightLaneChangeCmd()) {
            if (trajecotry_type ==  drive::common::planning::PlanningTrajectory::LANE_CHANGE_RIGHT){
                DLOG(INFO) << "Trajectory type: allow right lane change";
                _data_buffer.planning_general_info.setAllowLaneChange(true);
            }
            else{
                DLOG(INFO) << "Trajectory type: not allow right lane change";
                _data_buffer.planning_general_info.setAllowLaneChange(false);
            }
        }

        // get speed limit from map
        auto map_interface = PlusMapUtils::PlusMapPtr();
        LaneMapNode current_map_node;
        Vec2d ego_xy(_data_buffer.pose.x, _data_buffer.pose.y);
        double road_speed_limit_mps = 0;
        if ( map_interface != nullptr and map_interface->StatefulLocate(ego_xy, current_map_node) and
            map_interface->GetSpeedLimit(current_map_node, road_speed_limit_mps,0)){

            DLOG(INFO) << "get speed_limit from map : " << ConvertSpeedUnitFromMps(road_speed_limit_mps) << " " << _speed_unit;
            _data_buffer.planning_general_info.setSpeedLimit(ConvertSpeedUnitFromMps(road_speed_limit_mps));
        }else{
            DLOG(INFO) << "can't get speed limit from map, so use previous speed_limit: " << _data_buffer.planning_general_info.getSpeedLimit() << " " << _speed_unit;
        }

    }

    void MessageHandler::handleOdom(const nav_msgs::Odometry::ConstPtr& msg) {
        DLOG(INFO) << "handleOdom";
        tf::Quaternion q(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        const tf::Matrix3x3 m(q);
        auto& pose = _data_buffer.pose;
        m.getRPY(pose.roll, pose.pitch, pose.yaw);
        pose.x = msg->pose.pose.position.x;
        pose.y = msg->pose.pose.position.y;
        pose.z = msg->pose.pose.position.z;
        pose.vx = msg->twist.twist.linear.x;
        pose.vy = msg->twist.twist.linear.y;

    }
    void MessageHandler::handleObstacles(const std_msgs::String::ConstPtr& msg){
        DLOG(INFO) << "handleObstacles";
        auto& obstacle_extended_info_vec = _data_buffer.obstacle_extenged_info_vec;
        obstacle_extended_info_vec.clear();
        ObstacleDetection obstacle_detection;
        if (!obstacle_detection.ParseFromString(msg->data)) {
            DLOG(ERROR) << "failed to parse obstacles message";
            return;
        }
        DLOG(INFO) << "obstacle size: " << obstacle_detection.obstacle().size();
        _data_buffer.obstacle_general_info.setObstacleCount(obstacle_detection.obstacle().size());
        for(const auto& obstacle : obstacle_detection.obstacle()){
            std::vector<double> imu_point;
            ConvertWorld2IMU(_data_buffer.pose, imu_point, obstacle.motion().x(), obstacle.motion().y(), obstacle.motion().z());

            // filter out the rear obstacles
            if (imu_point[0] <= 0){
                continue;
            }

            bool is_threat = false;
            // only when AEB is active, we make leading obstacle as the threat
            if(_data_buffer.active_AEB){
                if( obstacle.ego_lane_presence() == drive::common::perception::PerceptionObstacle::IN_EGOLANE_LEAD_VEHICLE){
                    DLOG(INFO) << "threat obstacle id: " << obstacle.id();
                    is_threat = true;
                }
            }

            // naive lane_assignment
            // assume lane is straight, lane_width is 3.6 and we are driving in the middle of the lane
            // TODO we should switch to https://github.com/PlusAI/drive/pull/6904 when it is ready
            int lane_assignment = 0;
            double y = imu_point[1];
            if(std::abs(imu_point[1]) > 5.4){
                // skip the obstacle, if it was not in the ego/left/right lane
                continue;
            }
            else{
                if(std::abs(imu_point[1]) <= 1.8){
                    lane_assignment = 2; // ego lane
                }
                else if(imu_point[1] > 1.8){
                    lane_assignment = 1; // left lane
                    y = imu_point[1] - 3.6;
                }else{
                    lane_assignment = 2; // right lane
                    y = imu_point[1] + 3.6;
                }
            }

            unsigned int hmi_ob_id = _hmi_ob_id_cache.put(obstacle.id());
            hmi_message::ObstacleExtendedInfo obstacle_extended_info(hmi_ob_id, lane_assignment, imu_point[0], y, obstacle.type(),is_threat);
            obstacle_extended_info_vec.push_back(obstacle_extended_info);
        }
    }

    void MessageHandler::handleLongitudinalReport(const plusai_msgs::LongitudinalControlReport& msg) {
        DLOG(INFO) << "handleLongitudinalReport";
        DLOG(INFO) << "set speed: " << ConvertSpeedUnitFromMps(msg.v_target) << " " << _speed_unit;
        _data_buffer.planning_general_info.setSetSpeed(ConvertSpeedUnitFromMps(msg.v_target));
    }

    void MessageHandler::ConvertWorld2IMU(const Pose& pose, std::vector<double>& imu_point, const double& world_x, const double& world_y, const double& world_z) {
        Eigen::Isometry3d Tr_world_to_imu = pose.isometry3d().inverse();
        Eigen::Vector3d pt_world(world_x, world_y, world_z);
        Eigen::Vector3d pt_imu = Tr_world_to_imu * pt_world;
        imu_point.resize(3);
        imu_point[0] = pt_imu.x();
        imu_point[1] = pt_imu.y();
        imu_point[2] = pt_imu.z();
    }
    int MessageHandler::ConvertSpeedUnitFromMps(const double& speed_mps){
        if(_speed_unit == "kph"){
            return hmi_message::mps_to_kph(speed_mps);
        }
        else if( _speed_unit == "mph"){
            return hmi_message::mps_to_mph(speed_mps);
        }
        else {
            LOG(ERROR) << "unkown speed unit: " << _speed_unit << ", we can only support kph or mph";
        }
        return 0;
    }
}
}
}
