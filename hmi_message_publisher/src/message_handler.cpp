#include "hmi_message_publisher/message_handler.h"
#include "common/lane_detection.pb.h"
#include "common/obstacle_detection.pb.h"
#include "common/planning_trajectory.pb.h"
#include <glog/logging.h>

using namespace drive::common::perception;
using namespace drive::common::planning;
using namespace drive::common::math;

namespace HMI{
namespace SL4{
namespace hmi_message_publisher{
    
    MessageHandler::MessageHandler(){}
    
    void MessageHandler::handleSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg){
        DLOG(INFO) << "handleSteeringReport";
        DLOG(INFO) << "speed: " << msg->speed << " steering_angle: " << msg->steering_wheel_angle;    
        _data_buffer.vehicle_status_general_info.setSpeed(msg->speed);
        _data_buffer.vehicle_status_general_info.setSteeringAngle(msg->steering_wheel_angle);

    }

    void MessageHandler::handleDbwEnabledUpdate(const std_msgs::Bool::ConstPtr& msg){
        DLOG(INFO) << "handleDbwEnabledUpdate";
        DLOG(INFO) << "dbw_enable: " << msg->data;
        if (msg->data){
             _data_buffer.vehicle_status_general_info.setEngageStatus(hmi_message::VehicleEngageStatus::ENGAGED);
        }
    }

    void MessageHandler::handleCheckStatus(const std_msgs::String::ConstPtr& msg){
        DLOG(INFO) << "handleCheckStatus";
        std::string check_status_str = msg->data;
        // if the vehicle is engaged then keep status in engaged
        if (  _data_buffer.vehicle_status_general_info.getEngageStatus() == hmi_message::VehicleEngageStatus::ENGAGED){
            DLOG(INFO) << "ENGAGED";
            return;
        }

        if( check_status_str.find("\"sick\": false") != std::string::npos){
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
            LOG(ERROR) << "failed to parse lane detection message";
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
        DLOG(INFO) << "speed limit: " << planning_trajectory.speed_limit_used();
        _data_buffer.planning_general_info.setSpeedLimit(planning_trajectory.speed_limit_used());
        auto trajecotry_type = planning_trajectory.trajectory_type();
        
        if (trajecotry_type == drive::common::planning::PlanningTrajectory::AEB){
            DLOG(INFO) << "Trajectory type: AEB";
            _data_buffer.active_AEB = true;
            return;
        }
        else{
            _data_buffer.active_AEB = false;
            return;
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
            return;
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
            return;
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
        auto& obstacle_general_info_vec = _data_buffer.obstacle_general_info_vec;
        obstacle_general_info_vec.clear();
        ObstacleDetection obstacle_detection;
        if (!obstacle_detection.ParseFromString(msg->data)) {
            LOG(ERROR) << "failed to parse obstacles message";
            return;
        }
        DLOG(INFO) << "obstacle size: " << obstacle_detection.obstacle().size();
        for(const auto& obstacle : obstacle_detection.obstacle()){
            std::vector<double> imu_point;
            ConvertWorld2IMU(_data_buffer.pose, imu_point, obstacle.motion().x(), obstacle.motion().y(), obstacle.motion().z());                      
            bool is_threat = false;
            // only when AEB is active, we make leading obstacle as the threat
            if(_data_buffer.active_AEB){
                if( obstacle.ego_lane_presence() == drive::common::perception::PerceptionObstacle::IN_EGOLANE_LEAD_VEHICLE){
                    DLOG(INFO) << "threat obstacle id: " << obstacle.id();
                    is_threat = true;
                }
            }
            hmi_message::ObstacleGeneralInfo obstacle_general_info(obstacle.id(), imu_point[0], imu_point[1], obstacle.type(),is_threat); 
            obstacle_general_info_vec.push_back(obstacle_general_info);
        }
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
                                                                        
}
}
}
