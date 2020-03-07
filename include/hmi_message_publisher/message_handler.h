#pragma once
#include "hmi_message_publisher/data_buffer.h"

namespace HMI{
namespace SL4{
namespace hmi_message_publisher{

class MessageHandler{

  public:
    MessageHandler();

    void handleObstacles(const std_msgs::String::ConstPtr& msg);
    void handleLanePath(const std_msgs::String::ConstPtr& msg);
    void handleSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg);
    void handleTurnSignalCmd(const dbw_mkz_msgs::TurnSignalCmd::ConstPtr& msg);
    void handleDbwEnabledUpdate(const std_msgs::Bool::ConstPtr& msg);
    void handlePlanningTrajectory(const std_msgs::String::ConstPtr& msg);
    void handleCheckStatus(const std_msgs::String::ConstPtr& msg);
    void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void handleLongitudinalReport(const plusai_msgs::LongitudinalControlReport& msg);
    static std::string &SpeedUnit() {return _speed_unit;};

  private:
    void ConvertWorld2IMU(const drive::common::math::Pose& pose, std::vector<double>& imu_point, const double& world_x, const double& would_y, const double& wold_z);
    int ConvertSpeedUnitFromMps(const double& speed_mps);

    static std::string _speed_unit;

  protected:
    DataBuffer _data_buffer;
    hmi_message::LRUCache _hmi_ob_id_cache;
};

}
}
}
