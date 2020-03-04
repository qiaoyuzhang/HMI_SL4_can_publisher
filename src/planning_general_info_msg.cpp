#include "hmi_message_publisher/planning_general_info_msg.h"
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{
    PlanningGeneralInfo::PlanningGeneralInfo(){
        _data_vec.resize(PlanningGeneralInfo_data_size);
        setAllowLaneChange(false);
        setLeftLaneChangeCmd(false);
        setRightLaneChangeCmd(false);
        setSetSpeed(0);
        setSpeedLimit(0);
    }

    PlanningGeneralInfo::PlanningGeneralInfo(const bool& allow_lane_change, const bool& left_lane_change_cmd, const bool& right_lane_change_cmd, const double& set_speed, const double& speed_limit){
        _data_vec.resize(PlanningGeneralInfo_data_size);

        setAllowLaneChange(allow_lane_change);
        setLeftLaneChangeCmd(left_lane_change_cmd);
        setRightLaneChangeCmd(right_lane_change_cmd);
        setSetSpeed(set_speed);
        setSpeedLimit(speed_limit);
    }

    PlanningGeneralInfo::PlanningGeneralInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(PlanningGeneralInfo_data_size);
    }

    void PlanningGeneralInfo::setAllowLaneChange(const bool& allow_lane_change) {
        PlanningGeneralInfo_allow_lane_change.writeValueToDataVector(_data_vec, allow_lane_change);
    }

    bool PlanningGeneralInfo::getAllowLaneChange() {
        return PlanningGeneralInfo_allow_lane_change.recoverValueFromDataVector(_data_vec);
    }

    void PlanningGeneralInfo::setLeftLaneChangeCmd(const bool& left_lane_change_cmd) {
        PlanningGeneralInfo_left_lane_change_cmd.writeValueToDataVector(_data_vec, left_lane_change_cmd);
    }

    bool PlanningGeneralInfo::getLeftLaneChangeCmd() {
        return PlanningGeneralInfo_left_lane_change_cmd.recoverValueFromDataVector(_data_vec);
    }

    void PlanningGeneralInfo::setRightLaneChangeCmd(const bool& right_lane_change_cmd) {
        PlanningGeneralInfo_right_lane_change_cmd.writeValueToDataVector(_data_vec, right_lane_change_cmd);
    }

    bool PlanningGeneralInfo::getRightLaneChangeCmd() {
        return PlanningGeneralInfo_right_lane_change_cmd.recoverValueFromDataVector(_data_vec);
    }

    void PlanningGeneralInfo::setSetSpeed(const double& set_speed) {
        PlanningGeneralInfo_set_speed.writeValueToDataVector(_data_vec, set_speed);
    }

    double PlanningGeneralInfo::getSetSpeed() {
        return PlanningGeneralInfo_set_speed.recoverValueFromDataVector(_data_vec);
    }

    void PlanningGeneralInfo::setSpeedLimit(const double& speed_limit) {
        PlanningGeneralInfo_speed_limit.writeValueToDataVector(_data_vec, speed_limit);
    }

    double PlanningGeneralInfo::getSpeedLimit() {
        return PlanningGeneralInfo_speed_limit.recoverValueFromDataVector(_data_vec);
    }

}
}
}
