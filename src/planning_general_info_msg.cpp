#include "hmi_message_publisher/planning_general_info_msg.h"
#include "hmi_message_publisher/utils.h"
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
    
    void PlanningGeneralInfo::setAllowLaneChange(const bool& allow_lane_change){
        unsigned int data = PlanningGeneralInfo_allow_lane_change.getData(allow_lane_change); 
        writeDataToDataVector(_data_vec, PlanningGeneralInfo_allow_lane_change.data_start, PlanningGeneralInfo_allow_lane_change.len, data);
    }

    bool PlanningGeneralInfo::getAllowLaneChange(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, PlanningGeneralInfo_allow_lane_change.data_start, PlanningGeneralInfo_allow_lane_change.len, data);
        return PlanningGeneralInfo_allow_lane_change.recoverValue(data);
    }

    void PlanningGeneralInfo::setLeftLaneChangeCmd(const bool& left_lane_change_cmd){
        unsigned int data = PlanningGeneralInfo_left_lane_change_cmd.getData(left_lane_change_cmd); 
        writeDataToDataVector(_data_vec, PlanningGeneralInfo_left_lane_change_cmd.data_start, PlanningGeneralInfo_left_lane_change_cmd.len, data);
    }

    bool PlanningGeneralInfo::getLeftLaneChangeCmd(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, PlanningGeneralInfo_left_lane_change_cmd.data_start, PlanningGeneralInfo_left_lane_change_cmd.len, data);
        return PlanningGeneralInfo_left_lane_change_cmd.recoverValue(data);
    }   
    
    void PlanningGeneralInfo::setRightLaneChangeCmd(const bool& right_lane_change_cmd){
        unsigned int data = PlanningGeneralInfo_right_lane_change_cmd.getData(right_lane_change_cmd); 
        writeDataToDataVector(_data_vec, PlanningGeneralInfo_right_lane_change_cmd.data_start, PlanningGeneralInfo_right_lane_change_cmd.len, data);
    }

    bool PlanningGeneralInfo::getRightLaneChangeCmd(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, PlanningGeneralInfo_right_lane_change_cmd.data_start, PlanningGeneralInfo_right_lane_change_cmd.len, data);
        return PlanningGeneralInfo_right_lane_change_cmd.recoverValue(data);
    }

    void PlanningGeneralInfo::setSetSpeed(const double& set_speed) {
        unsigned int data = PlanningGeneralInfo_set_speed.getData(set_speed);
        writeDataToDataVector(_data_vec, PlanningGeneralInfo_set_speed.data_start, PlanningGeneralInfo_set_speed.len, data);
    }

    double PlanningGeneralInfo::getSetSpeed() {
        unsigned int data = 0;
        getDataFromDataVector(_data_vec, PlanningGeneralInfo_set_speed.data_start, PlanningGeneralInfo_set_speed.len, data);
        return PlanningGeneralInfo_set_speed.recoverValue(data);
    }

    void PlanningGeneralInfo::setSpeedLimit(const double& speed_limit){
        unsigned int data = PlanningGeneralInfo_speed_limit.getData(speed_limit); 
        writeDataToDataVector(_data_vec, PlanningGeneralInfo_speed_limit.data_start, PlanningGeneralInfo_speed_limit.len, data);
    }

    double PlanningGeneralInfo::getSpeedLimit(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, PlanningGeneralInfo_speed_limit.data_start, PlanningGeneralInfo_speed_limit.len, data);
        return PlanningGeneralInfo_speed_limit.recoverValue(data);
    }
    
}    
}
}
