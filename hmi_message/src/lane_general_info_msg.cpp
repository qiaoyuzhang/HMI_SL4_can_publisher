#include "hmi_message/lane_general_info_msg.h"
#include "hmi_message/utils.h"
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{
    LaneGeneralInfo::LaneGeneralInfo(){
        _data_vec.resize(LaneGeneralInfo_data_size);
    }
    
    LaneGeneralInfo::LaneGeneralInfo(const bool& left_lane_exist, const bool& right_lane_exist){ 
        _data_vec.resize(LaneGeneralInfo_data_size);

        setLeftLaneExist(left_lane_exist);
        setRightLaneExist(right_lane_exist);
    }
    
    LaneGeneralInfo::LaneGeneralInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(LaneGeneralInfo_data_size);   
    }
    void LaneGeneralInfo::setLeftLaneExist(const bool& left_lane_exist){
        unsigned int data = LaneGeneralInfo_left_lane_exist.getData(left_lane_exist);
        writeDataToDataVector(_data_vec, LaneGeneralInfo_left_lane_exist.data_start, LaneGeneralInfo_left_lane_exist.len, data);
    }    
    bool LaneGeneralInfo::getLeftLaneExist(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, LaneGeneralInfo_left_lane_exist.data_start, LaneGeneralInfo_left_lane_exist.len, data);
        return LaneGeneralInfo_left_lane_exist.recoverValue(data);
    }
    void LaneGeneralInfo::setRightLaneExist(const bool& right_lane_exist){
        unsigned int data = LaneGeneralInfo_right_lane_exist.getData(right_lane_exist);
        writeDataToDataVector(_data_vec, LaneGeneralInfo_right_lane_exist.data_start, LaneGeneralInfo_right_lane_exist.len, data);
    }    
    bool LaneGeneralInfo::getRightLaneExist(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, LaneGeneralInfo_right_lane_exist.data_start, LaneGeneralInfo_right_lane_exist.len, data);
        return LaneGeneralInfo_right_lane_exist.recoverValue(data);
    }
}
}
}
