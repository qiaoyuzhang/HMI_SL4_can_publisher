#include "hmi_message_publisher/vehicle_status_general_info_msg.h"
#include "hmi_message_publisher/utils.h"
#include <math.h>
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{  
   VehicleStatusGeneralInfo::VehicleStatusGeneralInfo(){
    _data_vec.resize(VehicleStatusGeneralInfo_data_size);
   }
   
   VehicleStatusGeneralInfo::VehicleStatusGeneralInfo(const int& speed, const int& steering_angle, const VehicleEngageStatus& engage_status){
    _data_vec.resize(VehicleStatusGeneralInfo_data_size);
    setSpeed(speed);
    setSteeringAngle(steering_angle);
    setEngageStatus(engage_status);     
   }
    
   VehicleStatusGeneralInfo::VehicleStatusGeneralInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(VehicleStatusGeneralInfo_data_size);   
    }
    void VehicleStatusGeneralInfo::setSpeed (const int& speed){
        unsigned int data = VehicleStatusGeneralInfo_speed.getData(speed); 
        writeDataToDataVector(_data_vec, VehicleStatusGeneralInfo_speed.data_start, VehicleStatusGeneralInfo_speed.len, data);
    }

    int VehicleStatusGeneralInfo::getSpeed(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, VehicleStatusGeneralInfo_speed.data_start, VehicleStatusGeneralInfo_speed.len, data);
        return VehicleStatusGeneralInfo_speed.recoverValue(data);
    }
    
    void VehicleStatusGeneralInfo::setSteeringAngle(const int& steering_angle){
        unsigned int data = VehicleStatusGeneralInfo_steering_angle.getData(steering_angle);
        writeDataToDataVector(_data_vec, VehicleStatusGeneralInfo_steering_angle.data_start, VehicleStatusGeneralInfo_steering_angle.len, data);
    }

    int VehicleStatusGeneralInfo::getSteeringAngle(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, VehicleStatusGeneralInfo_steering_angle.data_start, VehicleStatusGeneralInfo_steering_angle.len, data);
        return  VehicleStatusGeneralInfo_steering_angle.recoverValue(data);
    }
    
    void VehicleStatusGeneralInfo::setEngageStatus(const VehicleEngageStatus& engage_status){
        unsigned int data = VehicleStatusGeneralInfo_engage_status.getData(engage_status);
        writeDataToDataVector(_data_vec, VehicleStatusGeneralInfo_engage_status.data_start, VehicleStatusGeneralInfo_engage_status.len, data);
    }

    VehicleEngageStatus VehicleStatusGeneralInfo::getEngageStatus(){
        unsigned int data = 0;
        getDataFromDataVector(_data_vec, VehicleStatusGeneralInfo_engage_status.data_start, VehicleStatusGeneralInfo_engage_status.len, data);
        int value = VehicleStatusGeneralInfo_engage_status.recoverValue(data);
        switch(value) {
            case 0: 
                return VehicleEngageStatus::NOT_READY;
                break;
            case 1:
                return VehicleEngageStatus::READY;
                break;
            case 2: 
                return VehicleEngageStatus::ENGAGED;
                break;
            default:
                LOG(ERROR) << "get invalid EngageStatus value";
                return VehicleEngageStatus::NOT_READY;
                break;
        }
    }

}
}
}
