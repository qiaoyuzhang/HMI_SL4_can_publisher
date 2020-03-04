#include "hmi_message_publisher/vehicle_status_general_info_msg.h"
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

    void VehicleStatusGeneralInfo::setSpeed (const int& speed) {
        VehicleStatusGeneralInfo_speed.writeValueToDataVector(_data_vec, speed);
     }

    int VehicleStatusGeneralInfo::getSpeed() {
        return VehicleStatusGeneralInfo_speed.recoverValueFromDataVector(_data_vec);
    }

    void VehicleStatusGeneralInfo::setSteeringAngle(const int& steering_angle) {
        VehicleStatusGeneralInfo_steering_angle.writeValueToDataVector(_data_vec, steering_angle);
    }

    int VehicleStatusGeneralInfo::getSteeringAngle(){
        return VehicleStatusGeneralInfo_steering_angle.recoverValueFromDataVector(_data_vec);
    }

    void VehicleStatusGeneralInfo::setEngageStatus(const VehicleEngageStatus& engage_status) {
        VehicleStatusGeneralInfo_engage_status.writeValueToDataVector(_data_vec, engage_status);
    }

    VehicleEngageStatus VehicleStatusGeneralInfo::getEngageStatus() {
        int value = VehicleStatusGeneralInfo_engage_status.recoverValueFromDataVector(_data_vec);
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
