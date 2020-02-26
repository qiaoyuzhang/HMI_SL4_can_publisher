#include "hmi_message_publisher/obstacle_extended_info_msg.h"
#include "hmi_message_publisher/utils.h"
#include <math.h>
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{
    ObstacleExtendedInfo::ObstacleExtendedInfo(){
        _data_vec.resize(ObstacleExtendedInfo_data_size);
    }
    
    ObstacleExtendedInfo::ObstacleExtendedInfo(const unsigned int& ob_id, const int& dist_long, const int& dist_lat, const int&  ob_class, const bool& ob_threat){ 
        _data_vec.resize(ObstacleExtendedInfo_data_size);
    
        setObId(ob_id);
        setObClass(ob_class);
        setDistLong(dist_long);
        setDistLat(dist_lat);
        setObThreat(ob_threat);
    }
    
    ObstacleExtendedInfo::ObstacleExtendedInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(ObstacleExtendedInfo_data_size);   
    }
    
    void ObstacleExtendedInfo::setObId (const unsigned int& ob_id){
        unsigned int data = ObstacleExtendedInfo_ob_id.getData(ob_id); 
        writeDataToDataVector(_data_vec, ObstacleExtendedInfo_ob_id.data_start, ObstacleExtendedInfo_ob_id.len, data);
    }

    unsigned int ObstacleExtendedInfo::getObId(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleExtendedInfo_ob_id.data_start, ObstacleExtendedInfo_ob_id.len, data);
        return ObstacleExtendedInfo_ob_id.recoverValue(data);
    }
    
    void ObstacleExtendedInfo::setObClass(const int& ob_class){
        if (PerceptionObstacle_descriptor->FindValueByNumber(ob_class)){
            unsigned int data =  ObstacleExtendedInfo_ob_class.getData(ob_class);
            writeDataToDataVector(_data_vec,  ObstacleExtendedInfo_ob_class.data_start, ObstacleExtendedInfo_ob_class.len, data);
        }
        else{
            LOG(ERROR) << ob_class << " is not a valid PerceptionObstacle type";
        }
    }

    int ObstacleExtendedInfo::getObClass(){
        unsigned int data = 0;
        getDataFromDataVector(_data_vec, ObstacleExtendedInfo_ob_class.data_start, ObstacleExtendedInfo_ob_class.len, data);
        int number = ObstacleExtendedInfo_ob_class.recoverValue(data);
        if (PerceptionObstacle_descriptor->FindValueByNumber(number)){
            return number;
        }
        else{
            LOG(ERROR) <<"recovered " << number << " is not a valid PerceptionObstacle type";
            return -1;
        }
    }
    
    void ObstacleExtendedInfo::setDistLong(const double& dist_long){
        unsigned int data = ObstacleExtendedInfo_dist_long.getData(dist_long); 
        writeDataToDataVector(_data_vec, ObstacleExtendedInfo_dist_long.data_start, ObstacleExtendedInfo_dist_long.len, data);
    }

    double ObstacleExtendedInfo::getDistLong(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleExtendedInfo_dist_long.data_start, ObstacleExtendedInfo_dist_long.len, data);
        return ObstacleExtendedInfo_dist_long.recoverValue(data);
    }
    
    void ObstacleExtendedInfo::setDistLat(const double& dist_lat){
        unsigned int data = ObstacleExtendedInfo_dist_lat.getData(dist_lat); 
        writeDataToDataVector(_data_vec, ObstacleExtendedInfo_dist_lat.data_start, ObstacleExtendedInfo_dist_lat.len, data);
    }

    double ObstacleExtendedInfo::getDistLat(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleExtendedInfo_dist_lat.data_start, ObstacleExtendedInfo_dist_lat.len, data);
        return ObstacleExtendedInfo_dist_lat.recoverValue(data);
    }
    
    void ObstacleExtendedInfo::setObThreat(const bool& ob_threat){
        unsigned int data = ObstacleExtendedInfo_ob_threat.getData(ob_threat); 
        writeDataToDataVector(_data_vec, ObstacleExtendedInfo_ob_threat.data_start, ObstacleExtendedInfo_ob_threat.len, data);
    }
    
    bool ObstacleExtendedInfo::getObThreat(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleExtendedInfo_ob_threat.data_start, ObstacleExtendedInfo_ob_threat.len, data);
        return ObstacleExtendedInfo_ob_threat.recoverValue(data);
    }
}
}
}
