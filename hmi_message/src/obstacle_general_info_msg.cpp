#include "hmi_message/obstacle_general_info_msg.h"
#include "hmi_message/utils.h"
#include <math.h>
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{
    ObstacleGeneralInfo::ObstacleGeneralInfo(){
        _data_vec.resize(ObstacleGeneralInfo_data_size);
    }
    
    ObstacleGeneralInfo::ObstacleGeneralInfo(const unsigned int& ob_id, const int& dist_long, const int& dist_lat, const int&  ob_class, const bool& ob_threat){ 
        _data_vec.resize(ObstacleGeneralInfo_data_size);
    
        setObId(ob_id);
        setObClass(ob_class);
        setDistLong(dist_long);
        setDistLat(dist_lat);
        setObThreat(ob_threat);
    }
    
    ObstacleGeneralInfo::ObstacleGeneralInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(ObstacleGeneralInfo_data_size);   
    }
    
    void ObstacleGeneralInfo::setObId (const unsigned int& ob_id){
        unsigned int data = ObstacleGeneralInfo_ob_id.getData(ob_id); 
        writeDataToDataVector(_data_vec, ObstacleGeneralInfo_ob_id.data_start, ObstacleGeneralInfo_ob_id.len, data);
    }

    unsigned int ObstacleGeneralInfo::getObId(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleGeneralInfo_ob_id.data_start, ObstacleGeneralInfo_ob_id.len, data);
        return ObstacleGeneralInfo_ob_id.recoverValue(data);
    }
    
    void ObstacleGeneralInfo::setObClass(const int& ob_class){
        if (PerceptionObstacle_descriptor->FindValueByNumber(ob_class)){
            unsigned int data =  ObstacleGeneralInfo_ob_class.getData(ob_class);
            writeDataToDataVector(_data_vec,  ObstacleGeneralInfo_ob_class.data_start, ObstacleGeneralInfo_ob_class.len, data);
        }
        else{
            LOG(ERROR) << ob_class << " is not a valid PerceptionObstacle type";
        }
    }

    int ObstacleGeneralInfo::getObClass(){
        unsigned int data = 0;
        getDataFromDataVector(_data_vec, ObstacleGeneralInfo_ob_class.data_start, ObstacleGeneralInfo_ob_class.len, data);
        int number = ObstacleGeneralInfo_ob_class.recoverValue(data);
        if (PerceptionObstacle_descriptor->FindValueByNumber(number)){
            return number;
        }
        else{
            LOG(ERROR) <<"recovered " << number << " is not a valid PerceptionObstacle type";
            return -1;
        }
    }
    
    void ObstacleGeneralInfo::setDistLong(const double& dist_long){
        unsigned int data = ObstacleGeneralInfo_dist_long.getData(dist_long); 
        writeDataToDataVector(_data_vec, ObstacleGeneralInfo_dist_long.data_start, ObstacleGeneralInfo_dist_long.len, data);
    }

    double ObstacleGeneralInfo::getDistLong(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleGeneralInfo_dist_long.data_start, ObstacleGeneralInfo_dist_long.len, data);
        return ObstacleGeneralInfo_dist_long.recoverValue(data);
    }
    
    void ObstacleGeneralInfo::setDistLat(const double& dist_lat){
        unsigned int data = ObstacleGeneralInfo_dist_lat.getData(dist_lat); 
        writeDataToDataVector(_data_vec, ObstacleGeneralInfo_dist_lat.data_start, ObstacleGeneralInfo_dist_lat.len, data);
    }

    double ObstacleGeneralInfo::getDistLat(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleGeneralInfo_dist_lat.data_start, ObstacleGeneralInfo_dist_lat.len, data);
        return ObstacleGeneralInfo_dist_lat.recoverValue(data);
    }
    
    void ObstacleGeneralInfo::setObThreat(const bool& ob_threat){
        unsigned int data = ObstacleGeneralInfo_ob_threat.getData(ob_threat); 
        writeDataToDataVector(_data_vec, ObstacleGeneralInfo_ob_threat.data_start, ObstacleGeneralInfo_ob_threat.len, data);
    }
    
    bool ObstacleGeneralInfo::getObThreat(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleGeneralInfo_ob_threat.data_start, ObstacleGeneralInfo_ob_threat.len, data);
        return ObstacleGeneralInfo_ob_threat.recoverValue(data);
    }
}
}
}
