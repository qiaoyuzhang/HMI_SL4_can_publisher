#include "hmi_message_publisher/obstacle_general_info_msg.h"
#include "hmi_message_publisher/utils.h"
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{
    ObstacleGeneralInfo::ObstacleGeneralInfo(){
        _data_vec.resize(ObstacleGeneralInfo_data_size);
    }
    
    ObstacleGeneralInfo::ObstacleGeneralInfo(const int& num_obstacles){ 
        _data_vec.resize(ObstacleGeneralInfo_data_size);

        setObstacleCount(num_obstacles);
    }
    
    ObstacleGeneralInfo::ObstacleGeneralInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(ObstacleGeneralInfo_data_size);   
    }

    void ObstacleGeneralInfo::setObstacleCount(const int& num_obstacles){
        unsigned int data = ObstacleGeneralInfo_ob_num.getData(num_obstacles);
        writeDataToDataVector(_data_vec, ObstacleGeneralInfo_ob_num.data_start, ObstacleGeneralInfo_ob_num.len, data);
    }

    bool ObstacleGeneralInfo::getObstacleCount(){
        unsigned int data = 0; 
        getDataFromDataVector(_data_vec, ObstacleGeneralInfo_ob_num.data_start, ObstacleGeneralInfo_ob_num.len, data);
        return ObstacleGeneralInfo_ob_num.recoverValue(data);
    }
}
}
}
