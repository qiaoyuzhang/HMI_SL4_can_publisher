#include "hmi_message_publisher/obstacle_extended_info_msg.h"
#include <math.h>
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message{
    ObstacleExtendedInfo::ObstacleExtendedInfo(){
        _data_vec.resize(ObstacleExtendedInfo_data_size);
    }

    ObstacleExtendedInfo::ObstacleExtendedInfo(const unsigned int& ob_id, const unsigned int& lane_assignment, const double& dist_long, const double& dist_lat, const int&  ob_class, const bool& ob_threat){
        _data_vec.resize(ObstacleExtendedInfo_data_size);
        setObId(ob_id);
        setLaneAssignment(lane_assignment);
        setObClass(ob_class);
        setDistLong(dist_long);
        setDistLat(dist_lat);
        setObThreat(ob_threat);
    }

    ObstacleExtendedInfo::ObstacleExtendedInfo(const VectorDataType& data_vec){
        _data_vec = data_vec;
        _data_vec.resize(ObstacleExtendedInfo_data_size);
    }

    void ObstacleExtendedInfo::setObId (const unsigned int& ob_id) {
        ObstacleExtendedInfo_ob_id.writeValueToDataVector(_data_vec, ob_id);
    }

    unsigned int ObstacleExtendedInfo::getObId() {
        return ObstacleExtendedInfo_ob_id.recoverValueFromDataVector(_data_vec);
    }

    void ObstacleExtendedInfo::setLaneAssignment(const unsigned int& lane_assignment) {
        ObstacleExtendedInfo_lane_assignment.writeValueToDataVector(_data_vec, lane_assignment);
    }

    unsigned int ObstacleExtendedInfo::getLaneAssignment() {
        return ObstacleExtendedInfo_lane_assignment.recoverValueFromDataVector(_data_vec);
    }

    void ObstacleExtendedInfo::setObClass(const int& ob_class) {
        if (PerceptionObstacle_descriptor->FindValueByNumber(ob_class)){
            ObstacleExtendedInfo_ob_class.writeValueToDataVector(_data_vec, ob_class);
        }
        else{
            LOG(ERROR) << ob_class << " is not a valid PerceptionObstacle type";
        }
    }

    int ObstacleExtendedInfo::getObClass() {
        int number = ObstacleExtendedInfo_ob_class.recoverValueFromDataVector(_data_vec);
        if (PerceptionObstacle_descriptor->FindValueByNumber(number)){
            return number;
        }
        else{
            LOG(ERROR) <<"recovered " << number << " is not a valid PerceptionObstacle type";
            return -1;
        }
    }

    void ObstacleExtendedInfo::setDistLong(const double& dist_long) {
        ObstacleExtendedInfo_dist_long.writeValueToDataVector(_data_vec, dist_long);
    }

    double ObstacleExtendedInfo::getDistLong() {
        return ObstacleExtendedInfo_dist_long.recoverValueFromDataVector(_data_vec);
    }

    void ObstacleExtendedInfo::setDistLat(const double& dist_lat) {
        ObstacleExtendedInfo_dist_lat.writeValueToDataVector(_data_vec, dist_lat);
    }

    double ObstacleExtendedInfo::getDistLat() {
        return ObstacleExtendedInfo_dist_lat.recoverValueFromDataVector(_data_vec);
    }

    void ObstacleExtendedInfo::setObThreat(const bool& ob_threat) {
       ObstacleExtendedInfo_ob_threat.writeValueToDataVector(_data_vec, ob_threat);
    }

    bool ObstacleExtendedInfo::getObThreat() {
        return ObstacleExtendedInfo_ob_threat.recoverValueFromDataVector(_data_vec);
    }
}
}
}
