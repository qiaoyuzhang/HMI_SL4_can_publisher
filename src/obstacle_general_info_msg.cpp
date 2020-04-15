#include "hmi_message_publisher/obstacle_general_info_msg.h"
#include <glog/logging.h>

namespace HMI {
namespace SL4 {
namespace hmi_message {
ObstacleGeneralInfo::ObstacleGeneralInfo() {
    _data_vec.resize(ObstacleGeneralInfo_data_size);
}

ObstacleGeneralInfo::ObstacleGeneralInfo(const int& num_obstacles) {
    _data_vec.resize(ObstacleGeneralInfo_data_size);

    setObstacleCount(num_obstacles);
}

ObstacleGeneralInfo::ObstacleGeneralInfo(const VectorDataType& data_vec) {
    _data_vec = data_vec;
    _data_vec.resize(ObstacleGeneralInfo_data_size);
}

void ObstacleGeneralInfo::setObstacleCount(const int& num_obstacles) {
    ObstacleGeneralInfo_ob_num.writeValueToDataVector(_data_vec, num_obstacles);
}

bool ObstacleGeneralInfo::getObstacleCount() {
    return ObstacleGeneralInfo_ob_num.recoverValueFromDataVector(_data_vec);
}
}  // namespace hmi_message
}  // namespace SL4
}  // namespace HMI
