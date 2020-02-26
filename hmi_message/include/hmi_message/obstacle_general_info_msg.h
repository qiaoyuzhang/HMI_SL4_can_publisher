#include "common/obstacle_detection.pb.h"
#include "hmi_message/const_vars.h"
#include <vector>

namespace HMI {
namespace SL4 {
namespace hmi_message{
using VectorDataType = std::vector<unsigned char>;

// PerceptionObstacle::Type protobuf
//https://github.com/PlusAI/drive/blob/master/common/proto/perception/obstacle_detection.proto#L44
static const google::protobuf::EnumDescriptor *PerceptionObstacle_descriptor = drive::common::perception::PerceptionObstacle::Type_descriptor();

/*
 * Define hmi_sl4 Obstacle Message
 * If the input was outside of the range, it would store the min/max value instead
 */

class ObstacleGeneralInfo {
    public:
        ObstacleGeneralInfo();
        ObstacleGeneralInfo(const unsigned int& ob_id, const int& dist_long, const int& dist_lat, const int& ob_class, const bool& ob_threat);
        ObstacleGeneralInfo(const VectorDataType& data_vector);   
       
        void setObId(const unsigned int& obj_id);
        void setDistLong(const double& dist_long);
        void setDistLat(const double& dist_lat);
        void setObClass(const int& ob_class);
        void setObThreat(const bool& ob_threat);
        
        unsigned int getObId();
        double getDistLong();
        double getDistLat();
        int getObClass();
        bool getObThreat();
        
        VectorDataType getVectorData() const { return _data_vec; }
    
    private:
        
        VectorDataType _data_vec; 

       
    };
}
}  // namespace SL4
}  // namespace HMI
