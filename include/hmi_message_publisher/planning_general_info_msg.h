#include "common/planning_trajectory.pb.h"
#include "hmi_message_publisher/const_vars.h"
#include <vector>

namespace HMI {
namespace SL4 {
namespace hmi_message{
using VectorDataType = std::vector<unsigned char>;

// trajectorytype protobuf:
// https://github.com/PlusAI/drive/blob/master/common/proto/planning/planning_trajectory.proto#L95
static const google::protobuf::EnumDescriptor *TrajectoryType_descriptor = drive::common::planning::PlanningTrajectory::TrajectoryType_descriptor();

/*
 * Define hmi_sl4 Planning Message
 * If the input was outside of the range, it would store the min/max value instead
 */

class PlanningGeneralInfo {
    public:
        PlanningGeneralInfo();
        PlanningGeneralInfo(const bool& allow_lane_change, const bool& left_lane_change_cmd, const bool& right_lane_change_cmd, const int& set_speed, const int& speed_limit);
        PlanningGeneralInfo(const VectorDataType& data_vector);   
       
        void setAllowLaneChange(const bool& allow_lane_change);
        void setLeftLaneChangeCmd(const bool& left_lane_change_cmd);
        void setRightLaneChangeCmd(const bool& right_lane_change_cmd);
        void setSetSpeed(const int& set_speed);
        void setSpeedLimit(const int& speed_limit);
        
        bool getAllowLaneChange();
        bool getLeftLaneChangeCmd();
        bool getRightLaneChangeCmd();
        int getSetSpeed();
        int getSpeedLimit();
        
        VectorDataType getVectorData() const { return _data_vec; }
    
    private:
        VectorDataType _data_vec; 

    };
}
}  // namespace SL4
}  // namespace HMI
