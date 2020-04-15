#include <vector>
#include "hmi_message_publisher/const_vars.h"

namespace HMI {
namespace SL4 {
namespace hmi_message {
using VectorDataType = std::vector<unsigned char>;

/*
 * Define hmi_sl4 Lane general info Message
 * If the input was outside of the range, it would store the min/max value instead
 */

class LaneGeneralInfo {
  public:
    LaneGeneralInfo();
    LaneGeneralInfo(const bool& left_lane_exist, const bool& right_lane_exist);
    LaneGeneralInfo(const VectorDataType& data_vector);

    void setLeftLaneExist(const bool& left_lane_exist);
    void setRightLaneExist(const bool& right_lane_exist);

    bool getLeftLaneExist();
    bool getRightLaneExist();

    VectorDataType getVectorData() const { return _data_vec; }

  private:
    VectorDataType _data_vec;
};
}  // namespace hmi_message
}  // namespace SL4
}  // namespace HMI
