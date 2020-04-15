#include <vector>
#include "hmi_message_publisher/const_vars.h"

namespace HMI {
namespace SL4 {
namespace hmi_message {
using VectorDataType = std::vector<unsigned char>;

enum VehicleEngageStatus { NOT_READY = 0, READY = 1, ENGAGED = 2 };
/*
 * Define hmi_sl4  VechicleStatus Message
 * If the input was outside of the range, it would store the min/max value instead
 */

class VehicleStatusGeneralInfo {
  public:
    VehicleStatusGeneralInfo();
    VehicleStatusGeneralInfo(const int& speed,
                             const int& steering_angle,
                             const VehicleEngageStatus& engage_status);
    VehicleStatusGeneralInfo(const VectorDataType& data);

    void setSpeed(const int& speed);
    void setSteeringAngle(const int& steering_angle);
    void setEngageStatus(const VehicleEngageStatus& engage_status);

    int getSpeed();
    int getSteeringAngle();
    VehicleEngageStatus getEngageStatus();

    VectorDataType getVectorData() const { return _data_vec; }

  private:
    VectorDataType _data_vec;
};
}  // namespace hmi_message
}  // namespace SL4
}  // namespace HMI
