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

class ObstacleGeneralInfo {
  public:
    ObstacleGeneralInfo();
    ObstacleGeneralInfo(const int& num_obstacles);
    ObstacleGeneralInfo(const VectorDataType& data_vector);

    void setObstacleCount(const int& num_obstacles);

    bool getObstacleCount();

    VectorDataType getVectorData() const { return _data_vec; }

  private:
    VectorDataType _data_vec;
};
}  // namespace hmi_message
}  // namespace SL4
}  // namespace HMI
