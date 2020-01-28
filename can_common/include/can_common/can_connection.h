#pragma once

#include "can_common/connection.h"

namespace drive {
namespace common {
namespace can {

/*
 * Direct connection with CAN throuth SocketCAN
 */
class CANConnection final : public ConnectionInterface {
  public:
    explicit CANConnection(const int index) : ConnectionInterface(index) {}
    ~CANConnection();

    bool Initialize(const ros::NodeHandle& config_source) override;
    int WriteToCan(const MessageID msg_id, const VectorDataType& data, const bool standard = false) override;
    int ReadFromCan(MessageID& msg_id, VectorDataType& data) override;

  private:
    int _socket_fd = -1;
    bool _extended_id = false;
};

}  // namespace can
}  // namespace common
}  // namespace drive
