#pragma once

#include <memory>

#include "connection.h"

namespace drive {
namespace common {

/*
 * Main interfaces for writing to / reading from CAN. The underlying connection could be either
 * SocketCAN, CAN-Ethernet, etc. The connection type could be specified in launch file by parameter
 * "can_conn_type", which could be either "CAN" or "UDP" at present.
 *
 * Note, there could be multiple CANs configured in the launch file, they all share the same
 * parameter prefix, like "ifname", "can_conn_type", etc, and different postfix, the index.
 * For example, CAN0 maybe configured with "ifname0", "can_conn_type0", and CAN1 maybe configured
 * with "ifname1", "can_conn_type1".
 */
class CanInterface {
public:
    // index: which CAN it is, mainly used for reading configure.
    explicit CanInterface(const int index);

    // start CAN interface
    bool Start();
    // stop CAN interface
    bool Stop();

    // write to CAN with standard id if extended_id is false in launch file,
    // otherwise write with standard id.
    int WriteToCan(const can::MessageID msg_id, const can::VectorDataType& data);
    // write to CAN with standard id
    int WriteToCanStandard(const can::MessageID msg_id, const can::VectorDataType& data);
    // read from CAN, then store id and CAN frame data seperately
    int ReadFromCan(can::MessageID& msg_id, can::VectorDataType& data);

    // timeout eigher read error or write error based on the underlying connection
    bool Timeout() const noexcept;
    // retrieve the configured CAN name
    std::string GetIfName() const noexcept;

private:
    bool Init(const int index);
    std::unique_ptr<can::ConnectionInterface> _connection = nullptr;
};

/*
 * CanInterface's implementations are inlined and will call _connection's interfaces directly.
 */
inline bool CanInterface::Start() {
    return _connection->Start();
}
inline bool CanInterface::Stop() {
    return _connection->Stop();
}
inline int CanInterface:: WriteToCan(const can::MessageID msg_id, const can::VectorDataType& data) {
    return _connection->WriteToCan(msg_id, data, false);
}
inline int CanInterface:: WriteToCanStandard(const can::MessageID msg_id, const can::VectorDataType& data) {
    return _connection->WriteToCan(msg_id, data, true);
}
inline int CanInterface::ReadFromCan(can::MessageID& msg_id, can::VectorDataType& data) {
    return _connection->ReadFromCan(msg_id, data);
}
inline bool CanInterface::Timeout() const noexcept {
    return _connection->Timeout();
}
inline std::string CanInterface::GetIfName() const noexcept {
    return _connection->GetIfName();
}

}  // namespace common
}  // namespace drive
