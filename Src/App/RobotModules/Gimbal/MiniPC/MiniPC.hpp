#pragma once

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "MiniPCPacketImpl.hpp"

namespace GIMBAL {

class MiniPC : public COMM::TxPacket<EcToMiniPCPacket>, public COMM::RxPacket<MiniPCToEcPacket, 1> {
public:
    enum class PCState_e : uint8_t {
        OFFLINE = 0u,
        ONLINE,
    } pcState = PCState_e::OFFLINE;

    MiniPC();

private:
    void transmit(EcToMiniPCPacket::Data_u &_data) final;
    void registerCallback() final;
};

} // namespace GIMBAL
