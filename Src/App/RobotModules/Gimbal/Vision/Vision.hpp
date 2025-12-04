#pragma once

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "VisionPacketImpl.hpp"

namespace GIMBAL {

class Vision : public COMM::TxPacket<VisionTxPacket>, public COMM::RxPacket<VisionRxPacket, 3> {
public:
    enum class PCState_e : uint8_t {
        OFFLINE = 0u,
        ONLINE,
    } pcState = PCState_e::OFFLINE;

    Vision();

private:
    void transmit(uint8_t *_buf, uint16_t _len) final;
    void registerCallback() final;
};

} // namespace GIMBAL
