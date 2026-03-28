/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2026-03-17 16:20:33
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-18 13:04:04
 * @FilePath: \PinyDarts\Src\App\RobotModules\Gimbal\Vision\Vision.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "VisionPacketlmpl.hpp"

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
