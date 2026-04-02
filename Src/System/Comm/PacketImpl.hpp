#pragma once

#include "./utils.hpp"
#include "MsgImpl.hpp"

namespace COMM {

/**
 * @brief Sample Packet (Gimbal -> Chassis)
 * 
 */
class SamplePacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x21;
    struct ProtoData_s {
        bool gimbalReset;
        float vx;
        float vy;
        float gimbalYaw;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            bool gimbalReset : 1;
            int16_t vx : 16;
            int16_t vy : 16;
            int16_t gimbalYaw : 16;
            uint16_t reversed : 15;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int16_t>(_protoData.vx * 1000.f);
        d.vy = static_cast<int16_t>(_protoData.vy * 1000.f);
        d.gimbalYaw = convertToInt16(_protoData.gimbalYaw);
        d.reversed = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalReset = _data.content.gimbalReset;
        _protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        _protoData.gimbalYaw = convertToFloat(_data.content.gimbalYaw);
    }
};

class ChassisCtrlPacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x21;
    struct ProtoData_s {
        CHASSIS::FSMState_e state;
        float vx;
        float vy;
        float deltaYaw;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t state : 3;
            int16_t vx : 16;
            int16_t vy : 16;
            int16_t deltaYaw : 16;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.vx = static_cast<int16_t>(_protoData.vx * 1000.f);
        d.vy = static_cast<int16_t>(_protoData.vy * 1000.f);
        d.state = (uint8_t)_protoData.state;
        d.deltaYaw = convertToInt16(_protoData.deltaYaw);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        _protoData.state = (CHASSIS::FSMState_e)_data.content.state;
        _protoData.deltaYaw = convertToFloat(_data.content.deltaYaw);
    }
};

class UIGimbalMsgPacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x22;
    struct ProtoData_s {
        GIMBAL::FSMState_e gimbalState;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t gimbalState;
            uint8_t reversed[7];
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        _data.content.gimbalState = static_cast<uint8_t>(_protoData.gimbalState);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalState = static_cast<GIMBAL::FSMState_e>(_data.content.gimbalState);
    }
};

class UIArmMsgPacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x23;
    struct ProtoData_s {
        ARM::FSMState_e armState = ARM::FSMState_e::STOP;
         ARM::ClampState_e clampState = ARM::ClampState_e::NONE;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t armState;
            uint8_t clampState;
            uint8_t reversed[6];
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        _data.content.armState = static_cast<uint8_t>(_protoData.armState);
        _data.content.clampState = static_cast<uint8_t>(_protoData.clampState);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.armState = static_cast<ARM::FSMState_e>(_data.content.armState);
        _protoData.clampState = static_cast<ARM::ClampState_e>(_data.content.clampState);
    }
};


} // namespace COMM
