#pragma once

#include "./utils.hpp"
#include <cstring>
#include <cmath>

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
;
class ChassisCtrlPacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x21;
    struct ProtoData_s {
        bool gimbalReset;
        uint8_t state;
        float vx;
        float vy;
        float wz;
        float deltaYaw;
        uint8_t sentryPosture; //哨兵姿态系统
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            bool gimbalReset : 1;
            int16_t vx : 14;
            int16_t vy : 14;
            int16_t wz : 14;
            int16_t deltaYaw : 16;
            uint8_t state : 3;
            uint8_t sentryPosture : 2; //哨兵姿态系统
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int16_t>(std::fmax(-8192, std::fmin(8192, _protoData.vx * 1000.f)));
        d.vy = static_cast<int16_t>(std::fmax(-8192, std::fmin(8192, _protoData.vy * 1000.f)));
        d.wz = static_cast<int16_t>(std::fmax(-8192, std::fmin(8192, _protoData.wz * 1000.f)));
        d.state = _protoData.state;
        d.deltaYaw = convertToInt16(_protoData.deltaYaw);
        d.sentryPosture = _protoData.sentryPosture;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalReset = _data.content.gimbalReset;
        _protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        _protoData.wz = static_cast<float>(_data.content.wz) / 1000.f;
        _protoData.state = _data.content.state;
        _protoData.deltaYaw = convertToFloat(_data.content.deltaYaw);
        _protoData.sentryPosture = _data.content.sentryPosture;
    }
};

class ArmorBoosterPacketType {
public:
    static constexpr uint8_t LEN = 4;
    static constexpr uint16_t ID = 0x22;
    struct ProtoData_s {
        uint8_t state;
        bool fire;
    };
    union Data_u {
        struct Data_s {
            uint8_t state : 8;
            bool fire : 1;
            uint32_t reversed : 23;
        } content;
        uint8_t bytes[LEN];
    } data;
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.state = _protoData.state;
        d.fire = _protoData.fire;
        d.reversed = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.state = _data.content.state;
        _protoData.fire = _data.content.fire;
    }
};

} // namespace COMM
