#pragma once

#include "./utils.hpp"
#include <cstring>

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
        bool gimbalReset;
        uint8_t state;
        float vx;
        float vy;
        float deltaYaw;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            bool gimbalReset : 1;
            int16_t vx : 16;
            int16_t vy : 16;
            int16_t deltaYaw : 16;
            uint8_t state : 8;
            uint16_t reversed : 7;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static Data_u compress(const ProtoData_s &_protoData)
    {
        Data_u data;
        Data_u::Data_s &d = data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int16_t>(_protoData.vx * 1000.f);
        d.vy = static_cast<int16_t>(_protoData.vy * 1000.f);
        d.state = _protoData.state;
        d.deltaYaw = convertToInt16(_protoData.deltaYaw);
        d.reversed = 0;
        return data;
    }
    static ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData;
        protoData.gimbalReset = _data.content.gimbalReset;
        protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        protoData.state = _data.content.state;
        protoData.deltaYaw = convertToFloat(_data.content.deltaYaw);
        return protoData;
    }
};

} // namespace COMM
