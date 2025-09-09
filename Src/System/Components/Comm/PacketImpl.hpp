#pragma once

#include "utils.hpp"

namespace COMM {

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
    static Data_u compress(const ProtoData_s &_protoData)
    {
        Data_u data;
        Data_u::Data_s &d = data.content;
        d.gimbalReset = _protoData.gimbalReset;
        d.vx = static_cast<int16_t>(_protoData.vx * 1000.f);
        d.vy = static_cast<int16_t>(_protoData.vy * 1000.f);
        d.gimbalYaw = convertToInt16(_protoData.gimbalYaw);
        d.reversed = 0;
        return data;
    }
    static ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData;
        Data_u::Data_s d = _data.content;
        protoData.gimbalReset = d.gimbalReset;
        protoData.vx = static_cast<float>(d.vx) / 1000.f;
        protoData.vy = static_cast<float>(d.vy) / 1000.f;
        protoData.gimbalYaw = convertToFloat(d.gimbalYaw);
        return protoData;
    }
};

} // namespace COMM
