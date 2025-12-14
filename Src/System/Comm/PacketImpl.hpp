#pragma once

#include "./utils.hpp"
#include <cstring>

namespace COMM {

/**
 * @brief Synchronization Notice Packet
 * 
 */
class SyncNoticePacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x91;
    static constexpr uint8_t NOTICE_FLAG = 0x5A;
    struct ProtoData_s {
        uint8_t syncID;
        uint8_t operationFlag;
        float data;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t syncID : 8;
            uint8_t operationFlag : 8;
            int32_t data;
            uint16_t reversed : 16;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr Data_u compress(const ProtoData_s &_protoData)
    {
        Data_u data;
        Data_u::Data_s &d = data.content;
        d.syncID = ID;
        d.operationFlag = NOTICE_FLAG;
        memcpy(&d.data, &_protoData.data, sizeof(float));
        d.reversed = 0;
        return data;
    }
    static constexpr ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData;
        protoData.syncID = _data.content.syncID;
        protoData.operationFlag = _data.content.operationFlag;
        memcpy(&protoData.data, &_data.content.data, sizeof(float));
        return protoData;
    }
};

/**
 * @brief Synchronization ACK Packet
 * 
 */
class SyncACKPacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x92;
    static constexpr uint8_t ACK_FLAG = 0xA5;
    struct ProtoData_s {
        uint8_t syncID;
        uint8_t operationFlag;
        float data;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t syncID : 8;
            uint8_t operationFlag : 8;
            int32_t data;
            uint16_t reversed : 16;
        } content;
        uint8_t bytes[LEN];
    };
#pragma pack(pop)
    static constexpr Data_u compress(const ProtoData_s &_protoData)
    {
        Data_u data;
        Data_u::Data_s &d = data.content;
        d.syncID = ID;
        d.operationFlag = ACK_FLAG; // ACK flag
        memcpy(&d.data, &_protoData.data, sizeof(float));
        d.reversed = 0;
        return data;
    }
    static constexpr ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData;
        protoData.syncID = _data.content.syncID;
        protoData.operationFlag = _data.content.operationFlag;
        memcpy(&protoData.data, &_data.content.data, sizeof(float));
        return protoData;
    }
};

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
    static constexpr Data_u compress(const ProtoData_s &_protoData)
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
    static constexpr ProtoData_s decompress(const Data_u &_data)
    {
        ProtoData_s protoData;
        protoData.gimbalReset = _data.content.gimbalReset;
        protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        protoData.gimbalYaw = convertToFloat(_data.content.gimbalYaw);
        return protoData;
    }
};

} // namespace COMM
