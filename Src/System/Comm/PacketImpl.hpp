#pragma once

#include "./utils.hpp"
#include "RefereeProt.hpp"
#include <cstdint>
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
        bool useSuperCap;
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
            bool useSuperCap : 1;
            uint8_t reserved : 1;
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
        d.useSuperCap = _protoData.useSuperCap;
        d.reserved = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.gimbalReset = _data.content.gimbalReset;
        _protoData.vx = static_cast<float>(_data.content.vx) / 1000.f;
        _protoData.vy = static_cast<float>(_data.content.vy) / 1000.f;
        _protoData.wz = static_cast<float>(_data.content.wz) / 1000.f;
        _protoData.state = _data.content.state;
        _protoData.deltaYaw = convertToFloat(_data.content.deltaYaw);
        _protoData.useSuperCap = _data.content.useSuperCap;
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
            uint8_t fire : 1;
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

class RefereePacketType {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x23;
    struct ProtoData_s {
        uint16_t HP;
        uint16_t gameTime;
        uint8_t gameProgress;
        uint8_t robotId;
        uint8_t isPlay;
        uint32_t reserved;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint16_t HP : 10;
            uint16_t gameTime;
            uint8_t gameProgress : 4;
            uint8_t robotId : 8;
            uint8_t isPlay : 1;
            uint32_t reserved : 25;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.HP = _protoData.HP;
        d.gameTime = _protoData.gameTime;
        d.gameProgress = _protoData.gameProgress;
        d.robotId = _protoData.robotId;
        d.isPlay = _protoData.isPlay;
        d.reserved = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.HP = _data.content.HP;
        _protoData.gameTime = _data.content.gameTime;
        _protoData.gameProgress = _data.content.gameProgress;
        _protoData.robotId = _data.content.robotId;
        _protoData.isPlay = _data.content.isPlay;
    }
};

class SpeedMsgPacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x24;
    struct ProtoData_s {
        float shootSpeed;
        uint8_t bulletFreq;
        uint8_t reserved;
        uint16_t reserved1;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            float shootSpeed;
            uint8_t bulletFreq;
            uint8_t reserved;
            uint16_t reserved1;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.shootSpeed = _protoData.shootSpeed;
        d.bulletFreq = _protoData.bulletFreq;
        d.reserved = 0;
        d.reserved1 = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.shootSpeed = _data.content.shootSpeed;
        _protoData.bulletFreq = _data.content.bulletFreq;
        _protoData.reserved = _data.content.reserved;
        _protoData.reserved1 = _data.content.reserved1;
    }
};

// UL专用,英雄1,步兵2,哨兵7
/*
struct MapCommand_s {
    float targetPositionX;
    float targetPositionY;
    uint8_t cmdKeyboard;
    uint8_t targetRobotId;
    uint16_t cmdSource;
};
*/
class DecesionPacket1 {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x25;
    struct ProtoData_s {
        uint8_t currentChassisEnergy; // 当前底盘能量
        uint16_t allyOutpostHP;       // 己方前哨站血量
        uint16_t allyBaseHP;          // 己方基地血量
        uint8_t reserved[3];
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t currentChassisEnergy; // 当前底盘能量
            uint16_t allyOutpostHP;       // 己方前哨站血量
            uint16_t allyBaseHP;          // 己方基地血量
            uint8_t reserved[3];
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.currentChassisEnergy = _protoData.currentChassisEnergy;
        d.allyOutpostHP = _protoData.allyOutpostHP;
        d.allyBaseHP = _protoData.allyBaseHP;
        d.reserved[0] = 0;
        d.reserved[1] = 0;
        d.reserved[2] = 0;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.currentChassisEnergy = _data.content.currentChassisEnergy;
        _protoData.allyOutpostHP = _data.content.allyOutpostHP;
        _protoData.allyBaseHP = _data.content.allyBaseHP;
        _protoData.reserved[0] = _data.content.reserved[0];
        _protoData.reserved[1] = _data.content.reserved[1];
        _protoData.reserved[2] = _data.content.reserved[2];
    }
};

// class DecesionPacket2 {
// public:
//     static constexpr uint8_t LEN = 8;
//     static constexpr uint16_t ID = 0x26;
//     struct ProtoData_s {
//         REFEREE::MapCommand_s mapCommand;
//     };
// #pragma pack(push, 1)
//     union Data_u {
//         struct Data_s {
//             REFEREE::MapCommand_s mapCommand;
//         } content;
//         uint8_t bytes[LEN];
//     } data;
// #pragma pack(pop)
//     static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
//     {
//         Data_u::Data_s &d = _data.content;
//         d.mapCommand = _protoData.mapCommand;
//     }
//     static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
//     {
//         _protoData.mapCommand = _data.content.mapCommand;
//     }
// };


class SentryInfoPacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x29;
    struct ProtoData_s {
        REFEREE::SentryInfo_s midSentryInfo; // 哨兵裁判系统数据 byte:6
        uint8_t hurtHpDeductionReason;       // 受伤血量扣除原因  byte:1
        uint8_t reserved;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            REFEREE::SentryInfo_s midSentryInfo; // 哨兵裁判系统数据 byte:6
            uint8_t hurtHpDeductionReason;       // 受伤血量扣除原因  byte:1
            uint8_t reserved;
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.midSentryInfo = _protoData.midSentryInfo;
        d.hurtHpDeductionReason = _protoData.hurtHpDeductionReason;
        d.reserved = _protoData.reserved;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.midSentryInfo = _data.content.midSentryInfo;
        _protoData.hurtHpDeductionReason = _data.content.hurtHpDeductionReason;
        _protoData.reserved = _data.content.reserved;
    }
};

class SentryCmdPacket {
public:
    static constexpr uint8_t LEN = 8;
    static constexpr uint16_t ID = 0x30;
    struct ProtoData_s {
        REFEREE::SentryCmd_s midSentryCmd;
        bool isBumpArea;
    };
#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            REFEREE::SentryCmd_s midSentryCmd;
            bool isBumpArea;
            uint8_t reserved[3];
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop)
    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.midSentryCmd = _protoData.midSentryCmd;
        d.isBumpArea = _protoData.isBumpArea;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        _protoData.midSentryCmd = _data.content.midSentryCmd;
        _protoData.isBumpArea = _data.content.isBumpArea;
    }
};

} // namespace COMM
