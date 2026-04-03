#pragma once

#include <cstdint>

#include "Crc.hpp"
#include "StmLog.hpp"


#pragma pack(push, 1)
enum class SentryPosture_e : uint8_t {
    IDLE = 0,
    ATTACK,  // 攻击，1
    DEFENSE, // 防御，2
    MOVE,    // 移动，3
}; //默认姿态为移动

//发来的原始数据 byte:27
typedef struct {
    uint8_t visionHeader;
    uint8_t control;
    uint8_t fire;
    float yaw;
    float yawVel;
    float yawAcc;
    float pitch;
    float pitchVel;
    float pitchAcc;
} VisionToEc_s;

//byte:18
typedef struct {
    uint8_t visionHeader;
    uint8_t detectColor : 1;
    uint8_t taskMode : 2;
    bool resetTracker : 1;
    bool isPlay : 1;
    bool changeTarget : 1;
    uint8_t reserved : 2;
    float roll;
    float pitch;
    float yaw;
    float bulletSpeed;
} EcToVision_s;

//byte:13
typedef struct {
    uint8_t navHeader;
    float vx;
    float vy;
    float wz;
} NavToEc_s;

//byte:5
typedef struct {
    uint8_t navHeader;
    float yaw;
} EcToNav_s;

//byte:3
typedef struct {
    uint8_t decesionHeader;
    SentryPosture_e posture;
    uint8_t useSuperCap : 1;
    uint8_t reserved : 7;
} DecesionToEc_s;

//byte:26
typedef struct {
    uint8_t decesionHeader;
    uint16_t gameTime;
    uint8_t gameProgress;          // 比赛进程
    uint32_t currentChassisEnergy; //
    uint16_t myHP;                 // 导航的血量标志位
    uint32_t outpostStatus;        //前哨站
    uint32_t enemyBaseStatus;
    uint32_t enemyOutpostStatus;
    uint32_t eventData; //比赛增益
} EcToDecesion_s;
#pragma pack(pop)

class EcToMiniPCPacket {
public:
    static constexpr uint8_t LEN = 64; // usb一次最多64字节
    static constexpr uint8_t ID = 0x5A;
    static constexpr uint8_t VISION_ID = 0x6A;
    static constexpr uint8_t NAV_ID = 0x7A;
    static constexpr uint8_t DECESION_ID = 0x8A;

    struct ProtoData_s {
        EcToVision_s e2vData;   //18
        EcToNav_s e2nData;      //5
        EcToDecesion_s e2dData; //26
        uint8_t reserved[12];   //64-52
    };

#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t header;         //1
            EcToVision_s e2vData;   //18
            EcToNav_s e2nData;      //5
            EcToDecesion_s e2dData; //26
            uint8_t reserved[12];   //64-52
            uint16_t checksum;      //2
        } content;
        uint8_t bytes[LEN];

        Data_u() : content{} {} //值初始化
    } data;
#pragma pack(pop)

    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.header = ID;

        d.e2vData = _protoData.e2vData;
        d.e2vData.visionHeader = VISION_ID;

        d.e2nData = _protoData.e2nData;
        d.e2nData.navHeader = NAV_ID;

        d.e2dData = _protoData.e2dData;
        d.e2dData.decesionHeader = DECESION_ID;

        d.checksum = Get_CRC16_Check_Sum(_data.bytes, LEN - 2, CRC16_INIT);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        (void)_data;
        (void)_protoData;
    }
};

class MiniPCToEcPacket {
public:
    static constexpr uint8_t LEN = 64; // usb一次最多64字节
    static constexpr uint8_t ID = 0xA5;
    static constexpr uint8_t VISION_ID = 0xA6;
    static constexpr uint8_t NAV_ID = 0xA7;
    static constexpr uint8_t DECESION_ID = 0xA8;

    struct ProtoData_s {
        VisionToEc_s v2eData;   //27
        NavToEc_s n2eData;      //13
        DecesionToEc_s d2eData; //3
        uint8_t reserved[18];   //64-46
    };

#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t header;         //1
            VisionToEc_s v2eData;   //27
            NavToEc_s n2eData;      //13
            DecesionToEc_s d2eData; //3
            uint8_t reserved[18];   //64-46
            uint16_t checksum;      //2
        } content;
        uint8_t bytes[LEN];

        Data_u() : content{} {} //值初始化

    } data;
#pragma pack(pop)

    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        (void)_protoData;
        (void)_data;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        if (_data.content.header == ID && Verify_CRC16_Check_Sum(_data.bytes, sizeof(Data_u))) {
            _protoData.v2eData = _data.content.v2eData;
            _protoData.v2eData.visionHeader = VISION_ID;

            _protoData.n2eData = _data.content.n2eData;
            _protoData.n2eData.navHeader = NAV_ID;

            _protoData.d2eData = _data.content.d2eData;
            _protoData.d2eData.decesionHeader = DECESION_ID;
        }
    }
};