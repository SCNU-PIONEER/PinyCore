#pragma once
#include "Topic.hpp"

#include "MsgImpl.hpp"

#include "INS.hpp"

class TopicRouter {
public:
    static TopicRouter &instance();

    Topic<INS_SYS::INSData_s> insTopic{ "CompleteINSData" };

    Topic<GimbalEularMsg_s> gimbalEularTopic{ "GimbalEular" };

    Topic<DeltaYawMsg_s> deltaYawTopic{ "DeltaYaw" };
};
