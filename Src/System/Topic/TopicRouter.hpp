#pragma once
#include "Topic.hpp"

#include "MsgImpl.hpp"

#include "INS.hpp"

class TopicRouter {
public:
    static TopicRouter &instance()
    {
        static TopicRouter instance;
        return instance;
    }

    Topic<INS_SYS::INSData_s> insTopic{ "CompleteINSData" };

    Topic<GimbalYawMsg_s> gimbalEularTopic{ "GimbalEular" };

    Topic<GimbalYawMsg_s> gimbalYawTopic{ "GimbalYaw" };
};
