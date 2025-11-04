#pragma once
#include "Topic.hpp"
#include "MsgImpl.hpp"
#include "INS.hpp"

inline Topic<INS_SYS::INSData_s> insTopic{ "CompleteINSData" };

inline Topic<GimbalEularMsg_s> gimbalEularTopic{ "GimbalEular" };

inline Topic<GimbalYawMsg_s> gimbalYawTopic{ "GimbalYaw" };
