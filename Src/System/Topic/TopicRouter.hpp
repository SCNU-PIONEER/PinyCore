#pragma once
#include "Topic.hpp"
#include "MsgImpl.hpp"
#include "INS.hpp"

inline Topic<INS_SYS::INSData_s> insTopic{ "CompleteINSData" };
