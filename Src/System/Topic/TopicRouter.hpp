#include "Topic.hpp"

#include "INS.hpp"

class TopicRouter {
public:
    static TopicRouter &instance();

    Topic<INS_SYS::INSData_s> insTopic{ "CompleteINSData" };
};