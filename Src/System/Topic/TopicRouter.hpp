#include "Topic.hpp"

#include "INS.hpp"

class TopicRouter {
public:
    static TopicRouter &instance();

    Topic<INS_SYS::INSData_s> insTopic = Topic<INS_SYS::INSData_s>("INS");
};