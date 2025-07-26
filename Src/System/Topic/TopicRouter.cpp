#include "TopicRouter.hpp"

TopicRouter &TopicRouter::instance()
{
    static TopicRouter instance;
    return instance;
}
