#pragma once
#include "Singleton.hpp"

#include "MasterMC.hpp"
#include "SlaveMC.hpp"

class MessageCenter : public Singleton<MessageCenter> {
public:
    void update(void *_param)
    {
#if MASTER_MESSAGECENTER
        masterMC.masterUpdate(_param);
#endif
#if SLAVE_MESSAGECENTER
        slaveMC.slaveUpdate(_param);
#endif
    }

private:
    friend class Singleton<MessageCenter>;

    SlaveMC slaveMC;
    MasterMC masterMC;
};