#pragma once

class MasterMC {
public:
    MasterMC() {}
    ~MasterMC() = default;
    void masterUpdate(void *_param);

private:
    friend class MessageCenter;
};