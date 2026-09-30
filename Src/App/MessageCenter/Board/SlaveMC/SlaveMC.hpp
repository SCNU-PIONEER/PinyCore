#pragma once

class SlaveMC {
public:
    SlaveMC() {}
    ~SlaveMC() = default;
    void slaveUpdate(void *_param);

private:
    friend class MessageCenter;
};