#pragma once

#include <memory>
#include <unordered_map>

enum class FSMMode_e {
    NORMAL,
    CHANGE,
    PAUSE,
};

class FSMState {
public:
    FSMState() = default;
    virtual ~FSMState() = default;

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual uint8_t checkChange() = 0;

    FSMMode_e getMode();
    uint8_t getStateName();
    uint8_t getNextStateName();

    void setMode(FSMMode_e _mode);
    void setStateName(uint8_t _name);
    void setNextStateName(uint8_t _next);

private:
    uint8_t name_;
    uint8_t nextName_;
    FSMMode_e mode_ = FSMMode_e::NORMAL;
};


class StateFactory {
public:
    void init(FSMState *_state);

    void addState(uint8_t _name, std::unique_ptr<FSMState> _state);
    void removeState(uint8_t _name);

    FSMState *getNextState(uint8_t _next);

    void setState(FSMState &_state);

    void update();

private:
    FSMState *currentState_;
    FSMState *nextState_;
    std::unordered_map<uint8_t, std::unique_ptr<FSMState> > stateTable;
};
