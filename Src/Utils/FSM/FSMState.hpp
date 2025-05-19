#pragma once

#include <string>
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
    virtual std::string checkChange() = 0;

    FSMMode_e getMode();
    std::string getStateName();
    std::string getNextStateName();

    void setMode(FSMMode_e _mode);
    void setStateName(std::string _stateName);
    void setNextStateName(std::string _next);

private:
    std::string stateName;
    std::string nextStateName;
    FSMMode_e mode_ = FSMMode_e::NORMAL;
};


class StateFactory {
public:
    void init(FSMState *state);

    void addState(std::string _name, std::unique_ptr<FSMState> _state);

    FSMState *getNextState(std::string _next);

    void setState(FSMState *state);

    void update();

private:
    FSMState *currentState_;
    FSMState *nextState_;
    std::unordered_map<std::string, std::unique_ptr<FSMState> > stateTable;
};
