#pragma once

#include <memory>
#include <unordered_map>

enum class FSMMode_e : uint8_t { NORMAL, CHANGE };

template <typename T> class StateFactory;

class ChangingTime {
public:
    uint16_t current = 0;  // current time since transition started
    uint16_t duration = 0; // overall transition duration
};

template <typename T> class FSMState {
public:
    FSMState(T _name) : name_(_name), nextName_(_name) {}

    virtual ~FSMState() = default;

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual T checkChange() = 0;

    /* Manages state specific change */
    virtual bool change() { return true; };

    ChangingTime changingTime_;

private:
    /* only StateFactory can touch */
    template <typename t> friend class StateFactory;

    T name_;
    T nextName_;
    FSMMode_e mode_ = FSMMode_e::NORMAL;
};


template <typename T> class StateFactory {
public:
    void init(FSMState<T> *_state)
    {
        currentState_ = _state;
        nextState_ = _state;
    }

    void addState(T _name, std::unique_ptr<FSMState<T> > _state)
    {
        stateTable[_name] = std::move(_state);
    }

    FSMState<T> *getNextState(T _next)
    {
        auto it = stateTable.find(_next);
        if (it != stateTable.end()) {
            return it->second.get();
        }
        return nullptr;
    }

    void update()
    {
        if (currentState_->mode_ == FSMMode_e::NORMAL) {
            /* check first if we need to change state */
            currentState_->nextName_ = currentState_->checkChange();
            if (currentState_->nextName_ != currentState_->name_) {
                nextState_ = getNextState(currentState_->nextName_);
                currentState_->exit();
                currentState_ = nextState_;
                currentState_->mode_ = FSMMode_e::CHANGE;
            } else
                /* no need to change state, just run */
                currentState_->run();
        } else if (currentState_->mode_ == FSMMode_e::CHANGE) {
            /* change first for specific state */
            if (currentState_->change()) {
                currentState_->enter();
                currentState_->mode_ = FSMMode_e::NORMAL;
            }
        }
    }

private:
    FSMState<T> *currentState_;
    FSMState<T> *nextState_;
    std::unordered_map<T, std::unique_ptr<FSMState<T> > > stateTable;
};
