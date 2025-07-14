#include "FSMState.hpp"


FSMMode_e FSMState::getMode() { return mode_; };

uint8_t FSMState::getStateName() { return name_; };

uint8_t FSMState::getNextStateName() { return nextName_; };

void FSMState::setMode(FSMMode_e _mode) { mode_ = _mode; };

void FSMState::setStateName(uint8_t _name) { name_ = _name; };

void FSMState::setNextStateName(uint8_t _next)
{
    if (_next != -1)
        nextName_ = _next;
    else
        nextName_ = name_;
};

void StateFactory::init(FSMState *_state)
{
    currentState_ = _state;
    nextState_ = _state;
}

void StateFactory::addState(uint8_t _name, std::unique_ptr<FSMState> _state)
{
    stateTable[_name] = std::move(_state);
}

void StateFactory::removeState(uint8_t _name) { stateTable.erase(_name); }

FSMState *StateFactory::getNextState(uint8_t _next)
{
    auto it = stateTable.find(_next);
    if (it != stateTable.end()) {
        return it->second.get();
    }
    return nullptr;
}


void StateFactory::setState(FSMState &_state) { currentState_ = &_state; }

void StateFactory::update()
{
    if (currentState_ == nullptr) {
        return; // No state to update
    }
    if (currentState_->getMode() == FSMMode_e::NORMAL) {
        currentState_->run();
        currentState_->setNextStateName(currentState_->checkChange());
        if (currentState_->getNextStateName() !=
            currentState_->getStateName()) {
            uint8_t nextName = currentState_->getNextStateName();
            nextState_ = getNextState(nextName);

            currentState_->setMode(FSMMode_e::CHANGE);
        }
    } else if (currentState_->getMode() == FSMMode_e::CHANGE) {
        currentState_->exit();
        currentState_ = std::move(nextState_);
        if (currentState_) {
            currentState_->enter();
        }
        currentState_->setMode(FSMMode_e::NORMAL);
    } else if (currentState_->getMode() == FSMMode_e::PAUSE) {
        // do nothing
    } else {
        // error
    }
}
