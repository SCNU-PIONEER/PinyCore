#include "FSMState.hpp"


FSMMode_e FSMState::getMode() { return mode_; };

std::string FSMState::getStateName() { return stateName; };

std::string FSMState::getNextStateName() { return nextStateName; };

void FSMState::setMode(FSMMode_e _mode) { mode_ = _mode; };

void FSMState::setStateName(std::string _stateName) { stateName = _stateName; };

void FSMState::setNextStateName(std::string _next)
{
    if (_next != "")
        nextStateName = _next;
    else
        nextStateName = stateName;
};

void StateFactory::init(FSMState *state)
{
    currentState_ = state;
    nextState_ = state;
}

void StateFactory::addState(std::string _name, std::unique_ptr<FSMState> _state)
{
    stateTable[_name] = std::move(_state);
}

FSMState *StateFactory::getNextState(std::string _next)
{
    auto it = stateTable.find(_next);
    if (it != stateTable.end()) {
        return it->second.get();
    }
    return nullptr;
}

void StateFactory::setState(FSMState *state) { currentState_ = state; }

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
            std::string nextName = currentState_->getNextStateName();
            nextState_ = getNextState(nextName);

            currentState_->setMode(FSMMode_e::CHANGE);
        }
    } else if (currentState_->getMode() == FSMMode_e::CHANGE) {
        currentState_->exit();
        currentState_ = nextState_;
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
