# FSM

## how to use
1. create stateFactory
```c++
StateFactory stateFactory_;
```

2. add state
```c++
stateFactory_.addState("run", std::make_unique<RunState>(this));
```
3. update
```c++
stateFactory_.update();
```
4. state change
default no change: return ""
```
    std::string checkChange() override { return "";}
```
