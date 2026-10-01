# FSM v2.0.0

## 更新日志
1. 使用模板, 去除繁琐的 static_cast
2. 添加多帧 change() 函数，用于处理重复动作

## 状态机逻辑
1. 先判断是否需要切换状态
2. 如果不需要切换，则重复运行 run()
3. 如果需要切换，则先 exit() (非阻塞，只运行一次，必然跳进下一个状态)
  - 然后运行change() (默认实现为跳过，需要多帧处理才需要)
  - 正常情况则跳进下一个状态的 enter()，然后接着run()
  - 如果change持续，则返回false, 重复执行
  - 如果change失败, 则需要返回true且设置msg状态，在下一次 checkChange() 中直接转换状态

## how to use
1. create stateFactory
```c++
StateFactory<FSMState_e> stateFactory_;
```

2. add state
```c++
stateFactory.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
stateFactory.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
stateFactory.init(stateFactory.getNextState(FSMState_e::STOP));
```

3. update
```c++
stateFactory_.update();
```

4. state change
default no change: return self state
```stopState.cpp
FSMState_e checkChange() override { return FSMState_e::STOP;}
```

if need to change more time, such as:
```cpp
bool change() override{
        changingTime_.duration = 10;
        changingTime_.current++;
        if (changingTime_.current > changingTime_.duration) {
            changingTime_.current = 0;
            msg.state = State_e::STOP;
            LOG::info("run", " change failed");
            return true;
        } else if (changingTime_.current < changingTime_.duration) {
            LOG::info("run", " changing");
            return false;
        } else {
            return false;
        }
}
```

