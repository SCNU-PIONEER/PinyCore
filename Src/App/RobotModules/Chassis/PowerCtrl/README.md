# PowerCtrl v3.1.0

## 更新
1. 功率控制输出由电流变为力矩
2. 增加模型参数切换
3. 更新日志

## 主要成员变量
```c++
PowerController抽象类里：

std::vector<float> cmdPower_; // 原闭环控制器所设定的功率
float powerRatio = 1.f;      // 功率分配比例
float maxPower = 0.f;        // 允许最大输出功率
std::vector<float> setPower_; // 功率控制后所得的功率
std::vector<float> setTorq_;  // 最终设定输出力矩
CAP *cap_;
```
派生类里(如QuadricycleController):
```c++
PowerModel_s Wheel            //底盘模型结构体
PowerModel_s::ModelParam_s LaunchMotion 
PowerModel_s::ModelParam_s UniformMotion   //底盘模型参数
RLS<PowerModel_s::FIT_RANK> *wheelRLS_    

static constexpr float VEL_THESHOLD = 200.f;  //切换模型参数速度阈值

//RLS拟合的为整个机构的模型参数，如底盘有不同结构
//如哨兵有舵则需定义不同PowerModel_s和RLS,如swerveController

    RLS<PowerModel_s::FIT_RANK> *wheelRLS_;
#if ENABLE_STEER_RLS
    RLS<PowerModel_s::FIT_RANK> *steerRLS_;
#endif

```
## 控制逻辑
1. 更新裁判系统缓冲能量，根据电容实际能量占比与期望能量占比计算允许最大输出功率
2. 各型号电机计算各自原始功率，判断是否超功率，若超功率则按比例分配功率
3. 依照比例分配完功率后，根据各型号电机的模型参数，计算最终设定输出力矩
4. RLS动态拟合，更新模型参数

## 函数
```c++
update()          //更新最大输出功率和裁判系统数据
cmdPowerCalc()    //计算模型原始功率
relPowerCalc()    //计算模型实际功率(与反馈功率比较观察模型是否拟合)
torqueCalc()      //计算最终设定输出力矩
rlsUpdate()       //RLS动态拟合
```

## 使用
前提：kconfig中开启PowerCtrl， superCap， referee以及PowerCtrl中的rls,

1. create PowerController 并new
```c++
hpp中
SwerveController *powerCtrl_;(使用对应控制器)
CAP cap_{&HCAN1};(使用对应can)

cpp中
#if APP_USE_POWERCTRL
    powerCtrl_ = new SwerveController(ChassisType_e::SWERVE, &cap_);
#endif
```

2. 在底盘的update()里调用
```c++
#if APP_USE_POWERCTRL
    cap_.capTask(powerCtrl_->capEnable_, powerCtrl_->systemRestart_, powerCtrl_->clearError_, powerCtrl_->enableCharge_,
                 powerCtrl_->chargeRatioLimit_, refereeMsg.chassisPowerLimit, refereeMsg.chassisPowerBuffer);
#endif
    调用后确保超电数据接收正常，如不正常大多是因为超电的can设置错误,数据不正常需要重烧功率板代码
```

3. ctrl
功率控制需自己计算pid，不能使用电机库封装的pid计算
```c++

    float cmd[4];
    for (uint8_t i = 0; i < 4; i++) {
        velPid_[i].torq =
                velPid_[i].pid->calc(refWSpeed._[i], wSpeed_._[i]);
        cmd[i] = velPid_[i].torq;
    }
    std::vector<float> torq;
    torq = powerCtrl_->powerCtrl(refWSpeed._, motors_._, cmd, refereeMsg);

    for (uint8_t i = 0; i < 4; i++) {
#if APP_USE_POWERCTRL
        motors_._[i]->cmdTorq(torq[i]);
#else
        motors_._[i]->cmdTorq(cmd[i]);
#endif
    }

    注意：
    powerCtrl_->powerCtrl(refWSpeed._, &motors_._[0], cmd, refereeMsg)的四个参数
    1.refWSpeed的单位需为rads/s，如单位为rpm则传入需变单位
    2. motors_._为电机指针数组
    3.cmd为pid计算得到的力矩
    4.refereeMsg 裁判系统msg，需手动        
    if (xQueueReceive((((MsgBus_s *)_param)->refereeQueue), &refereeMsg, 0) ==
            pdTRUE) {
        };   
```

## 调试(建议使用本手册调试方法，确保功率控制效果)
调试需在freemaster等软件中观察

1. 调底盘pid参数，实际跟随效果要好，但不能太硬
先使用正常pid输出的力矩
```c++
#if APP_USE_POWERCTRL
        motors_._[i]->cmdTorq(cmd[i]);
#else
        motors_._[i]->cmdTorq(cmd[i]);
#endif
注：调太软会让底盘达不到功率墙
```
2. 静态功耗参数

首先先断控,看chassisRealPower的值,填到PowerModel_s::ModelParam_s LaunchMotion 
和UniformMotion的LeakagePower中(LeakagePower为静态功耗建议取中间值偏上，因为有噪声)

LeakagePower的作用为在没有速度时保证拟合效果，此时需要拟合的参数数据此时趋近0
而实际上有功耗，因此在减去静态功耗后就可以保证拟合效果

如果像舵轮这种有两个底盘模块时需分别断电不同模块电机,分别填入对应的LeakagePower中

3. LaunchMotion和UniformMotion的参数拟合
### 说明
车体运动的过程可以抽象看成起步过程和趋近匀速过程。
起步过程车体需要克服静摩擦且需要较大加速度，因此需要电机输出更大的力矩，此时需要功率更大。
而趋近匀速过程车体加速度小或者没有，只需要保持匀速，因此需要电机输出较小的力矩，此时需要功率小。
所以两个过程的参数是不同的。

而RLS在进行拟合的大部分参数来自趋近匀速过程，因此此时想要拟合预估功率与超电功率，电流项的参数就会增大。
RLS的拟合需要时间，不可能在收敛出趋近匀速过程的参数后，在短时间的起步过程收敛出起步过程的参数。

所以会出现在起步时，还是用趋近匀速过程的参数，此时预估功率与超电功率偏差较大，
导致预估功率与超电功率偏差较大，导致超功率

### 步骤
####  LaunchMotion参数拟合（上场前要重新拟合一下确保没问题，因为整车重量变化参数会不一样）
拟合LaunchMotion参数时还是使用
```c++
#if APP_USE_POWERCTRL
        motors_._[i]->cmdTorq(cmd[i]);
#else
        motors_._[i]->cmdTorq(cmd[i]);
#endif
```
freemaster中观察起步过程chassisRealPower_, chassisRawPower_, chassisFitPower_拟合效果
拟合效果好，则将得到的参数填入PowerModel_s::ModelParam_s LaunchMotion中
#注： 一定要是起步加减速过程，不能匀速，最好先停止拟合再抄（如四轮车断控，或者写debug模式）

检验拟合效果：换回模型输出力矩
```c++
#if APP_USE_POWERCTRL
        motors_._[i]->cmdTorq(torq[i]);
#else
        motors_._[i]->cmdTorq(cmd[i]);
#endif
```
观察是否超功率，拟合好效果如图：(如果不行重新拟合)
![起步拟合](<Pitures/LaunchMotion.png>)

#### UniformMotion参数拟合
UniformMotion的参数只需在匀速过程中记下参数填入即可（减少场上拟合时间）
![匀速拟合](<Pitures/UniformMotion.png>)

#### 速度阈值调整
调试软件中观察功率波形以及实际轮子速度（注意为未经过减速箱速度）
速度阈值一般为功率开始稳定时的速度，如上图第二段功率下降到稳定时的速度(下步为200)

如果调完速度阈值后发现有尖峰，如图
![起步拟合](<Pitures/Theshold.png>)则为阈值不够

调好后如图
![起步拟合](<Pitures/Success.png>)


## 后续优化方向
1. 功率分配算法优化
不只使用比例分配，不同情况使用不同方法使功率分配更合理
2. 错误状态处理
如超电，裁判系统断连等错误状态处理
3. 超功率保护
超功率后主动降低限制，防止连续超功率

