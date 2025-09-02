# PowerCtrl v1.3.0

## 更新
1. 函数传参变化
2. 增加RLS动态拟合参数

## 主要成员变量
PowerController抽象类里：

std::vector<float> cmdPower; // 原闭环控制器所设定的功率
float powerRatio = 1.f;      // 功率分配比例
float maxPower = 0.f;        // 允许最大输出功率（经过动态规划）
std::vector<float> setPower; // 功率控制后所得的功率
std::vector<float> setIq;    // 最终设定输出电流
CAP cap_;

派生类里(如QuadricycleController):

MotorParam_s M3508  //3508的模型参数
RLS<3> wheelRLS_    //RLS
(如有不同型号电机需定义不同MotorParam_s和RLS)

## 控制逻辑
1. 更新裁判系统缓冲能量，根据电容实际能量占比与期望能量占比计算允许最大输出功率
2. 各型号电机计算各自原始功率，判断是否超功率，若超功率则按比例分配功率
3. 依照比例分配完功率后，根据各型号电机的模型参数，计算最终设定输出电流
4. RLS动态拟合，更新模型参数

## 函数
dynamicPower()    //计算允许最大输出功率

cmdPowerCalc()    //计算模型原始功率
relPowerCalc()    //计算模型实际功率(与反馈功率比较观察模型是否拟合)
currentCalc()     //计算最终设定输出电流
(不同型号电机需分别使用不同MotorParam_s计算功率再求和，电流也分别计算)

rlsUpdata()        //RLS动态拟合
(注意如果有多个型号电机时请勿在比赛开启，仅用于调试时得出各型号电机的参数
 原因：RLS需使用此时各电机型号真实功率，超电只能反馈所有电机功率，无法区分各型号电机)

## 使用
1. create PowerController
```c++
SwerveController *powerCtrl_;(使用对应控制器)
```
2. init
```c++
if constexpr (APP_USE_POWERCTRL) {
    powerCtrl_ = new SwerveController(ChassisType_e::SWERVE);
}
```
3. ctrl
```c++
float cmd[8];
for (uint8_t i = 0; i < 8; i++)
    cmd[i] = velPid_[i].elec;

if constexpr (APP_USE_POWERCTRL) {
    std::vector<float> elec;
    elec = powerCtrl_->powerCtrl(expWheels._, &motors_._[0], cmd);

    for (uint8_t i = 0; i < 8; i++)
        motors_._[i]->cmdElec(elec[i]);
} else {
    for (uint8_t i = 0; i < 8; i++)
        motors_._[i]->cmdElec(cmd[i]);
}

velPid_[i].elec 即pid得到的cmd电流
```

```

