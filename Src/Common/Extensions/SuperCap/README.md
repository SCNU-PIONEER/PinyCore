# SuperCap v2.0.0

## 更新

1. 新功率板协议更新
2. 重构电容模组部分代码，更新变量及函数签名

## 使用

包体内容可见同目录下SuperCapMsg.hpp

SuperCap类中有设置各命令值的函数。

快速使用说明：

1. 包含头文件
   
    #include "SuperCap.hpp"

2. 构造
   
    SuperCap cap(hcanPtr, /*cmdId*/0x222, /*dataId*/0x223, /*txFreq*/200.f);
   
   - `hcanPtr`：指向 `canHandle` 的句柄（平台/项目中 CAN 句柄类型）。
   - `txFreq`：命令发送频率（Hz），默认 200Hz。

3. 在 RTOS 任务循环中调用 task()

4. 设置命令 / 控制行为（示例）
   
   常用命令，控制电容放电
    cap.enableDischarge();
    cap.disableDischarge();
   
   电容组默认设置充电功率上限为所设置的chargePowerLimit，即使调用unlimitCharge()
    cap.limitCharge();
    cap.unlimitCharge();
    cap.setChargelimitRatio(0.8f); // 0..1
    cap.setChargePowerLimit(120);
    cap.setChargeEnergySlack(60);
   
    cap.setClearErrorFlag(true);
    cap.setSystemRestartFlag(true);

5. 读取状态
   
    auto &data = cap.getCapData();
    float rxHz = cap.getRxFreq();