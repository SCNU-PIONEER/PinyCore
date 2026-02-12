# System Identification v1.1.0

## 更新
1. 增加yaw轴系统辨识
2. 遗忘因子参数更新

## 说明
一个准确的系统模型可以显著提高控制系统的性能，因此系统辨识是控制理论中非常重要的一环。本模块实现了基于单杆控制模型的参数辨识，
包括转动惯量、阻尼系数等参数。使用RLS递归最小二乘算法进行在线系统辨识。

使用的模型为：
```math
\tau = J \ddot{\theta} + B \dot{\theta} + G \cos(\theta) + F \operatorname{sgn}(\dot{\theta})
```

## 主要成员变量
```c++
Matrix<RANK, 1> systemMatrix_{ Matrix<RANK, 1>::ones() * 0.0001f };  //需要辨识的参数矩阵
RLS<RANK> rls_{ 0.999f }; //RLS递归最小二乘算法

RLS的遗忘因子为0.999，可以根据实际情况进行调整。遗忘因子越大，RLS算法对历史数据更注重，但可能会导致系统不稳定。
遗忘因子越小，RLS算法对新数据更注重，但会导致系统收敛速度变慢。

float ctrlFreq_ = 0.f;     //控制频率
FILTER::IIR3 velFilter_;   //速度滤波器
FILTER::IIR3 torqFilter_;  //力矩滤波器
FILTER::LPF iFilter_;      //转动惯量滤波器

float fitTorq_ = 0.f;      //拟合力矩（用于实际力矩对比评估拟合效果）
float inertia_ = 0.f;      //转动惯量
float friction_ = 0.f;     //阻尼系数
float gravity_ = 0.f;      //重力矩
float coulomb_ = 0.f;      //库仑摩擦力矩
```
yaw轴的拟合阶数只有3个，不含重力项。
pitch由于有重力影响，因此拟合阶数为4个。

RLS对噪声非常敏感，因此需要使用滤波器对输入信号进行滤波，以减少噪声的影响。
且由于拟合需要各个参数同相位，因此对于传入参数需要进行同一滤波器同一参数进行滤波。
因为拟合使用了角速度和角加速度，为了保持同相位，角加速度的计算使用角速度滤波后的值计算。

推荐角速度使用ins的角速度，因为角速度在ins中已经经过EKF滤波，因此角速度的噪声较小。
在进行同一相位滤波时截止频率可以不用太低。截止频率太低会增加数据的幅度失真，导致拟合效果变差。

## 使用
在需要辨识的机构，如云台，创建需要辨识的对象，并传入该机构的控制频率和截止频率
```c++
YawSI yawLink{ 1000.f, 10.f };
PitchSI pitchLink{ 1000.f, 10.f };
```
然后在控制状态中调用update函数，传入控制频率和角速度、力矩
```c++
gimbal_->yawLink.update(gimbal_->yaw()->data().torq, gimbal_->ins().body.gz);

gimbal_->pitchLink.update(gimbal_->pitch()->data().torq, gimbal_->ins().body.gy,
                            gimbal_->pitchState().pos);
```
由电机的运动方程可以看出，转动惯量J，和角加速度dw／dt是以乘积的形式出现，因此在转动惯量辨识
过程中必须保证dw／dt不为零，即只有当电机角速度有一定变化时惯量辨识才有意义。
因此辨识时最好时刻保持加减速运动，避免电机处于静止状态。

## 后续
pitch辨识未实测，因此暂不上传


