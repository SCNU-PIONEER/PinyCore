# INS (Inertial Navigation System) v1.0.0



## 更新日志

1. 创建本README



## 如何应用到自己的项目

### 传感器焊接误差矫正

使用十二面校准法进行矫正，详细请见

[SCNU-PIONEER/IMUCalibrate](https://github.com/SCNU-PIONEER/IMUCalibrate)

生成矫正矩阵后填写进INS.cpp的ACC_CALI

```cpp
static constexpr AccCali_s ACC_CALI = {
    // default accelerometer calibration
    .accel_T = { { 1.010860f, 0.015129f, -0.001459f },
                 { 0.001142f, 1.009152f, 0.006399f },
                 { -0.005477f, 0.002071f, 1.013539f } },
    .accel_offs = { -34.944336f, -3.310059f, 107.792969f }
};
```

注：该工程暂时仅适配于C板，将考虑在未来版本将矫正程序集成进PinyCore中



### 静态零漂误差矫正

1. 启用INS系统
2. 保证在接下来直至标定结束，六轴加速度计不会发生任何的移动
3. 监视结构体对象ins->imuCali\_.gyro\_cali\_ 内的数值
4. 当上述数值趋于稳定时（如等待5分钟，数据浮动≤0.1），记录数值

5. 将记录的数值覆盖INS.cpp的GYRO_CALI的bias项

```cpp
static constexpr GyroCali_s GYRO_CALI = {
    // default gyroscope calibration
    .gx_bias = -0.898322f, .gy_bias = -4.99465f, .gz_bias = -0.234681f,
    .gx_tco_k = 0.f,       .gx_tco_b0 = 0.f,     .gy_tco_k = 0.f,
    .gy_tco_b0 = 0.f,      .gz_tco_k = 0.f,      .gz_tco_b0 = 0.f
};
```

注：笔者这里没有使用温控



### 安装位置矫正

自行测量传感器（六轴加速度计）距离车体中心的x、y偏移，单位为米，填入INS.cpp

```cpp
// When there is no offset present
static constexpr float IMU_OFFSET_X = 0;
static constexpr float IMU_OFFSET_Y = 0;
```

注：遵循机器人右手系，用±符号来表示偏移方向



### 安装方向矫正

INS.cpp中，安装方向说明如下

```cpp
// the order of axis is defined as:
/*
              Z
              |
              |
              |
              |     X: IN FRONT OF YOUR HEAD
              |   /
              | /
    Y <-------ROBOT 
*/
// load raw INS needed data, you must transform the raw imu data to correct order
// Roll: Clockwise increase(+) when looking straight ahead
// Pitch: Head up decrease(-)
// Yaw: Clockwise decrease(-) when viewed from above
auto &data = instance->rawDat_;
data = {
    .a = { .x = cali.getOutput().ax,
           .y = cali.getOutput().ay,
           .z = cali.getOutput().az },
    .g = { .x = cali.getOutput().gx,
           .y = cali.getOutput().gy,
           .z = cali.getOutput().gz },
};
```

`cali.getOutput()`得到的值基于传感器本体的坐标，而我们需要将其映射到机器人的坐标

先将data改为上文所示的样子，启动INS，监视`ins->insData_.body`的变量大小即变化方向，基于机器人右手系修正data的内容

如25赛季全向轮步兵（牢全）最终矫正的值如下

```cpp
data = {
    .a = { .x = cali.getOutput().ax,
           .y = cali.getOutput().ay,
           .z = -cali.getOutput().az },
    .g = { .x = -cali.getOutput().gx,
           .y = -cali.getOutput().gy,
           .z = -cali.getOutput().gz },
};
```



### 其他矫正

若因硬件导致传感器原始数据模态变化（如电源电压纹波大、信号传输受扰强等）会导致INS的数值异常，此时也许需要重新整定算法DcmAHRS内部的EKF特征参数



## 注意事项

1. v1.0.0版本的世界系加速度、角速度不完全准确
2. INS仅提供姿态数据，不提供控制接口（不排除在未来版本加入的可能性）