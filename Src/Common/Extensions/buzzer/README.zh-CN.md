# Buzzer (蜂鸣器驱动) v1.0.0

## 更新日志

1. 创建本README v1.0.0  
2. 添加基于Bsp内的bsp_pwm的定时器驱动，支持任意频率输出  
3. 支持预定义音效（Dji、Wechat、WindowsXP、Victory、Press、PinyCore、AllNote）  
4. 实现FreeRTOS队列缓冲，支持非阻塞播放和序列播放  

---

## 如何应用

### 硬件配置

Buzzer模块使用定时器的PWM输出驱动蜂鸣器，需要以下硬件资源：

- 一个支持PWM输出的定时器（如`TIM4`）
- 一个PWM通道（如`TIM_CHANNEL_3`）
- 蜂鸣器连接在该PWM输出引脚上，通常需要外部驱动电路（三极管/MOSFET）

在项目中需要提供（一般在DefaultConf.hpp.in中配置）：
- 定时器句柄 `BEEP_TIMER`
- 通道号 `uint32_t`（如`BEEP_TIM_CHANNEL`）
- 定时器时钟频率 `uint32_t`（例如`BEEP_APB_FREQ`）

### 初始化蜂鸣器

在应用初始化代码中（例如`AppManager::initApp()`），创建`Buzzer`对象并调用其方法：

```cpp
#include "Buzzer.hpp"

// 假设有定时器句柄 htim2，通道 TIM_CHANNEL_1，定时器频率 80MHz
BUZZER::Buzzer myBuzz(&htim2, TIM_CHANNEL_1, 80'000'000);
```

PinyCore默认使用`Lazy`在Buzzer.hpp中延迟构造，且已经在初始化代码中将其初始化（如示例代码所示）：

```cpp
Lazy<BUZZER::Buzzer> buzz;
// 在 initApp 中：
buzz.init(&BEEP_TIMER, BEEP_TIM_CHANNEL, BEEP_APB_FREQ);
```

### 播放单个音符

使用`playNote`方法，可以指定音高（`Tone_e`枚举）和持续时间（毫秒）：

```cpp
buzz->playNote(BUZZER::Tone_e::C5, 500);  // 播放中央C，持续500ms
buzz->playNote(BUZZER::Note_s{BUZZER::Tone_e::A4, 300});  // 使用Note_s结构
```

**注意，该操作会无阻塞地往蜂鸣器任务队列发送Note，若队列满则发送无效。**

### 播放预定义音效

Buzzer模块预定义了一些常见音效，可以直接调用：

```cpp
buzz->playNote(BUZZER::Dji);        // DJI提示音
buzz->playNote(BUZZER::Wechat);     // 微信消息提示音
buzz->playNote(BUZZER::WindowsXP);  // Windows XP启动音
buzz->playNote(BUZZER::Victory);    // 胜利音效
buzz->playNote(BUZZER::Press);      // 按键声
buzz->playNote(BUZZER::PinyCore);   // 自定义PinyCore旋律
buzz->playNote(BUZZER::AllNote);    // 所有音符依次播放
```

 该操作**非阻塞**：调用后立即返回，音效会在后台任务中依次播放，不会阻塞主线程。

### 自定义音效序列

你可以定义自己的音符序列，然后使用`playSequence`播放：

```cpp
// 定义自己的旋律
const BUZZER::Note_s myMelody[] = {
    { BUZZER::Tone_e::C4, 200 },
    { BUZZER::Tone_e::E4, 200 },
    { BUZZER::Tone_e::G4, 200 },
    { BUZZER::Tone_e::C5, 400 },
    { BUZZER::Tone_e::REST, 100 }
};

// 播放
buzz->playNote(myMelody);
```

`playSequence`会自动计算数组长度，无需手动指定。

### 队列缓冲与任务调度

Buzzer内部创建了一个FreeRTOS队列（大小10），并运行一个后台任务（优先级`MID5`）。所有播放请求（`playNote`和`playSequence`）最终都会通过队列发送给后台任务，由后台任务依次执行PWM输出。这种设计保证了：

- 播放请求不会阻塞调用者
- 多个音符可以排队，按顺序播放
- 不同来源（中断、多个任务）可同时发送音符，无需担心冲突

---

## 注意事项

1. **定时配置**  
   定时器的PWM输出频率计算方式为：  
   `period = (timerFreq / (freq * prescaler_)) - 1`  
   其中`prescaler_`固定为100，可以通过调用`setPSC()`来更改。该值影响可播放音符的范围，请确保定时器时钟频率足够高，以支持目标频率（最高约5kHz，实际取决于蜂鸣器特性）。

2. **任务栈大小**  
   Buzzer后台任务栈大小为128字（`Task<Buzzer, 128>`），序列播放任务栈大小为256字。若自定义序列很长或需要额外处理，可适当调整。

3. **队列大小**  
   队列默认容量为10，若短时间内有超过10个音符未处理，后续音符会阻塞等待。如果需要更高的并发能力，可修改`Buzzer`构造函数中的`xQueueCreate`参数。

4. **PWM占空比**  
   播放音符时，PWM占空比固定为50%，产生对称方波。蜂鸣器通常对占空比不敏感，如需调节音量，可修改`play()`函数中的`pwm_.setDutyCycle(period/2)`。

5. **静音处理**  
   当音符为`Tone_e::REST`时，PWM输出被关闭，仅延时指定时间。这用于产生休止符。

6. **与FreeRTOS集成**  
   Buzzer模块依赖FreeRTOS，请确保系统已正确初始化FreeRTOS内核。

7. **线程安全**  
   `playNote`内部通过队列发送消息，是线程安全的。可以在中断服务函数中调用，但需注意中断中不能使用`playSequence`,其中的`portMAX_DELAY`（会阻塞），建议使用`playNote`的`xQueueSend`超时参数（默认为0）。目前播放单个音符的`playNote`使用`xQueueSend(queue_, &_note, 0)`，因此可在中断中安全使用。

8. **PWM初始化**  
   Buzzer构造时不会启动PWM输出，只有收到播放请求时才会调用`pwm_.start()`。播放完毕后PWM输出关闭。