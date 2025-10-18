# LED v1.0.1



## 更新日志 

1. 创建本README v1.0.0 
2. WS2812B的PWM DMA驱动中，DMA缓冲区使用ram_alloc创建 v1.0.1


## 如何使用

主要使用的API都存在于LEDManager中



### 1 . 创建灯组

```cpp
void addLEDs(LEDDriver *_driver);
```

其中`_driver`用`create`函数来创建

目前支持

1. PWM DMA驱动WS2812B
2. SPI DMA驱动WS2812B
3. 常规高低电平控制光电二极管

```cpp
template <PWMChipsets_e Chip> LEDDriver *create(Pwm *_handle, int _num)
template <SPIChipsets_e Chip> LEDDriver *create(SPI_HandleTypeDef *_handle, int _num)
template <IOChipsets_e Chip>
    static LEDDriver *create(RGBLEDDriver::lightTuner _setR(uint8_t),
                             RGBLEDDriver::lightTuner _setG(uint8_t),
                             RGBLEDDriver::lightTuner _setB(uint8_t))
```

示例（创建10个WS2812的灯组，在C板中使用TIM8 CH1的PWM DMA驱动）：

```cpp
LED::LEDs::instance().addLEDs(
                LED::LEDs::instance().create<LED::PWM_WS2812B>(
                        std::make_unique<Pwm>(&htim8, TIM_CHANNEL_1).get(), 10));
```



### 2. 控制灯组

系统通过创建阻塞任务来实现对灯的控制，存储控制灯组命令的队列为空时，任务阻塞

使用以下API来发送命令到队列中

```cpp
static void ctrl(CmdType_e _type, uint8_t _index, uint8_t _ctrlNum);

static void off();
```

如控制10个灯珠展现彩虹灯效

```cpp
LED::LEDs::ctrl(LED::CmdType_e::RAINBOW_FLOW, 0, 10);
```

第一个参数表示该次控制命令作用范围的起始位置

第二个参数表示该次控制命令作用范围的大小（数量）