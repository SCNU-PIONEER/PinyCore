# Filter v1.1.0

## 更新
1. FIR滤波器更新
2. IIR滤波器大更新

## 说明
本文档只做滤波器的使用说明，原理后续会更新在飞书上
具体不同滤波器使用会与频谱辨识一起更新


## FIR滤波器
```c++
template <uint8_t RANK> class FIR
```
此滤波器需与matlab 或python等滤波器参数生成工具搭配使用（后续更新）

## IIR滤波器
### IIR基类
```c++
template <uint8_t RANK> class IIR
```
由于巴特沃斯，切比雪夫等滤波器都为IIR滤波器，因此IIR滤波器为基类，其他滤波器继承IIR

主要变量
```c++
template <uint8_t RANK> struct IIRCoeffs_s {
    float NUM[RANK + 1]{};
    float DEN[RANK + 1]{};
    float x[RANK]{};
    float y[RANK]{};
    float sampleRate_ = 0.f;
    float cutoffFreq_ = 0.f;
    struct NormParam_s {
        float a[RANK + 1]{};
        float epsilon_ = 0.f;
    } normParam_;
};
```

### 继承类
```c++
class IIR2 : public IIR<2>;
class IIR3 : public IIR<3>;
```
巴特沃斯的归一化系数和切比雪夫的归一化系数不同，因此在构造函数需传入FilterType_e
并且切比雪夫在设计滤波器时所需求的纹波不同时归一化系数也不同，因此需要传入Ripple_e，选择不同的纹波

阶数不同时对应的归一化参数不同    
```c++
const std::unordered_map<Ripple_e, IIRCoeffs_s<2>::NormParam_s> IIR2PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, std::numbers::sqrt2, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 1.5162026f, 1.4256245f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 1.1025103f, 1.0977343f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 0.8230603f, 0.8038164f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 0.7079478f, 0.6448996f, 1.f }, .epsilon_ = 0.9976283f } }
    };
```
Ripple_e::NONE对应的就是巴特沃斯滤波器

目前手搓滤波器只有2阶和3阶，4阶后续更新，更高阶需使用matlab等工具生成系数然后使用
```c++
template <uint8_t RANK> class IIRN
```
使用方法
```c++
FILTER::IIRN<3> yawVelButter{ 
(float[4]){ 0.000864626588f, 0.0025938797648f, 0.00259387976489f, 0.00086462658829830f },
(float[4]){ 1.0f, -2.707831133387657f,  2.495045192704085756f, -0.78029704661004f } };
```

## 兼容旧版的参数
短时间没有调整滤波器的需求情况下，
可以将以前的IIR2换成
```c++
FILTER::IIR2 yawVelIIR_{ 500.f, 30.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE };
```
IIR3换成
```c++
FILTER::IIR3 yawVelIIR_{ 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE };
```

## 后续优化方向
1. 更多的滤波器类型,如椭圆滤波器
2. 不同滤波器特性说明
3. 滤波器参数生成工具使用方法


