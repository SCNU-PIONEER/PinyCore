# superCap v2.0.0

## 更新
1. 新功率板协议更新

## 主要成员变量
```c++
canHandle *hcan_;
uint16_t cmdId_;
uint16_t dataId_;

CapData_s capData_;
RawCapData_s rawCapData_;
CapCmd_s capCmd_;

```
capData_ 为解析后数据
rawCapData_ 为原始数据
capCmd_ 为发送命令

## 主要成员函数
```c++
void capTask(bool _capEnable, bool _systemRestart, bool _clearError, bool _enChargeLimit, uint8_t _chargeRatioLimit,
                 uint16_t _powerLimit, uint16_t _energyBuffer);
```
需要持续运行，不能在上控情况下调用
```c++
CapData_s &getCapData() { return capData_; }
```
功率板数据接口

## 使用
```c++
struct CapCmd_s {
    uint8_t enableDCDC : 1;    // 允许启动DCDC
    uint8_t systemRestart : 1; // 系统重启
    uint8_t reserved1 : 3;
    uint8_t clearError : 1;    // 手动清除可清除的错误
    uint8_t enChargeLimit : 1; // 是否启用主动充电限制
    uint8_t useFeedback : 1;   // 是否使用反馈消息

    uint16_t powerLimit;      // 裁判限制功率，单位W
    uint16_t energyBuffer;    // 裁判能量缓冲，单位J
    uint8_t chargeRatioLimit; // 主动充电限制比例（能量），0-255
    int16_t reserved2;
};//变量说明

enableDCDC为电容以及无线充电 充放电开关，阵亡应为0
systemRestart，默认为0，功率板代码数据错误时，可设置为1，重新启动
clearError，默认为0，可设置为1，清除超电错误
enChargeLimit，默认为1，启用主动充电限制
chargeRatioLimit, 量程为0-255，对应于0-28.8v，即能量比例为0-100%
powerLimit，单位为W，功率限制
*说明*：功率板充电上限功率（电容充电功率），电容放电逻辑：功率板检测功率超过发送的powerLimit后，以检测功率减去powerLimit后的功率放电
energyBuffer，单位为J，缓冲能量

```