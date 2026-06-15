# Piny Log
# 指标
1. 基本功能：输出信息,可变参数（c风格), 自动换行  
2. 可选性：颜色、文件定位、等级、时间戳(未实现)  
3. 安全性：线程安全, 类型检查  
4. 拓展性: 支持RTT, vofa  

# example
```

    LOG::Logger &log = LOG::Logger::instance();

    uint8_t i = 10;
    std::string str = "PinyCore init start.";

    log.raw("Piny");
    LOG::info( "Piny", "This is a info.");
    LOG::warn( "Piny", "This is a warn.");
    LOG::error( "Piny", "This is a error.");

    log.info("Piny", "num i: %d", i);
    log.info("Piny", "str: %s", str.c_str());
```
# 特性

## 安全性
### 线程安全 
1. RTT提供汇编级的锁机制  
2. 单例模式确保全局唯一  
3. 提供分线程多缓冲区  
4. 单次调用write，不会出现串行  

### 多缓冲区
```
SEGGER_RTT_ConfigUpBuffer(1, "Performance", PerfBuffer, sizeof(PerfBuffer),
                          SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
```

# Reference
[Segger RTT](https://kb.segger.com/RTT)
