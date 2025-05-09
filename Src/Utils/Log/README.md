# Piny Log
# 指标
1. 基本功能：输出信息,可变参数（c风格）
2. 可选性：颜色、文件定位、等级、时间戳(未实现)  
3. 安全性：线程安全, 类型检查  
4. 拓展性: 支持RTT, vofa  

# example
```

    uint8_t i = 10;
    std::string str = "PinyCore init start.";

    log.info("Piny", "This is a info.");
    log.warn("Piny", "This is a warn.");
    log.error("Piny", "This is a error.");

    log.info("Piny", "num i: %d", i);
    log.info("Piny", "str: %s", str.c_str());
```
