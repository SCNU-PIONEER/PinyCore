# Bsp
using CRTP to Implement a singleton bsp class  
## Advantages
1. Thread safety  
2. High performance, no virtual function consumption  
3. Decoupling design  

## Bsp Can
Please use the following function to register the callback function for fdcan or can to receive new messages
```
Can::instance().registerCallback(&hfdcan1, 0x201, canCallback);
```

## Bsp Uart
Please use the following function to register the callback function for uart to receive new messages
```
Uart::instance().registerCallback(huart, &Rc::callBackFromISR);
```
