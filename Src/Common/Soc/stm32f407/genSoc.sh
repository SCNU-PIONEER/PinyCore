#!/bin/bash

rm -rf ./hal/Drivers

mkdir -p hal/Drivers/CMSIS/Device/ST/STM32F4xx
cp -r ../../../../ThirdParty/CMSIS_Device_F4/Include/ hal/Drivers/CMSIS/Device/ST/STM32F4xx
cp -r ../../../../ThirdParty/STM32F4xx_HAL_Driver/ hal/Drivers
cp -r ../../../../ThirdParty/CMSIS5/Core/Include/ hal/Drivers/CMSIS/
