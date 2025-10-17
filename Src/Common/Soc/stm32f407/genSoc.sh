#!/bin/bash

rm -rf ./hal/Drivers

mkdir -p hal/Drivers/CMSIS/Device/ST/STM32F4xx
cp -r ./CMSIS_Device/Include/ ./hal/Drivers/CMSIS/Device/ST/STM32F4xx/
cp -r ./HAL_Driver/ ./hal/Drivers/STM32F4xx_HAL_Driver/
cp -r ../CMSIS5/Core/Include/ ./hal/Drivers/CMSIS/

MAIN_C_FILE="hal/Core/Src/main.c"
MAIN_H_FILE="hal/Core/Inc/main.h"

VARIABLES=$(sed -n '/\/\* Private variables [-]*\*\//,/\/\* USER CODE BEGIN PV \*\//p' "$MAIN_C_FILE" |
  grep -E '^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]+[A-Za-z_][A-Za-z0-9_]*[[:space:]]*;' |
  sed 's/^[[:space:]]*//;s/[[:space:]]*$//')

if [[ -z "$VARIABLES" ]]; then
  echo "未找到有效的变量声明"
  exit 1
fi

echo "找到以下变量声明:"
echo "$VARIABLES"
echo ""

TEMP_FILE=$(mktemp)

# 为每个变量声明添加extern到 /* USER CODE BEGIN ET */ 下方
echo "$VARIABLES" | while IFS= read -r line; do
  # 去掉分号（如果有）
  var_decl=$(echo "$line" | sed 's/;[[:space:]]*$//')

  # 检查是否已经存在相同的extern声明
  if ! grep -q "extern $var_decl;" "$MAIN_H_FILE"; then
    # 在 /* USER CODE BEGIN ET */ 行后添加extern声明
    sed -i "/\/\* USER CODE BEGIN ET \*\//a\\
extern $var_decl;" "$MAIN_H_FILE"
    echo "已添加: extern $var_decl;"
  else
    echo "已存在: extern $var_decl;"
  fi
done

echo "处理完成"
