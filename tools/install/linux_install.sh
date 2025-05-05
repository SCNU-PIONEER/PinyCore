#!/bin/bash

# 获取发行版ID
if [ -f /etc/os-release ]; then
  . /etc/os-release
  OS=$ID
else
  echo "无法确定Linux发行版"
  exit 1
fi

# 根据发行版设置包管理器
case $OS in
debian | ubuntu)
  PKG_MANAGER="apt-get"
  ;;
arch | manjaro)
  PKG_MANAGER="pacman"
  ;;
*)
  echo "不支持的发行版: $OS"
  exit 1
  ;;
esac

echo "检测到系统: $OS, 使用包管理器: $PKG_MANAGER"
echo "正在更新包管理器..."
if [ "$PKG_MANAGER" = "apt-get" ]; then
  sudo apt-get update
elif [ "$PKG_MANAGER" = "pacman" ]; then
  sudo pacman -Syu
fi

if command -v cmake &>/dev/null; then
  echo "cmake 已安装，版本: $(cmake --version | head -n 1)"
  exit 0
else
  echo "正在安装cmake..."
  if [ "$PKG_MANAGER" = "apt-get" ]; then
    sudo apt-get install -y cmake
  elif [ "$PKG_MANAGER" = "pacman" ]; then
    sudo pacman -S --noconfirm cmake
  fi
fi

if command -v ninja &>/dev/null; then
  echo "ninja 已安装，版本: $(ninja --version | head -n 1)"
  exit 0
else
  echo "正在安装ninja..."
  if [ "$PKG_MANAGER" = "apt-get" ]; then
    sudo apt-get install -y ninja-build
  elif [ "$PKG_MANAGER" = "pacman" ]; then
    sudo pacman -S --noconfirm ninja
  fi
fi

if command -v openocd &>/dev/null; then
  echo "openocd 已安装，版本: $(openocd --version | head -n 1)"
  exit 0
else
  echo "正在安装openocd..."
  if [ "$PKG_MANAGER" = "apt-get" ]; then
    sudo apt-get install -y openocd
  elif [ "$PKG_MANAGER" = "pacman" ]; then
    sudo pacman -S --noconfirm openocd
  fi
fi

if command -v openocd &>/dev/null; then
  echo "openocd 已安装，版本: $(openocd --version | head -n 1)"
  exit 0
else
  echo "正在安装openocd..."
  if [ "$PKG_MANAGER" = "apt-get" ]; then
    sudo apt-get install -y openocd
  elif [ "$PKG_MANAGER" = "pacman" ]; then
    sudo pacman -S --noconfirm openocd
  fi
fi

if kconfig -h &>/dev/null; then
  echo "kconfig-frontends 已安装"
  exit 0
else
  echo "正在安装kconfig-frontends..."
  if [ "$PKG_MANAGER" = "apt-get" ]; then
    sudo apt-get install -y kconfig-frontends
  elif [ "$PKG_MANAGER" = "pacman" ]; then
    sudo pacman -S --noconfirm kconfig-frontends
  fi
fi

if command -v arm-eabi-gcc &>/dev/null; then
  echo "arm-eabi-gcc 已安装，版本: $(arm-eabi-gcc --version | head -n 1)"
  exit 0
else
  echo "正在安装arm-eabi-gcc..."
  if [ "$PKG_MANAGER" = "apt-get" ]; then
    sudo apt-get install -y gcc-arm-none-eabi
    sudo apt install --only-upgrade gcc-arm-none-eabi
  elif [ "$PKG_MANAGER" = "pacman" ]; then
    sudo pacman -S --noconfirm arm-none-eabi-gcc
  fi
fi

# 验证安装
echo "验证安装..."
cmake -v
ninja -v
openocd -v
arm-none-eabi-gcc -v

echo "安装完成！"
