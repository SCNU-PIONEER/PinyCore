# PinyEngineer
<p align="center">
    <a href="http://commitizen.github.io/cz-cli/"><img
            src="https://img.shields.io/badge/commitizen-friendly-brightgreen.svg"
            alt="Commitizen friendly"/></a>
    <a href="https://github.com/semantic-release/semantic-release"><img
            src="https://img.shields.io/badge/semantic--release-angular-e10079?logo=semantic-release"
            alt="semantic-release: angular"/></a>
</p>

## Todo

### 底盘
- [ ] 1. 遥控器/键盘控制
- [ ] 2. 功率控制
- [ ] 3. 小陀螺
- [ ] 4. 公路区跨越

### 云台
- [ ] 1. 云台跟随

### 臂
- [ ] 1. 复位
- [ ] 2. 遥控器/键盘控制
- [ ] 3. 一键上台阶
- [ ] 4. 一键取矿
- [ ] 5. 示教器跟踪

### 示教器
- [ ] 1. 裁判系统通信
- [ ] 2. 控制机械臂
- [ ] 3. 重力补偿
- [ ] 4. 力反馈
- [ ] 5. 无线调试

### 中间层
- [ ] 1. 双板通信
- [ ] 2. ui

### 特色功能
- [ ] 1. 翻倒自救
- [ ] 2. 其他机器人翻倒救援

****

👉 [Click here to view details](./.docs/profile.md)

# 🎯Requirements

1. cmake >= 3.24
2. ninja >= 1.1
3. [arm-none-eabi-toolchains](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) >= 10.3.1 or [llvm](https://github.com/arm/arm-toolchain) (experimental)
4. python >= 3.2
5. [kconfiglib](https://github.com/ulfalizer/Kconfiglib)

we also suggest to install following software:
1. [clangd](https://clangd.llvm.org/) >= 20.0
1. clang-format >= 20.0
2. [commitizen](https://github.com/commitizen/cz-cli)

# 🌟Getting started
## 🏗️Build

**构建**

```sh
cmake -B build -G Ninja
```

**编译**

```sh
ninja -C build
```

****

支持基于Kconfig对项目进行配置

```sh
cmake --build ./build --target menuconfig
```

支持直接根据预制的配置文件来构建项目 (应存在于根目录，本例文件名为"yourconfig")

```sh
cmake -B build -G Ninja -DCONFIG_NAME=yourconfig
```

## 🐞Debug

1. openocd >= 0.12.0
2. [cortex-debug](https://github.com/Marus/cortex-debug) / [codelldb](https://github.com/vadimcn/codelldb) (vscode-plugin)
3. Ozone 3.24
4. FreeMASTER/FreeMASTER Lite >= 3.1.3 (后者支持Linux，需装前置Java JRE)
5. systemview

## ⚠️Notice

1. 在main函数执行之前调用HAL库函数是危险的！请避免使调用HAL库的构造函数的类对象成为全局变量（可以创建全局的指针，或使用./Src/Utils/Lazy中的工具，并在初始化阶段构造）；
1. 首次构建时，本项目会自动从github拉取第三方库，请确保网络可用。

## 🧩Framework

![frame](.docs/frame.png)

# 🙌Contributing

Contributions are always welcome!

See [CONTRIBUTING](./.docs/CONTRIBUTING.md) for ways to get started.

Please adhere to this project's [CODE_OF_CONDUCT](./.docs/CODE_OF_CONDUCT.md).

> [!IMPORTANT]
> PinyCore is still in early development, and is not yet complete. It should be stable enough and we have
> been daily driving it for quite a while, but expect some bugs and possibly breaking changes to the
> config file.
