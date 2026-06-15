<p align="center">
    <img src="./.assets/logo.png" alt="PinyCore Logo" width="240"/>
</p>

# PinyCore

<p align="center">
    <a href="./README.md">English</a> | <a href="./README.zh-CN.md">中文</a>
</p>

<p align="center">
    <a href="http://commitizen.github.io/cz-cli/"><img
            src="https://img.shields.io/badge/commitizen-friendly-brightgreen.svg"
            alt="Commitizen friendly"/></a>
    <a href="https://github.com/semantic-release/semantic-release"><img
            src="https://img.shields.io/badge/semantic--release-angular-e10079?logo=semantic-release"
            alt="semantic-release: angular"/></a>
</p>

**PinyCore** is a generic, reusable control framework for embedded robot systems, **and is not limited to RoboMaster**. It provides the common infrastructure shared by different units, including SoC/BSP hardware abstraction, FreeRTOS-based task scheduling and communication, logging and fault handling, state-machine support, and Kconfig-driven build configuration. The project is intended to separate application logic from the underlying hardware and provide stable interfaces for building concrete unit implementations.

PinyCore does not implement any specific unit type or application-specific behavior. The repository provides only the core framework; concrete robot implementations should live in the application layer or in a separate project that depends on PinyCore.

To keep PinyCore reusable and compatible across different projects, only generic code should be submitted to this repository. Application-specific modules, board-specific configuration, and logic tied to a particular unit should be kept outside the core. PRs to PinyCore are intended to be opened only when:

- an application-layer module has been implemented and stabilized, and its generic portion can be reused without breaking compatibility with existing code; or
- changes are being collected during a specific integration phase, such as a final form review.

The following parts are not provided as ready-to-use implementations and must be completed by the user:

1. **Application layer** — unit-specific control logic, task organization, state transitions, and high-level behavior.
2. **Command-input logic configuration** — input mapping, filtering, mode handling, and command dispatch for the application's control sources.

****

👉 [Click here to view details](./.docs/profile.md)

# 🎯Requirements

1. cmake >= 3.24
2. ninja >= 1.1
3. [arm-none-eabi-toolchains](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) >= 10.3.1 or [llvm](https://github.com/arm/arm-toolchain) (experimental)
4. python >= 3.2 (maybe also need python-is-python3 in ubuntu)
5. [kconfiglib](https://github.com/ulfalizer/Kconfiglib)

we also suggest to install following software:
1. [clangd](https://clangd.llvm.org/) >= 20.0
2. clang-format >= 20.0
3. [commitizen](https://github.com/commitizen/cz-cli)
4. ccache

# 🌟Getting started
## 🏗️Build & Compile

**Build**

```sh
cmake -B build -G Ninja
```

**Compile**

```sh
ninja -C build
```

****

Support configuring projects based on Kconfig

```sh
cmake --build ./build --target menuconfig
```

Support building projects directly based on pre-configured files (which should exist in the root directory, with the file name "yourconfig" in this example)

```sh
cmake -B build -G Ninja -DCONFIG_NAME=yourconfig
```

## 🐞Debug

1. openocd >= 0.12.0
2. [cortex-debug](https://github.com/Marus/cortex-debug) / [codelldb](https://github.com/vadimcn/codelldb) (vscode-plugin)
3. Ozone 3.24
4. FreeMASTER/FreeMASTER Lite >= 3.1.3 (Lite supports Linux and requires the installation of the prerequisite JRE)
5. systemview

## ⚠️Notice

1. It is dangerous to call HAL library functions before the main function is executed! Please avoid making class objects that call HAL library constructors global variables (you can create global pointers, or use the tools in ./Src/Utils/Lazy and construct them during the initialization phase);
2. During the initial build, this project will automatically pull third-party libraries from GitHub. Please ensure that the network is available.

## 🧩Framework

![frame](.docs/frame.png)

# 🙌Contributing

Contributions are always welcome!

See [CONTRIBUTING](./.docs/CONTRIBUTING.md) for ways to get started.

Please adhere to this project's [CODE_OF_CONDUCT](./.docs/CODE_OF_CONDUCT.md).
