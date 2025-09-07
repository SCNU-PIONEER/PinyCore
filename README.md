# PinyCore
<p align="center">
    <a href="http://commitizen.github.io/cz-cli/"><img
            src="https://img.shields.io/badge/commitizen-friendly-brightgreen.svg"
            alt="Commitizen friendly"/></a>
    <a href="https://github.com/semantic-release/semantic-release"><img
            src="https://img.shields.io/badge/semantic--release-angular-e10079?logo=semantic-release"
            alt="semantic-release: angular"/></a>
</p>

目前本仓库不会实现任何具体兵种，只会提供一个基础框架, 只有通用的代码需要pr到本仓库, 当app层模块稳定实现且兼容所有代码或特定时期（如完整形态考核等）才需要pr到本仓库，其他需要自己实现的地方：

1. app层
2. cmd输入的逻辑配置

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

```sh
cmake -B build -G Ninja
ninja -C build
```

### menuconfig

```sh
cmake --build ./build --target menuconfig
```

## 🐞Debug

1. openocd >= 0.12.0
2. [cortex-debug](https://github.com/Marus/cortex-debug) / [codelldb](https://github.com/vadimcn/codelldb) (vscode-plugin)
3. Ozone 3.24
4. systemview

## 📝Note

1. 全局变量构造不要有 hal 库的操作，但可以初始化 hal 指针，因为 hal 是在 main 中初始化，而全局变量是在 main 之前构造!

# 🙌Contributing

Contributions are always welcome!

See [CONTRIBUTING](./.docs/CONTRIBUTING.md) for ways to get started.

Please adhere to this project's [CODE_OF_CONDUCT](./.docs/CODE_OF_CONDUCT.md).

> [!IMPORTANT]
> PinyCore is still in early development, and is not yet complete. It should be stable enough and we have
> been daily driving it for quite a while, but expect some bugs and possibly breaking changes to the
> config file.
