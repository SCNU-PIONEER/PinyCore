# Contributing to PinyCore

Welcome to PinyCore! This document is a guideline about how to contribute to PinyCore.
If you find something incorrect or missing, please leave comments / suggestions.

## Before you get started

### Code of Conduct

Please make sure to read and observe our [Code of Conduct](./CODE_OF_CONDUCT.md).

### Setting up your development environment

#### need
1. cmake >= 3.24
2. ninja >= 1.1
3. [arm-none-eabi-toolchains](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) >= 10.3.1 or [llvm](https://github.com/arm/arm-toolchain)
4. python
5. kconfiglib
6. clang-format

#### install script
##### linux
ubuntu/arch
```
sh ./tools/install/linux_install.sh
```
##### windows
click 'tools_install.bat' and 'win_install.ps1'

#### build
```
cmake --build ./build --target menuconfig
cmake -B build -G Ninja
ninja -C build
```

## Contributing

We are always very happy to have contributions, whether for typo fix, bug fix or big new features.
Please do not ever hesitate to ask a question or send a pull request.

### GitHub workflow

We use the `develop` branch as the development branch, which indicates that this is a unstable branch.

Here are the workflow for contributors:

1. Fork to your own
2. Clone fork to local repository
3. Create a new branch and work on it
4. Keep your branch in sync
5. Commit your changes (make sure your commit message concise)
6. Push your commits to your forked repository
7. Create a pull request

Please follow [the pull request template](./.github/PULL_REQUEST_TEMPLATE.md).
Please make sure the PR has a corresponding issue.

After creating a PR, one or more reviewers will be assigned to the pull request.
The reviewers will review the code.

Before merging a PR, squash any fix review feedback, typo, merged, and rebased sorts of commits.
The final commit message should be clear and concise.

### Open an issue / PR

We use [GitHub Issues](https://github.com/pwntester/octo.nvim/issues) and [Pull Requests](https://github.com/pwntester/octo.nvim/pulls) for trackers.

If you find a typo in document, find a bug in code, or want new features, or want to give suggestions,
you can [open an issue on GitHub](https://github.com/pwntester/octo.nvim/issues/new) to report it.
Please follow the guideline message in the issue template.

If you want to contribute, please follow the [contribution workflow](#github-workflow) and create a new pull request.
If your PR contains large changes, e.g. component refactor or new components, please write detailed documents
about its design and usage.

Note that a single PR should not be too large. If heavy changes are required, it's better to separate the changes
to a few individual PRs.

### Code review

All code should be well reviewed by one or more committers. Some principles:

- Readability: Important code should be well-documented. Comply with our code style.
- Elegance: New functions, classes or components should be well designed.

# 为PinyCore做贡献

欢迎来到PinyCore！本文档是关于如何为PinyCore做出贡献的指南。
如果你发现一些不正确或缺失的东西，请留下评论/建议。

## 在你开始之前

### 行为准则

请务必阅读并遵守我们的[行为准则](./CODE_OF_CONDUCT.md)。

### 设置您的开发环境

#### 需要
1. cmake >= 3.24
2. ninja >= 1.1
3. [arm-none-eabi-toolchains](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) >= 10.3.1或[llvm](https://github.com/arm/arm-toolchain)
4. python
5. kconfiglib
6. clang-format

#### 安装脚本
##### linux
ubuntu/arch
```
sh ./tools/install/linux_install.sh
```
##### windows
点击 'tools_install.bat' 和 'win_install.ps1'

#### 构建
```
cmake --build ./build --target menuconfig
make -B build -G Ninja
ninja - build
```

## 贡献

我们总是很高兴有贡献，无论是拼写错误修复，错误修复还是大的新功能。
请不要犹豫，问一个问题或发送拉请求。

### GitHub工作流

我们使用'develop'分支作为开发分支，这表明这是一个不稳定的分支。

以下是贡献者的工作流程：

1. fork你自己的仓库
2. 将fork克隆到本地存储库
3. 创建一个新的分支并对其进行处理
4. 保持分支同步
5. 提交更改（确保提交信息简洁）
6. 将提交推送到分叉的存储库
7. 创建拉取请求

请按照[PULL_REQUEST_TEMPLATE](../.github/PULL_REQUEST_TEMPLATE.md)。
请确认PR有相应的问题。

在创建PR之后，将为拉取请求分配一个或多个审阅者。
审查者将审查代码。

在合并PR之前，将所有的修复审查反馈、错别字、合并和重基类型的提交都压扁。
最后的提交信息应该清晰而简洁。

### 打开issue / PR

我们使用[GitHub Issues](https://github.com/pwntester/octo.nvim/issues)和[Pull Requests](https://github.com/pwntester/octo.nvim/pulls)作为跟踪器。

如果你在文档中发现了一个错别字，在代码中发现了一个错误，或者想要新的功能，或者想要提出建议，
你可以[在GitHub上打开问题](https://github.com/pwntester/octo.nvim/issues/new)报告它。
请遵循问题模板中的指导信息。

如果你想贡献，请遵循[贡献工作流](#github-workflow)并创建一个新的拉取请求。
如果您的PR包含较大的更改，例如组件重构或新组件，请编写详细的文档
关于它的设计和使用。

注意，单个PR不应该太大。如果需要进行大量更改，最好将更改分开
给个别pr。

### 代码审查

所有的代码都应该由一个或多个提交者很好地审查。一些原则:

可读性：重要的代码应该有良好的文档。遵守我们的代码风格。
-优雅：新的函数、类或组件应该设计得很好。
