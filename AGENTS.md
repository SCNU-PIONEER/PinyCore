# PinyCore AI Development Guidelines

## Platform And Toolchain

- C++23 / C11
- arm-none-eabi
- FreeRTOS
- STM32 HAL

## Build

```bash
cmake -B build -G Ninja
cmake -B build -G Ninja -DCONFIG_NAME=your_config_name # Build with a preset configuration
```

## Compile

```bash
ninja -C build
```

When adding a new module, make sure it is included in the build. Follow the existing `CMakeLists.txt` style and pay attention to how module build options are connected with Kconfig.

## Coding Standards

### Naming Rules

| Category | Style | Examples |
|---|---|---|
| Class | PascalCase | `Chassis`, `PositionalPid` |
| Enum type | PascalCase + `_e` | `RegId_e`, `ErrorCode_e` |
| Struct type | PascalCase + `_s` | `Empty_s`, `ProtoData_s` |
| Union type, C style | PascalCase + `_u` | `Pids_u`, `Motors_u` |
| Union type, C++ style | PascalCase + `_v` | `Command_v` |
| Enum member | UPPER_SNAKE_CASE | `DM_REG_UV_VALUE`, `NO_ERROR` |
| Namespace | UPPER_SNAKE_CASE | `PINYMOTOR`, `INS_SYS`, `LED` |
| Function / variable | camelCase | `enable()`, `bspInit()`, `createApp()` |
| Private class member | camelCase + trailing `_` | `tasks_`, `insDat_` |
| Function parameter | leading `_` + camelCase | `_flag`, `_ctx` |

### Abbreviations

Except for UPPER_SNAKE_CASE names, abbreviations with exactly three letters should stay fully uppercase. Other abbreviations should be treated as normal words: capitalize the first letter and keep the rest lowercase.

### Keyword Conflicts

If an identifier conflicts with a C++ keyword, append an underscore:

```cpp
float class_ = 0; // `class` is a C++ keyword
```

### Code Style

Follow `./.clang-tidy` and `./.clang-format`.

- Write all comments in English.
- Prefer `enum class` over unscoped enums.
- Do not add trailing underscores to struct fields.
- Use `#pragma once` in header files.
- Mark classes or member functions as `final` when they are not intended to be inherited or overridden.
- Terminate every namespace with a closing comment.
- Do not construct objects before the HAL library is initialized. Use `./Src/Utils/Lazy` for lazy initialization of global objects.
- Use `new` only for allocations that are intentionally never deleted. Otherwise, use smart pointers.
- Use `__always_inline` instead of `inline`.
- Prefer `static constexpr` constants over `#define`.
- Confirm with the user before modifying the HAL library or any third-party library.
- Prefer interface-based designs. Do not hard-code handles before the design is confirmed by the user.
- **Less is more. Keep code concise**.

## Commit Convention

Use the Commitizen format:

```text
<type>[scope]: <description>
```

Example:

```text
feat(motor): add DJI Motor GM6020
```

Do not add `Co-Authored-By` trailers or AI tool signatures.
