set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# define compiler toolchain for Clang,
# such as set(TOOLCHAIN_PREFIX "~/apps/ATfE-20.1.0-Linux-x86_64/bin")
set(TOOLCHAIN_PREFIX "")
set(TOOLCHAIN_PREFIX "/home/yjy/apps/ATfE-20.1.0-Linux-x86_64/bin")

# Clang compiler
set(CMAKE_C_COMPILER                "${TOOLCHAIN_PREFIX}/clang")
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}/clang")
set(CMAKE_ASM_COMPILER              "${TOOLCHAIN_PREFIX}/clang")  # Clang support build asm

# linker and tools
set(CMAKE_AR                        "${TOOLCHAIN_PREFIX}/llvm-ar")
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}/llvm-objcopy")
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}/llvm-size")
set(CMAKE_STRIP                     "${TOOLCHAIN_PREFIX}/llvm-strip")

set(CMAKE_LINKER                    "${TOOLCHAIN_PREFIX}/ld.lld")

# .elf
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

# debug build type
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")

# release build type
set(CMAKE_C_FLAGS_RELEASE "-Oz -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Oz -g3")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
# set(ARM_LIB_DIR "home/yjy/apps/ATfE-20.1.0-Linux-x86_64/lib/clang-runtimes/arm-none-eabi/armv7m_hard_fpv4_sp_d16")
# set newlib_dir
# such as set(NEWLIB_DIR "~/apps/ATfE-20.1.0-Linux-x86_64/ATfE-newlib-nano-overlay-20.1.0/lib/clang-runtimes/newlib-nano")
set(NEWLIB_DIR "")
set(NEWLIB_DIR "/home/yjy/apps/ATfE-20.1.0-Linux-x86_64/ATfE-newlib-nano-overlay-20.1.0/lib/clang-runtimes/newlib-nano")

# set(TARGET_FLAGS "--target=armv7m-none-eabi -mfpu=${fpu} -mfloat-abi=${float-abi} -mthumb")
set(TARGET_FLAGS "--target=armv7m-none-eabi -mfpu=${fpu} -mfloat-abi=${float-abi} -mthumb --sysroot=${NEWLIB_DIR}" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${soc_folder}/link.ld\"")
# set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,crt0-rdimon -lrdimon")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc++ -lc++abi -Wl,--end-group")

# color config
set(CMAKE_C_FLAGS                   "${CMAKE_C_FLAGS} -fcolor-diagnostics")
set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")

# LLDB debug config
set(CMAKE_DEBUGGER                  lldb-mi)

