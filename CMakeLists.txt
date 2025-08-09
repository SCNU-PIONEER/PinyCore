cmake_minimum_required(VERSION 3.24)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Version info
set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")


# Define the build type
set(CMAKE_BUILD_TYPE "Release")

# Set the project name and version
set(CMAKE_PROJECT_NAME PinyCore)

# Include toolchain file
include("tools/Kconfig/Kconfig.cmake")
add_subdirectory(Src/Common/Soc)  # BUG: waiting for fix
include("tools/cmake/GccArmNoneEabi.cmake")
# include("tools/cmake/Clang.cmake")
include("tools/cmake/FindInclude.cmake")
include("tools/cmake/ShowMessage.cmake")
include("Src/sdkconfig.cmake" OPTIONAL)
include("tools/cmake/Warning.cmake")

# Core project settings
project(${CMAKE_PROJECT_NAME} VERSION ${VERSION})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)


# 启用彩色诊断信息
add_compile_options($<$<C_COMPILER_ID:GNU>:-fdiagnostics-color=always>)
add_compile_options($<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} Src/PinyCore.cpp
)

set(CMSISCORE "${soc_folder}/hal/Drivers/CMSIS")

# Add subdirectory

add_subdirectory(Lib/Segger)
add_subdirectory(Lib/Eigen3)
# add_subdirectory(Lib/TinyMPC)

add_subdirectory(Src/App)
add_subdirectory(${soc_folder}/hal)
add_subdirectory(Src/Common/Extensions)
add_subdirectory(Src/Common/Bsp)
add_subdirectory(Src/System)
add_subdirectory(Src/Utils)
add_subdirectory(Src)
add_subdirectory(Lib/CMSIS-DSP/Source)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE Src/PinyCore.hpp)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        robot_core/  
        Inc  
        Drivers/STM32F4xx_HAL_Driver/Inc  
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy  
        Drivers/CMSIS/Include  
        Drivers/CMSIS/Device/ST/STM32F4xx/Include  
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc  
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc  
        robot_core/algorithm  
        robot_core/base_drv  
        robot_core/base_drv/drv_bmi08x  
        robot_core/base_drv/drv_motor  
        robot_core/OS  
        robot_core/OS/drv_freertos/include  
        robot_core/OS/drv_freertos/portable/RVDS/ARM7_LPC21xx  
        robot_core/OS/drv_freertos/portable/RVDS/ARM_CM0  
        robot_core/OS/drv_freertos/portable/RVDS/ARM_CM3  
        robot_core/OS/drv_freertos/portable/RVDS/ARM_CM4_MPU  
        robot_core/OS/drv_freertos/portable/RVDS/ARM_CM4F  
        robot_core/OS/drv_freertos/portable/RVDS/ARM_CM7  
        robot_core/tests  
        robot_core/tools 
        robot_core/Src
        robot_core/Src/App/State
        robot_core/Src/App/State/StateBase
        robot_core/Src/App/RobotModules
        robot_core/Src/App/RobotModules/Chassis
        # robot_core/Src/App/State/StateBase/StateBase.cpp
        # robot_core/Src/App/State/StateBase/StateBase.h
      
        )
        # 添加 Gimbal 模块的头文件路径
#include_directories(Src/App/Gimbal)

# Add project symbols

# Add linked libraries
target_link_libraries(
  ${CMAKE_PROJECT_NAME} PRIVATE
  Piny
  App
  Extensions
  System
  Bsp
  stm32cubemx
  segger
  Eigen3
  CMSISDSP
  # TinyMPC
)

showMessage()
