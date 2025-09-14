cmake_minimum_required(VERSION 3.24)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

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
include("tools/cmake/Version.cmake")

# Core project settings
project(${CMAKE_PROJECT_NAME} VERSION ${PROJECT_VERSION})

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
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE )

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
