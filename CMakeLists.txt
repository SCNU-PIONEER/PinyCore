cmake_minimum_required(VERSION 3.24)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Version info
set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

# set(CMSISCORE "${CMAKE_SOURCE_DIR}/cubemx/Drivers/CMSIS")

# Define the build type
set(CMAKE_BUILD_TYPE "Debug")

# Set the project name and version
set(CMAKE_PROJECT_NAME PinyCore)

# Include toolchain file
include("tools/Kconfig/Kconfig.cmake")
add_subdirectory(Src/Common/Soc)  # BUG: waiting for fix
include("tools/cmake/GccArmNoneEabi.cmake")
include("tools/cmake/FindInclude.cmake")
include("tools/cmake/ShowMessage.cmake")
include("Src/sdkconfig.cmake" OPTIONAL)

# Core project settings
project(${CMAKE_PROJECT_NAME} VERSION ${VERSION})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# 关闭指定警告
add_compile_options(-Wno-unused-variable)
add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-function)
add_compile_options(-Wno-unused-value)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-missing-braces)
add_compile_options(-Wno-sequence-point)
add_compile_options(-Wno-pedantic)
add_compile_options(-Wno-deprecated-volatile)
add_compile_options(-Wno-missing-field-initializers)
# add_compile_options(-O0) add_compile_options(-Ofast)

# 启用彩色诊断信息
add_compile_options($<$<C_COMPILER_ID:GNU>:-fdiagnostics-color=always>)
add_compile_options($<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} Src/PinyCore.cpp
)

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
  # TinyMPC
  syscall
)

showMessage()
