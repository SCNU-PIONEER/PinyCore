cmake_minimum_required(VERSION 3.24)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_PROJECT_NAME PinyCore)

include("tools/Kconfig/Kconfig.cmake")
add_subdirectory(Src/Common/Soc) # BUG: waiting for fix
include("tools/cmake/GccArmNoneEabi.cmake")
include("tools/cmake/FindInclude.cmake")
include("tools/cmake/ShowMessage.cmake")
include("Src/Config/sdkconfig.cmake" OPTIONAL)
include("tools/cmake/Warning.cmake")
include("tools/cmake/Version.cmake")

project(${CMAKE_PROJECT_NAME} VERSION ${PROJECT_VERSION})

enable_language(C CXX ASM)

include("tools/cmake/FindLib/FindLib.cmake")

add_compile_options($<$<C_COMPILER_ID:GNU>:-fdiagnostics-color=always>)
add_compile_options($<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>)

add_executable(${CMAKE_PROJECT_NAME})

set(CMSISCORE "${soc_folder}/hal/Drivers/CMSIS")

add_subdirectory(Lib/Eigen3)
# add_subdirectory(Lib/TinyMPC)

add_subdirectory(Src/App)
add_subdirectory(${soc_folder}/hal/cmake/stm32cubemx)
add_subdirectory(Src/Common/Extensions)
add_subdirectory(Src/Common/Bsp)
add_subdirectory(Src/System)
add_subdirectory(Src/Utils)
add_subdirectory(Lib/CMSIS-DSP/Source)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE Src/PinyCore.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Src)

target_link_libraries(
  ${CMAKE_PROJECT_NAME}
  Extensions
  Bsp
  utils
  freertos_kernel
  # TinyMPC
)

showmessage()
