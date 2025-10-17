include(FetchContent)

set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/.deps)

# freertos
# why not choice cmsis_freertos?
# - because cmsis_freertos is a wrapper above FreeRTOS, although it has some advantages
#   such as mempool and safety malloc functions
#   but we have operated new/delete by FreeRTOS heap functions directly(not thread safe),
#   and we sure that we only use FreeRTOS instead of some other RTOS like zephyr or RT-Thread...
#   our rule is simple: don't do more abstraction layer than necessary
message("-------------------- find lib -----------------------")
FetchContent_Declare(
  FreeRTOS
  URL https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/refs/tags/V11.2.0.tar.gz
  URL_HASH
    SHA256=e148d3a442ac7652c998e0a932c4cc2a3c6abff66a3a0a96a7678c0cd092d41c
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_DIR ${CMAKE_SOURCE_DIR}/Lib/FreeRTOS)

add_library(freertos_config INTERFACE)

# Set path to folder containing FreeRTOSConfig.h
target_include_directories(freertos_config
                           INTERFACE "${CMAKE_SOURCE_DIR}/Src/Config")

target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

target_sources(freertos_config INTERFACE
  "${CMAKE_SOURCE_DIR}/Lib/systemview/Sample/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.c"
  "${CMAKE_SOURCE_DIR}/Lib/systemview/Sample/FreeRTOSV11/Config/Cortex-M/SEGGER_SYSVIEW_Config_FreeRTOS.c")

target_include_directories(freertos_config INTERFACE
  "${CMAKE_SOURCE_DIR}/Lib/systemview/Sample/FreeRTOSV11")

target_link_libraries(freertos_config INTERFACE segger)

target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

set(FREERTOS_HEAP
    "4"
    CACHE STRING "" FORCE)

# Select the native compile PORT
# note: stm32h7 use GCC_ARM_CM4F instead of GCC_ARM_CM7
# GCC_ARM_CM7 only support r0p1, and cubemx default is GCC_ARM_CM4F
# so we don't abstraction until we need to support other board like cortex-m3
set(FREERTOS_PORT
    "GCC_ARM_CM4F"
    CACHE STRING "" FORCE)

FetchContent_MakeAvailable(FreeRTOS)

# reference to https://kb.segger.com/FreeRTOS_with_SystemView
FetchContent_Declare(
  RTT
  GIT_REPOSITORY https://github.com/SEGGERMicro/RTT.git
  GIT_TAG main
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/Lib/RTT")

FetchContent_Declare(
  systemview
  GIT_REPOSITORY https://github.com/SEGGERMicro/SystemView.git
  GIT_TAG main
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/Lib/systemview")

file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS 
  "${CMAKE_SOURCE_DIR}/Lib/RTT/RTT/*.c"
  "${CMAKE_SOURCE_DIR}/Lib/RTT/RTT/*.S"
  "${CMAKE_SOURCE_DIR}/Lib/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c"
  "${CMAKE_SOURCE_DIR}/Lib/systemview/SYSVIEW/SEGGER_SYSVIEW.c")

add_library(segger STATIC ${SOURCES})

target_include_directories(segger PUBLIC
  "${CMAKE_SOURCE_DIR}/Lib/RTT/Config"
  "${CMAKE_SOURCE_DIR}/Lib/RTT/RTT"
  "${CMAKE_SOURCE_DIR}/Lib/systemview/Config"
  "${CMAKE_SOURCE_DIR}/Lib/systemview/SEGGER"
  "${CMAKE_SOURCE_DIR}/Lib/systemview/SYSVIEW")

FetchContent_MakeAvailable(RTT systemview)

FetchContent_Declare(
  cmsis5
  URL https://github.com/STMicroelectronics/cmsis-core/archive/refs/tags/v5.9.0.tar.gz
  URL_HASH
    SHA256=6cbeb8e53d029f18bae2fbf4be3c33abdce8e8e1a27a2cab69a8550478614fc7
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_DIR "${CMAKE_SOURCE_DIR}/Lib/CMSIS5")

FetchContent_MakeAvailable(cmsis5)

FetchContent_Declare(
  cmsis_device_f4
  GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
  GIT_TAG v2.6.11
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/Lib/CMSIS_Device_F4")

FetchContent_MakeAvailable(cmsis_device_f4)
FetchContent_Declare(
  hal
  GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
  GIT_TAG v1.8.5
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/Lib/STM32F4xx_HAL_Driver")

FetchContent_MakeAvailable(hal)

# FetchContent_Declare(
#   tinympc
#   GIT_REPOSITORY https://github.com/TinyMPC/TinyMPC.git
#   GIT_TAG main
#   GIT_SHALLOW TRUE
#   GIT_PROGRESS TRUE
#   SOURCE_DIR "${CMAKE_SOURCE_DIR}/Lib/TinyMPC")

# FetchContent_MakeAvailable(tinympc)


# execute_process(
#   COMMAND ${CMAKE_SOURCE_DIR}/tools/script/genSoc.sh
#   RESULT_VARIABLE hal_result
#   OUTPUT_VARIABLE hal_output
#   ERROR_VARIABLE hal_error
#   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/script)
#
# if(NOT hal_result EQUAL 0)
#   message(WARNING "genSoc run success: ${hal_error}")
# else()
#   message(STATUS "genSoc run failed: ${hal_output}")
# endif()
