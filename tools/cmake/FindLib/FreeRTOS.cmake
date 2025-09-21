  FetchContent_Declare(
    FreeRTOS
    URL https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/refs/tags/V11.2.0.tar.gz
    URL_HASH
      SHA256=e148d3a442ac7652c998e0a932c4cc2a3c6abff66a3a0a96a7678c0cd092d41c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_DIR "${THIRD_PARTY_DIR}/FreeRTOS")

add_library(freertos_config INTERFACE)

# Set path to folder containing FreeRTOSConfig.h
target_include_directories(freertos_config INTERFACE
  "${CMAKE_SOURCE_DIR}/Src/Config")

target_sources(freertos_config INTERFACE
  "${THIRD_PARTY_DIR}/systemview/Sample/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.c"
  "${THIRD_PARTY_DIR}/systemview/Sample/FreeRTOSV11/Config/Cortex-M/SEGGER_SYSVIEW_Config_FreeRTOS.c")

target_include_directories(freertos_config INTERFACE
  "${THIRD_PARTY_DIR}/systemview/Sample/FreeRTOSV11")

target_link_libraries(freertos_config INTERFACE segger)

target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

set(FREERTOS_HEAP
    "4"
    CACHE STRING "" FORCE)

# Select the native compile PORT
set(FREERTOS_PORT
    "GCC_ARM_CM4F"
    CACHE STRING "" FORCE)

FetchContent_MakeAvailable(FreeRTOS)
