FetchContent_Declare(
    FreeRTOS
    URL https://github.com/FreeRTOS/FreeRTOS-Kernel/releases/download/V11.3.0/FreeRTOS-KernelV11.3.0.zip
    URL_HASH
    SHA256=e9187bb79db6115c81b50f4d9fc48ac91a1d42a738378f56a540965fd26feee7
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
