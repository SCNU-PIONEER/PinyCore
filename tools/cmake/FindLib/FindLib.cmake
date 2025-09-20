include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/.deps)

set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/ThirdParty)

message("-------------------- find lib -----------------------")

include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/CMSIS5.cmake")
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/FreeRTOS.cmake")
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/Segger.cmake")
# include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/CMSIS_DSP.cmake")

FetchContent_Declare(
  cmsis_device_f4
  GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
  GIT_TAG v2.6.11
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS_Device_F4")

FetchContent_MakeAvailable(cmsis_device_f4)
FetchContent_Declare(
  hal
  GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
  GIT_TAG v1.8.5
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver")

FetchContent_MakeAvailable(hal)
# include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/TinyMPC.cmake")

