INCLUDE(../../Config/sdkconfig.cmake option)

add_library(Soc INTERFACE)

if(DEFINED TARGET_STM32H723)
    set(soc_target "stm32h723")
    set(cortex "cortex-m7" PARENT_SCOPE)
    set(fpu "fpv5-d16" PARENT_SCOPE)
    set(float-abi "hard" PARENT_SCOPE)
elseif(DEFINED TARGET_STM32F407)
    set(soc_target "stm32f407")
    set(cortex "cortex-m4" PARENT_SCOPE)
    set(fpu "fpv4-sp-d16" PARENT_SCOPE)
    set(float-abi "hard" PARENT_SCOPE)
else()
    message(WARNING "Using defaust Soc board: stm32f407" )
    set(soc_target "stm32f407")
    set(cortex "cortex-m4" PARENT_SCOPE)
    set(fpu "fpv4-sp-d16" PARENT_SCOPE)
    set(float-abi "hard" PARENT_SCOPE)
endif()

message("soc board: ${soc_target}")

set(soc_folder "${CMAKE_CURRENT_SOURCE_DIR}/${soc_target}" PARENT_SCOPE)

set(target_folder "${CMAKE_CURRENT_SOURCE_DIR}/${soc_target}")
target_include_directories(Soc INTERFACE ${target_folder})

target_link_libraries(Soc INTERFACE stm32cubemx)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/.deps)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS5")
  FetchContent_Declare(
    cmsis5
    URL https://github.com/STMicroelectronics/cmsis-core/archive/refs/tags/v5.9.0.tar.gz
    URL_HASH
      SHA256=6cbeb8e53d029f18bae2fbf4be3c33abdce8e8e1a27a2cab69a8550478614fc7
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS5")
  FetchContent_MakeAvailable(cmsis5)
endif()

if(DEFINED TARGET_STM32F407)
  if(NOT EXISTS "${target_folder}/hal/Drivers/CMSIS/Device" OR NOT EXISTS )
    FetchContent_Declare(
      cmsis_device_f4
      GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
      GIT_TAG v2.6.11
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${target_folder}/CMSIS_Device")
    FetchContent_MakeAvailable(cmsis_device_f4)
  endif()

  if(NOT EXISTS "${target_folder}/HAL_Driver")
    FetchContent_Declare(
      hal_f4
      GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
      GIT_TAG v1.8.5
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${target_folder}/HAL_Driver")
    FetchContent_MakeAvailable(hal_f4)
  endif()

else(DEFINED TARGET_STM32H723)
  if(NOT EXISTS "${target_folder}/CMSIS_Device")
    FetchContent_Declare(
      cmsis_device_h7
      GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-h7.git
      GIT_TAG v1.10.6
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${target_folder}/CMSIS_Device")
    FetchContent_MakeAvailable(cmsis_device_h7)
  endif()

  if(NOT EXISTS "${target_folder}/HAL_Driver")
    FetchContent_Declare(
      hal_h7
      GIT_REPOSITORY https://github.com/STMicroelectronics/stm32h7xx-hal-driver.git
      GIT_TAG v1.11.5
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${target_folder}/HAL_Driver")
    FetchContent_MakeAvailable(hal_h7)
  endif()
endif()

if(NOT EXISTS "${target_folder}/hal/Drivers/CMSIS/Device" OR NOT EXISTS "${target_folder}/hal/Drivers/CMSIS/Include")
  message(WARNING "No HAL drivers from ${target_folder}/hal/Drivers, generating...")
  execute_process(
    COMMAND ${Python_EXECUTABLE} ${target_folder}/genSoc.py
    RESULT_VARIABLE soc_result
    OUTPUT_VARIABLE soc_output
    ERROR_VARIABLE soc_error
    ECHO_ERROR_VARIABLE
    WORKING_DIRECTORY ${target_folder})

  if(NOT soc_result EQUAL 0)
    message(ERROR "genSoc.sh run failed: ${soc_error}")
  else()
    message(STATUS "genSoc.sh run success: ${soc_output}")
  endif()
endif()
