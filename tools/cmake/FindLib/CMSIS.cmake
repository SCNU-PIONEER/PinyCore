if(DEFINED TARGET_STM32F407)
  if(NOT EXISTS ${THIRD_PARTY_DIR}/CMSIS_Device_F4)
    FetchContent_Declare(
      cmsis_device_f4
      GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
      GIT_TAG v2.6.11
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS_Device_F4")
    FetchContent_MakeAvailable(cmsis_device_f4)
  endif()

  if(NOT EXISTS ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver)
    FetchContent_Declare(
      hal_f4
      GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
      GIT_TAG v1.8.5
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver")
    FetchContent_MakeAvailable(hal_f4)
  endif()

else(DEFINED TARGET_STM32H723)
  if(NOT EXISTS ${THIRD_PARTY_DIR}/CMSIS_Device_H7)
    FetchContent_Declare(
      cmsis_device_h7
      GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-h7.git
      GIT_TAG v1.10.6
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS_Device_H7")
    FetchContent_MakeAvailable(cmsis_device_h7)
  endif()

  if(NOT EXISTS ${THIRD_PARTY_DIR}/STM32H7xx_HAL_Driver)
    FetchContent_Declare(
      hal_h7
      GIT_REPOSITORY https://github.com/STMicroelectronics/stm32h7xx-hal-driver.git
      GIT_TAG v1.11.5
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
      SOURCE_DIR "${THIRD_PARTY_DIR}/STM32H7xx_HAL_Driver")
    FetchContent_MakeAvailable(hal_h7)
  endif()
endif()

