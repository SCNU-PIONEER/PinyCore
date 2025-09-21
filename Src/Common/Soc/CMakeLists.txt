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

if(NOT EXISTS "${target_folder}/hal/Drivers")
  message(WARNING "No HAL drivers from ${target_folder}/hal/Drivers")
  execute_process(
    COMMAND ${target_folder}/genSoc.sh
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
