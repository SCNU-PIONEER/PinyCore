INCLUDE(../../sdkconfig.cmake option)
add_library(Soc INTERFACE)

if(DEFINED TARGET_STM32H723)
    set(target_folder "stm32h723")
    set(cortex "cortex-m7" PARENT_SCOPE)
    set(fpu "fpv5-d16" PARENT_SCOPE)
    set(float-abi "hard" PARENT_SCOPE)
elseif(DEFINED TARGET_STM32F407)
    set(target_folder "stm32f407")
    set(cortex "cortex-m4" PARENT_SCOPE)
    set(fpu "fpv4-sp-d16" PARENT_SCOPE)
    set(float-abi "hard" PARENT_SCOPE)
    # elseif(CONFIG_TARGET STREQUAL "stm32f446")
    #   set(target_folder "stm32f446" )
    #   set(cortex "cortex-m4" PARENT_SCOPE)
    # elseif(CONFIG_TARGET STREQUAL "stm32g473")
    #   set(target_folder "stm32fg473")
    #   set(cortex "cortex-m4" PARENT_SCOPE)
else()
    message(WARNING "Using defaust Soc board: stm32f407" )
    set(target_folder "stm32f407")
    set(cortex "cortex-m4" PARENT_SCOPE)
    set(fpu "fpv4-sp-d16" PARENT_SCOPE)
    set(float-abi "hard" PARENT_SCOPE)
endif()

message( "soc board: ${target_folder}" )
set(soc_folder "${CMAKE_CURRENT_SOURCE_DIR}/${target_folder}" PARENT_SCOPE)

# includes
target_include_directories(Soc
    INTERFACE ${target_folder})

