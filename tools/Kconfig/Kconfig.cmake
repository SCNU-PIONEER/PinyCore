find_program(KCONFIG_DEFCONF defconfig)
find_program(KCONFIG_MCONF menuconfig)
find_package(Python REQUIRED COMPONENTS Interpreter)

if(KCONFIG_MCONF)
    message(STATUS "Found menuconfig: ${KCONFIG_MCONF}")
else()
    message(FATAL_ERROR "Could not find kconfiglib,\
                        please use 'pip install kconfiglib' to install it")
endif()

set(PYTHON_SCRIPT "${CMAKE_SOURCE_DIR}/tools/script/Kconfig2h.py")

add_custom_target(clean_all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove .config
    COMMENT "Removing build/ and .config"
)

add_custom_target(menuconfig
  COMMAND ${KCONFIG_MCONF} "${CMAKE_SOURCE_DIR}/Kconfig"
  COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
  COMMAND ${CMAKE_COMMAND} -B "${CMAKE_BINARY_DIR}" -G Ninja
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Launching menuconfig (Kconfig interface)"
  USES_TERMINAL
)

function (apply_config config_name)
    if(NOT EXISTS "${CMAKE_SOURCE_DIR}/${config_name}")
        message(FATAL_ERROR "${CMAKE_SOURCE_DIR}/${config_name} not found!")
    endif()

    message(STATUS "Applying configuration: ${config_name}")
    execute_process(
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND ${KCONFIG_DEFCONF} ${config_name}
        ERROR_VARIABLE kconfig_error
        RESULT_VARIABLE kconfig_result
    )
    if(NOT kconfig_result EQUAL 0)
        message(FATAL_ERROR 
            "Failed to apply configuration: ${config_name}\n"
            "Error: ${kconfig_error}"
        )
    endif()
    message(STATUS "Successfully generate ${CMAKE_SOURCE_DIR}/.config")

    execute_process(
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
        ERROR_VARIABLE py_error
        RESULT_VARIABLE py_result
    )
    if(NOT py_result EQUAL 0)
        message(FATAL_ERROR 
            "Failed to generate sdkconfig files\n"
            "Error: ${py_error}"
        )
    endif()
endfunction()

if(DEFINED CONFIG_NAME)
  apply_config(${CONFIG_NAME})
endif()

if(NOT EXISTS "${CMAKE_BINARY_DIR}/build.ninja" OR NOT EXISTS "${CMAKE_SOURCE_DIR}/Src/Config/sdkconfig.h")
    message(WARNING "No found build, Using default config")

    if(EXISTS ${CMAKE_SOURCE_DIR}/defconfig)
        execute_process(
        COMMAND ${KCONFIG_DEFCONF} ${CMAKE_SOURCE_DIR}/defconfig
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    else()
        execute_process(
        COMMAND ${KCONFIG_DEFCONF} ${CMAKE_SOURCE_DIR}/Kconfig
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif()

    execute_process(
    COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE kconfig_result)
endif()
