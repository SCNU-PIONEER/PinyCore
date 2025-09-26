find_program(KCONFIG_DEFCONF alldefconfig)
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

if(NOT EXISTS "${CMAKE_BINARY_DIR}/build.ninja" OR NOT EXISTS "${CMAKE_SOURCE_DIR}/Src/Config/sdkconfig.h")
    message(WARNING "No found build, Using default config")

    # kconfig default config
    execute_process(
    COMMAND ${KCONFIG_DEFCONF} ${CMAKE_SOURCE_DIR}/Kconfig
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
    execute_process(
    # COMMAND ${KCONFIG_DEFCONF} ${CMAKE_SOURCE_DIR}/Kconfig
    COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE kconfig_result
)
endif()

