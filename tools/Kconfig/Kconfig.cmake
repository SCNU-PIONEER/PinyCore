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

# BUG: wait for check update
# file(TIMESTAMP sdkconfig.h OLD_TIME)
# file(TIMESTAMP sdkconfig.h NEW_TIME)

add_custom_target(clean_all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove .config
    COMMENT "Removing build/ and .config"
)

add_custom_target(menuconfig
  COMMAND ${KCONFIG_MCONF} "${CMAKE_SOURCE_DIR}/Kconfig"
  COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
  # COMMAND ${CMAKE_COMMAND}
  #   -DARG1=${OLD_TIME}
  #   -DARG2=${NEW_TIME}
  #   -P "${CMAKE_CURRENT_LIST_DIR}/checkUpdate.cmake"
  COMMAND ${CMAKE_COMMAND} -B "${CMAKE_BINARY_DIR}" -G Ninja --fresh
  COMMAND ${CMAKE_COMMAND} -B "${CMAKE_BINARY_DIR}" -G Ninja   # BUG: need to fresh two times
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Launching menuconfig (Kconfig interface)"
  USES_TERMINAL
)

if(NOT EXISTS "${CMAKE_BINARY_DIR}/build.ninja" OR NOT EXISTS "${CMAKE_SOURCE_DIR}/Src/sdkconfig.h")
  message(WARNING "No found build, Using default config")

  # kconfig default config
  execute_process(
    COMMAND ${KCONFIG_DEFCONF} ${CMAKE_SOURCE_DIR}/Kconfig
    COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
    COMMAND -B build --fresh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE kconfig_result
)
endif()

