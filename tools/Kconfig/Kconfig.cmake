find_program(KCONFIG_CONF kconfig-conf)
find_program(KCONFIG_MCONF kconfig-mconf)
find_package(Python REQUIRED COMPONENTS Interpreter)

if(KCONFIG_CONF)
  message(STATUS "Found kconfig-conf: ${KCONFIG_CONF}")
else()
  message(FATAL_ERROR "Could not find kconfig-conf/mconf,\
                        please install kconfig-frontends")
endif()

set(PYTHON_SCRIPT "${CMAKE_SOURCE_DIR}/tools/script/Kconfig2h.py")

add_custom_target(menuconfig
  COMMAND ${KCONFIG_MCONF} "${CMAKE_SOURCE_DIR}/Kconfig"
  COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
  COMMAND ${CMAKE_COMMAND} -B "${CMAKE_BINARY_DIR}" -G Ninja --fresh
  COMMAND ${CMAKE_COMMAND} -B "${CMAKE_BINARY_DIR}" -G Ninja   # BUG: need to fresh two times
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Launching menuconfig (Kconfig interface)"
  USES_TERMINAL
)

if(EXISTS "${CMAKE_BINARY_DIR}")
  message(STATUS "Found build: ${build_exists}")
else()
  message(WARNING "No found build, Using default config")

  # kconfig default config
  execute_process(
    COMMAND ${KCONFIG_CONF}
            --defconfig=${CMAKE_SOURCE_DIR}/Kconfig
            ${CMAKE_SOURCE_DIR}/Kconfig
    COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
    COMMAND -B build --fresh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE kconfig_result
)
endif()

file(GLOB CONFIG_FILE "${CMAKE_SOURCE_DIR}/Src/sdkconfig.h")
if(CONFIG_FILE)
  message(STATUS "Found config file: ${CONFIG_FILE}")
else()
  message(WARNING "No config file found, Using default config")

  # kconfig default config
  execute_process(
    COMMAND ${KCONFIG_CONF}
            --defconfig=${CMAKE_SOURCE_DIR}/Kconfig
            ${CMAKE_SOURCE_DIR}/Kconfig
    COMMAND ${Python_EXECUTABLE} ${PYTHON_SCRIPT} "${CMAKE_SOURCE_DIR}/.config"
    COMMAND -B build --fresh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE kconfig_result
)
endif()
