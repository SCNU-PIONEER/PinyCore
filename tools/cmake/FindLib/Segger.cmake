# reference to https://kb.segger.com/FreeRTOS_with_SystemView
FetchContent_Declare(
  RTT
  GIT_REPOSITORY https://github.com/SEGGERMicro/RTT.git
  GIT_TAG main
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/RTT")

FetchContent_Declare(
  systemview
  GIT_REPOSITORY https://github.com/SEGGERMicro/SystemView.git
  GIT_TAG main
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/systemview")

FetchContent_MakeAvailable(RTT systemview)

file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS 
  "${THIRD_PARTY_DIR}/RTT/RTT/*.c"
  "${THIRD_PARTY_DIR}/RTT/RTT/*.S"
  "${THIRD_PARTY_DIR}/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c"
  "${THIRD_PARTY_DIR}/systemview/SYSVIEW/SEGGER_SYSVIEW.c")

add_library(segger STATIC ${SOURCES})

target_include_directories(segger PUBLIC
  "${THIRD_PARTY_DIR}/RTT/Config"
  "${THIRD_PARTY_DIR}/RTT/RTT"
  "${THIRD_PARTY_DIR}/systemview/Config"
  "${THIRD_PARTY_DIR}/systemview/SEGGER"
  "${THIRD_PARTY_DIR}/systemview/SYSVIEW")
