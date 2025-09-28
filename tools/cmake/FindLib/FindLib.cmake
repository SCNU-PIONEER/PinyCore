include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/.deps)

set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/ThirdParty)

message("-------------------- find lib -----------------------")

include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/FreeRTOS.cmake")
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/Segger.cmake")
# include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/CMSIS_DSP.cmake")
# include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/TinyMPC.cmake")

