# Add the library
add_library(utils STATIC)

target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# TODO: use kconfig to choice the algorithm
if(USE_ALGORITHM)
  file(GLOB_RECURSE ALGORITHM_SOURCES CMAKE_CONFIGURE_DEPENDS "*.c" "*.cpp")
  target_sources(utils PRIVATE ${ALGORITHM_SOURCES})
  target_include_directories(
    utils
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/PID
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/Filter
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/Smooth
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/CRC
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/RLS
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/Matrix
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/LQR
           ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/KF)
endif()

if(USE_DWT)
  target_sources(utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Dwt/Dwt.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dwt)
endif()

if(USE_FSM)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/FSM)
endif()

if(USE_HARDFAULT)
  target_sources(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Hardfault/Hardfault.c)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Hardfault)
endif()

if(USE_STM_LOG)
  target_sources(utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Log/StmLog.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log)
endif()

if(USE_VOFA)
  target_sources(utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Vofa/Vofa.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Vofa)
endif()

target_sources(${CMAKE_PROJECT_NAME}
               PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/OperatorNew/OperatorNew.cpp)

# link
target_link_libraries(utils PRIVATE
  Soc
  segger
  Bsp
  freertos_kernel
  PUBLIC
  CMSISDSP)
