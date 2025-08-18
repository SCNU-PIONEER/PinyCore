# Add the library
add_library(Utils STATIC)

target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(USE_ALGORITHM)
  file(GLOB_RECURSE ALGORITHM_SOURCES CMAKE_CONFIGURE_DEPENDS "*.c" "*.cpp")
  target_sources(Utils PRIVATE ${ALGORITHM_SOURCES})
  target_include_directories(Utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/PID
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/Filter
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/Smooth
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/CRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/RLS
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/Matrix
    ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/LQR
  )
endif()

if(USE_DWT)
  target_sources(Utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Dwt/Dwt.c)
  target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dwt)
endif()

if(USE_FSM)
  target_sources(Utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/FSM/FSMState.cpp)
  target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/FSM)
endif()

if(USE_HARDFAULT)
  target_sources(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Hardfault/Hardfault.c)
  target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Hardfault)
endif()

if(USE_STM_LOG)
  target_sources(Utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Log/StmLog.cpp)
  target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log)
endif()

if(USE_VOFA)
  target_sources(Utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Vofa/Vofa.cpp)
  target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Vofa)
endif()

# link
target_link_libraries(Utils PRIVATE stm32cubemx Soc segger Bsp PUBLIC CMSISDSP)

