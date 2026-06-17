# Add the library
add_library(utils STATIC)

# TODO: use kconfig to choice the mathematica modules
if(USE_MATHEMATICA)
  file(GLOB_RECURSE MATHEMATICA_SOURCES CONFIGURE_DEPENDS
       "${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/*.c"
       "${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/*.cpp")
  target_sources(utils PRIVATE ${MATHEMATICA_SOURCES})
  target_include_directories(
    utils
    PRIVATE ../../Lib/CMSIS-DSP/Include
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
  target_sources(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dwt/Dwt.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dwt)
endif()

if(USE_FSM)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/FSM)
endif()

if(USE_HARDFAULT)
  target_sources(utils
                 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Hardfault/Hardfault.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Hardfault)
endif()

if(USE_STM_LOG)
  target_sources(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log/StmLog.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log)
endif()

if(USE_VOFA)
  target_sources(utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Vofa/Vofa.cpp)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Vofa)
endif()

if(USE_LAZY)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Lazy)
endif()

if(USE_SINGLETON)
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Singleton)
endif()

target_sources(${CMAKE_PROJECT_NAME}
               PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/OperatorNew/OperatorNew.cpp)

# link
target_link_libraries(
  utils
  PRIVATE Soc segger Bsp freertos_kernel)

if(USE_MATHEMATICA AND NOT DEFINED TARGET_STM32F103C8TX)
  target_link_libraries(utils PUBLIC CMSISDSP)
endif()
