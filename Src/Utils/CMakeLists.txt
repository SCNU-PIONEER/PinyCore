# Add the library
add_library(utils STATIC)

if(USE_MATHEMATICA)
  function(add_mathematica_module config_name module_name)
    if(${config_name})
      file(GLOB_RECURSE MODULE_SOURCES
           "${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/${module_name}/*.c"
           "${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/${module_name}/*.cpp")
      if(MODULE_SOURCES)
        target_sources(utils PRIVATE ${MODULE_SOURCES})
      endif()
      target_include_directories(
        utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Mathematica/${module_name})
    endif()
  endfunction()

  add_mathematica_module(USE_MATHEMATICA_BALLISTIC Ballistic)
  add_mathematica_module(USE_MATHEMATICA_CORDIC Cordic)
  add_mathematica_module(USE_MATHEMATICA_CRC CRC)
  add_mathematica_module(USE_MATHEMATICA_FFT FFT)
  add_mathematica_module(USE_MATHEMATICA_FILTER Filter)
  add_mathematica_module(USE_MATHEMATICA_KF KF)
  add_mathematica_module(USE_MATHEMATICA_LQR LQR)
  add_mathematica_module(USE_MATHEMATICA_MATRIX Matrix)
  add_mathematica_module(USE_MATHEMATICA_OLS OLS)
  add_mathematica_module(USE_MATHEMATICA_PID PID)
  add_mathematica_module(USE_MATHEMATICA_RAMP Ramp)
  add_mathematica_module(USE_MATHEMATICA_RICCATI Riccati)
  add_mathematica_module(USE_MATHEMATICA_RLS RLS)
  add_mathematica_module(USE_MATHEMATICA_SMC SMC)
  add_mathematica_module(USE_MATHEMATICA_SMOOTH Smooth)
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
  if(LOG_OUTPUT_RTT)
    target_sources(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log/Output/RttOutput.cpp)
  endif()
  if(LOG_OUTPUT_UART)
    target_sources(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log/Output/UartOutput.cpp)
  endif()
  target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Log)
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
