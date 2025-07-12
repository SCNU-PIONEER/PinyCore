include(${CMAKE_SOURCE_DIR}/tools/cmake/FindInclude.cmake)

# Add the sourses
file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS "*.c" "*.cpp" "*.s")

# Add the library
add_library(stm32cubemx INTERFACE )

# Include the header directories
FIND_INCLUDE_DIR(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(stm32cubemx INTERFACE ${HEADERS})

# Add the source files
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${SOURCES})

# link
target_link_libraries(stm32cubemx
  INTERFACE
  Piny
  segger
  Utils
)

# compile
target_compile_definitions(stm32cubemx INTERFACE
  USE_HAL_DRIVER
  STM32F407xx
  ARM_MATH_CM4
  ARM_MATH_MATRIX_CHECK
  ARM_MATH_ROUNDING
  ARM_MATH_LOOPUNROLL
  DISABLEFLOAT16
  $<$<CONFIG:Debug>:DEBUG>
)
