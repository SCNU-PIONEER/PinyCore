include (${CMAKE_SOURCE_DIR}/tools/cmake/FindInclude.cmake)

# Add the sourses
file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS "*.c" "*.cpp" "*.s")

list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32h7xx.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32h7xx_it.c")

file(GLOB_RECURSE SYSCALL CMAKE_CONFIGURE_DEPENDS
       "syscalls.c" "sysmem.c" "system_stm32h7xx.c" "stm32h7xx_it.c")

# Add the library
add_library(stm32cubemx STATIC ${SOURCES})
add_library(syscall OBJECT ${SYSCALL})

# Include the header directories
FIND_INCLUDE_DIR(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(stm32cubemx PUBLIC ${HEADERS})

# Include the header directories
FIND_INCLUDE_DIR(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(syscall PUBLIC ${HEADERS})

# link
target_link_libraries(stm32cubemx
  PUBLIC
  App
  Utils
  Piny
  segger
)

# compile
target_compile_definitions(syscall PRIVATE
  STM32H723xx
)

target_compile_options(stm32cubemx PUBLIC -Wno-deprecated-volatile)

# compile
target_compile_definitions(stm32cubemx PUBLIC
  USE_HAL_DRIVER
  STM32H723xx
  ARM_MATH_CM7
  ARM_MATH_MATRIX_CHECK
  ARM_MATH_ROUNDING
  ARM_MATH_LOOPUNROLL
  DISABLEFLOAT16
)
