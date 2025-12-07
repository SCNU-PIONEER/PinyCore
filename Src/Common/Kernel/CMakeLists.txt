target_sources(freertos_config INTERFACE Memory.cpp)

add_library(kernel)

target_include_directories(kernel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(kernel PUBLIC freertos_kernel)
