include (${CMAKE_SOURCE_DIR}/tools/cmake/FindInclude.cmake)

# Add the sourses
file(GLOB_RECURSE SOURCES  CMAKE_CONFIGURE_DEPENDS "*.c" "*.cpp")

# Add the library
add_library(Extensions STATIC ${SOURCES})

# Include the header directories
FIND_INCLUDE_DIR(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(Extensions PUBLIC ${HEADERS})

# link
target_link_libraries(Extensions PRIVATE Bsp utils Soc freertos_kernel)
