# pid
file(GLOB_RECURSE UTIL_SOURCES CMAKE_CONFIGURE_DEPENDS
     "../../Src/Utils/Mathematica/PID/*.cpp")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${UTIL_SOURCES} test_pid.cpp)
target_include_directories(${CMAKE_PROJECT_NAME}
                           PRIVATE ../../Src/Utils/Mathematica/PID)

add_test(NAME positional_pid COMMAND ${CMAKE_PROJECT_NAME} "[positional pid]")
add_test(NAME incremental_pid COMMAND ${CMAKE_PROJECT_NAME} "[incremental pid]")
