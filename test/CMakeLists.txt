cmake_minimum_required(VERSION 3.24)
project(pinyTest LANGUAGES CXX)
include(CTest)
enable_testing()

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.11.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)

FetchContent_MakeAvailable(Catch2)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_CXX_STANDARD 23)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

add_executable(pinyTest)

add_subdirectory(utils)

target_link_libraries(pinyTest PRIVATE Catch2::Catch2WithMain)
