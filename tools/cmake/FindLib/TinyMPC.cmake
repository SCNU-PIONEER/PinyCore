# Reset stale download stamps when the source checkout has been removed.
if(NOT EXISTS "${THIRD_PARTY_DIR}/TinyMPC/CMakeLists.txt")
  file(REMOVE_RECURSE "${FETCHCONTENT_BASE_DIR}/tinympc-subbuild")
endif()

FetchContent_Declare(
  tinympc
  GIT_REPOSITORY https://github.com/TinyMPC/TinyMPC.git
  GIT_TAG main
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/TinyMPC"
  SOURCE_SUBDIR src/tinympc)

FetchContent_MakeAvailable(tinympc)

# Build the solver without host-only code generation or example executables.
get_target_property(TINYMPC_SOURCES tinympcstatic SOURCES)
list(REMOVE_ITEM TINYMPC_SOURCES codegen.cpp)
set_target_properties(tinympcstatic PROPERTIES
  SOURCES "${TINYMPC_SOURCES}"
  POSITION_INDEPENDENT_CODE OFF)

# Suppress upstream unused-variable warnings only in the affected source file.
set_property(SOURCE "${tinympc_SOURCE_DIR}/src/tinympc/admm.cpp"
  TARGET_DIRECTORY tinympcstatic
  APPEND PROPERTY COMPILE_OPTIONS -Wno-unused-variable)

# Suppress upstream ABI notes only in the Eigen-heavy source files.
set_property(SOURCE
  "${tinympc_SOURCE_DIR}/src/tinympc/rho_benchmark.cpp"
  "${tinympc_SOURCE_DIR}/src/tinympc/tiny_api.cpp"
  TARGET_DIRECTORY tinympcstatic
  APPEND PROPERTY COMPILE_OPTIONS -Wno-psabi)
