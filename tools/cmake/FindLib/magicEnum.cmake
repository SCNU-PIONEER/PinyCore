FetchContent_Declare(
  magic_enum
  GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/magic_enum")

FetchContent_MakeAvailable(magic_enum)
