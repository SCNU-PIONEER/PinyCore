FetchContent_Declare(
  etl
  GIT_REPOSITORY https://github.com/ETLCPP/etl.git
  GIT_TAG 20.44.1
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/etl")

FetchContent_MakeAvailable(etl)

set(ETL_CONFIG
    ETL_CPP23_SUPPORTED
    ETL_NO_STL
    ETL_NO_64BIT_TYPES
)

target_compile_definitions(etl INTERFACE ${ETL_CONFIG})
