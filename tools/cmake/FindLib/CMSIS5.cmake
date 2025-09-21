if(NOT EXISTS "${THIRD_PARTY_DIR}/CMSIS5")
  FetchContent_Declare(
    cmsis5
    URL https://github.com/STMicroelectronics/cmsis-core/archive/refs/tags/v5.9.0.tar.gz
    URL_HASH
      SHA256=6cbeb8e53d029f18bae2fbf4be3c33abdce8e8e1a27a2cab69a8550478614fc7
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS5")

  FetchContent_MakeAvailable(cmsis5)
endif()
