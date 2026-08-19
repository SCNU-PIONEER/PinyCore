FetchContent_Declare(
  cmsis_dsp
  GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
  GIT_TAG v1.17.1
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS-DSP")

set(CMSISCORE "${soc_folder}/hal/Drivers/CMSIS")

FetchContent_MakeAvailable(cmsis_dsp)

target_compile_options(CMSISDSP PRIVATE -O3  -g3 -ffast-math)
