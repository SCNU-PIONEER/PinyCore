FetchContent_Declare(
  cmsis_dsp
  GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
  GIT_TAG v1.16.2
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS-DSP")

FetchContent_MakeAvailable(cmsis_dsp)

set(CMSISCORE "${soc_folder}/hal/Drivers/CMSIS")
