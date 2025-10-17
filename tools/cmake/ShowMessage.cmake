function(showMessage)
  message("------------------- build info ----------------------")
  message("-- Build type:" ${CMAKE_BUILD_TYPE})
  message("-- C Standard: " ${CMAKE_C_STANDARD})
  message("-- C++ Standard: " ${CMAKE_CXX_STANDARD})
  message("-- c compiler: " ${CMAKE_C_COMPILER_VERSION})
  message("-- c++ compiler: " ${CMAKE_CXX_COMPILER_VERSION})
  message("-- robot version: " ${GIT_VERSION})
  message("-----------------------------------------------------")
  message(
    "    ____   _                  ______             \n"
    "   / __ \\ (_)____   __  __   / ____/____   _____ ___ \n"
    "  / /_/ // // __ \\ / / / /  / /    / __ \\ / ___// _ \\\n"
    " / ____// // / / // /_/ /  / /___ / /_/ // /   /  __/\n"
    "/_/    /_//_/ /_/ \\__, /   \\____/ \\____//_/    \\___/ \n"
    "                 /____/                              \n")
endfunction()
