include(${CMAKE_SOURCE_DIR}/tools/cmake/FindInclude.cmake)

# file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS "*.c" "*.cpp")

# Cmd
file (GLOB_RECURSE CMD_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Cmd/Cmd.cpp"
    "Cmd/Handler/Handler.cpp")
list(APPEND SOURCES ${CMD_SOURCES})

# RcMsgHandler
if(EXTENSION_RC)
file (GLOB_RECURSE RCMSGHANDLER_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Cmd/Handler/RcMsgHandler/RcMsgHandler.cpp")
list(APPEND SOURCES ${RCMSGHANDLER_SOURCES})
endif()

# RefereeHandler
if(EXTENSION_REFEREE)
file (GLOB_RECURSE REFEREEHANDLER_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Cmd/Handler/RefereeHandler/RefereeHandler.cpp")
list(APPEND SOURCES ${REFEREEHANDLER_SOURCES})
endif()

# RttMsgHandler
file (GLOB_RECURSE RTTMSGHANDLER_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Cmd/Handler/RttMsgHandler/RttMsgHandler.cpp")
list(APPEND SOURCES ${RTTMSGHANDLER_SOURCES})

# Comm
file (GLOB_RECURSE COMM_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Comm/*.cpp")
list(APPEND SOURCES ${COMM_SOURCES})

# Daemons
file (GLOB_RECURSE DAEMONS_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Daemons/Daemons.cpp")
list(APPEND SOURCES ${DAEMONS_SOURCES})

# Identification
if(USE_MATHEMATICA_RLS AND USE_MATHEMATICA_IIR AND USE_MATHEMATICA_LPF)
file (GLOB_RECURSE IDENTIFICATION_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Identification/*.cpp"
    "Identification/YawSI/YawSI.cpp")
list(APPEND SOURCES ${IDENTIFICATION_SOURCES})
endif()

# INS
if(APP_USE_INS)
file (GLOB_RECURSE INS_SOURCES CMAKE_CONFIGURE_DEPENDS
    "INS/INS.cpp"
    "INS/Algorithm/Calibrate/calibrate.cpp"
    "INS/Algorithm/Calibrate/AccelCali/AccelCali.cpp"
    "INS/Algorithm/DcmAHRS/DcmAHRS.cpp"
    "INS/Algorithm/MahonyAHRS/MahonyAHRS.cpp")
list(APPEND SOURCES ${INS_SOURCES})
endif()

# Topic
file (GLOB_RECURSE TOPIC_SOURCES CMAKE_CONFIGURE_DEPENDS
    "Topic/*.cpp")
list(APPEND SOURCES ${TOPIC_SOURCES})

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SOURCES})

find_include_dir(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${HEADERS})

target_link_libraries(
  ${CMAKE_PROJECT_NAME} segger
)
