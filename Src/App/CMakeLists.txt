include(${CMAKE_SOURCE_DIR}/tools/cmake/FindInclude.cmake)

file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS "AppManager.cpp")

# chassis type
if(APP_USE_MECANUM)
  set(CHASSIS_TYPE "Mecanum")
elseif(APP_USE_OMNI)
  set(CHASSIS_TYPE "Omni")
elseif(APP_USE_BIPEDAL)
  set(CHASSIS_TYPE "Bipedal")
elseif(APP_USE_SWERVE)
  set(CHASSIS_TYPE "Swerve")
endif()

file(GLOB_RECURSE CHASSIS_SOURCES CMAKE_CONFIGURE_DEPENDS
     "RobotModules/Chassis/Actuators/${CHASSIS_TYPE}/*.cpp")
list(APPEND SOURCES ${CHASSIS_SOURCES})

# chassis power control
if(APP_USE_POWERCTRL)
  file(GLOB_RECURSE POWERCTRL_SOURCES CMAKE_CONFIGURE_DEPENDS
       "RobotModules/Chassis/PowerControl/*.cpp")
  list(APPEND SOURCES ${POWERCTRL_SOURCES})
endif()

# gimbal type
if(APP_USE_STANDARD)
  set(GIMBAL_TYPE "Standard")
elseif(APP_USE_STACKED)
  set(GIMBAL_TYPE "Stacked")
endif()

file(GLOB_RECURSE GIMBAL_SOURCES CMAKE_CONFIGURE_DEPENDS
  "RobotModules/Gimbal/Actuators/${GIMBAL_TYPE}/*.cpp")
list(APPEND SOURCES ${GIMBAL_SOURCES})

# ui
if(APP_USE_UI)
  file(GLOB_RECURSE UI_SOURCES CMAKE_CONFIGURE_DEPENDS "UI/*.cpp")
  list(APPEND SOURCES ${UI_SOURCES})
endif()

# test
if(APP_USE_TEST)
  file(GLOB_RECURSE TEST_SOURCES CMAKE_CONFIGURE_DEPENDS "Test/*.cpp")
  list(APPEND SOURCES ${TEST_SOURCES})
endif()

# daemons
if(APP_USE_DAEMONS)
  file(GLOB_RECURSE DAEMONS_SOURCES CMAKE_CONFIGURE_DEPENDS "Daemons/*.cpp")
  list(APPEND SOURCES ${DAEMONS_SOURCES})
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SOURCES})

find_include_dir(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${HEADERS})
