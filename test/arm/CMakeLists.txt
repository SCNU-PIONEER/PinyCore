file(GLOB_RECURSE UTIL_SOURCES CMAKE_CONFIGURE_DEPENDS
     "../../Src/App/RobotModules/Arm/ArmKinematic.cpp")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${UTIL_SOURCES} test_arm.cpp)
target_include_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE ../../Src/App/RobotModules/Arm
                                ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TEST)

add_test(NAME rotation2euler COMMAND ${CMAKE_PROJECT_NAME} "[rotation2euler]")
add_test(NAME dh COMMAND ${CMAKE_PROJECT_NAME} "[dh]")
add_test(NAME fk COMMAND ${CMAKE_PROJECT_NAME} "[fk]")
add_test(NAME jacob COMMAND ${CMAKE_PROJECT_NAME} "[jacob]")
add_test(NAME ik COMMAND ${CMAKE_PROJECT_NAME} "[ik]")
