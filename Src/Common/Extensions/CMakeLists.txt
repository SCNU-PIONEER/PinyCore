# Add the sources
set(SOURCES)

if(EXTENSION_MOTOR)
  set(MOTOR_SOURCES
      Motor/Core/MotorManager.cpp
      Motor/Core/Base/IMotor.cpp
      Motor/Core/Base/QuadMotorBase.cpp
      Motor/Core/Base/TripMotorBase.cpp)

  if(MOTOR_DJI_M3508 OR MOTOR_DJI_M2006 OR MOTOR_DJI_GM6020)
    list(APPEND MOTOR_SOURCES
         Motor/Core/Imp/DJI/DJIMotor.cpp
         Motor/Core/Imp/DJI/WorkMode.cpp)
  endif()
  if(MOTOR_DJI_M3508)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DJI/M3508/M3508.cpp)
  endif()
  if(MOTOR_DJI_M2006)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DJI/M2006/M2006.cpp)
  endif()
  if(MOTOR_DJI_GM6020)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DJI/GM6020/GM6020.cpp)
  endif()

  if(MOTOR_DJI_OD_GM3510)
    list(APPEND MOTOR_SOURCES
         Motor/Core/Imp/DJI_OD/DJIOldMotor.cpp
         Motor/Core/Imp/DJI_OD/WorkMode.cpp
         Motor/Core/Imp/DJI_OD/GM3510/GM3510.cpp)
  endif()

  if(MOTOR_DM10010L OR MOTOR_DM3507 OR MOTOR_DM3519 OR MOTOR_DM4310
     OR MOTOR_DM4340 OR MOTOR_DM6006 OR MOTOR_DM8009)
    list(APPEND MOTOR_SOURCES
         Motor/Core/Imp/DAMIAO/DMMotor.cpp
         Motor/Core/Imp/DAMIAO/RegOperator.cpp
         Motor/Core/Imp/DAMIAO/WorkMode.cpp)
  endif()
  if(MOTOR_DM10010L)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM10010L/DM10010L.cpp)
  endif()
  if(MOTOR_DM3507)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM3507/DM3507.cpp)
  endif()
  if(MOTOR_DM3519)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM3519/DM3519.cpp)
  endif()
  if(MOTOR_DM4310)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM4310/DM4310.cpp)
  endif()
  if(MOTOR_DM4340)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM4340/DM4340.cpp)
  endif()
  if(MOTOR_DM6006)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM6006/DM6006.cpp)
  endif()
  if(MOTOR_DM8009)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/DAMIAO/DM8009/DM8009.cpp)
  endif()

  if(MOTOR_LK_MF9025 OR MOTOR_LK_MG4005 OR MOTOR_LK_MG8016E_I6_RS485)
    # LK picks its CAN/RS485 backend at compile time via the bus-mode choice
    # in LK/Kconfig (MF9025_CAN default -> LKMotorCAN, *_RS485 -> LKMotorRS485,
    # *_CAN_BROADCAST -> legacy LKMotor). Compile all three backends so the
    # link succeeds regardless of the selected bus mode.
    list(APPEND MOTOR_SOURCES
         Motor/Core/Imp/LK/LKMotor.cpp
         Motor/Core/Imp/LK/LKMotorCan.cpp
         Motor/Core/Imp/LK/LKMotorRS485.cpp)
  endif()
  if(MOTOR_LK_MF9025)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/LK/MF9025/MF9025.cpp)
  endif()
  if(MOTOR_LK_MG4005)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/LK/MG4005/MG4005.cpp)
  endif()
  if(MOTOR_LK_MG8016E_I6_RS485)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/LK/MG8016E-i6/MG8016E_i6_RS485.cpp)
  endif()

  if(MOTOR_RMD_X2_7 OR MOTOR_RMD_X4_36 OR MOTOR_RMD_X4_10)
    list(APPEND MOTOR_SOURCES
         Motor/Core/Imp/MAITA/MTMotor.cpp
         Motor/Core/Imp/MAITA/ReadMTDescription.cpp
         Motor/Core/Imp/MAITA/WorkCode.cpp)
  endif()
  if(MOTOR_RMD_X2_7)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/MAITA/RMD-X2-7/RMD-X2-7.cpp)
  endif()
  if(MOTOR_RMD_X4_36)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/MAITA/RMD-X4-36/RMD-X4-36.cpp)
  endif()
  if(MOTOR_RMD_X4_10)
    list(APPEND MOTOR_SOURCES Motor/Core/Imp/MAITA/RMD-X4-10/RMD-X4-10.cpp)
  endif()

  if(MOTOR_UNITREE_UT8010_6)
    list(APPEND MOTOR_SOURCES
         Motor/Core/Imp/Unitree/UTMotor.cpp
         Motor/Core/Imp/Unitree/UT8010_6/UT8010_6.cpp)
    if(NOT USE_MATHEMATICA_CRC)
      list(APPEND MOTOR_SOURCES
           ${CMAKE_SOURCE_DIR}/Src/Utils/Mathematica/CRC/Crc.cpp)
    endif()
  endif()

  list(LENGTH MOTOR_SOURCES MOTOR_SOURCE_COUNT)
  if(MOTOR_SOURCE_COUNT LESS 5)
    message(FATAL_ERROR
            "EXTENSION_MOTOR is enabled, but no motor model was selected. "
            "Pick a motor model in menuconfig (e.g. CONFIG_MOTOR_DJI_GM6020).")
  endif()
  list(APPEND SOURCES ${MOTOR_SOURCES})
endif()

if(EXTENSION_BUZZER)
  list(APPEND SOURCES "buzzer/Buzzer.cpp")
endif()

if(EXTENSION_LED)
    list(APPEND SOURCES "LED/LEDManager.cpp")
    if(LED_WS2812_SPI)
        list(APPEND SOURCES "LED/Driver/WS2812/WS2812SPIDriver.cpp")
    endif()
    if(LED_WS2812_PWM)
        list(APPEND SOURCES "LED/Driver/WS2812/WS2812PWMDriver.cpp")
    endif()
    if(LED_RPGLED)
        list(APPEND SOURCES "LED/Driver/RPGLED/RPGLEDDriver.cpp")
    endif()
endif()

if(EXTENSION_DT7)
  list(APPEND SOURCES "RemoteControl/DT7/DT7.cpp")
endif()
if(EXTENSION_ET08A)
  list(APPEND SOURCES "RemoteControl/ET08A/ET08A.cpp")
endif()
if(EXTENSION_VT13)
  list(APPEND SOURCES "RemoteControl/VT13/VT13.cpp")
endif()

if(EXTENSION_REFEREE)
  list(APPEND SOURCES "Referee/Referee.cpp")
endif()

if(EXTENSION_SENSOR)
  list(APPEND SOURCES "Sensor/BMI088/Bmi088.cpp" "Sensor/BMI088/Middleware.cpp")
endif()

if(EXTENSION_SUPERCAP)
  list(APPEND SOURCES "SuperCap/SuperCap.cpp")
endif()

# Add the library
add_library(Extensions STATIC ${SOURCES})

# Include the header directories
find_include_dir(HEADERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(Extensions PUBLIC ${HEADERS})

# link
target_link_libraries(Extensions PRIVATE Bsp utils Soc kernel)
