# Add the sourses
file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS "Motor/*.cpp")

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
elseif(EXTENSION_ET08A)
  list(APPEND SOURCES "RemoteControl/ET08A/ET08A.cpp")
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
