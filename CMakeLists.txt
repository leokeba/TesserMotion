# ESP-IDF component when built by idf.py, host library + tests otherwise.
set(TESSERMOTION_CORE_SRCS
    src/tessermotion/dc_motor.cpp
    src/tessermotion/tb6612.cpp
    src/tessermotion/mt6701.cpp
    src/tessermotion/adxl345.cpp
    src/tessermotion/nmea.cpp
    src/tessermotion/angle_correction.cpp
    src/tessermotion/axis.cpp
    src/tessermotion/motion.cpp
    src/tessermotion/tilt.cpp
    src/tessermotion/nmea_gps.cpp
    src/tessermotion/axis_tree.cpp
    src/tessermotion/motion_tree.cpp
    src/tessermotion/tilt_tree.cpp
    src/tessermotion/gps_tree.cpp)

if(ESP_PLATFORM)
    # TesserAPI is a hard dependency. Its component name depends on how it
    # was added: the repository's directory (EXTRA_COMPONENT_DIRS), or
    # "<namespace>__tesserapi" (component manager). Requirements are resolved
    # before the component list is known, so it is found here and linked
    # below rather than listed in REQUIRES.
    idf_build_get_property(TESSERMOTION_BUILD_COMPONENTS BUILD_COMPONENTS)
    set(TESSERMOTION_TESSERAPI "")
    foreach(c ${TESSERMOTION_BUILD_COMPONENTS})
        string(TOLOWER "${c}" c_lower)
        if(c_lower MATCHES "(^|__)tesserapi$")
            set(TESSERMOTION_TESSERAPI ${c})
        endif()
    endforeach()
    if(NOT TESSERMOTION_TESSERAPI AND NOT CMAKE_BUILD_EARLY_EXPANSION)
        message(FATAL_ERROR "TesserMotion needs TesserAPI (https://github.com/leokeba/TesserAPI): add it to "
                            "EXTRA_COMPONENT_DIRS or as a component manager dependency.")
    endif()
    # The drivers left "driver" in ESP-IDF 5.3.
    if(IDF_VERSION_MAJOR GREATER 5 OR (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 3))
        set(TESSERMOTION_I2C_REQ esp_driver_i2c)
        set(TESSERMOTION_DRIVER_REQS esp_driver_gpio esp_driver_ledc esp_driver_uart)
    else()
        set(TESSERMOTION_I2C_REQ driver)
        set(TESSERMOTION_DRIVER_REQS "")
    endif()
    idf_component_register(
        SRCS ${TESSERMOTION_CORE_SRCS}
             src/tessermotion/platform/esp/gpio_esp.cpp
             src/tessermotion/platform/esp/pwm_esp.cpp
             src/tessermotion/platform/esp/i2c_bus.cpp
             src/tessermotion/platform/esp/task_esp.cpp
             src/tessermotion/platform/esp/uart_port.cpp
        INCLUDE_DIRS "src"
        REQUIRES ${TESSERMOTION_I2C_REQ}  # i2c_bus.h includes driver/i2c_master.h
        PRIV_REQUIRES ${TESSERMOTION_DRIVER_REQS} esp_timer log)
    # Public: the trees' header includes TesserAPI's.
    if(TESSERMOTION_TESSERAPI)
        idf_component_get_property(TESSERMOTION_TESSERAPI_LIB ${TESSERMOTION_TESSERAPI} COMPONENT_LIB)
        target_link_libraries(${COMPONENT_LIB} PUBLIC ${TESSERMOTION_TESSERAPI_LIB})
    endif()
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(tessermotion CXX)

option(TESSERMOTION_BUILD_TESTS "Build host-side tests" ON)
option(TESSERMOTION_SANITIZE "Build tests with ASan/UBSan" ON)
set(TESSERAPI_DIR "" CACHE PATH "TesserAPI source tree (default: ../TesserAPI if present, else fetched from GitHub)")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# TesserAPI, for tesser_core (and ArduinoJson through it).
set(TESSER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
if(NOT TARGET tesser_core)
    if(NOT TESSERAPI_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../TesserAPI/CMakeLists.txt")
        get_filename_component(TESSERAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../TesserAPI" ABSOLUTE)
        set(TESSERAPI_DIR "${TESSERAPI_DIR}" CACHE PATH "" FORCE)
    endif()
    if(TESSERAPI_DIR)
        message(STATUS "TesserAPI: ${TESSERAPI_DIR}")
        add_subdirectory("${TESSERAPI_DIR}" "${CMAKE_BINARY_DIR}/tesserapi")
    else()
        message(STATUS "TesserAPI: fetching https://github.com/leokeba/TesserAPI.git")
        include(FetchContent)
        FetchContent_Declare(tesserapi
            GIT_REPOSITORY https://github.com/leokeba/TesserAPI.git
            GIT_TAG main
            GIT_SHALLOW ON)
        FetchContent_MakeAvailable(tesserapi)
    endif()
endif()

set(TESSERMOTION_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror -fno-exceptions -fno-rtti)

add_library(tessermotion_core ${TESSERMOTION_CORE_SRCS})
target_include_directories(tessermotion_core PUBLIC src)
target_link_libraries(tessermotion_core PUBLIC tesser_core)
set_target_properties(tessermotion_core PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_compile_options(tessermotion_core PRIVATE ${TESSERMOTION_WARNINGS})

# The fakes and the simulated plant (test/fake_hw.*, test/plant.*), for the
# host tests of TesserMotion and of applications built on it: link
# tessermotion_testing, include "fake_hw.h" and "plant.h".
#
# fake_hw.cpp defines the platform hooks tessermotion_core calls (platform.h),
# so its objects go into each executable rather than into the archive: GNU ld
# scans an archive once, and libtessermotion_testing.a comes before
# libtessermotion_core.a on the link line.
add_library(tessermotion_fake_hw OBJECT test/fake_hw.cpp)
target_include_directories(tessermotion_fake_hw PUBLIC test)
target_link_libraries(tessermotion_fake_hw PUBLIC tessermotion_core)
add_library(tessermotion_testing test/plant.cpp)
target_sources(tessermotion_testing INTERFACE $<TARGET_OBJECTS:tessermotion_fake_hw>)
target_include_directories(tessermotion_testing PUBLIC test)
target_link_libraries(tessermotion_testing PUBLIC tessermotion_core)
foreach(t tessermotion_fake_hw tessermotion_testing)
    set_target_properties(${t} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    target_compile_options(${t} PRIVATE ${TESSERMOTION_WARNINGS})
endforeach()

if(TESSERMOTION_BUILD_TESTS)
    enable_testing()
    add_executable(tessermotion_tests test/test_main.cpp)
    add_executable(tessermotion_fixtures test/fixtures_main.cpp)
    add_executable(tessermotion_link_check test/link_check_main.cpp)
    foreach(t tessermotion_tests tessermotion_fixtures tessermotion_link_check)
        target_link_libraries(${t} PRIVATE tessermotion_testing)
        set_target_properties(${t} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
        target_compile_options(${t} PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
    endforeach()
    if(TESSERMOTION_SANITIZE)
        # tesser_core too, so the whole binary is instrumented alike.
        foreach(t tesser_core tessermotion_core tessermotion_fake_hw tessermotion_testing tessermotion_tests
                  tessermotion_fixtures tessermotion_link_check)
            target_compile_options(${t} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
            target_link_options(${t} PRIVATE -fsanitize=address,undefined)
        endforeach()
    endif()
    add_test(NAME tessermotion_tests COMMAND tessermotion_tests)
    # Checks that the generator runs cleanly; the committed fixtures are
    # regenerated with `./build/tessermotion_fixtures fixtures`.
    add_test(NAME tessermotion_fixtures COMMAND tessermotion_fixtures ${CMAKE_CURRENT_BINARY_DIR}/fixtures)
    add_test(NAME tessermotion_link_check COMMAND tessermotion_link_check)
endif()
