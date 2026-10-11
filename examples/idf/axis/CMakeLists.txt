cmake_minimum_required(VERSION 3.16)
# TesserMotion is this repository; TesserAPI, TesserUI and TesserKIT (for
# Wi-Fi and the web interface) are found through TESSERAPI_PATH,
# TESSERUI_PATH and TESSERKIT_PATH, by default checkouts next to it.
get_filename_component(TESSERMOTION_PATH "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
foreach(dep TesserAPI TesserUI TesserKIT)
    string(TOUPPER ${dep} DEP)
    if(DEFINED ENV{${DEP}_PATH})
        get_filename_component(${DEP}_PATH "$ENV{${DEP}_PATH}" ABSOLUTE)
    else()
        get_filename_component(${DEP}_PATH "${TESSERMOTION_PATH}/../${dep}" ABSOLUTE)
    endif()
    if(NOT EXISTS "${${DEP}_PATH}/CMakeLists.txt")
        message(FATAL_ERROR "${dep} not found at ${${DEP}_PATH}: clone it there, or set ${DEP}_PATH")
    endif()
endforeach()
set(EXTRA_COMPONENT_DIRS "${TESSERMOTION_PATH}" "${TESSERAPI_PATH}" "${TESSERUI_PATH}" "${TESSERKIT_PATH}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tessermotion_axis)
