cmake_minimum_required(VERSION 3.16)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(paw3395_ble_mouse)

# Specify source files and include directories
set(COMPONENT_SRCS "main/main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "." "..")
