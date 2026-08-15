cmake_minimum_required(VERSION 3.16)

set(COMPONENT_SRCS "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ".")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(paw3395_ble_mouse)
