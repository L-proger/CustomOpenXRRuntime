cmake_minimum_required(VERSION 3.16)

project(customxr C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_library(customxr SHARED)

target_sources(customxr
PRIVATE
   "Src/Main.cpp"
   "Src/DxgiUtils.h"
)

target_include_directories(customxr
PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../openxr_loader_windows-1.0.27/openxr_loader_windows/include
    ${CMAKE_CURRENT_LIST_DIR}/../OpenXR-SDK-Source/src/loader
    ${CMAKE_CURRENT_LIST_DIR}/../OpenXR-SDK-Source/src/common
)


target_compile_definitions(customxr
PRIVATE
    XR_OS_WINDOWS
    XR_USE_PLATFORM_WIN32
)

