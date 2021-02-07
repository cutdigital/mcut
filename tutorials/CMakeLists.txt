cmake_minimum_required(VERSION 3.1)
project(mcut_tutorials)

include(DownloadProject.cmake)

if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

add_subdirectory(CmdLineTool)
add_subdirectory(HelloWorld)
add_subdirectory(TextureCoordinates)