#
# You must provide the following variable:
#	MCUT_INCLUDE_DIR - the MCUT include directory (mcut-headers)
#
# This script will define the following cache variables:
# 	MCUT_LIBRARY - the path to the MCUT library
#

cmake_minimum_required(VERSION 3.10)

project(mcut VERSION 1.0)

#
# User inputs
#
option (MCUT_ARBITRARY_PRECISION_NUMBERS "exact math" OFF)
option (MCUT_BUILD_AS_SHARED_LIB ".dll/.so" ON)

if(NOT DEFINED MCUT_INCLUDE_DIR)
	message(SEND_ERROR "MCUT_INCLUDE_DIR unspecified")
endif()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC" AND ${MCUT_BUILD_AS_SHARED_LIB})
	set ( CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set( CMAKE_CXX_STANDARD 11)
set ( CMAKE_CXX_STANDARD_REQUIRED True)

set ( CMAKE_EXPORT_COMPILE_COMMANDS ON)

set ( CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)
set ( CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)

set (src_files ${CMAKE_CURRENT_SOURCE_DIR}/src/mcut.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/halfedge_mesh.cpp)
set (library_name mcut)

if (MCUT_BUILD_AS_SHARED_LIB)
	add_library(${library_name} SHARED ${src_files})
else()
	add_library(${library_name} STATIC ${src_files})
endif()

target_include_directories(${library_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/incl ${MCUT_INCLUDE_DIR})

if (MCUT_BUILD_AS_SHARED_LIB)
	set_target_properties(${library_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MCUT_ARBITRARY_PRECISION_NUMBERS)
  target_compile_definitions(${library_name} PUBLIC -DARBITRARY_PRECISION_NUMBERS)
endif()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
	target_compile_options(${library_name} PUBLIC -Wall -Wextra)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
	target_compile_options(${library_name} PUBLIC -w3 -diag-disable:remark)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
	target_compile_options(${library_name} PUBLIC /W4)
	target_compile_definitions(${library_name} PUBLIC -D_CRT_SECURE_NO_WARNINGS)
endif()

set (MCUT_LIBRARY $<TARGET_FILE:${library_name}>)

message(STATUS "MCUT_INCLUDE_DIR=${MCUT_INCLUDE_DIR}")
message(STATUS "MCUT_LIBRARY=${MCUT_LIBRARY}")