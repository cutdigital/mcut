cmake_minimum_required(VERSION 3.10)

project(mcut VERSION 1.0)

option(MCUT_ARBITRARY_PRECISION_NUMBERS "exact math" OFF)
option(MCUT_BUILD_AS_SHARED_LIB ".dll/.so" ON)

set ( CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set ( CMAKE_CXX_STANDARD_REQUIRED True)
set ( CMAKE_EXPORT_COMPILE_COMMANDS ON)
set ( CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)
set ( CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)

set (src_files ${CMAKE_CURRENT_SOURCE_DIR}/src/mcut.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/halfedge_mesh.cpp)

if (MCUT_BUILD_AS_SHARED_LIB)
	add_library(mcut SHARED ${src_files})
else()
	add_library(mcut STATIC ${src_files})
endif()

target_include_directories(mcut PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/incl ${CMAKE_CURRENT_SOURCE_DIR}/extern/mcut-headers)

if (MCUT_BUILD_AS_SHARED_LIB)
	set_target_properties(mcut PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MCUT_ARBITRARY_PRECISION_NUMBERS)
  target_compile_definitions(mcut PUBLIC -DARBITRARY_PRECISION_NUMBERS)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_compile_options(mcut PUBLIC -Wall -Wextra)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
	target_compile_options(mcut PUBLIC -w3 -diag-disable:remark)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	target_compile_options(mcut PUBLIC /W4)
	target_compile_definitions(mcut PUBLIC -D_CRT_SECURE_NO_WARNINGS)
endif()

