#
# You must provide the following variable:
#	MCUT_INCLUDE_DIR - the MCUT include directory
#
# You can also set the following option (optional):
#	 MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS - enable exact arthmetic
#
# This script will define the following cache variables:
# 	MCUT_LIBRARY - the path to the MCUT library
#

cmake_minimum_required(VERSION 3.10)

# Set default build type to "Release".
# NOTE: this should be done before the project command since the latter can set
# CMAKE_BUILD_TYPE itself (it does so for nmake).
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	FORCE)
endif()

project(mcut VERSION 1.0 LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message(STATUS "System name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "MCUT version: ${mcut_VERSION}")

#
# User inputs
#
option (MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS "exact math" OFF)
option (MCUT_BUILD_AS_SHARED_LIB ".dll/.so" ON)

if(NOT DEFINED MCUT_INCLUDE_DIR)
	message(SEND_ERROR "MCUT_INCLUDE_DIR unspecified")
endif()

if(${MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS})
	
	if (MSVC)
		# MPIR (GMP)
		find_package(MCUT_MPIR REQUIRED)

		message(STATUS "MCUT_MPIR_INCLUDE_DIR=${MCUT_MPIR_INCLUDE_DIR}")
		message(STATUS "MCUT_MPIR_LIBRARY=${MCUT_MPIR_LIBRARY}")
	endif()

	# MPFR
	find_package(MCUT_MPFR REQUIRED)

	message(STATUS "MCUT_MPFR_INCLUDE_DIR=${MCUT_MPFR_INCLUDE_DIR}")
	message(STATUS "MCUT_MPFR_LIBRARY=${MCUT_MPFR_LIBRARY}")
endif()

if (MSVC AND ${MCUT_BUILD_AS_SHARED_LIB})
	set ( CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set( CMAKE_CXX_STANDARD 11)
set ( CMAKE_CXX_STANDARD_REQUIRED True)

set ( CMAKE_EXPORT_COMPILE_COMMANDS ON)

set ( CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)
set ( CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)

set (src_files 	${CMAKE_CURRENT_SOURCE_DIR}/src/mcut.cpp 
				${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp 
				${CMAKE_CURRENT_SOURCE_DIR}/src/halfedge_mesh.cpp
				${CMAKE_CURRENT_SOURCE_DIR}/src/math.cpp
				${CMAKE_CURRENT_SOURCE_DIR}/src/number.cpp)
set (library_name mcut)

if (MCUT_BUILD_AS_SHARED_LIB)
	add_library(${library_name} SHARED ${src_files})
	message(STATUS "${library_name} will be built as a shared library.")
	set_property(TARGET ${library_name} PROPERTY VERSION "1.0")
    set_property(TARGET ${library_name} PROPERTY SOVERSION 1)
else()
	add_library(${library_name} STATIC ${src_files})
	message(STATUS "${library_name} will be built as a static library.")
endif()

target_include_directories(${library_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/incl ${MCUT_INCLUDE_DIR})

if (MCUT_BUILD_AS_SHARED_LIB)
	set_target_properties(${library_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)
	target_include_directories(${library_name} PRIVATE ${MCUT_MPFR_INCLUDE_DIR} ${MCUT_MPIR_INCLUDE_DIR})
	target_compile_definitions(${library_name} PRIVATE -DMCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1)
	target_link_libraries(${library_name} PRIVATE ${MCUT_MPFR_LIBRARY} ${MCUT_MPIR_LIBRARY})
endif()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
	target_compile_options(${library_name} PRIVATE -Wall -Wextra)
	set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp PROPERTIES COMPILE_FLAGS "-Wno-unused-but-set-variable -Wno-unused-variable")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
	target_compile_options(${library_name} PRIVATE -w3 -diag-disable:remark)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
	target_compile_options(${library_name} PRIVATE /W4)
	set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp PROPERTIES COMPILE_FLAGS "/wd4100 /wd4189")
	target_compile_definitions(${library_name} PRIVATE -D_CRT_SECURE_NO_WARNINGS)
endif()

set (MCUT_LIBRARY $<TARGET_FILE:${library_name}>)

message(STATUS "MCUT_INCLUDE_DIR=${MCUT_INCLUDE_DIR}")
message(STATUS "MCUT_LIBRARY=${MCUT_LIBRARY}")
message(STATUS "MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS=${MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS}")
