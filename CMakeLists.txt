# Copyright (c) 2020-2021 CutDigital Ltd.
# All rights reserved.
# 
# NOTE: This file is licensed under GPL-3.0-or-later (default). 
# A commercial license can be purchased from CutDigital Ltd. 
#  
# License details:
# 
# (A)  GNU General Public License ("GPL"); a copy of which you should have 
#      recieved with this file.
# 	    - see also: <http://www.gnu.org/licenses/>
# (B)  Commercial license.
#      - email: contact@cut-digital.com
# 
# The commercial license options is for users that wish to use MCUT in 
# their products for comercial purposes but do not wish to release their 
# software products under the GPL license. 
# 
# Author(s)     : Floyd M. Chitalu
#
############################################################################################
#
# You can configure MCUT with the following CMake options:
#
#	MCUT_BUILD_AS_SHARED_LIB - Build MCUT as a shared/dynamic library (.so/.dll).
#	MCUT_BUILD_THE_DOCS - Build documentation (explicit dependancy on Doxygen)
#	MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS - Build using arbitrary precision numbers (explicit dependancy on MPFR).
#	MCUT_BUILD_TESTS - Build the tests (implicit dependancy on GoogleTest)
#	MCUT_BUILD_TUTORIALS - Build tutorials
#
# This script will define the following cache variables:
#
#	MCUT_INCLUDE_DIR - the MCUT include directory
# 	MCUT_SHARED_LIB_PATH - path to the MCUT shared library 

cmake_minimum_required(VERSION 3.12...3.13 FATAL_ERROR)

project(mcut LANGUAGES CXX C)

set ( DESCRIPTION "2-manifold surface-mesh cutting library." )

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose build type (None Debug Release RelWithDebInfo MinSizeRel)." FORCE)
endif()

# ... location of mpfr and mpir (gmp) search modules
list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(MCUT_MAJOR 1)
set(MCUT_MINOR 0)
set(MCUT_PATCH 0)

set( MCUT_VERSION "${MCUT_MAJOR}.${MCUT_MINOR}.${MCUT_PATCH}" )

message(STATUS "[MCUT] version: ${MCUT_VERSION}")

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (project_API_version_string "${MCUT_VERSION}")
set (project_build_version_string "${MCUT_VERSION}")
set (project_namespace_name MCUT)

#
# User options
#
option(MCUT_BUILD_THE_DOCS "Configure to build docs with Doxygen" OFF) # OFF by default
option(MCUT_BUILD_TESTS "Configure to build tests with GoogleTest" OFF)
option(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS "Build the arbitrary precision library as well" OFF) # OFF by default
option(MCUT_BUILD_AS_SHARED_LIB "Configure to build MCUT as a shared/dynamic library" OFF)
option(MCUT_BUILD_TUTORIALS "Configure to build MCUT tutorials" OFF)

#
# machine-precision-numbers library targets
#
set (target_name mcut)

#
# arbitrary-precision-numbers library target & dependencies
#
if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)

	#
	# find MPFR & MPIR
	#
	if (MSVC)
		message(STATUS "[MCUT] searching for MPIR in system environment")
		find_package(MCUT_MPIR REQUIRED) # GMP
	endif()

	find_package(MCUT_MPFR REQUIRED)

	list(APPEND include_dirs ${MCUT_MPFR_INCLUDE_DIR} ${MCUT_MPIR_INCLUDE_DIR}) 
	list(APPEND extra_libs ${MCUT_MPFR_LIBRARY} ${MCUT_MPIR_LIBRARY}) 
endif() # if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)

#
# MCUT compilation variables/settings
#
set (MCUT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

message(STATUS "[MCUT] MCUT_INCLUDE_DIR=${MCUT_INCLUDE_DIR}")

list(APPEND include_dirs  ${MCUT_INCLUDE_DIR})
list(APPEND compilation_flags "")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	list(APPEND compilation_flags -Wall -Wextra)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	list(APPEND compilation_flags w3 -diag-disable:remark)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	list(APPEND compilation_flags /W4)
	list(APPEND preprocessor_defs _CRT_SECURE_NO_WARNINGS)
endif()

message(STATUS "[MCUT] compilation_flags=${compilation_flags}")
message(STATUS "[MCUT] preprocessor_defs=${preprocessor_defs}")

# disable redandant warnings for kernel.cpp
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	set(kernel_source_file_flags "-Wno-unused-but-set-variable -Wno-unused-variable")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	# pass
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	set(kernel_source_file_flags "/wd4100 /wd4189")
endif()

set_source_files_properties(
	SOURCE 
	${CMAKE_CURRENT_SOURCE_DIR}/source/kernel.cpp 
	PROPERTY COMPILE_FLAGS ${kernel_source_file_flags})

set (	project_source_files  
		${CMAKE_CURRENT_SOURCE_DIR}/source/mcut.cpp 
		${CMAKE_CURRENT_SOURCE_DIR}/source/kernel.cpp 
		${CMAKE_CURRENT_SOURCE_DIR}/source/halfedge_mesh.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/source/math.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/source/geom.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/source/number.cpp)

#
# Create MCUT target(s)
#

# This function invokes commands which create a library target (static, shared etc.)
function(create_library_target LIBRARY_TYPE ARBITRARY_PRECISION_NUMBERS_DEF)
    message(STATUS "[MCUT] create target: name=${target_name} type=${LIBRARY_TYPE}")
	
	add_library(${target_name} ${LIBRARY_TYPE} ${project_source_files})

	target_include_directories(${target_name} PRIVATE ${include_dirs})
	target_link_libraries(${target_name} PRIVATE ${extra_libs})
	target_compile_options(${target_name} PRIVATE ${compilation_flags})
	target_compile_definitions(${target_name} PRIVATE ${preprocessor_defs} ${ARBITRARY_PRECISION_NUMBERS_DEF})

	set_property(TARGET ${target_name} PROPERTY VERSION ${project_build_version_string})
	set_property(TARGET ${target_name} PROPERTY SOVERSION ${project_API_version_string})

	get_target_property(target_type ${target_name} TYPE)

	if ("${target_type}" STREQUAL "SHARED")
		set_property(TARGET ${target_name} PROPERTY POSITION_INDEPENDENT_CODE ON)
		if(MSVC)
			set_property(TARGET ${target_name} PROPERTY WINDOWS_EXPORT_ALL_SYMBOLS ON)
		endif()
	endif()
endfunction()

if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)
	#
	# arbitrary-precision numbers builds
	#
	if(MCUT_BUILD_AS_SHARED_LIB)
		create_library_target(SHARED MCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1 )
	else()
		create_library_target(STATIC MCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1)
	endif()

	message(STATUS "[MCUT] MCUT_MPFR_INCLUDE_DIR=${MCUT_MPFR_INCLUDE_DIR}")
	message(STATUS "[MCUT] MCUT_MPFR_LIBRARY=${MCUT_MPFR_LIBRARY}")
	message(STATUS "[MCUT] MCUT_MPIR_INCLUDE_DIR=${MCUT_MPIR_INCLUDE_DIR}")
	message(STATUS "[MCUT] MCUT_MPIR_LIBRARY=${MCUT_MPIR_LIBRARY}")
else()
	#
	# machine-precision numbers builds
	#
	if(MCUT_BUILD_AS_SHARED_LIB)
		create_library_target(SHARED "") 
	else()
		create_library_target(STATIC "")
	endif()
endif() # if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)

set (MCUT_LIB_PATH $<TARGET_FILE:mcut>)
message(STATUS "[MCUT] MCUT_LIB_PATH=${MCUT_LIB_PATH}")
#
# tests
#
if(MCUT_BUILD_TESTS)
	add_subdirectory(tests)
endif()

#
# documentation
#
if(MCUT_BUILD_THE_DOCS)
	add_subdirectory(docs)
endif()

#
# Tutorials
#
if(MCUT_BUILD_TUTORIALS)
	add_subdirectory(tutorials)
endif()

########################################################
###                   PACKAGING                      ###
### This is a quite INCOMPLETE set of variables that ###
### should be set for the various generators.        ###
### Consult the CPack documentations for a full set. ###
########################################################

# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cpack/Component-Install-With-CPack
# https://stackoverflow.com/questions/6003374/what-is-cmake-equivalent-of-configure-prefix-dir-make-all-install

# TODO: package documentation files

if(MCUT_BUILD_AS_SHARED_LIB)
	#
	# dynamic libs
	#
	if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)
		install(TARGETS ${apn_shared_lib_name}
				LIBRARY
				DESTINATION lib/apn/shared
				COMPONENT dynamic_libraries)
	else()
		install(TARGETS ${mpn_shared_lib_name}
				LIBRARY
				DESTINATION lib/mpn/shared
				COMPONENT dynamic_libraries)
	endif() # if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)
else()
	#
	# static libs
	#
	if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)
		install(TARGETS ${apn_static_lib_name} 
				ARCHIVE
				DESTINATION lib/apn/static
				COMPONENT static_libraries)
	else()
		install(TARGETS ${mpn_static_lib_name}
				ARCHIVE
				DESTINATION lib/mpn/static
				COMPONENT static_libraries)
	endif() # if(MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS)
endif()

#
# headers
#
install(FILES ${MCUT_INCLUDE_DIR}/mcut/mcut.h ${MCUT_INCLUDE_DIR}/mcut/platform.h
  		DESTINATION include/mcut
	  	COMPONENT headers)
	  
install(FILES 
		${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt
		${CMAKE_CURRENT_SOURCE_DIR}/README.md
		DESTINATION ./
		COMPONENT text_files)

#
# notify CPack of the names of all of the components in the project
#
set(CPACK_COMPONENTS_ALL static_libraries dynamic_libraries headers text_files) # applications

set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME "MCUT Application")
set(CPACK_COMPONENT_STATIC_LIBRARIES_DISPLAY_NAME "Static Libraries")
set(CPACK_COMPONENT_DYNAMIC_LIBRARIES_DISPLAY_NAME "Dynamics Libraries")
set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers")

set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION
  "A simple application using MCUT")
set(CPACK_COMPONENT_STATIC_LIBRARIES_DESCRIPTION
  "Static libraries used to build programs with MCUT")
  set(CPACK_COMPONENT_DYNAMIC_LIBRARIES_DESCRIPTION
  "Dynamic libraries used to build programs with MCUT")
set(CPACK_COMPONENT_HEADERS_DESCRIPTION
  "C/C++ header files for use with MCUT")

#  
# component dependencies
#
set(CPACK_COMPONENT_HEADERS_DEPENDS static_libraries dynamic_libraries)

set(CPACK_COMPONENT_APPLICATIONS_GROUP "Runtime")
set(CPACK_COMPONENT_STATIC_LIBRARIES_GROUP "Development")
set(CPACK_COMPONENT_DYNAMIC_LIBRARIES_GROUP "Development")
set(CPACK_COMPONENT_HEADERS_GROUP "Development")

set(CPACK_COMPONENT_GROUP_DEVELOPMENT_DESCRIPTION
  "All of the tools you'll ever need to develop software")

set (CPACK_PACKAGE_NAME "MCUT")
set (CPACK_PACKAGE_VENDOR "CutDigital Software Ltd")
set (CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set (CPACK_PACKAGE_VERSION_MAJOR "${MCUT_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${MCUT_MINOR}")
set (CPACK_PACKAGE_VERSION_PATCH "${MCUT_PATCH}")
#set (CPACK_PACKAGE_DESCRIPTION "MCUT (pronounced ‘emcut’) is a tool for cutting meshes.")
#set (CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/DESCRIPTION.txt)
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "MCUT (pronounced ‘emcut’) is a tool for cutting meshes.")
set (CPACK_PACKAGE_HOMEPAGE_URL "https://mcut.github.io/")
set (CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
# set (CPACK_PACKAGE_ICON ) 
set (CPACK_PACKAGE_CHECKSUM SHA256)
#set (CPACK_PROJECT_CONFIG_FILE ) 
set (CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt) # must also include in install command
set (CPACK_RESOURCE_FILE_README ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
#set (CPACK_RESOURCE_FILE_WELCOME ${CMAKE_CURRENT_SOURCE_DIR}/WELCOME.txt)

if (WIN32)
    if (USE_WIX_TOOLSET)
        set(CPACK_GENERATOR "WIX") # this need WiX Tooset installed and a path to candle.exe
    else ()
        set(CPACK_GENERATOR "NSIS") # this needs NSIS installed, and available
    endif ()
elseif ( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
    set(CPACK_GENERATOR "PackageMake")
else ()
    set(CPACK_GENERATOR "TGZ")
endif ()

#set (CPACK_OUTPUT_CONFIG_FILE ) # Defaults to CPackConfig.cmake.
#set (CPACK_PACKAGE_EXECUTABLES ) 
set (CPACK_STRIP_FILES TRUE)
# set (CPACK_VERBATIM_VARIABLES )
# set (CPACK_SOURCE_PACKAGE_FILE_NAME )
# set (CPACK_SOURCE_STRIP_FILES )
# set (CPACK_SOURCE_GENERATOR )
# set (CPACK_SOURCE_OUTPUT_CONFIG_FILE )
# set (CPACK_SOURCE_IGNORE_FILES )
# set (CPACK_CMAKE_GENERATOR )
# set (CPACK_INSTALL_CMAKE_PROJECTS ) 
# set (CPACK_INSTALL_CMAKE_PROJECTS )
# set (CPACK_SYSTEM_NAME )
# set (CPACK_PACKAGE_VERSION )
# set (CPACK_TOPLEVEL_TAG )
# set (CPACK_INSTALL_COMMANDS )
# set (CPACK_INSTALLED_DIRECTORIES )
# set ( )


include(CPack)

# eof