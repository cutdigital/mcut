#
# You must provide the following variable:
#	MCUT_INCLUDE_DIR - the MCUT include directory
#
# This script will define the following cache variables:
# 	MCUT_APPROX_MATH_SHARED_LIB_PATH - path to the MCUT shared library (machine precision)
#   MCUT_APPROX_MATH_STATIC_LIB_PATH - path to the MCUT static library (machine precision)
# 	MCUT_EXACT_SHARED_LIB_PATH - path to the MCUT shared library (exact precision)
#   MCUT_EXACT_STATIC_LIB_PATH - path to the MCUT static library (exact precision)

cmake_minimum_required(VERSION 3.12...3.13 FATAL_ERROR)

project(mcut LANGUAGES CXX C)

set ( DESCRIPTION "2-manifold surface-mesh cutting library." )

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	FORCE)
endif()

list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ### NOTE: *** Adjust VERSION when required ***
# read 'version' file into a variable (stripping any newlines or spaces)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/VERSION.txt versionFile)

if (NOT versionFile)
    message(FATAL_ERROR "Unable to determine version. RELEASE_VERSON file is missing.")
endif()

string(STRIP "${versionFile}" VERSION_STRING)
string(REPLACE "." ";" VERSION_LIST ${VERSION_STRING})

list (GET VERSION_LIST 0 MCUT_MAJOR)
list (GET VERSION_LIST 1 MCUT_MINOR)
list (GET VERSION_LIST 2 MCUT_PATCH)

set( MCUT_VERSION "${MCUT_MAJOR}.${MCUT_MINOR}.${MCUT_PATCH}" )

message(STATUS "System name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "MCUT version: ${MCUT_VERSION}")

if(NOT DEFINED MCUT_INCLUDE_DIR)
	message(SEND_ERROR "MCUT_INCLUDE_DIR unspecified")
endif()

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (project_API_version_string 1)
set (project_build_version_string "1.0")
set (project_namespace_name MCUT)
set (approx_math_shared_lib_name mcut_shared_approx)
set (approx_math_static_lib_name mcut_static_approx)
set (exact_math_shared_lib_name mcut_shared_exact)
set (exact_math_static_lib_name mcut_static_exact)

list(APPEND include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/incl ${MCUT_INCLUDE_DIR})

#
# MPFR
#
if (MSVC)
	find_package(MCUT_MPIR REQUIRED) # GMP
endif()

find_package(MCUT_MPFR REQUIRED)

list(APPEND include_dirs ${MCUT_MPFR_INCLUDE_DIR} ${MCUT_MPIR_INCLUDE_DIR})
list(APPEND extra_libs ${MCUT_MPFR_LIBRARY} ${MCUT_MPIR_LIBRARY})
list(APPEND compilation_flags ${MCUT_MPFR_LIBRARY} ${MCUT_MPIR_LIBRARY})

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
	list(APPEND compilation_flags -Wall -Wextra)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
	list(APPEND compilation_flags w3 -diag-disable:remark)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
	list(APPEND compilation_flags /W4)
	list(APPEND preprocessor_defs _CRT_SECURE_NO_WARNINGS)
endif()

#
# disable redandant warnings for kernel.cpp
#
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
	set(kernel_dot_cpp_flags "-Wno-unused-but-set-variable -Wno-unused-variable")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
	# pass
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
	set(kernel_dot_cpp_flags "/wd4100 /wd4189")
endif()

set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp PROPERTIES COMPILE_FLAGS ${kernel_dot_cpp_flags})

set (	project_source_files  
		${CMAKE_CURRENT_SOURCE_DIR}/src/mcut.cpp 
		${CMAKE_CURRENT_SOURCE_DIR}/src/kernel.cpp 
		${CMAKE_CURRENT_SOURCE_DIR}/src/halfedge_mesh.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/math.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/number.cpp)

function(create_library_target LIBRARY_TARGET_NAME LIBRARY_TYPE ARBITRARY_PRECISION_NUMBERS_DEF OUTPUT_FILENAME)
    message(STATUS "create target: name=${LIBRARY_TARGET_NAME} type=${LIBRARY_TYPE}")
	
	add_library(${LIBRARY_TARGET_NAME} ${LIBRARY_TYPE} ${project_source_files})

	target_include_directories(${LIBRARY_TARGET_NAME} PRIVATE ${include_dirs})
	target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE ${extra_libs})
	target_compile_options(${LIBRARY_TARGET_NAME} PRIVATE ${compilation_flags})
	target_compile_definitions(${LIBRARY_TARGET_NAME} PRIVATE ${preprocessor_defs} ${ARBITRARY_PRECISION_NUMBERS_DEF})

	set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY VERSION ${project_build_version_string})
	set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY SOVERSION ${project_API_version_string})
	set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY OUTPUT_NAME ${OUTPUT_FILENAME})

	get_target_property(target_type ${LIBRARY_TARGET_NAME} TYPE)

	if (target_type STREQUAL "SHARED")
		set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
		if(MSVC)
			set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY WINDOWS_EXPORT_ALL_SYMBOLS ON)
		endif()
	endif()
endfunction()

#
# approximate math libraries
#
create_library_target(${approx_math_shared_lib_name} SHARED "" mcut-fp) # fixed precision
create_library_target(${approx_math_static_lib_name} STATIC "" mcut-fp)

set (MCUT_APPROX_MATH_SHARED_LIB_PATH $<TARGET_FILE:${project_namespace_name}::${approx_math_shared_lib_name}>)
set (MCUT_APPROX_MATH_STATIC_LIB_PATH $<TARGET_FILE:${project_namespace_name}::${approx_math_static_lib_name}>)

#
# exact math libraries
#
create_library_target(${exact_math_shared_lib_name} SHARED ${exact_math_object_library} MCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1 mcut)
create_library_target(${exact_math_static_lib_name} STATIC ${exact_math_object_library} MCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1 mcut)

set (MCUT_EXACT_MATH_SHARED_LIB_PATH $<TARGET_FILE:${project_namespace_name}::${exact_math_shared_lib_name}>)
set (MCUT_EXACT_MATH_STATIC_LIB_PATH $<TARGET_FILE:${project_namespace_name}::${exact_math_static_lib_name}>)

message(STATUS "MCUT_INCLUDE_DIR=${MCUT_INCLUDE_DIR}")
message(STATUS "MCUT_APPROX_MATH_SHARED_LIB_PATH=${MCUT_APPROX_MATH_SHARED_LIB_PATH}")
message(STATUS "MCUT_APPROX_MATH_STATIC_LIB_PATH=${MCUT_APPROX_MATH_STATIC_LIB_PATH}")
message(STATUS "MCUT_EXACT_MATH_SHARED_LIB_PATH=${MCUT_EXACT_MATH_SHARED_LIB_PATH}")
message(STATUS "MCUT_EXACT_MATH_STATIC_LIB_PATH=${MCUT_EXACT_MATH_STATIC_LIB_PATH}")
message(STATUS "MCUT_MPFR_INCLUDE_DIR=${MCUT_MPFR_INCLUDE_DIR}")
message(STATUS "MCUT_MPFR_LIBRARY=${MCUT_MPFR_LIBRARY}")
message(STATUS "MCUT_MPIR_INCLUDE_DIR=${MCUT_MPIR_INCLUDE_DIR}")
message(STATUS "MCUT_MPIR_LIBRARY=${MCUT_MPIR_LIBRARY}")


########################################################
###                   PACKAGING                      ###
### This is a quite INCOMPLETE set of variables that ###
### should be set for the various generators.        ###
### Consult the CPack documentations for a full set. ###
########################################################

# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cpack/Component-Install-With-CPack
# https://stackoverflow.com/questions/6003374/what-is-cmake-equivalent-of-configure-prefix-dir-make-all-install

# static libs

install(TARGETS ${exact_math_static_lib_name} 
		ARCHIVE
		DESTINATION lib/exact/static
		COMPONENT static_libraries)

install(TARGETS ${approx_math_static_lib_name}
		ARCHIVE
		DESTINATION lib/fp/static
		COMPONENT static_libraries)

# dynamic libs

install(TARGETS ${exact_math_shared_lib_name}
		LIBRARY
		DESTINATION lib/exact/shared
		COMPONENT dynamic_libraries)

install(TARGETS ${approx_math_shared_lib_name}
		LIBRARY
		DESTINATION lib/fp/shared
		COMPONENT dynamic_libraries)

# headers

install(FILES ${MCUT_INCLUDE_DIR}/mcut/mcut.h ${MCUT_INCLUDE_DIR}/mcut/platform.h
  		DESTINATION include/mcut
	  	COMPONENT headers)
	  
install(FILES 
		${CMAKE_CURRENT_SOURCE_DIR}/DESCRIPTION.txt 
		${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt
		${CMAKE_CURRENT_SOURCE_DIR}/README.txt
		${CMAKE_CURRENT_SOURCE_DIR}/VERSION.txt
		${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.txt
		DESTINATION ./
		COMPONENT text_files)

# notify CPack of the names of all of the components in the project
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

# component dependencies
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
set (CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/DESCRIPTION.txt)
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "MCUT (pronounced ‘emcut’) is a tool for cutting meshes.")
set (CPACK_PACKAGE_HOMEPAGE_URL "https://cutdigital.github.io/")
set (CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
# set (CPACK_PACKAGE_ICON ) 
set (CPACK_PACKAGE_CHECKSUM SHA256)
#set (CPACK_PROJECT_CONFIG_FILE ) 
set (CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt) # must also include in install command
set (CPACK_RESOURCE_FILE_README ${CMAKE_CURRENT_SOURCE_DIR}/README.txt)
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