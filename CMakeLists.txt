#
# You must provide the following variable:
#	MCUT_INCLUDE_DIR - the MCUT include directory
#
# You can also set the following option (optional):
#	 MCUT_BUILD_WITH_ARBITRARY_PRECISION_NUMBERS - enable exact arthmetic
#
# This script will define the following cache variables:
# 	MCUT_APPROX_MATH_SHARED_LIB_PATH - path to the MCUT shared library (machine precision)
#   MCUT_APPROX_MATH_STATIC_LIB_PATH - path to the MCUT static library (machine precision)
# 	MCUT_EXACT_SHARED_LIB_PATH - path to the MCUT shared library (exact precision)
#   MCUT_EXACT_STATIC_LIB_PATH - path to the MCUT static library (exact precision)

cmake_minimum_required(VERSION 3.12...3.13  )

set (HOMEPAGE_URL https://cutdigital.github.io/)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	FORCE)
endif()

project(mcut VERSION 1.0 LANGUAGES CXX C)

set ( DESCRIPTION "2-manifold surface-mesh cutting library." )

list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message(STATUS "System name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "MCUT version: ${mcut_VERSION}")

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

function(create_library_target LIBRARY_TARGET_NAME LIBRARY_TYPE ARBITRARY_PRECISION_NUMBERS_DEF)
    message(STATUS "create target: name=${LIBRARY_TARGET_NAME} type=${LIBRARY_TYPE}")
	
	add_library(${LIBRARY_TARGET_NAME} ${LIBRARY_TYPE} ${project_source_files})

	target_include_directories(${LIBRARY_TARGET_NAME} PRIVATE ${include_dirs})
	target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE ${extra_libs})
	target_compile_options(${LIBRARY_TARGET_NAME} PRIVATE ${compilation_flags})
	target_compile_definitions(${LIBRARY_TARGET_NAME} PRIVATE ${preprocessor_defs} ${ARBITRARY_PRECISION_NUMBERS_DEF})

	set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY VERSION ${project_build_version_string})
	set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY SOVERSION ${project_API_version_string})

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
create_library_target(${approx_math_shared_lib_name} SHARED "")
create_library_target(${approx_math_static_lib_name} STATIC "")

set (MCUT_APPROX_MATH_SHARED_LIB_PATH $<TARGET_FILE:${project_namespace_name}::${approx_math_shared_lib_name}>)
set (MCUT_APPROX_MATH_STATIC_LIB_PATH $<TARGET_FILE:${project_namespace_name}::${approx_math_static_lib_name}>)

#
# exact math libraries
#
create_library_target(${exact_math_shared_lib_name} SHARED ${exact_math_object_library} MCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1)
create_library_target(${exact_math_static_lib_name} STATIC ${exact_math_object_library} MCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1)

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
