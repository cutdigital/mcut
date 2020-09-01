cmake_minimum_required(VERSION 3.10)

project(mcut VERSION 1.0)

option(MCUT_USE_NATIVE_FLOATING_POINT_NUMBERS "Exact geometric math" ON)
option(MCUT_BUILD_AS_SHARED_LIB ".dll/.so" ON)

### Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)

if(EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "You cannot build in a source directory.")
endif()

set ( CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set ( CMAKE_CXX_STANDARD_REQUIRED True)
set ( CMAKE_EXPORT_COMPILE_COMMANDS ON)
set ( CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)
set ( CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/bin)
set ( mcut_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/incl)
set ( mcut_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)

#
# Compiler settings
#
if(MSVC)
	#if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
	#	string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	#else()
	#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
	#endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

set (src_files ${mcut_src_dir}/mcut.cpp ${mcut_src_dir}/kernel.cpp ${mcut_src_dir}/halfedge_mesh.cpp)

if (MCUT_BUILD_AS_SHARED_LIB)
	add_library(mcut SHARED ${src_files})
else()
	add_library(mcut STATIC ${src_files})
endif()

target_include_directories(mcut PUBLIC ${mcut_include_dir})
#target_link_libraries(mcut ${EXTRA_LIBS})

if (MCUT_BUILD_AS_SHARED_LIB)
	set_target_properties(mcut PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MCUT_USE_NATIVE_FLOATING_POINT_NUMBERS)
  target_compile_definitions(mcut PUBLIC -DUSE_NATIVE_FLOATING_POINT_NUMBERS)
endif()

target_compile_definitions(mcut PUBLIC -D_CRT_SECURE_NO_WARNINGS)


