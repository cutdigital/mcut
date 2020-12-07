cmake_minimum_required(VERSION 3.10)

project(mcut_tests VERSION 1.0)

### Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)

if(EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "[MCUT] You cannot build in a source directory.")
endif()

enable_testing()
include(GoogleTest)

# https://github.com/google/googletest/blob/master/googletest/README.md
# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
	RESULT_VARIABLE result
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )

if(result)
	message(FATAL_ERROR "[MCUT] CMake step for googletest failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
	RESULT_VARIABLE result
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )

if(result)
	message(FATAL_ERROR "[MCUT] Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
					${CMAKE_CURRENT_BINARY_DIR}/googletest-build
					EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
	include_directories("${gtest_SOURCE_DIR}/include")
endif()

# keeps CACHE cleaner
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# mini off file loader
add_library(offLoad STATIC ${CMAKE_CURRENT_SOURCE_DIR}/off.cpp)

# Now simply link against gtest or gtest_main as needed. Eg

macro(package_add_test TESTNAME test_source_files use_exact_lib)
    message(STATUS "[MCUT] create target: ${TESTNAME}")
    # create an exectuable in which the tests will be stored
    add_executable(${TESTNAME} ${test_source_files})
    target_include_directories(${TESTNAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${MCUT_INCLUDE_DIR}")
    
    # link the Google test infrastructure, mocking library, and a default main function to
    # the test executable.  Remove g_test_main if writing your own main function.
    list(APPEND linked_libs gtest gmock gtest_main offLoad)

    if(use_exact_lib)
        list(APPEND linked_libs ${apn_static_lib_name})
    else()
        list(APPEND linked_libs ${mpn_static_lib_name})
    endif()

    target_link_libraries(${TESTNAME} ${linked_libs})

    # gtest_discover_tests replaces gtest_add_tests,
    # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it
    gtest_discover_tests(${TESTNAME}
        # set a working directory so your project root so that you can find test data via paths relative to the project root
        WORKING_DIRECTORY ${PROJECT_DIR}
        PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
    )

    target_compile_definitions(${TESTNAME} PUBLIC -DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

    if(use_exact_lib)
        target_compile_definitions(${TESTNAME} PRIVATE -DMCUT_WITH_ARBITRARY_PRECISION_NUMBERS=1)
    endif()

endmacro()

if(MCUT_BUILD_ARBITRARY_PRECISION_NUMBERS_LIB)
    package_add_test(api-tests-apn ${CMAKE_CURRENT_SOURCE_DIR}/api-tests.cpp TRUE)
    package_add_test(cutting-tests-apn ${CMAKE_CURRENT_SOURCE_DIR}/cutting-tests.cpp TRUE) 
else()
    package_add_test(api-tests ${CMAKE_CURRENT_SOURCE_DIR}/api-tests.cpp FALSE)
    package_add_test(cutting-tests ${CMAKE_CURRENT_SOURCE_DIR}/cutting-tests.cpp FALSE) 
endif()



