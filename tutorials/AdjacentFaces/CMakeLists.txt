

add_executable(AdjacentFaces AdjacentFaces.cpp)

target_link_libraries(AdjacentFaces mcut)
target_include_directories(AdjacentFaces PRIVATE ${MCUT_INCLUDE_DIR} )
target_compile_definitions(AdjacentFaces PRIVATE -DOUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_compile_options(AdjacentFaces PRIVATE ${compilation_flags})
target_compile_definitions(AdjacentFaces PRIVATE ${preprocessor_defs})