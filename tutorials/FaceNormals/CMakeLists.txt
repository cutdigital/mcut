

add_executable(FaceNormals FaceNormals.cpp)
target_link_libraries(FaceNormals mcut)
target_include_directories(FaceNormals PRIVATE ${MCUT_INCLUDE_DIR} ${libigl_include_dir} ${eigen_include_dir})#
target_compile_definitions(FaceNormals PRIVATE -DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data" -DOUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")