

add_executable(CSGBoolean CSGBoolean.cpp)
target_link_libraries(CSGBoolean mcut)
target_include_directories(CSGBoolean PRIVATE ${MCUT_INCLUDE_DIR} ${libigl_include_dir} ${eigen_include_dir})#
target_compile_definitions(CSGBoolean PRIVATE -DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data" -DOUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")