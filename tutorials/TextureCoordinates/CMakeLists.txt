download_project(PROJ                libigl
                 GIT_REPOSITORY      https://github.com/libigl/libigl.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

set(libigl_include_dir ${CMAKE_BINARY_DIR}/libigl-src/include)

set(LIBIGL_EIGEN_VERSION 3.3.7 CACHE STRING "Default version of Eigen used by libigl.")

download_project(PROJ                eigen
                 GIT_REPOSITORY      https://gitlab.com/libeigen/eigen.git
                 GIT_TAG             ${LIBIGL_EIGEN_VERSION}
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

set(eigen_include_dir ${CMAKE_BINARY_DIR}/eigen-src)

add_executable(TextureCoordinates TextureCoordinates.cpp)
target_link_libraries(TextureCoordinates mcut)
target_include_directories(TextureCoordinates PRIVATE ${MCUT_INCLUDE_DIR} ${libigl_include_dir} ${eigen_include_dir})#
target_compile_definitions(TextureCoordinates PRIVATE -DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data" -DOUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")