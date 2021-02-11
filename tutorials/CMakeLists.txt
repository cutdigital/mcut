cmake_minimum_required(VERSION 3.1)
project(mcut_tutorials)

include(DownloadProject.cmake)

if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

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

#add_subdirectory(CmdLineTool)
add_subdirectory(HelloWorld)
add_subdirectory(TextureCoordinates)
add_subdirectory(VertexNormals)
add_subdirectory(FaceNormals)