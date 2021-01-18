download_project(PROJ                libigl
                 GIT_REPOSITORY      https://github.com/libigl/libigl.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

set(libigl_include_dir ${CMAKE_BINARY_DIR}/libigl-src/include)

add_executable(TextureCoordinates TextureCoordinates.cpp)
target_link_libraries(TextureCoordinates mcut)
target_include_directories(TextureCoordinates PRIVATE ${MCUT_INCLUDE_DIR} ${libigl_include_dir})