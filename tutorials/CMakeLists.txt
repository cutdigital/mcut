cmake_minimum_required(VERSION 3.1)
project(mcut_tutorials)

list(APPEND preprocessor_defs -DEIGEN_WARNINGS_DISABLED)

#add_subdirectory(BasicGuiApp)
add_subdirectory(BasicCmdLineApp)
add_subdirectory(HelloWorld)
add_subdirectory(TextureCoordinates)
add_subdirectory(VertexNormals)
add_subdirectory(FaceNormals)
add_subdirectory(CSGBoolean)
add_subdirectory(AdjacentFaces)
