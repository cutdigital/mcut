cmake_minimum_required(VERSION 3.1)

project(mcut_tutorials)

function(create_tutorial_target tutorial_name)
    add_executable(${tutorial_name} "${CMAKE_CURRENT_SOURCE_DIR}/${tutorial_name}/${tutorial_name}.cpp")
    target_link_libraries(${tutorial_name} mcut mio)
    target_include_directories(${tutorial_name} PRIVATE ${MCUT_INCLUDE_DIR}  ${mio_include_dir})
    target_compile_definitions(${tutorial_name} PRIVATE -DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/${tutorial_name}/data" -DOUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")
    target_compile_options(${tutorial_name} PRIVATE ${compilation_flags})
    target_compile_definitions(${tutorial_name} PRIVATE ${preprocessor_defs})
endfunction()

create_tutorial_target(AdjacentFaces)
create_tutorial_target(FaceNormals)

#add_subdirectory(BasicCmdLineApp)
#add_subdirectory(HelloWorld)
#add_subdirectory(TextureCoordinates)
#add_subdirectory(VertexNormals)
#add_subdirectory(CSGBoolean)
#add_subdirectory(Triangulation)
#add_subdirectory(TriangulatedFaceMaps)
#add_subdirectory(EventCallback)
#add_subdirectory(MultipleContextsInParallel)
#add_subdirectory(ReversedConnectedComponentFaces)
#add_subdirectory(PlanarSectioning)
#add_subdirectory(QuerySeamVerticesUnsorted)
#add_subdirectory(QuerySeamVerticesSorted)
#add_subdirectory(InOutQuery)


