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
create_tutorial_target(VertexNormals)
create_tutorial_target(TextureCoordinates) # untested
create_tutorial_target(BasicCmdLineApp) 
create_tutorial_target(HelloWorld)
create_tutorial_target(CSGBoolean)
create_tutorial_target(Triangulation)
create_tutorial_target(TriangulatedFaceMaps)
create_tutorial_target(EventCallback) 
create_tutorial_target(ReversedConnectedComponentFaces)
create_tutorial_target(QuerySeamVerticesUnsorted)
create_tutorial_target(QuerySeamVerticesSorted)
create_tutorial_target(MultipleContextsInParallel)
create_tutorial_target(PlanarSectioning)
create_tutorial_target(InOutQuery)

#
# TODO
#
#add_subdirectory(ConnectedComponentProperties)
#add_subdirectory(EnableDebugMode)


