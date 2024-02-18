/***************************************************************************
 *  This file is part of the MCUT project, which is comprised of a library 
 *  for surface mesh cutting, example programs and test programs.
 * 
 *  Copyright (C) 2024 CutDigital Enterprise Ltd
 *  
 *  MCUT is dual-licensed software that is available under an Open Source 
 *  license as well as a commercial license. The Open Source license is the 
 *  GNU Lesser General Public License v3+ (LGPL). The commercial license 
 *  option is for users that wish to use MCUT in their products for commercial 
 *  purposes but do not wish to release their software under the LGPL. 
 *  Email <contact@cut-digital.com> for further information.
 *
 *  You may not use this file except in compliance with the License. A copy of 
 *  the Open Source license can be obtained from
 *
 *      https://www.gnu.org/licenses/lgpl-3.0.en.html.
 *
 *  For your convenience, a copy of this License has been included in this
 *  repository.
 *
 *  MCUT is distributed in the hope that it will be useful, but THE SOFTWARE IS 
 *  PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR 
 *  A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF 
 *  OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  MultipleContextsInParallel.cpp
 *
 *  \brief:
 *  This tutorial shows how to setup MCUT to run with multiple contexts in 
 *  parallel. This allows users to dispatch multiple cutting tasks in parallel.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"
#include "mio/mio.h"

#include <chrono>
#include <cstring>
#include <inttypes.h> // PRId64
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

void MCAPI_PTR mcDebugOutputCALLBACK(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

struct Timer {
    std::string m_name;
    std::chrono::time_point<std::chrono::steady_clock> m_startpoint;
    Timer(std::string name)
        : m_name(name)
        , m_startpoint(std::chrono::steady_clock::now())
    {
    }

    ~Timer()
    {
        const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        const std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startpoint);
        unsigned long long elapsed_ = elapsed.count();
        printf(">> Task %s [%llums]\n", m_name.c_str(), elapsed_);
        fflush(stdout);
    }
};

int main()
{
    Timer scoped_timer_main_fn("main-function");

    MioMesh srcMesh = {
		nullptr, // pVertices
		nullptr, // pNormals
		nullptr, // pTexCoords
		nullptr, // pFaceSizes
		nullptr, // pFaceVertexIndices
		nullptr, // pFaceVertexTexCoordIndices
		nullptr, // pFaceVertexNormalIndices
		0, // numVertices
		0, // numNormals
		0, // numTexCoords
		0, // numFaces
	};

	MioMesh cutMesh = srcMesh;

    McResult status = MC_NO_ERROR;

    std::vector<McContext> contexts_array;

    {
        Timer scoped_timer("init-function");

        //
	// read-in the source-mesh from file
	//

	mioReadOFF(DATA_DIR "/source-mesh.off",
			   &srcMesh.pVertices,
			   &srcMesh.pFaceVertexIndices,
               &srcMesh.pFaceSizes,
			   &srcMesh.numVertices,
			   &srcMesh.numFaces);

	//
	// read-in the cut-mesh from file
	//

	mioReadOFF(DATA_DIR "/cut-mesh.off",
			   &cutMesh.pVertices,
			   &cutMesh.pFaceVertexIndices,
               &cutMesh.pFaceSizes,
			   &cutMesh.numVertices,
			   &cutMesh.numFaces);

        printf("\n>> Create MCUT contexts\n");

        const McUint32 num_system_threads = std::thread::hardware_concurrency();
        const McUint32 num_contexts_to_create = num_system_threads / 2; // since we are using "MC_OUT_OF_ORDER_EXEC_MODE_ENABLE"

        contexts_array.resize(num_contexts_to_create);

        for (McUint32 i = 0; i < num_contexts_to_create; ++i) {

            status = mcCreateContext(&contexts_array[i], MC_DEBUG | MC_OUT_OF_ORDER_EXEC_MODE_ENABLE);

            if (status != MC_NO_ERROR) {
                printf("mcCreateContext failed (err=%d)", (int)status);
                exit(1);
            }

            //
            // config debug output
            // 

            McFlags contextFlags = MC_NULL_HANDLE;

            status = mcGetInfo(contexts_array[i], MC_CONTEXT_FLAGS, sizeof(McFlags), &contextFlags, nullptr);

            if (status != MC_NO_ERROR) {
                printf("1:mcGetInfo(MC_CONTEXT_FLAGS) failed (err=%d)", (int)status);
                exit(1);
            }

            if (contextFlags & MC_DEBUG) {
                status = mcDebugMessageCallback(contexts_array[i], mcDebugOutputCALLBACK, nullptr);

                if (status != MC_NO_ERROR) {
                    printf("mcDebugMessageCallback failed (err=%d)", (int)status);
                    exit(1);
                }

                status = mcDebugMessageControl(contexts_array[i], McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);

                if (status != MC_NO_ERROR) {
                    printf("mcDebugMessageControl failed (err=%d)", (int)status);
                    exit(1);
                }
            }
        }

        printf("\n>> Have %u MCUT contexts\n", num_contexts_to_create);
    }

    const McUint32 num_contexts = (McUint32)contexts_array.size();

    //
    // do the cutting
    // 

    std::vector<std::vector<McConnectedComponent>> per_context_connected_components(num_contexts);

    {
        Timer scoped_timer("dispatch-function");

        std::vector<std::vector<McEvent>> per_context_dispatch_events(num_contexts);
        std::vector<McUint32> per_context_CC_count_array(num_contexts, MC_NULL_HANDLE);

        auto schedule_dispatch_call = [&](McContext context, std::vector<McEvent>& emittedEvents, McUint32& numConnComps) {

            status = mcEnqueueDispatch(
                context,
                MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
                // source mesh
                srcMesh.pVertices,
                srcMesh.pFaceVertexIndices,
                srcMesh.pFaceSizes,
                srcMesh.numVertices,
                srcMesh.numFaces,
                // cut mesh
                cutMesh.pVertices,
                cutMesh.pFaceVertexIndices,
                cutMesh.pFaceSizes,
                cutMesh.numVertices,
                cutMesh.numFaces,
                // events to wait for
                0,
                NULL,
                // emitted event
                &emittedEvents[0]);

            if (status != MC_NO_ERROR) {
                printf("mcEnqueueDispatch failed (err=%d)\n", (int)status);
                return status;
            }

            // query for all available connected components:
            status = mcEnqueueGetConnectedComponents(
                context,
                MC_CONNECTED_COMPONENT_TYPE_ALL,
                0, NULL,
                &numConnComps, // This variable will be updated when "eventQueryNumCCsAfterDispatch" has completed
                1,
                &emittedEvents[0], // NOTE: MCUT make an internal copy of this handle (i.e. its value)
                &emittedEvents[1]);

            if (status != MC_NO_ERROR) {
                printf("mcEnqueueGetConnectedComponents failed (err=%d)\n", (int)status);
                return status;
            }

            return MC_NO_ERROR;
        };

        //
        // **Fork**
        //

        // Spin-off multiple dispatch calls that will run in parallel
        for (McUint32 i = 0; i < num_contexts; ++i) {

            per_context_dispatch_events[i].resize(2); // there are two enqueue commands in "schedule_dispatch_call"

            status = schedule_dispatch_call(contexts_array[i], per_context_dispatch_events[i], per_context_CC_count_array[i]);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "schedule_dispatch_call failed (err=%d)\n", (int)status);
                exit(1);
            }
        }

        // ** We could also do some other stuff here while the events in per_context_dispatch_events are running
        // If you have some other pending work in your application, you can query an MCUT command's status with
        // mcGetEventInfo and passing MC_EVENT_COMMAND_EXECUTION_STATUS

        //
        // **Join**
        //

        {
            std::vector<McEvent> dispatch_events_array; // collection of all events into one list

            for (McUint32 i = 0; i < num_contexts; ++i) {

                dispatch_events_array.insert(
                    dispatch_events_array.cend(),
                    per_context_dispatch_events[i].cbegin(),
                    per_context_dispatch_events[i].cend());
            }

            // wait for all events
            status = mcWaitForEvents((McUint32)dispatch_events_array.size(), &dispatch_events_array[0]);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "mcWaitForEvents failed (err=%d)\n", (int)status);
                exit(1);
            }

            //
            // We no longer need the mem of input meshes, so we can free it!
            //
            //  CAUTION: Make sure that all dispatch functions that refer
            //  to user allocated memory (like input-mesh pointers) have completed 
            //  before freeing this memory. Thus, the earliest we can call "mioFreeMesh"
            //  is after waiting for the dispatch functions to finish with "mcWaitForEvents"
            //
            mioFreeMesh(&srcMesh);
            mioFreeMesh(&cutMesh);

            // check for any runtime errors and request the connected components

            for (McUint32 i = 0; i < num_contexts; ++i) {

                for (McUint32 j = 0; j < (McUint32)per_context_dispatch_events[i].size(); ++j) {

                    McResult schedule_dispatch_call_fn_status = MC_NO_ERROR;

                    status = mcGetEventInfo(per_context_dispatch_events[i][j], MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &schedule_dispatch_call_fn_status, NULL);

                    if (status != MC_NO_ERROR) {
                        fprintf(stderr, "1:mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS...) failed (err=%d)\n", (int)status);
                        exit(1);
                    }

                    if (schedule_dispatch_call_fn_status != MC_NO_ERROR) {
                        fprintf(stderr, "schedule_dispatch_call failed internally (err=%d)\n", (int)schedule_dispatch_call_fn_status);
                        exit(1);
                    }
                }

                const McUint32 num_connected_components = per_context_CC_count_array[i]; // ... in context

                if (num_connected_components == 0) {
                    printf("context %p has no connected components\n", contexts_array[i]);
                    return status;
                }

                per_context_connected_components[i].resize(num_connected_components);

                status = mcGetConnectedComponents( // NOTE: blocking call because we need the result immediately (see below)
                    contexts_array[i],
                    MC_CONNECTED_COMPONENT_TYPE_ALL,
                    num_connected_components,
                    &per_context_connected_components[i][0],
                    NULL);

                if (status != MC_NO_ERROR) {
                    printf("mcGetConnectedComponents failed (err=%d)\n", (int)status);
                    return status;
                }
            }

            status = mcReleaseEvents((McUint32)dispatch_events_array.size(), &dispatch_events_array[0]);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)status);
                exit(1);
            }
        }
    }

    //
    // Request connected component data [size] in bytes so that we can allocate memory
    // 

    std::vector< // context
        std::vector< // cc
            McSize // queried vertices array size
            >>
        per_context_per_cc_vertices_bytes(num_contexts);
    std::vector< // context
        std::vector< // cc
            McSize // queried triangles array size
            >>
        per_context_per_cc_triangles_bytes(num_contexts);

    {
        Timer scoped_timer("request-CC-data-size");

        auto schedule_cc_data_bytesize_query = [](
                                                   const McContext context,
                                                   const std::vector<McConnectedComponent>& context_connected_components,
                                                   std::vector<std::vector<McEvent>>& per_cc_event_waitlist,
                                                   std::vector<McSize>& per_cc_vertices_bytes,
                                                   std::vector<McSize>& per_cc_triangles_bytes) {

            const McUint32 num_connected_components = (McUint32)context_connected_components.size();

            for (McUint32 i = 0; i < num_connected_components; ++i) {
                McConnectedComponent connCompId = context_connected_components[i]; // connected compoenent id

                const McUint32 async_queries_per_cc = 2; // vertices and triangles

                std::vector<McEvent>& cc_events = per_cc_event_waitlist[i];
                cc_events.resize(async_queries_per_cc);

                McSize& cc_vertices_bytes = per_cc_vertices_bytes[i];
                McSize& cc_triangles_bytes = per_cc_triangles_bytes[i];

                cc_vertices_bytes = 0;
                McResult status = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE,
                    0, NULL,
                    &cc_vertices_bytes,
                    0, NULL,
                    &cc_events[0]);

                if (status != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (err=%d)\n", (int)status);
                    return status;
                }

                cc_triangles_bytes = 0;
                status = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION,
                    0, NULL,
                    &cc_triangles_bytes, 0, NULL,
                    &cc_events[1]);

                if (status != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION) failed (err=%d)\n", (int)status);
                    return status;
                }
            }

            return MC_NO_ERROR;
        };

        std::vector< // context
            std::vector< // cc
                std::vector< // async task
                    McEvent>>>
            per_context_per_cc_event_waitlist(num_contexts);

        //
        // **Fork**
        //
        for (McUint32 i = 0; i < num_contexts; ++i) {

            const McUint32 num_connected_components = (McUint32)per_context_connected_components[i].size();

            per_context_per_cc_event_waitlist[i].resize(num_connected_components);
            per_context_per_cc_vertices_bytes[i].resize(num_connected_components);
            per_context_per_cc_triangles_bytes[i].resize(num_connected_components);

            status = schedule_cc_data_bytesize_query(
                contexts_array[i],
                per_context_connected_components[i],
                per_context_per_cc_event_waitlist[i],
                per_context_per_cc_vertices_bytes[i],
                per_context_per_cc_triangles_bytes[i]);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "schedule_dispatch_call failed (err=%d)\n", (int)status);
                exit(1);
            }
        }

        // ** Can do something else here while events are running

        //
        // wait for events
        // 

        {
            std::vector<McEvent> bytes_size_query_events_array; // collection of all events into one list

            for (McUint32 i = 0; i < num_contexts; ++i) {

                const McUint32 num_CCs = (McUint32)per_context_connected_components[i].size();

                for (McUint32 j = 0; j < num_CCs; ++j) {
                    bytes_size_query_events_array.insert(
                        bytes_size_query_events_array.cend(),
                        per_context_per_cc_event_waitlist[i][j].cbegin(),
                        per_context_per_cc_event_waitlist[i][j].cend());
                }
            }

            status = mcWaitForEvents((McUint32)bytes_size_query_events_array.size(), &bytes_size_query_events_array[0]);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "mcWaitForEvents failed (err=%d)\n", (int)status);
                exit(1);
            }

            //
            // check for any errors
            //
            for (McUint32 i = 0; i < num_contexts; ++i) {

                const McUint32 num_CCs = (McUint32)per_context_connected_components[i].size();

                for (McUint32 j = 0; j < num_CCs; ++j) {

                    const McUint32 num_events = (McUint32)per_context_per_cc_event_waitlist[i][j].size();

                    for (McUint32 k = 0; k < num_events; ++k) {

                        McResult schedule_cc_data_bytesize_query_fn_status = MC_NO_ERROR;
                         status = mcGetEventInfo(per_context_per_cc_event_waitlist[i][j][k], MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &schedule_cc_data_bytesize_query_fn_status, NULL);

                        if (status != MC_NO_ERROR) {
                            fprintf(stderr, "1:mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS...) failed (err=%d)\n", (int)status);
                            exit(1);
                        }

                        if (schedule_cc_data_bytesize_query_fn_status != MC_NO_ERROR) {
                            fprintf(stderr, "schedule_cc_data_bytesize_query failed internally (err=%d)\n", (int)schedule_cc_data_bytesize_query_fn_status);
                            exit(1);
                        }
                    }
                }
            }

            status = mcReleaseEvents((McUint32)bytes_size_query_events_array.size(), &bytes_size_query_events_array[0]);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)status);
                exit(1);
            }
        }
    }

    //
    // Query connected component data now we know the amount of bytes to allocate
    // 

    std::vector< // context
        std::vector< // cc
            std::vector<McDouble> // queried vertices array
            >>
        per_context_per_cc_vertices_array(num_contexts);
    std::vector< // context
        std::vector< // cc
            std::vector<McUint32> // queried triangles array size
            >>
        per_context_per_cc_triangles_array(num_contexts);

    {
        Timer scoped_timer("request-CC-data");

        auto schedule_cc_data_query = [](
                                          const McContext context,
                                          const std::vector<McConnectedComponent>& context_connected_components,
                                          std::vector<std::vector<McEvent>>& per_cc_event_waitlist,
                                          const std::vector<McSize>& per_cc_vertices_bytes,
                                          const std::vector<McSize>& per_cc_triangles_bytes,
                                          std::vector<std::vector<McDouble>>& per_cc_vertices_array,
                                          std::vector<std::vector<McUint32>>& per_cc_triangles_array) {
            const McUint32 num_connected_components = (McUint32)context_connected_components.size();

            for (McUint32 i = 0; i < num_connected_components; ++i) {
                McConnectedComponent connCompId = context_connected_components[i]; // connected compoenent id

                std::vector<McEvent>& cc_events = per_cc_event_waitlist[i];
                
                const McSize& cc_vertices_bytes = per_cc_vertices_bytes[i];
                const McSize& cc_triangles_bytes = per_cc_triangles_bytes[i];

                std::vector<McDouble>& cc_vertices_array = per_cc_vertices_array[i];
                std::vector<McUint32>& cc_triangles_array = per_cc_triangles_array[i];

                McResult status = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE,
                    cc_vertices_bytes, &cc_vertices_array[0],
                    NULL,
                    0, NULL,
                    &cc_events[0]);

                if (status != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (err=%d)\n", (int)status);
                    return status;
                }

                status = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION,
                    cc_triangles_bytes, &cc_triangles_array[0],
                    NULL, 0, NULL,
                    &cc_events[1]);

                if (status != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION) failed (err=%d)\n", (int)status);
                    return status;
                }
            }

            return MC_NO_ERROR;
        };

        std::vector< // context
            std::vector< // cc
                std::vector< // async task
                    McEvent>>>
            per_context_per_cc_event_waitlist(num_contexts);

        for (McUint32 i = 0; i < num_contexts; ++i) {

            const McUint32 num_connected_components = (McUint32)per_context_connected_components[i].size();

            
            per_context_per_cc_event_waitlist[i].resize(num_connected_components);
            per_context_per_cc_vertices_array[i].resize(num_connected_components);
            per_context_per_cc_triangles_array[i].resize(num_connected_components);

            // allocate arrays (since we know how much to allocated)
            for(McUint32 j = 0; j < num_connected_components; ++j)
            {
                const McUint32 async_queries_per_cc = 2; // vertices and triangles

                per_context_per_cc_event_waitlist[i][j].resize(async_queries_per_cc);

                const McUint32 num_vertex_array_doubles = (McUint32)per_context_per_cc_vertices_bytes[i][j]/sizeof(McDouble);
                const McUint32 num_triangles_array_uints = (McUint32)per_context_per_cc_vertices_bytes[i][j]/sizeof(McUint32);

                per_context_per_cc_vertices_array[i][j].resize(num_vertex_array_doubles);
                per_context_per_cc_triangles_array[i][j].resize(num_triangles_array_uints);
            }

            schedule_cc_data_query(
                contexts_array[i],
                per_context_connected_components[i],
                per_context_per_cc_event_waitlist[i],
                per_context_per_cc_vertices_bytes[i],
                per_context_per_cc_triangles_bytes[i],
                per_context_per_cc_vertices_array[i],
                per_context_per_cc_triangles_array[i]);
        }

        // ** Can do something else here while events are running

        //
        // wait for events (join)
        // 
        std::vector<McEvent> data_query_events_array; // collection of all events into one list

        for (McUint32 i = 0; i < num_contexts; ++i) {
            const McUint32 num_CCs = (McUint32)per_context_connected_components[i].size();

            for (McUint32 j = 0; j < num_CCs; ++j) {
                data_query_events_array.insert(
                    data_query_events_array.cend(),
                    per_context_per_cc_event_waitlist[i][j].cbegin(),
                    per_context_per_cc_event_waitlist[i][j].cend());
            }
        }

        status = mcWaitForEvents((McUint32)data_query_events_array.size(), &data_query_events_array[0]);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "mcWaitForEvents failed (err=%d)\n", (int)status);
            exit(1);
        }

        //
        // check for any errors
        //
        for (McUint32 i = 0; i < num_contexts; ++i) {

            const McUint32 num_CCs = (McUint32)per_context_connected_components[i].size();

            for (McUint32 j = 0; j < num_CCs; ++j) {

                const McUint32 num_events = (McUint32)per_context_per_cc_event_waitlist[i][j].size();

                for (McUint32 k = 0; k < num_events; ++k) {

                    McResult exec_status = MC_NO_ERROR;
                    status = mcGetEventInfo(per_context_per_cc_event_waitlist[i][j][k], MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &exec_status, NULL);

                    if (status != MC_NO_ERROR) {
                        fprintf(stderr, "mcGetEventInfo(MC_EVENT_RUNTIME_EXECUTION_STATUS...) failed (err=%d)\n", (int)status);
                        exit(1);
                    }

                    if (exec_status != MC_NO_ERROR) {
                        fprintf(stderr, "schedule_cc_data_bytesize_query failed internally (err=%d)\n", (int)exec_status);
                        exit(1);
                    }
                }
            }
        }

        status = mcReleaseEvents((McUint32)data_query_events_array.size(), &data_query_events_array[0]);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)status);
            exit(1);
        }
    }

    //
    // Write the connected components to file
    // 
    {
        Timer scoped_timer("save-files");

        for (McUint32 i = 0; i < num_contexts; ++i) {

            const McUint32 num_CCs = (McUint32)per_context_per_cc_vertices_array[i].size();

            for (McUint32 j = 0; j < num_CCs; ++j) {

                const std::vector<McDouble>& vertex_array = per_context_per_cc_vertices_array[i][j];
                const std::vector<McUint32>& triangles_array = per_context_per_cc_triangles_array[i][j];
                std::vector<McUint32> face_sizes(triangles_array.size() / 3, 3); // required by "mioWriteOBJ"

                //
                // save connected component (mesh) to an .obj file
                // 

                char fnameBuf[256];
                sprintf(fnameBuf, "ctxt%d-conncomp%d.obj", i, j);
                std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

                mioWriteOBJ(
                    fpath.c_str(), 
                    (McDouble*)vertex_array.data(), 
                    nullptr, // pNormals
                    nullptr, // pTexCoords
                    face_sizes.data(), 
                    (McUint32*)triangles_array.data(), 
                    nullptr, // pFaceVertexTexCoordIndices
                    nullptr, // pFaceVertexNormalIndices 
                    vertex_array.size()/3, 
                    0, // numNormals 
                    0, // numTexCoords
                    (McUint32)face_sizes.size());
            }
        }
    }
    
    //
    // Tear down contexts
    // 

    {
        Timer scoped_timer("teardown");

        for (McUint32 i = 0; i < num_contexts; ++i) {

            McContext context = contexts_array[i];

            //
            // destroy internal data associated with each connected component
            //
            status = mcReleaseConnectedComponents(context, (McUint32)per_context_connected_components[i].size(), per_context_connected_components[i].data());

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)status);
                exit(1);
            }

            //
            // destroy context
            //
            status = mcReleaseContext(context);

            if (status != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseContext failed (err=%d)\n", (int)status);
                exit(1);
            }
        }
    }

    return 0;
}

void MCAPI_PTR mcDebugOutputCALLBACK(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{

    // printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
    // printf("userParam=%p\n", userParam);

    std::string debug_src;
    switch (source) {
    case MC_DEBUG_SOURCE_API:
        debug_src = "API";
        break;
    case MC_DEBUG_SOURCE_KERNEL:
        debug_src = "KERNEL";
        break;
    case MC_DEBUG_SOURCE_FRONTEND:
        debug_src = "FRONTEND";
    case MC_DEBUG_SOURCE_ALL:case MC_DEBUG_SOURCE_IGNORE:
        break;
    }
    std::string debug_type;
    switch (type) {
    case MC_DEBUG_TYPE_ERROR:
        debug_type = "ERROR";
        break;
    case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        debug_type = "DEPRECATION";
        break;
    case MC_DEBUG_TYPE_OTHER:
        debug_type = "OTHER";
        break;
    case MC_DEBUG_TYPE_ALL:case MC_DEBUG_TYPE_IGNORE:
        break;
    }

    std::string severity_str;

    switch (severity) {
    case MC_DEBUG_SEVERITY_HIGH:
        severity_str = "HIGH";
        break;
    case MC_DEBUG_SEVERITY_MEDIUM:
        severity_str = "MEDIUM";
        break;
    case MC_DEBUG_SEVERITY_LOW:
        severity_str = "LOW";
        break;
    case MC_DEBUG_SEVERITY_NOTIFICATION:
        severity_str = "NOTIFICATION";
        break;
    case MC_DEBUG_SEVERITY_ALL:
        break;
    }

    printf("MCUT[%d:%p,%s:%s:%s:%zu] %s\n", id, userParam, debug_src.c_str(), debug_type.c_str(), severity_str.c_str(), length, message);
}

void mcCheckError_(McResult err, const char* file, int line)
{
    std::string error;
    switch (err) {
    case MC_OUT_OF_MEMORY:
        error = "MC_OUT_OF_MEMORY";
        break;
    case MC_INVALID_VALUE:
        error = "MC_INVALID_VALUE";
        break;
    case MC_INVALID_OPERATION:
        error = "MC_INVALID_OPERATION";
        break;
    case MC_NO_ERROR:
        error = "MC_NO_ERROR";
        break;
    case MC_RESULT_MAX_ENUM:
        error = "UNKNOWN";
        break;
    }
    if (err) {
        std::cout << error << " | " << file << " (" << line << ")" << std::endl;
    }
}