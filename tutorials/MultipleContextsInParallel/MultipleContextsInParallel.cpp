/**
 * Copyright (c) 2021-2023 Floyd M. Chitalu.
 * All rights reserved.
 *
 * NOTE: This file is licensed under GPL-3.0-or-later (default).
 * A commercial license can be purchased from Floyd M. Chitalu.
 *
 * License details:
 *
 * (A)  GNU General Public License ("GPL"); a copy of which you should have
 *      recieved with this file.
 * 	    - see also: <http://www.gnu.org/licenses/>
 * (B)  Commercial license.
 *      - email: floyd.m.chitalu@gmail.com
 *
 * The commercial license options is for users that wish to use MCUT in
 * their products for comercial purposes but do not wish to release their
 * software products under the GPL license.
 *
 * Author(s)     : Floyd M. Chitalu
 */

#include "mcut/mcut.h"
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

void readOFF(
    const char* fpath,
    double** pVertices,
    unsigned int** pFaceIndices,
    unsigned int** pFaceSizes,
    unsigned int* numVertices,
    unsigned int* numFaces);

void writeOFF(
    const char* fpath,
    const double* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces);

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

int main(int argc, char* argv[])
{
    Timer scoped_timer_main_fn("main-function");

    double* srcMeshVertices = NULL;
    uint32_t* srcMeshFaceIndices = NULL;
    uint32_t* srcMeshFaceSizes = NULL;
    uint32_t srcMeshNumFaces = 0;
    uint32_t srcMeshNumVertices = 0;

    double* cutMeshVertices = NULL;
    uint32_t* cutMeshFaceIndices = NULL;
    uint32_t* cutMeshFaceSizes = NULL;
    uint32_t cutMeshNumFaces = 0;
    uint32_t cutMeshNumVertices = 0;

    McResult api_err = MC_NO_ERROR;

    std::vector<McContext> contexts_array;

    {
        Timer scoped_timer("init-function");

        const char* srcMeshFilePath = DATA_DIR "/source-mesh.off";
        const char* cutMeshFilePath = DATA_DIR "/cut-mesh.off";

        printf(">> source-mesh file: %s\n", srcMeshFilePath);
        printf(">> cut-mesh file: %s\n", cutMeshFilePath);

        // load meshes
        // -----------

        readOFF(srcMeshFilePath, &srcMeshVertices, &srcMeshFaceIndices, &srcMeshFaceSizes, &srcMeshNumVertices, &srcMeshNumFaces);

        printf(">> src-mesh vertices=%u faces=%u\n", srcMeshNumVertices, srcMeshNumFaces);

        readOFF(cutMeshFilePath, &cutMeshVertices, &cutMeshFaceIndices, &cutMeshFaceSizes, &cutMeshNumVertices, &cutMeshNumFaces);

        printf(">> cut-mesh vertices=%u faces=%u\n", cutMeshNumVertices, cutMeshNumFaces);

        printf("\n>> Create MCUT contexts\n");

        const uint32_t num_system_threads = std::thread::hardware_concurrency();
        const uint32_t num_contexts_to_create = num_system_threads / 2; // since we are using "MC_OUT_OF_ORDER_EXEC_MODE_ENABLE"

        contexts_array.resize(num_contexts_to_create);

        for (uint32_t i = 0; i < num_contexts_to_create; ++i) {

            api_err = mcCreateContext(&contexts_array[i], MC_DEBUG | MC_OUT_OF_ORDER_EXEC_MODE_ENABLE);

            if (api_err != MC_NO_ERROR) {
                printf("mcCreateContext failed (err=%d)", (int)api_err);
                exit(1);
            }

            // config debug output
            // -------------------------------------------------------------------------

            McFlags contextFlags = MC_NULL_HANDLE;

            api_err = mcGetInfo(contexts_array[i], MC_CONTEXT_FLAGS, sizeof(McFlags), &contextFlags, nullptr);

            if (api_err != MC_NO_ERROR) {
                printf("1:mcGetInfo(MC_CONTEXT_FLAGS) failed (err=%d)", (int)api_err);
                exit(1);
            }

            if (contextFlags & MC_DEBUG) {
                api_err = mcDebugMessageCallback(contexts_array[i], mcDebugOutputCALLBACK, nullptr);

                if (api_err != MC_NO_ERROR) {
                    printf("mcDebugMessageCallback failed (err=%d)", (int)api_err);
                    exit(1);
                }

                api_err = mcDebugMessageControl(contexts_array[i], McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);

                if (api_err != MC_NO_ERROR) {
                    printf("mcDebugMessageControl failed (err=%d)", (int)api_err);
                    exit(1);
                }
            }
        }

        printf("\n>> Have %u MCUT contexts\n", num_contexts_to_create);
    }

    const uint32_t num_contexts = contexts_array.size();

    // do the cutting
    // -------------------------------------------------------------------------

    std::vector<std::vector<McConnectedComponent>> per_context_connected_components(num_contexts);

    {
        Timer scoped_timer("dispatch-function");

        std::vector<std::vector<McEvent>> per_context_dispatch_events(num_contexts);
        std::vector<McUint32> per_context_CC_count_array(num_contexts, MC_NULL_HANDLE);

        auto schedule_dispatch_call = [&](McContext context, std::vector<McEvent>& emittedEvents, McUint32& numConnComps) {

            api_err = mcEnqueueDispatch(
                context,
                MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
                // source mesh
                (const McVoid*)srcMeshVertices,
                (const McUint32*)srcMeshFaceIndices,
                (const McUint32*)srcMeshFaceSizes,
                (McUint32)srcMeshNumVertices,
                (McUint32)srcMeshNumFaces,
                // cut mesh
                (const McVoid*)cutMeshVertices,
                (const McUint32*)cutMeshFaceIndices,
                (const McUint32*)cutMeshFaceSizes,
                (McUint32)cutMeshNumVertices,
                (McUint32)cutMeshNumFaces,
                // events to wait for
                0,
                NULL,
                // emitted event
                &emittedEvents[0]);

            if (api_err != MC_NO_ERROR) {
                printf("mcEnqueueDispatch failed (err=%d)\n", (int)api_err);
                return api_err;
            }

            // query for all available connected components:
            api_err = mcEnqueueGetConnectedComponents(
                context,
                MC_CONNECTED_COMPONENT_TYPE_ALL,
                0, NULL,
                &numConnComps, // This variable will be updated when "eventQueryNumCCsAfterDispatch" has completed
                1,
                &emittedEvents[0], // NOTE: MCUT make an internal copy of this handle (i.e. its value)
                &emittedEvents[1]);

            if (api_err != MC_NO_ERROR) {
                printf("mcEnqueueGetConnectedComponents failed (err=%d)\n", (int)api_err);
                return api_err;
            }

            return MC_NO_ERROR;
        };

        //
        // **Fork**
        //

        // Spin-off multiple dispatch calls that will run in parallel
        for (uint32_t i = 0; i < num_contexts; ++i) {

            per_context_dispatch_events[i].resize(2); // there are two enqueue commands in "schedule_dispatch_call"

            api_err = schedule_dispatch_call(contexts_array[i], per_context_dispatch_events[i], per_context_CC_count_array[i]);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "schedule_dispatch_call failed (err=%d)\n", (int)api_err);
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

            for (uint32_t i = 0; i < num_contexts; ++i) {

                dispatch_events_array.insert(
                    dispatch_events_array.cend(),
                    per_context_dispatch_events[i].cbegin(),
                    per_context_dispatch_events[i].cend());
            }

            api_err = mcWaitForEvents((McUint32)dispatch_events_array.size(), &dispatch_events_array[0]);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "mcWaitForEvents failed (err=%d)\n", (int)api_err);
                exit(1);
            }

            // check for any runtime errors and request the connected components

            for (uint32_t i = 0; i < num_contexts; ++i) {

                for (uint32_t j = 0; j < (uint32_t)per_context_dispatch_events[i].size(); ++j) {

                    McResult schedule_dispatch_call_fn_status = MC_NO_ERROR;

                    api_err = mcGetEventInfo(per_context_dispatch_events[i][j], MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &schedule_dispatch_call_fn_status, NULL);

                    if (api_err != MC_NO_ERROR) {
                        fprintf(stderr, "1:mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS...) failed (err=%d)\n", (int)api_err);
                        exit(1);
                    }

                    if (schedule_dispatch_call_fn_status != MC_NO_ERROR) {
                        fprintf(stderr, "schedule_dispatch_call failed internally (err=%d)\n", (int)schedule_dispatch_call_fn_status);
                        exit(1);
                    }
                }

                const uint32_t num_connected_components = per_context_CC_count_array[i]; // ... in context

                if (num_connected_components == 0) {
                    printf("context %p has no connected components\n", contexts_array[i]);
                    return api_err;
                }

                per_context_connected_components[i].resize(num_connected_components);

                api_err = mcGetConnectedComponents( // NOTE: blocking call because we need the result immediately (see below)
                    contexts_array[i],
                    MC_CONNECTED_COMPONENT_TYPE_ALL,
                    num_connected_components,
                    &per_context_connected_components[i][0],
                    NULL);

                if (api_err != MC_NO_ERROR) {
                    printf("mcGetConnectedComponents failed (err=%d)\n", (int)api_err);
                    return api_err;
                }
            }

            api_err = mcReleaseEvents((McUint32)dispatch_events_array.size(), &dispatch_events_array[0]);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)api_err);
                exit(1);
            }
        }
    }

    // Request connected component data size in bytes
    // -------------------------------------------------------------------------

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
            const uint32_t num_connected_components = (uint32_t)context_connected_components.size();

            for (uint32_t i = 0; i < num_connected_components; ++i) {
                McConnectedComponent connCompId = context_connected_components[i]; // connected compoenent id

                const uint32_t async_queries_per_cc = 2; // vertices and triangles

                std::vector<McEvent>& cc_events = per_cc_event_waitlist[i];
                cc_events.resize(async_queries_per_cc);

                McSize& cc_vertices_bytes = per_cc_vertices_bytes[i];
                McSize& cc_triangles_bytes = per_cc_triangles_bytes[i];

                cc_vertices_bytes = 0;
                McResult api_err = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE,
                    0, NULL,
                    &cc_vertices_bytes,
                    0, NULL,
                    &cc_events[0]);

                if (api_err != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (err=%d)\n", (int)api_err);
                    return api_err;
                }

                cc_triangles_bytes = 0;
                api_err = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION,
                    0, NULL,
                    &cc_triangles_bytes, 0, NULL,
                    &cc_events[1]);

                if (api_err != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION) failed (err=%d)\n", (int)api_err);
                    return api_err;
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
        for (uint32_t i = 0; i < num_contexts; ++i) {

            const uint32_t num_connected_components = (uint32_t)per_context_connected_components[i].size();

            per_context_per_cc_event_waitlist[i].resize(num_connected_components);
            per_context_per_cc_vertices_bytes[i].resize(num_connected_components);
            per_context_per_cc_triangles_bytes[i].resize(num_connected_components);

            api_err = schedule_cc_data_bytesize_query(
                contexts_array[i],
                per_context_connected_components[i],
                per_context_per_cc_event_waitlist[i],
                per_context_per_cc_vertices_bytes[i],
                per_context_per_cc_triangles_bytes[i]);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "schedule_dispatch_call failed (err=%d)\n", (int)api_err);
                exit(1);
            }
        }

        // ** Can do something else here while events are running

        // wait for events
        // -------------------------------------------------------------------------

        {
            std::vector<McEvent> bytes_size_query_events_array; // collection of all events into one list

            for (uint32_t i = 0; i < num_contexts; ++i) {

                const uint32_t num_CCs = per_context_connected_components[i].size();

                for (uint32_t j = 0; j < num_CCs; ++j) {
                    bytes_size_query_events_array.insert(
                        bytes_size_query_events_array.cend(),
                        per_context_per_cc_event_waitlist[i][j].cbegin(),
                        per_context_per_cc_event_waitlist[i][j].cend());
                }
            }

            api_err = mcWaitForEvents((McUint32)bytes_size_query_events_array.size(), &bytes_size_query_events_array[0]);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "mcWaitForEvents failed (err=%d)\n", (int)api_err);
                exit(1);
            }

            //
            // check for any errors
            //
            for (uint32_t i = 0; i < num_contexts; ++i) {

                const uint32_t num_CCs = per_context_connected_components[i].size();

                for (uint32_t j = 0; j < num_CCs; ++j) {

                    const uint32_t num_events = per_context_per_cc_event_waitlist[i][j].size();

                    for (uint32_t k = 0; k < num_events; ++k) {

                        McResult schedule_cc_data_bytesize_query_fn_status = MC_NO_ERROR;
                        const McResult api_err = mcGetEventInfo(per_context_per_cc_event_waitlist[i][j][k], MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &schedule_cc_data_bytesize_query_fn_status, NULL);

                        if (api_err != MC_NO_ERROR) {
                            fprintf(stderr, "1:mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS...) failed (err=%d)\n", (int)api_err);
                            exit(1);
                        }

                        if (schedule_cc_data_bytesize_query_fn_status != MC_NO_ERROR) {
                            fprintf(stderr, "schedule_cc_data_bytesize_query failed internally (err=%d)\n", (int)schedule_cc_data_bytesize_query_fn_status);
                            exit(1);
                        }
                    }
                }
            }

            api_err = mcReleaseEvents((McUint32)bytes_size_query_events_array.size(), &bytes_size_query_events_array[0]);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)api_err);
                exit(1);
            }
        }
    }

    // Query connected component data now we know the amount of bytes to allocate
    // -------------------------------------------------------------------------

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
            const uint32_t num_connected_components = (uint32_t)context_connected_components.size();

            for (uint32_t i = 0; i < num_connected_components; ++i) {
                McConnectedComponent connCompId = context_connected_components[i]; // connected compoenent id

                std::vector<McEvent>& cc_events = per_cc_event_waitlist[i];
                
                const McSize& cc_vertices_bytes = per_cc_vertices_bytes[i];
                const McSize& cc_triangles_bytes = per_cc_triangles_bytes[i];

                std::vector<McDouble>& cc_vertices_array = per_cc_vertices_array[i];
                std::vector<McUint32>& cc_triangles_array = per_cc_triangles_array[i];

                McResult api_err = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE,
                    cc_vertices_bytes, &cc_vertices_array[0],
                    NULL,
                    0, NULL,
                    &cc_events[0]);

                if (api_err != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (err=%d)\n", (int)api_err);
                    return api_err;
                }

                api_err = mcEnqueueGetConnectedComponentData(
                    context,
                    connCompId,
                    MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION,
                    cc_triangles_bytes, &cc_triangles_array[0],
                    NULL, 0, NULL,
                    &cc_events[1]);

                if (api_err != MC_NO_ERROR) {
                    printf("mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION) failed (err=%d)\n", (int)api_err);
                    return api_err;
                }
            }

            return MC_NO_ERROR;
        };

        std::vector< // context
            std::vector< // cc
                std::vector< // async task
                    McEvent>>>
            per_context_per_cc_event_waitlist(num_contexts);

        for (uint32_t i = 0; i < num_contexts; ++i) {

            const uint32_t num_connected_components = (uint32_t)per_context_connected_components[i].size();

            
            per_context_per_cc_event_waitlist[i].resize(num_connected_components);
            per_context_per_cc_vertices_array[i].resize(num_connected_components);
            per_context_per_cc_triangles_array[i].resize(num_connected_components);

            // allocate arrays (since we know how much to allocated)
            for(uint32_t j = 0; j < num_connected_components; ++j)
            {
                const uint32_t async_queries_per_cc = 2; // vertices and triangles

                per_context_per_cc_event_waitlist[i][j].resize(async_queries_per_cc);

                const uint32_t num_vertex_array_doubles = per_context_per_cc_vertices_bytes[i][j]/sizeof(McDouble);
                const uint32_t num_triangles_array_uints = per_context_per_cc_vertices_bytes[i][j]/sizeof(McUint32);

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

        // wait for events (join)
        // -------------------------------------------------------------------------
        std::vector<McEvent> data_query_events_array; // collection of all events into one list

        for (uint32_t i = 0; i < num_contexts; ++i) {
            const uint32_t num_CCs = per_context_connected_components[i].size();

            for (uint32_t j = 0; j < num_CCs; ++j) {
                data_query_events_array.insert(
                    data_query_events_array.cend(),
                    per_context_per_cc_event_waitlist[i][j].cbegin(),
                    per_context_per_cc_event_waitlist[i][j].cend());
            }
        }

        api_err = mcWaitForEvents((McUint32)data_query_events_array.size(), &data_query_events_array[0]);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "mcWaitForEvents failed (err=%d)\n", (int)api_err);
            exit(1);
        }

        //
        // check for any errors
        //
        for (uint32_t i = 0; i < num_contexts; ++i) {

            const uint32_t num_CCs = per_context_connected_components[i].size();

            for (uint32_t j = 0; j < num_CCs; ++j) {

                const uint32_t num_events = per_context_per_cc_event_waitlist[i][j].size();

                for (uint32_t k = 0; k < num_events; ++k) {

                    McResult exec_status = MC_NO_ERROR;
                    const McResult api_err = mcGetEventInfo(per_context_per_cc_event_waitlist[i][j][k], MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &exec_status, NULL);

                    if (api_err != MC_NO_ERROR) {
                        fprintf(stderr, "mcGetEventInfo(MC_EVENT_RUNTIME_EXECUTION_STATUS...) failed (err=%d)\n", (int)api_err);
                        exit(1);
                    }

                    if (exec_status != MC_NO_ERROR) {
                        fprintf(stderr, "schedule_cc_data_bytesize_query failed internally (err=%d)\n", (int)exec_status);
                        exit(1);
                    }
                }
            }
        }

        api_err = mcReleaseEvents((McUint32)data_query_events_array.size(), &data_query_events_array[0]);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)api_err);
            exit(1);
        }
    }

    // Write the connected components to file
    // -------------------------------------------------------------------------
    {
        Timer scoped_timer("save-files");

        for (uint32_t i = 0; i < num_contexts; ++i) {

            const uint32_t num_CCs = per_context_per_cc_vertices_array[i].size();

            for (uint32_t j = 0; j < num_CCs; ++j) {

                const std::vector<McDouble>& vertex_array = per_context_per_cc_vertices_array[i][j];
                const std::vector<McUint32>& triangles_array = per_context_per_cc_triangles_array[i][j];
                std::vector<McUint32> face_sizes(triangles_array.size() / 3, 3); // required by "writeOFF"

                char fnameBuf[256];
                sprintf(fnameBuf, "ctxt%d-cc%d.obj", i, j);

                writeOFF(fnameBuf,
                    vertex_array.data(),
                    (uint32_t*)triangles_array.data(),
                    (uint32_t*)face_sizes.data(),
                    (uint32_t)vertex_array.size() / 3,
                    (uint32_t)triangles_array.size() / 3);
            }
        }
    }

    // Tear down contexts
    // -------------------------------------------------------------------------

    {
        Timer scoped_timer("teardown");

        for (uint32_t i = 0; i < num_contexts; ++i) {

            McContext context = contexts_array[i];

            // destroy internal data associated with each connected component
            api_err = mcReleaseConnectedComponents(context, (uint32_t)per_context_connected_components[i].size(), per_context_connected_components[i].data());

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseEvents failed (err=%d)\n", (int)api_err);
                exit(1);
            }

            // destroy context
            api_err = mcReleaseContext(context);

            if (api_err != MC_NO_ERROR) {
                fprintf(stderr, "mcReleaseContext failed (err=%d)\n", (int)api_err);
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
    case MC_DEBUG_SOURCE_ALL:
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
        // printf("Type: Other");
        debug_type = "OTHER";
        break;
    case MC_DEBUG_TYPE_ALL:
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)

// https://stackoverflow.com/questions/735126/are-there-alternate-implementations-of-gnu-getline-interface/735472#735472
/* Modifications, public domain as well, by Antti Haapala, 11/10/17
- Switched to getc on 5/23/19 */
#include <errno.h>
#include <stdint.h>

// if typedef doesn't exist (msvc, blah)
typedef intptr_t ssize_t;

ssize_t getline(char** lineptr, size_t* n, FILE* stream)
{
    size_t pos;
    int c;

    if (lineptr == NULL || stream == NULL || n == NULL) {
        errno = EINVAL;
        return -1;
    }

    c = getc(stream);
    if (c == EOF) {
        return -1;
    }

    if (*lineptr == NULL) {
        *lineptr = (char*)malloc(128);
        if (*lineptr == NULL) {
            return -1;
        }
        *n = 128;
    }

    pos = 0;
    while (c != EOF) {
        if (pos + 1 >= *n) {
            size_t new_size = *n + (*n >> 2);
            if (new_size < 128) {
                new_size = 128;
            }
            char* new_ptr = (char*)realloc(*lineptr, new_size);
            if (new_ptr == NULL) {
                return -1;
            }
            *n = new_size;
            *lineptr = new_ptr;
        }

        ((unsigned char*)(*lineptr))[pos++] = (unsigned char)c;
        if (c == '\n') {
            break;
        }
        c = getc(stream);
    }

    (*lineptr)[pos] = '\0';
    return pos;
}
#endif // #if defined (_WIN32)

bool readLine(FILE* file, char** lineBuf, size_t* len)
{
    while (getline(lineBuf, len, file)) {
        if (strlen(*lineBuf) > 1 && (*lineBuf)[0] != '#') {
            return true;
        }
    }
    return false;
}

void readOFF(
    const char* fpath,
    double** pVertices,
    unsigned int** pFaceIndices,
    unsigned int** pFaceSizes,
    unsigned int* numVertices,
    unsigned int* numFaces)
{
    // using "rb" instead of "r" to prevent linefeed conversion
    // See: https://stackoverflow.com/questions/27530636/read-text-file-in-c-with-fopen-without-linefeed-conversion
    FILE* file = fopen(fpath, "rb");

    if (file == NULL) {
        fprintf(stderr, "error: failed to open `%s`", fpath);
        exit(1);
    }

    char* lineBuf = NULL;
    size_t lineBufLen = 0;
    bool lineOk = true;
    int i = 0;

    // file header
    lineOk = readLine(file, &lineBuf, &lineBufLen);

    if (!lineOk) {
        fprintf(stderr, "error: .off file header not found\n");
        exit(1);
    }

    if (strstr(lineBuf, "OFF") == NULL) {
        fprintf(stderr, "error: unrecognised .off file header\n");
        exit(1);
    }

    // #vertices, #faces, #edges
    lineOk = readLine(file, &lineBuf, &lineBufLen);

    if (!lineOk) {
        fprintf(stderr, "error: .off element count not found\n");
        exit(1);
    }

    int nedges = 0;
    sscanf(lineBuf, "%d %d %d", numVertices, numFaces, &nedges);
    *pVertices = (double*)malloc(sizeof(double) * (*numVertices) * 3);
    *pFaceSizes = (unsigned int*)malloc(sizeof(unsigned int) * (*numFaces));

    // vertices
    for (i = 0; i < (double)(*numVertices); ++i) {
        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off vertex not found\n");
            exit(1);
        }

        double x, y, z;
        sscanf(lineBuf, "%lf %lf %lf", &x, &y, &z);

        (*pVertices)[(i * 3) + 0] = x;
        (*pVertices)[(i * 3) + 1] = y;
        (*pVertices)[(i * 3) + 2] = z;
    }
#if _WIN64
    __int64 facesStartOffset = _ftelli64(file);
#else
    long int facesStartOffset = ftell(file);
#endif
    int numFaceIndices = 0;

    // faces
    for (i = 0; i < (int)(*numFaces); ++i) {
        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off file face not found\n");
            exit(1);
        }

        int n; // number of vertices in face
        sscanf(lineBuf, "%d", &n);

        if (n < 3) {
            fprintf(stderr, "error: invalid vertex count in file %d\n", n);
            exit(1);
        }

        (*pFaceSizes)[i] = n;
        numFaceIndices += n;
    }

    (*pFaceIndices) = (unsigned int*)malloc(sizeof(unsigned int) * numFaceIndices);

#if _WIN64
    int err = _fseeki64(file, facesStartOffset, SEEK_SET);
#else
    int err = fseek(file, facesStartOffset, SEEK_SET);
#endif
    if (err != 0) {
        fprintf(stderr, "error: fseek failed\n");
        exit(1);
    }

    int indexOffset = 0;
    for (i = 0; i < (int)(*numFaces); ++i) {

        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off file face not found\n");
            exit(1);
        }

        int n; // number of vertices in face
        sscanf(lineBuf, "%d", &n);

        char* lineBufShifted = lineBuf;
        int j = 0;

        while (j < n) { // parse remaining numbers on lineBuf
            lineBufShifted = strstr(lineBufShifted, " ") + 1; // start of next number

            int val;
            sscanf(lineBufShifted, "%d", &val);

            (*pFaceIndices)[indexOffset + j] = val;
            j++;
        }

        indexOffset += n;
    }

    free(lineBuf);

    fclose(file);
}

void writeOFF(
    const char* fpath,
    const double* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces)
{
    fprintf(stdout, "write: %s\n", fpath);

    FILE* file = fopen(fpath, "w");

    if (file == NULL) {
        fprintf(stderr, "error: failed to open `%s`", fpath);
        exit(1);
    }

    fprintf(file, "OFF\n");
    fprintf(file, "%d %d %d\n", numVertices, numFaces, 0 /*numEdges*/);
    int i;
    for (i = 0; i < (int)numVertices; ++i) {
        const double* vptr = pVertices + (i * 3);
        fprintf(file, "%f %f %f\n", vptr[0], vptr[1], vptr[2]);
    }

    int faceBaseOffset = 0;
    for (i = 0; i < (int)numFaces; ++i) {
        uint32_t faceVertexCount = pFaceSizes[i];
        fprintf(file, "%d", (int)faceVertexCount);
        int j;
        for (j = 0; j < (int)faceVertexCount; ++j) {
            const uint32_t* fptr = pFaceIndices + faceBaseOffset + j;
            fprintf(file, " %d", *fptr);
        }
        fprintf(file, "\n");
        faceBaseOffset += faceVertexCount;
    }

    fclose(file);
}