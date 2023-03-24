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

/*
This test creates multiple contexts that run in parallel.

There are as many contexts as half the number of hardware threads.

Each context is run in its own thread.
*/

#include "utest.h"
#include <future>
#include <mcut/mcut.h>
#include <thread>
#include <vector>
struct ConcurrentSynchronizedContexts {
    std::vector<McContext> contexts;

    //
    // Same inputs as Hello World tutorial
    //

    std::vector<double> cubeVertices;
    std::vector<uint32_t> cubeFaces;
    int numCubeVertices;
    int numCubeFaces;

    std::vector<uint32_t> cubeFaceSizes;

    // Cutting Shape:

    std::vector<double> cutMeshVertices;

    std::vector<uint32_t> cutMeshFaces;

    uint32_t numCutMeshVertices;
    uint32_t numCutMeshFaces;
};

UTEST_F_SETUP(ConcurrentSynchronizedContexts)
{
    utest_fixture->contexts.resize(std::thread::hardware_concurrency() / 2);

    // create the context objects
    for (int i = 0; i < (int)utest_fixture->contexts.size(); ++i) {
        utest_fixture->contexts[i] = MC_NULL_HANDLE;
        uint32_t helpers = ((i % 2 == 0) ? 1 : 0);
        ASSERT_EQ(mcCreateContextWithHelpers(&utest_fixture->contexts[i], MC_OUT_OF_ORDER_EXEC_MODE_ENABLE, helpers), MC_NO_ERROR);
        ASSERT_TRUE(utest_fixture->contexts[i] != nullptr);
    }

    utest_fixture->cubeVertices = {
        -1, -1, 1, // 0
        1, -1, 1, // 1
        -1, 1, 1, // 2
        1, 1, 1, // 3
        -1, -1, -1, // 4
        1, -1, -1, // 5
        -1, 1, -1, // 6
        1, 1, -1 // 7
    };
    utest_fixture->cubeFaces = {
        0, 3, 2, // 0
        0, 1, 3, // 1
        1, 7, 3, // 2
        1, 5, 7, // 3
        5, 6, 7, // 4
        5, 4, 6, // 5
        4, 2, 6, // 6
        4, 0, 2, // 7
        2, 7, 6, // 8
        2, 3, 7, // 9
        4, 1, 0, // 10
        4, 5, 1, // 11

    };
    utest_fixture->numCubeVertices = 8;
    utest_fixture->numCubeFaces = 12;

    utest_fixture->cubeFaceSizes = {
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
    };

    // Cutting Shape:

    utest_fixture->cutMeshVertices = {
        -1.2, 1.6, 0.994070,
        1.4, -1.3, 0.994070,
        -1.2, 1.6, -1.005929,
        1.4, -1.3, -1.005929
    };

    utest_fixture->cutMeshFaces = {
        1, 2, 0,
        1, 3, 2
    };

    utest_fixture->numCutMeshVertices = 4;
    utest_fixture->numCutMeshFaces = 2;
}

UTEST_F_TEARDOWN(ConcurrentSynchronizedContexts)
{
    for (int i = 0; i < (int)utest_fixture->contexts.size(); ++i) {
        EXPECT_EQ(mcReleaseContext(utest_fixture->contexts[i]), MC_NO_ERROR);
    }
}

UTEST_F(ConcurrentSynchronizedContexts, sequentialDispatchCalls)
{
    for (int i = 0; i < (int)utest_fixture->contexts.size(); ++i) {
        McEvent dispatchEvent = MC_NULL_HANDLE;
        ASSERT_EQ(mcEnqueueDispatch(
                      utest_fixture->contexts[i],
                      MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
                      utest_fixture->cubeVertices.data(),
                      utest_fixture->cubeFaces.data(),
                      utest_fixture->cubeFaceSizes.data(),
                      utest_fixture->numCubeVertices,
                      utest_fixture->numCubeFaces,
                      utest_fixture->cutMeshVertices.data(),
                      utest_fixture->cutMeshFaces.data(),
                      nullptr, // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
                      utest_fixture->numCutMeshVertices,
                      utest_fixture->numCutMeshFaces,
                      0, NULL, &dispatchEvent),
            MC_NO_ERROR);

        ASSERT_TRUE(dispatchEvent != MC_NULL_HANDLE);

        //
        // wait until event is complete
        //
        ASSERT_EQ(mcWaitForEvents(1, &dispatchEvent), MC_NO_ERROR);

        size_t bytes = 0;

        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McEventCommandExecStatus));

        McEventCommandExecStatus dispatchEventStatus = (McEventCommandExecStatus)MC_UNDEFINED_VALUE;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &dispatchEventStatus, NULL), MC_NO_ERROR);

        ASSERT_TRUE(dispatchEventStatus == McEventCommandExecStatus::MC_COMPLETE);

        ASSERT_EQ(mcReleaseEvents(1, &dispatchEvent), MC_NO_ERROR);
    }
}

UTEST_F(ConcurrentSynchronizedContexts, parallelButUnsynchronisedDispatchCalls)
{
    // NOTE: the input meshes being cut are the same per context (keeping things simple)
    auto myDispatchWrapperFunc = [&](McContext threadSpecificContext) {
        McEvent dispatchEvent = MC_NULL_HANDLE;
        ASSERT_EQ(mcEnqueueDispatch(
                      threadSpecificContext,
                      MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
                      utest_fixture->cubeVertices.data(),
                      utest_fixture->cubeFaces.data(),
                      utest_fixture->cubeFaceSizes.data(),
                      utest_fixture->numCubeVertices,
                      utest_fixture->numCubeFaces,
                      utest_fixture->cutMeshVertices.data(),
                      utest_fixture->cutMeshFaces.data(),
                      nullptr, // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
                      utest_fixture->numCutMeshVertices,
                      utest_fixture->numCutMeshFaces,
                      0, NULL, &dispatchEvent),
            MC_NO_ERROR);

        ASSERT_TRUE(dispatchEvent != MC_NULL_HANDLE);

        //
        // wait until event is complete
        //
        ASSERT_EQ(mcWaitForEvents(1, &dispatchEvent), MC_NO_ERROR);

        size_t bytes = 0;

        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McEventCommandExecStatus));

        McEventCommandExecStatus dispatchEventStatus = (McEventCommandExecStatus)MC_UNDEFINED_VALUE;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &dispatchEventStatus, NULL), MC_NO_ERROR);

        {
            if (dispatchEventStatus != McEventCommandExecStatus::MC_COMPLETE)
            {
                ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &dispatchEventStatus, NULL), MC_NO_ERROR);
            }
        }
        ASSERT_EQ(dispatchEventStatus, McEventCommandExecStatus::MC_COMPLETE);

        ASSERT_EQ(mcReleaseEvents(1, &dispatchEvent), MC_NO_ERROR);
    };

    std::vector<std::future<void>> futures(utest_fixture->contexts.size());

    // run multiple dispatch calls in parallel!
    for (int i = 0; i < (int)utest_fixture->contexts.size(); ++i) {
        futures[i] = std::async(std::launch::async, myDispatchWrapperFunc, utest_fixture->contexts[i]);
    }

    // wait for our dispatch call to finish
    for (int i = 0; i < (int)utest_fixture->contexts.size(); ++i) {
        futures[i].wait();
    }
}