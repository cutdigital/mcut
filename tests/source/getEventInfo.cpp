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

#include "utest.h"
#include <mcut/mcut.h>
#include <vector>

struct GetEventInfo {
    McContext context_;

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

UTEST_F_SETUP(GetEventInfo)
{
    McResult err = mcCreateContext(&utest_fixture->context_, MC_PROFILING_ENABLE);
    ASSERT_TRUE(utest_fixture->context_ != nullptr);
    ASSERT_EQ(err, MC_NO_ERROR);

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

UTEST_F_TEARDOWN(GetEventInfo)
{
    McResult err = mcReleaseContext(utest_fixture->context_);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F(GetEventInfo, mcEnqueueDispatchAPI)
{
    McEvent dispatchEvent = MC_NULL_HANDLE;
    ASSERT_EQ(mcEnqueueDispatch(
                  utest_fixture->context_,
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

    size_t bytes = 0;
    {
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McFlags));

        McFlags dispatchEventStatus = MC_UNDEFINED_VALUE;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &dispatchEventStatus, NULL), MC_NO_ERROR);

        // can be any because task runs asynchronously
        ASSERT_TRUE(dispatchEventStatus == MC_QUEUED || //
            dispatchEventStatus == MC_SUBMITTED || //
            dispatchEventStatus == MC_RUNNING || //
            dispatchEventStatus == MC_COMPLETE);
    }

    //
    // wait until event is complete
    //
    ASSERT_EQ(mcWaitForEvents(1, &dispatchEvent), MC_NO_ERROR);

    {
        bytes = 0;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McResult));

        McResult dispatchEventRuntimeStatus = MC_NO_ERROR;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS, bytes, &dispatchEventRuntimeStatus, NULL), MC_NO_ERROR);

        ASSERT_TRUE(dispatchEventRuntimeStatus == MC_NO_ERROR); // verify
    }

    //
    // query profiling time info
    // -------------------------

    McSize submit = 0;
    {
        bytes = 0;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_SUBMIT, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McSize));

        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_SUBMIT, bytes, &submit, NULL), MC_NO_ERROR);

        ASSERT_TRUE(submit != 0);
    }

    McSize queued = 0;

    {
        bytes = 0;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_QUEUED, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McSize));

        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_QUEUED, bytes, &queued, NULL), MC_NO_ERROR);

        ASSERT_TRUE(queued != 0);
    }

    

    McSize start = 0;
    {
        bytes = 0;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_START, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McSize));

        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_START, bytes, &start, NULL), MC_NO_ERROR);

        ASSERT_TRUE(start != 0);
    }

    McSize end = 0;
    {
        bytes = 0;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_END, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McSize));

        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_TIMESTAMP_END, bytes, &end, NULL), MC_NO_ERROR);

        ASSERT_TRUE(end != 0);
    }

    ASSERT_LT(submit, queued );

    ASSERT_GT( queued -submit, (McSize)0);

    ASSERT_LT(queued, start);

    ASSERT_GT(start - queued, (McSize)0);

    ASSERT_LT(start, end);

    ASSERT_GT(end - start, (McSize)0);
}