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
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "utest.h"
#include <mcut/mcut.h>
#include <vector>

struct GetEventInfo {
    McContext context_ = MC_NULL_HANDLE;

    //
    // Same inputs as Hello World tutorial
    //

    std::vector<McDouble> cubeVertices = {};
	std::vector<McUint32> cubeFaces = {};
    McInt32 numCubeVertices=0;
    McInt32 numCubeFaces=0;

    std::vector<McUint32> cubeFaceSizes = {};

    // Cutting Shape:

    std::vector<McDouble> cutMeshVertices = {};

    std::vector<McUint32> cutMeshFaces = {};

    McUint32 numCutMeshVertices=0;
    McUint32 numCutMeshFaces=0;
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

    McSize bytes = 0;
    {
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, 0, NULL, &bytes), MC_NO_ERROR);
        ASSERT_EQ(bytes, sizeof(McFlags));

        McFlags dispatchEventStatus = MC_UNDEFINED_VALUE;
        ASSERT_EQ(mcGetEventInfo(dispatchEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &dispatchEventStatus, NULL), MC_NO_ERROR);

        // can be any because task runs asynchronously
        ASSERT_TRUE(
            dispatchEventStatus & MC_SUBMITTED || //
            dispatchEventStatus & MC_RUNNING || //
            dispatchEventStatus & MC_COMPLETE);
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

    ASSERT_LT(submit, start);

    ASSERT_GT(start - submit, (McSize)0);

    ASSERT_LT(start, end);

    ASSERT_GT(end - start, (McSize)0);

    ASSERT_EQ(mcReleaseEvents(1, &dispatchEvent), MC_NO_ERROR);
}