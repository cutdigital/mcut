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
#include <string>

struct BooleanOperation {
    McContext myContext = MC_NULL_HANDLE;
    std::vector<McConnectedComponent> pConnComps_;
    std::vector<McFloat> srcMeshVertices;
    std::vector<McUint32> meshFaceIndices;
    std::vector<McUint32> meshFaceSizes;
};

//static void MCAPI_PTR mcDebugOutput_(McDebugSource source,
//    McDebugType type,
//    McInt32 id,
//    McDebugSeverity severity,
//    size_t length,
//    const char* message,
//    const void* userParam)
//{
//
//    //printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
//    //printf("userParam=%p\n", userParam);
//
//    std::string debug_src;
//    switch (source) {
//    case MC_DEBUG_SOURCE_API:
//        debug_src = "API";
//        break;
//    case MC_DEBUG_SOURCE_KERNEL:
//        debug_src = "KERNEL";
//        break;
//    case MC_DEBUG_SOURCE_ALL:
//        break;
//    }
//    std::string debug_type;
//    switch (type) {
//    case MC_DEBUG_TYPE_ERROR:
//        debug_type = "ERROR";
//        break;
//    case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
//        debug_type = "DEPRECATION";
//        break;
//    case MC_DEBUG_TYPE_OTHER:
//        //printf("Type: Other");
//        debug_type = "OTHER";
//        break;
//    case MC_DEBUG_TYPE_ALL:
//        break;
//
//    }
//
//    std::string severity_str;
//
//    switch (severity) {
//    case MC_DEBUG_SEVERITY_HIGH:
//        severity_str = "HIGH";
//        break;
//    case MC_DEBUG_SEVERITY_MEDIUM:
//        severity_str = "MEDIUM";
//        break;
//    case MC_DEBUG_SEVERITY_LOW:
//        severity_str = "LOW";
//        break;
//    case MC_DEBUG_SEVERITY_NOTIFICATION:
//        severity_str = "NOTIFICATION";
//        break;
//    case MC_DEBUG_SEVERITY_ALL:
//        break;
//    }
//
//    printf("MCUT[%d:%p,%s:%s:%s:%zu] %s\n", id, userParam, debug_src.c_str(), debug_type.c_str(), severity_str.c_str(), length, message);
//}

UTEST_F_SETUP(BooleanOperation)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_NULL_HANDLE), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);

    //// config debug output
    //// -----------------------
    //McSize numBytes = 0;
    //McFlags contextFlags;
    //EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes), MC_NO_ERROR);


    //EXPECT_TRUE(sizeof(McFlags) == numBytes);

    //EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr), MC_NO_ERROR);


    //if (contextFlags & MC_DEBUG) {
    //    EXPECT_EQ(mcDebugMessageCallback(utest_fixture->myContext, mcDebugOutput_, nullptr), MC_NO_ERROR);
    //    EXPECT_EQ(mcDebugMessageControl(utest_fixture->myContext, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, MC_FALSE), MC_NO_ERROR);
    //}

    utest_fixture->srcMeshVertices = {
        -1.f, -1.f, 1.f, // 0
        1.f, -1.f, 1.f, // 1
        1.f, -1.f, -1.f, // 2
        -1.f, -1.f, -1.f, //3
        -1.f, 1.f, 1.f, //4
        1.f, 1.f, 1.f, //5
        1.f, 1.f, -1.f, //6
        -1.f, 1.f, -1.f //7
    };

    utest_fixture->meshFaceIndices = {
        3, 2, 1, 0, // bottom
        4, 5, 6, 7, //top
        0, 1, 5, 4, //front
        1, 2, 6, 5, // right
        2, 3, 7, 6, //back
        3, 0, 4, 7 // left
    };

    utest_fixture->meshFaceSizes = { 4, 4, 4, 4, 4, 4 };
}

UTEST_F_TEARDOWN(BooleanOperation)
{
    EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (McUint32)utest_fixture->pConnComps_.size(), utest_fixture->pConnComps_.data()), MC_NO_ERROR);
    EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
}

// Performing a Boolean "union" operation with the same object, while forgetting to
// pass the appropriate MC_DISPATCH_ENFORCE_GENERAL_POSITION flag.
UTEST_F(BooleanOperation, selfUnionWithoutGeneralPositionEnforcement)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_INVALID_OPERATION);
}

// Performing a Boolean "union" operation with the same object, while allowing general
// position enforcement.
UTEST_F(BooleanOperation, selfUnionWithGeneralPositionEnforcement)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_NO_ERROR);
}

// Performing a Boolean "union" operation with the same object, while allowing general
// position enforcement (with MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE).
UTEST_F(BooleanOperation, selfUnionWithGeneralPositionEnforcement_abs)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags | MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_NO_ERROR);
}

// Performing a Boolean "diff(A,B)" operation.
UTEST_F(BooleanOperation, differenceA_Not_B)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<McFloat> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (McInt32 i = 0; i < (McInt32)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanA_Not_BFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanA_Not_BFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(cutMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(McUint32(3), numConnComps);
}

UTEST_F(BooleanOperation, differenceB_Not_A)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<McFloat> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (McInt32 i = 0; i < (McInt32)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanB_Not_AFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanB_Not_AFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(cutMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(numConnComps, McUint32(3));
}

UTEST_F(BooleanOperation, unionOp)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<McFloat> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (McInt32 i = 0; i < (McInt32)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanB_Not_AFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanB_Not_AFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(cutMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(McUint32(3), numConnComps);
}

UTEST_F(BooleanOperation, intersectionOp)
{
    const std::vector<McFloat>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<McUint32>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<McUint32>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<McFloat> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (McInt32 i = 0; i < (McInt32)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanB_Not_AFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanB_Not_AFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(srcMeshVertices.size() / 3), (McUint32)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (McUint32)(cutMeshVertices.size() / 3), (McUint32)meshFaceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(numConnComps, McUint32(3));
}
