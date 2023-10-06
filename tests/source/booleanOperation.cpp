/**
 * Copyright (c) 2021-2022 Floyd M. Chitalu.
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

struct BooleanOperation {
    McContext myContext = MC_NULL_HANDLE;
    std::vector<McConnectedComponent> pConnComps_;
    std::vector<float> srcMeshVertices;
    std::vector<uint32_t> meshFaceIndices;
    std::vector<uint32_t> meshFaceSizes;
};

static void MCAPI_PTR mcDebugOutput_(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{

    //printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
    //printf("userParam=%p\n", userParam);

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
        //printf("Type: Other");
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

UTEST_F_SETUP(BooleanOperation)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_DEBUG), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);

    // config debug output
    // -----------------------
    McSize numBytes = 0;
    McFlags contextFlags;
    EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes), MC_NO_ERROR);


    EXPECT_TRUE(sizeof(McFlags) == numBytes);

    EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr), MC_NO_ERROR);


    if (contextFlags & MC_DEBUG) {
        EXPECT_EQ(mcDebugMessageCallback(utest_fixture->myContext, mcDebugOutput_, nullptr), MC_NO_ERROR);
        EXPECT_EQ(mcDebugMessageControl(utest_fixture->myContext, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true), MC_NO_ERROR);
    }

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
    EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (uint32_t)utest_fixture->pConnComps_.size(), utest_fixture->pConnComps_.data()), MC_NO_ERROR);
    EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
}

// Performing a Boolean "union" operation with the same object, while forgetting to
// pass the appropriate MC_DISPATCH_ENFORCE_GENERAL_POSITION flag.
UTEST_F(BooleanOperation, selfUnionWithoutGeneralPositionEnforcement)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_INVALID_OPERATION);
}

// Performing a Boolean "union" operation with the same object, while allowing general
// position enforcement.
UTEST_F(BooleanOperation, selfUnionWithGeneralPositionEnforcement)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);
}

// Performing a Boolean "union" operation with the same object, while allowing general
// position enforcement (with MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE).
UTEST_F(BooleanOperation, selfUnionWithGeneralPositionEnforcement_abs)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags | MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);
}

// Performing a Boolean "diff(A,B)" operation.
UTEST_F(BooleanOperation, differenceA_Not_B)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<float> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (int i = 0; i < (int)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanA_Not_BFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanA_Not_BFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(cutMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(uint32_t(3), numConnComps);
}

UTEST_F(BooleanOperation, differenceB_Not_A)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<float> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (int i = 0; i < (int)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanB_Not_AFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanB_Not_AFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(cutMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(numConnComps, uint32_t(3));
}

UTEST_F(BooleanOperation, unionOp)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<float> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (int i = 0; i < (int)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanB_Not_AFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanB_Not_AFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(cutMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(uint32_t(3), numConnComps);
}

UTEST_F(BooleanOperation, intersectionOp)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;
    std::vector<float> cutMeshVertices = srcMeshVertices;

    // shifted so that the front-bottom-left vertex is located at (0,0,0) and the centre is at (1,1,1)
    for (int i = 0; i < (int)cutMeshVertices.size(); ++i) {
        cutMeshVertices[i] += 1.f;
    }

    const McFlags booleanB_Not_AFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanB_Not_AFlags,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &cutMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(cutMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    ASSERT_EQ(numConnComps, uint32_t(3));
}
