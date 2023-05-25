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

struct DegenerateInput {
    McContext myContext = MC_NULL_HANDLE;
};

UTEST_F_SETUP(DegenerateInput)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_NULL_HANDLE), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);
}

UTEST_F_TEARDOWN(DegenerateInput)
{
    EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
}

// An intersection between two triangles where one intersection point would be the result of
// two edges intersecting, which is not allowed (if perturbation is disabled).
UTEST_F(DegenerateInput, edgeEdgeIntersection)
{
    std::vector<float> srcMeshVertices = {
        0.f, 0.f, 0.f,
        3.f, 0.f, 0.f,
        0.f, 3.f, 0.f
    };

    std::vector<uint32_t> srcMeshFaceIndices = { 0, 1, 2 };
    uint32_t srcMeshFaceSizes = 3; // array of one

    std::vector<float> cutMeshVertices = {
        0.f, 2.f, -1.f,
        3.f, 2.f, -1.f,
        0.f, 2.f, 2.f
    };

    std::vector<uint32_t> cutMeshFaceIndices = { 0, 1, 2 };
    uint32_t cutMeshFaceSizes = 3; // array of one

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, MC_DISPATCH_VERTEX_ARRAY_FLOAT, //
                  &srcMeshVertices[0], &srcMeshFaceIndices[0], &srcMeshFaceSizes, 3, 1, //
                  &cutMeshVertices[0], &cutMeshFaceIndices[0], &cutMeshFaceSizes, 3, 1),
        MC_INVALID_OPERATION);
}

#if 0
// An intersection between two triangles where a vertex from the cut-mesh triangle
// lies on the src-mesh triangle. (This will only work with exact arith)
UTEST_F(DegenerateInput, faceVertexIntersection)
{
    std::vector<float> srcMeshVertices = {
        0.f, 0.f, 0.f,
        3.f, 0.f, 0.f,
        0.f, 3.f, 0.f
    };

    std::vector<uint32_t> srcMeshFaceIndices = { 0, 1, 2 };
    uint32_t srcMeshFaceSizes = 3; // array of one

    std::vector<float> cutMeshVertices = {
        1.f, 1.f, -3.f,
        3.f, 1.f, -3.f,
        1.f, 1.f, 0.f
    };

    std::vector<uint32_t> cutMeshFaceIndices = { 0, 1, 2 };
    uint32_t cutMeshFaceSizes = 3; // array of one

    // NOTE: this test will fail due to limitations in floating point precisions
    // It should pass for exact numbers build
    ASSERT_EQ(mcDispatch(utest_fixture->myContext, MC_DISPATCH_VERTEX_ARRAY_FLOAT, //
                  &srcMeshVertices[0], &srcMeshFaceIndices[0], &srcMeshFaceSizes, 3, 1, //
                  &cutMeshVertices[0], &cutMeshFaceIndices[0], &cutMeshFaceSizes, 3, 1),
        MC_INVALID_OPERATION);
}
#endif

// TODO: add vertex-edge and vertex-vertex intersection tests

UTEST_F(DegenerateInput, faceWithZeroArea)
{
    std::vector<float> srcMeshVertices = {
        -1.f, -1.f, 0.f,
        1.f, -1.f, 0.f,
        1.f, 1.f, 0.f,
        -1.f, -1.f, 0.f
    };

    std::vector<uint32_t> srcMeshFaceIndices = { 0, 1, 2, 0, 2, 3 };
    std::vector<uint32_t>  srcMeshFaceSizes = {3, 3}; 

    std::vector<float> cutMeshVertices = {
        -1.f, 0.f, 1.f,
        2.f, 0.f, 1.f,
        2.f, 0.f, -1.f,
        -1.f, 0.f, -1.f,
    };

    std::vector<uint32_t> cutMeshFaceIndices = { 0, 1, 2 , 3};
    uint32_t cutMeshFaceSizes = 4; // array of one

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, MC_DISPATCH_VERTEX_ARRAY_FLOAT, //
                  &srcMeshVertices[0], &srcMeshFaceIndices[0], &srcMeshFaceSizes[0], 3, 1, //
                  &cutMeshVertices[0], &cutMeshFaceIndices[0], &cutMeshFaceSizes, 3, 1),
        MC_INVALID_OPERATION);
}
