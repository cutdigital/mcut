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
    std::vector<McFloat> srcMeshVertices = {
        0.f, 0.f, 0.f,
        3.f, 0.f, 0.f,
        0.f, 3.f, 0.f
    };

    std::vector<McUint32> srcMeshFaceIndices = { 0, 1, 2 };
    McUint32 srcMeshFaceSizes = 3; // array of one

    std::vector<McFloat> cutMeshVertices = {
        0.f, 2.f, -1.f,
        3.f, 2.f, -1.f,
        0.f, 2.f, 2.f
    };

    std::vector<McUint32> cutMeshFaceIndices = { 0, 1, 2 };
    McUint32 cutMeshFaceSizes = 3; // array of one

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
    std::vector<McFloat> srcMeshVertices = {
        0.f, 0.f, 0.f,
        3.f, 0.f, 0.f,
        0.f, 3.f, 0.f
    };

    std::vector<McUint32> srcMeshFaceIndices = { 0, 1, 2 };
    McUint32 srcMeshFaceSizes = 3; // array of one

    std::vector<McFloat> cutMeshVertices = {
        1.f, 1.f, -3.f,
        3.f, 1.f, -3.f,
        1.f, 1.f, 0.f
    };

    std::vector<McUint32> cutMeshFaceIndices = { 0, 1, 2 };
    McUint32 cutMeshFaceSizes = 3; // array of one

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
    std::vector<McDouble> srcMeshVertices = {
        -1., -1., 0.,//
        1., -1., 0.,//
        1., 1., 0.,//
        -1., -1., 0.//
    };

    std::vector<McUint32> srcMeshFaceIndices = { 0, 1, 2, 0, 2, 3 };
    std::vector<McUint32>  srcMeshFaceSizes = {3, 3}; 

    std::vector<McDouble> cutMeshVertices = {
        -1., 0., 1.,//
        2., 0., 1.,//
        2., 0., -1.,//
        -1., 0., -1.,//
    };

    std::vector<McUint32> cutMeshFaceIndices = { 0, 1, 2 , 3};
    McUint32 cutMeshFaceSizes = 4; // array of one

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, MC_DISPATCH_VERTEX_ARRAY_FLOAT, //
                  &srcMeshVertices[0], &srcMeshFaceIndices[0], &srcMeshFaceSizes[0], 4, 2, //
                  &cutMeshVertices[0], &cutMeshFaceIndices[0], &cutMeshFaceSizes, 4, 1),
        MC_INVALID_OPERATION);
}
