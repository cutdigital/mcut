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
#include <string>
#include <vector>

#include "off.h"

#ifdef _WIN32
#pragma warning(disable : 26812) // Unscoped enums from mcut.h
#endif // _WIN32

struct PolygonsWithHoles { 
    McContext context_ {};

    McFloat* pSrcMeshVertices {};
    McUint32* pSrcMeshFaceIndices {};
    McUint32* pSrcMeshFaceSizes {};
    McUint32 numSrcMeshVertices{};
    McUint32 numSrcMeshFaces {};

    McFloat* pCutMeshVertices {};
    McUint32* pCutMeshFaceIndices {};
    McUint32* pCutMeshFaceSizes {};
    McUint32 numCutMeshVertices {};
    McUint32 numCutMeshFaces {};
};

UTEST_F_SETUP(PolygonsWithHoles)
{
    McResult err = mcCreateContext(&utest_fixture->context_, 0);
    EXPECT_TRUE(utest_fixture->context_ != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    utest_fixture->pSrcMeshVertices = nullptr;
    utest_fixture->pSrcMeshFaceIndices = nullptr;
    utest_fixture->pSrcMeshFaceSizes = nullptr;
    utest_fixture->numSrcMeshVertices = 0;
    utest_fixture->numSrcMeshFaces = 0;

    utest_fixture->pCutMeshVertices = nullptr;
    utest_fixture->pCutMeshFaceIndices = nullptr;
    utest_fixture->pCutMeshFaceSizes = nullptr;
    utest_fixture->numCutMeshVertices = 0;
    utest_fixture->numCutMeshFaces = 0;
}

UTEST_F_TEARDOWN(PolygonsWithHoles)
{
    EXPECT_EQ(mcReleaseContext(utest_fixture->context_), MC_NO_ERROR);

    if (utest_fixture->pSrcMeshVertices)
        free(utest_fixture->pSrcMeshVertices);

    if (utest_fixture->pSrcMeshFaceIndices)
        free(utest_fixture->pSrcMeshFaceIndices);

    if (utest_fixture->pSrcMeshFaceSizes)
        free(utest_fixture->pSrcMeshFaceSizes);

    if (utest_fixture->pCutMeshVertices)
        free(utest_fixture->pCutMeshVertices);

    if (utest_fixture->pCutMeshFaceIndices)
        free(utest_fixture->pCutMeshFaceIndices);

    if (utest_fixture->pCutMeshFaceSizes)
        free(utest_fixture->pCutMeshFaceSizes);
}

UTEST_F(PolygonsWithHoles, outputWillHaveHoles)
{
    // partial cut intersection between a cube and a quad

    const std::string srcMeshPath = std::string(MESHES_DIR) + "/cube-flattened.off";

    readOFF(srcMeshPath.c_str(), &utest_fixture->pSrcMeshVertices, &utest_fixture->pSrcMeshFaceIndices, &utest_fixture->pSrcMeshFaceSizes, &utest_fixture->numSrcMeshVertices, &utest_fixture->numSrcMeshFaces);

    ASSERT_TRUE(utest_fixture->pSrcMeshVertices != nullptr);
    ASSERT_TRUE(utest_fixture->pSrcMeshFaceIndices != nullptr);
    ASSERT_TRUE(utest_fixture->pSrcMeshVertices != nullptr);
    ASSERT_GT((McInt32)utest_fixture->numSrcMeshVertices, 2);
    ASSERT_GT((McInt32)utest_fixture->numSrcMeshFaces, 0);

    const std::string cutMeshPath = std::string(MESHES_DIR) + "/cube-flattened-with-holes.off";

    readOFF(cutMeshPath.c_str(), &utest_fixture->pCutMeshVertices, &utest_fixture->pCutMeshFaceIndices, &utest_fixture->pCutMeshFaceSizes, &utest_fixture->numCutMeshVertices, &utest_fixture->numCutMeshFaces);
    ASSERT_TRUE(utest_fixture->pCutMeshVertices != nullptr);
    ASSERT_TRUE(utest_fixture->pCutMeshFaceIndices != nullptr);
    ASSERT_TRUE(utest_fixture->pCutMeshFaceSizes != nullptr);
    ASSERT_GT((McInt32)utest_fixture->numCutMeshVertices, 2);
    ASSERT_GT((McInt32)utest_fixture->numCutMeshFaces, 0);

    ASSERT_EQ(mcDispatch(
                  utest_fixture->context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                  utest_fixture->pSrcMeshVertices,
                  utest_fixture->pSrcMeshFaceIndices,
                  utest_fixture->pSrcMeshFaceSizes,
                  utest_fixture->numSrcMeshVertices,
                  utest_fixture->numSrcMeshFaces,
                  utest_fixture->pCutMeshVertices,
                  utest_fixture->pCutMeshFaceIndices,
                  utest_fixture->pCutMeshFaceSizes,
                  utest_fixture->numCutMeshVertices,
                  utest_fixture->numCutMeshFaces),
        MC_INVALID_OPERATION);
}
