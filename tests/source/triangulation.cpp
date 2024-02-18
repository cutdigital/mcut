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


struct Triangulation {
    McContext myContext = MC_NULL_HANDLE;
    std::vector<McConnectedComponent> pConnComps_;
    struct {
        std::vector<McFloat> vertices;
        std::vector<McUint32> faceIndices;
        std::vector<McUint32> faceSizes;
    } srcMesh, cutMesh;
};

UTEST_F_SETUP(Triangulation)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_NULL_HANDLE), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);

    //    // config debug output
    //// -----------------------
    //McSize numBytes = 0;
    //McFlags contextFlags;
    //EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes), MC_NO_ERROR);
    //

    //EXPECT_EQ(sizeof(McFlags),numBytes);

    //EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr), MC_NO_ERROR);

    //if (contextFlags & MC_DEBUG) {
    //    mcDebugMessageCallback(utest_fixture->myContext, mcDebugOutput, nullptr);
    //    mcDebugMessageControl(utest_fixture->myContext, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);
    //}
}

UTEST_F_TEARDOWN(Triangulation)
{
    EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (McUint32)utest_fixture->pConnComps_.size(), utest_fixture->pConnComps_.data()), MC_NO_ERROR);
    EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
}

UTEST_F(Triangulation, oneEdgePartialCut)
{
    //
    // source mesh
    //
    utest_fixture->srcMesh.vertices = {
        .0f, .0f, .0f, // 0 (origin)
        1.0f, .0f, .0f, // 1 (right)
        .0f, 1.0f, .0f, // 2 (up)
    };
    utest_fixture->srcMesh.faceIndices = { 0, 1, 2 };
    utest_fixture->srcMesh.faceSizes = { 3 };

    //
    // cut mesh
    //
    utest_fixture->cutMesh.vertices = {
        .25f, .5f, .5f, // 0
        .25f, .5f, -.5f, // 1
        1.0f, .5f, .5f, // 2
    };
    utest_fixture->cutMesh.faceIndices = { 0, 1, 2 };
    utest_fixture->cutMesh.faceSizes = { 3 };

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (McUint32)(utest_fixture->srcMesh.vertices.size() / 3), (McUint32)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (McUint32)(utest_fixture->cutMesh.vertices.size() / 3), (McUint32)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (McInt32 c = 0; c < (McInt32)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        McSize connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, McSize(0));
        ASSERT_GE(connCompVerticesBytes, McSize(sizeof(McFloat) * 9)); // triangle
        const McUint32 numberOfVertices = (McUint32)(connCompVerticesBytes / (sizeof(McFloat) * 3));

        std::vector<McFloat> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        McSize connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, McSize(0));
        ASSERT_GE(connCompTriIndicesBytes, McSize(sizeof(McUint32) * 3)); // triangle
        std::vector<McUint32> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(McUint32));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (McInt32 v = 0; v < (McInt32)triangleIndices.size(); ++v) {
            ASSERT_GE((McUint32)triangleIndices[v], (McUint32)0);
            ASSERT_LT((McUint32)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        //writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
    }
}

UTEST_F(Triangulation, twoEdgePartialCut)
{
    //
    // source mesh
    //
    utest_fixture->srcMesh.vertices = {
        .0f, .0f, .0f, // 0 (origin)
        1.0f, .0f, .0f, // 1 (right)
        .0f, 1.0f, .0f, // 2 (up)
    };
    utest_fixture->srcMesh.faceIndices = { 0, 1, 2 };
    utest_fixture->srcMesh.faceSizes = { 3 };

    //
    // cut mesh
    //
    utest_fixture->cutMesh.vertices = {
        .25f, .5f, .5f, // 0
        .25f, .5f, -.5f, // 1
        1.0f, .5f, .5f, // 2
        // quad
        .25f, .25f, .5f, // 3
        .25f, .25f, -.5f // 4
    };
    utest_fixture->cutMesh.faceIndices = { 0, 2, 1,  0, 1, 4,3};
    utest_fixture->cutMesh.faceSizes = { 3, 4 };

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (McUint32)(utest_fixture->srcMesh.vertices.size() / 3), (McUint32)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (McUint32)(utest_fixture->cutMesh.vertices.size() / 3), (McUint32)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (McInt32 c = 0; c < (McInt32)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        McSize connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, McSize(0));
        ASSERT_GE(connCompVerticesBytes, McSize(sizeof(McFloat) * 9)); // triangle
        const McUint32 numberOfVertices = (McUint32)(connCompVerticesBytes / (sizeof(McFloat) * 3));

        std::vector<McFloat> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        McSize connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, McSize(0));
        ASSERT_GE(connCompTriIndicesBytes, McSize(sizeof(McUint32) * 3)); // triangle
        std::vector<McUint32> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(McUint32));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (McInt32 v = 0; v < (McInt32)triangleIndices.size(); ++v) {
            ASSERT_GE((McUint32)triangleIndices[v], (McUint32)0);
            ASSERT_LT((McUint32)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        //writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
    }
}

UTEST_F(Triangulation, threeEdgePartialCut)
{
    //
    // source mesh
    //
    utest_fixture->srcMesh.vertices = {
        .0f, .0f, .0f, // 0 (origin)
        1.0f, .0f, .0f, // 1 (right)
        .0f, 1.0f, .0f, // 2 (up)
    };
    utest_fixture->srcMesh.faceIndices = { 0, 1, 2 };
    utest_fixture->srcMesh.faceSizes = { 3 };

    //
    // cut mesh
    //
    utest_fixture->cutMesh.vertices = {
        .25f, .5f, .5f, // 0
        .25f, .5f, -.5f, // 1
        1.0f, .5f, .5f, // 2
        // quad
        .25f, .25f, .5f, // 3
        .25f, .25f, -.5f, // 4
        // quad
        .25f, .15f, .5f, // 5
        .25f, .15f, -.5f // 6
    };
    utest_fixture->cutMesh.faceIndices = { 
        0, 2, 1,  //
        0, 1, 4, 3, //
        3, 4, 6, 5
        };
    utest_fixture->cutMesh.faceSizes = { 3, 4, 4 };

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (McUint32)(utest_fixture->srcMesh.vertices.size() / 3), (McUint32)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (McUint32)(utest_fixture->cutMesh.vertices.size() / 3), (McUint32)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (McInt32 c = 0; c < (McInt32)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        McSize connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, McSize(0));
        ASSERT_GE(connCompVerticesBytes, McSize(sizeof(McFloat) * 9)); // triangle
        const McUint32 numberOfVertices = (McUint32)(connCompVerticesBytes / (sizeof(McFloat) * 3));

        std::vector<McFloat> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        McSize connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, McSize(0));
        ASSERT_GE(connCompTriIndicesBytes, McSize(sizeof(McUint32) * 3)); // triangle
        std::vector<McUint32> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(McUint32));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (McInt32 v = 0; v < (McInt32)triangleIndices.size(); ++v) {
            ASSERT_GE((McUint32)triangleIndices[v], (McUint32)0);
            ASSERT_LT((McUint32)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        //writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
    }
}

UTEST_F(Triangulation, threeEdgeVerticalPartialCut)
{
    //
    // source mesh
    //
    utest_fixture->srcMesh.vertices = {
        .0f, .0f, .0f, // 0 (origin)
        1.0f, .0f, .0f, // 1 (right)
        .0f, 1.0f, .0f, // 2 (up)
    };
    utest_fixture->srcMesh.faceIndices = { 0, 1, 2 };
    utest_fixture->srcMesh.faceSizes = { 3 };

    //
    // cut mesh
    //
    utest_fixture->cutMesh.vertices = {
        .15f, .5f, .5f, // 0
        .15f, .5f, -.5f, // 1
        .15f, 1.5f, .5f, // 2
        // quad
        .15f, .25f, .5f, // 3
        .15f, .25f, -.5f, // 4
        // quad
        .15f, .15f, .5f, // 5
        .15f, .15f, -.5f // 6
    };
    utest_fixture->cutMesh.faceIndices = { 
        0, 2, 1,  //
        0, 1, 4, 3, //
        3, 4, 6, 5
        };
    utest_fixture->cutMesh.faceSizes = { 3, 4, 4 };

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (McUint32)(utest_fixture->srcMesh.vertices.size() / 3), (McUint32)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (McUint32)(utest_fixture->cutMesh.vertices.size() / 3), (McUint32)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (McInt32 c = 0; c < (McInt32)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        McSize connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, McSize(0));
        ASSERT_GE(connCompVerticesBytes, McSize(sizeof(McFloat) * 9)); // triangle
        const McUint32 numberOfVertices = (McUint32)(connCompVerticesBytes / (sizeof(McFloat) * 3));

        std::vector<McFloat> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        McSize connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, McSize(0));
        ASSERT_GE(connCompTriIndicesBytes, McSize(sizeof(McUint32) * 3)); // triangle
        std::vector<McUint32> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(McUint32));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (McInt32 v = 0; v < (McInt32)triangleIndices.size(); ++v) {
            ASSERT_GE((McUint32)triangleIndices[v], (McUint32)0);
            ASSERT_LT((McUint32)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        //writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
    }
}