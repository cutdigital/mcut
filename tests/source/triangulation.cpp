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
#include "off.h"
#include <vector>


struct Triangulation {
    McContext myContext = MC_NULL_HANDLE;
    std::vector<McConnectedComponent> pConnComps_;
    struct {
        std::vector<float> vertices;
        std::vector<uint32_t> faceIndices;
        std::vector<uint32_t> faceSizes;
    } srcMesh, cutMesh;
};

UTEST_F_SETUP(Triangulation)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_DEBUG), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);

        // config debug output
    // -----------------------
    uint64_t numBytes = 0;
    McFlags contextFlags;
    EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes), MC_NO_ERROR);
    

    EXPECT_EQ(sizeof(McFlags),numBytes);

    EXPECT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr), MC_NO_ERROR);

    if (contextFlags & MC_DEBUG) {
        mcDebugMessageCallback(utest_fixture->myContext, mcDebugOutput, nullptr);
        mcDebugMessageControl(utest_fixture->myContext, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);
    }
}

UTEST_F_TEARDOWN(Triangulation)
{
    EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (uint32_t)utest_fixture->pConnComps_.size(), utest_fixture->pConnComps_.data()), MC_NO_ERROR);
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
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (uint32_t)(utest_fixture->srcMesh.vertices.size() / 3), (uint32_t)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (uint32_t)(utest_fixture->cutMesh.vertices.size() / 3), (uint32_t)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (int c = 0; c < (int)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        uint64_t connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, uint64_t(0));
        ASSERT_GE(connCompVerticesBytes, uint64_t(sizeof(float) * 9)); // triangle
        const uint32_t numberOfVertices = (uint32_t)(connCompVerticesBytes / (sizeof(float) * 3));

        std::vector<float> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        uint64_t connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, uint64_t(0));
        ASSERT_GE(connCompTriIndicesBytes, uint64_t(sizeof(uint32_t) * 3)); // triangle
        std::vector<uint32_t> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(uint32_t));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (int v = 0; v < (int)triangleIndices.size(); ++v) {
            ASSERT_GE((uint32_t)triangleIndices[v], (uint32_t)0);
            ASSERT_LT((uint32_t)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
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
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (uint32_t)(utest_fixture->srcMesh.vertices.size() / 3), (uint32_t)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (uint32_t)(utest_fixture->cutMesh.vertices.size() / 3), (uint32_t)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (int c = 0; c < (int)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        uint64_t connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, uint64_t(0));
        ASSERT_GE(connCompVerticesBytes, uint64_t(sizeof(float) * 9)); // triangle
        const uint32_t numberOfVertices = (uint32_t)(connCompVerticesBytes / (sizeof(float) * 3));

        std::vector<float> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        uint64_t connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, uint64_t(0));
        ASSERT_GE(connCompTriIndicesBytes, uint64_t(sizeof(uint32_t) * 3)); // triangle
        std::vector<uint32_t> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(uint32_t));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (int v = 0; v < (int)triangleIndices.size(); ++v) {
            ASSERT_GE((uint32_t)triangleIndices[v], (uint32_t)0);
            ASSERT_LT((uint32_t)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
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
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (uint32_t)(utest_fixture->srcMesh.vertices.size() / 3), (uint32_t)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (uint32_t)(utest_fixture->cutMesh.vertices.size() / 3), (uint32_t)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (int c = 0; c < (int)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        uint64_t connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, uint64_t(0));
        ASSERT_GE(connCompVerticesBytes, uint64_t(sizeof(float) * 9)); // triangle
        const uint32_t numberOfVertices = (uint32_t)(connCompVerticesBytes / (sizeof(float) * 3));

        std::vector<float> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        uint64_t connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, uint64_t(0));
        ASSERT_GE(connCompTriIndicesBytes, uint64_t(sizeof(uint32_t) * 3)); // triangle
        std::vector<uint32_t> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(uint32_t));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (int v = 0; v < (int)triangleIndices.size(); ++v) {
            ASSERT_GE((uint32_t)triangleIndices[v], (uint32_t)0);
            ASSERT_LT((uint32_t)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
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
                  &utest_fixture->srcMesh.vertices[0], &utest_fixture->srcMesh.faceIndices[0], &utest_fixture->srcMesh.faceSizes[0], (uint32_t)(utest_fixture->srcMesh.vertices.size() / 3), (uint32_t)utest_fixture->srcMesh.faceSizes.size(), //
                  &utest_fixture->cutMesh.vertices[0], &utest_fixture->cutMesh.faceIndices[0], &utest_fixture->cutMesh.faceSizes[0], (uint32_t)(utest_fixture->cutMesh.vertices.size() / 3), (uint32_t)utest_fixture->cutMesh.faceSizes.size()),
        MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }
    std::vector<McConnectedComponent> connComps;
    connComps.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps.size(), connComps.data(), NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (int c = 0; c < (int)connComps.size(); ++c) {
        McConnectedComponent cc = connComps[c]; // connected compoenent id

        // vertex array
        uint64_t connCompVerticesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompVerticesBytes, uint64_t(0));
        ASSERT_GE(connCompVerticesBytes, uint64_t(sizeof(float) * 9)); // triangle
        const uint32_t numberOfVertices = (uint32_t)(connCompVerticesBytes / (sizeof(float) * 3));

        std::vector<float> vertices(numberOfVertices * 3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

        // triangle indices
        uint64_t connCompTriIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &connCompTriIndicesBytes), MC_NO_ERROR);
        ASSERT_GT(connCompTriIndicesBytes, uint64_t(0));
        ASSERT_GE(connCompTriIndicesBytes, uint64_t(sizeof(uint32_t) * 3)); // triangle
        std::vector<uint32_t> triangleIndices;
        triangleIndices.resize(connCompTriIndicesBytes / sizeof(uint32_t));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, connCompTriIndicesBytes, triangleIndices.data(), NULL), MC_NO_ERROR);

        for (int v = 0; v < (int)triangleIndices.size(); ++v) {
            ASSERT_GE((uint32_t)triangleIndices[v], (uint32_t)0);
            ASSERT_LT((uint32_t)triangleIndices[v], numberOfVertices); //  "out of bounds vertex index"
        }

        writeOBJ("tri-cc"+std::to_string(c) + ".obj", vertices, triangleIndices);
    }
}