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

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS 1
#endif

#include "utest.h"
#include "mcut/mcut.h"
#include "mio/mio.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#define NUMBER_OF_BENCHMARKS 61 //

struct Benchmark {
    McContext myContext = MC_NULL_HANDLE;
    McInt32 benchmarkIndex = 0;

    MioMesh srcMesh = {
		nullptr, // pVertices
		nullptr, // pNormals
		nullptr, // pTexCoords
		nullptr, // pFaceSizes
		nullptr, // pFaceVertexIndices
		nullptr, // pFaceVertexTexCoordIndices
		nullptr, // pFaceVertexNormalIndices
		0, // numVertices
		0, // numNormals
		0, // numTexCoords
		0, // numFaces
	};

    MioMesh cutMesh = {
		nullptr, // pVertices
		nullptr, // pNormals
		nullptr, // pTexCoords
		nullptr, // pFaceSizes
		nullptr, // pFaceVertexIndices
		nullptr, // pFaceVertexTexCoordIndices
		nullptr, // pFaceVertexNormalIndices
		0, // numVertices
		0, // numNormals
		0, // numTexCoords
		0, // numFaces
	};
};

UTEST_I_SETUP(Benchmark)
{
    if (utest_index < NUMBER_OF_BENCHMARKS) {
        // create with no flags (default)
        EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_NULL_HANDLE), MC_NO_ERROR);
        EXPECT_TRUE(utest_fixture->myContext != nullptr);
        utest_fixture->benchmarkIndex = (McInt32)utest_index;
    }
}

UTEST_I_TEARDOWN(Benchmark)
{
    if (utest_index < NUMBER_OF_BENCHMARKS) {
        EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
    }

    mioFreeMesh(&utest_fixture->srcMesh);
    mioFreeMesh(&utest_fixture->cutMesh);
}

UTEST_I(Benchmark, inputID, NUMBER_OF_BENCHMARKS)
{
    std::vector<std::pair<std::string, std::string>> benchmarkMeshPairs;

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(3) << utest_fixture->benchmarkIndex;
    std::string s = ss.str();
    benchmarkMeshPairs.emplace_back("src-mesh" + s + ".off", "cut-mesh" + s + ".off");

    for (auto& i : benchmarkMeshPairs) {
        const std::string srcMeshName = i.first;
        const std::string cutMeshName = i.second;

        //
        // read-in the source-mesh from file
        //

        mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/" + srcMeshName).c_str(),
                    &utest_fixture->srcMesh.pVertices,
                    &utest_fixture->srcMesh.pFaceVertexIndices,
                    &utest_fixture->srcMesh.pFaceSizes,
                    &utest_fixture->srcMesh.numVertices,
                    &utest_fixture->srcMesh.numFaces);

            
        //
        // read-in the cut-mesh from file
        // NOTE: the icosphere (cut-mesh) provided lies inside the cube (source-mesh)
        //

        mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/" + cutMeshName).c_str(),
                    &utest_fixture->cutMesh.pVertices,
                    &utest_fixture->cutMesh.pFaceVertexIndices,
                    &utest_fixture->cutMesh.pFaceSizes,
                    &utest_fixture->cutMesh.numVertices,
                    &utest_fixture->cutMesh.numFaces);

        //
        // do the cutting
        // 
        ASSERT_EQ(mcDispatch(
                    utest_fixture->myContext,
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
                    // source mesh
                    utest_fixture->srcMesh.pVertices,
                    utest_fixture->srcMesh.pFaceVertexIndices,
                    utest_fixture->srcMesh.pFaceSizes,
                    utest_fixture->srcMesh.numVertices,
                    utest_fixture->srcMesh.numFaces,
                    // cut mesh
                    utest_fixture->cutMesh.pVertices,
                    utest_fixture->cutMesh.pFaceVertexIndices,
                    utest_fixture->cutMesh.pFaceSizes,
                    utest_fixture->cutMesh.numVertices,
                    utest_fixture->cutMesh.numFaces),
            MC_NO_ERROR);

        //
        // We no longer need the mem of input meshes, so we can free it!
        //
		mioFreeMesh(&utest_fixture->srcMesh);
		mioFreeMesh(&utest_fixture->cutMesh);
        
        McUint32 connectedComponentCount = 0;

        ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount), MC_NO_ERROR);

        if (connectedComponentCount == 0) {
            printf("no connected component found\n");
        }

        std::vector<McConnectedComponent> connectedComponents;
        connectedComponents.resize(connectedComponentCount);

        ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL), MC_NO_ERROR);

        //
        // query connected component data
        //
        for (McInt32 c = 0; c < (McInt32)connectedComponents.size(); ++c) {
            McConnectedComponent cc = connectedComponents[c]; // connected compoenent id

            // vertex array
            McSize connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
            ASSERT_GT(connCompVerticesBytes, McSize(0));
            ASSERT_GE(connCompVerticesBytes, McSize(sizeof(McFloat) * 9)); // triangle
            const McUint32 numberOfVertices = (McUint32)(connCompVerticesBytes / (sizeof(McFloat) * 3));

            std::vector<McFloat> vertices(numberOfVertices*3);
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

            // face indices
            McSize connCompFaceIndicesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &connCompFaceIndicesBytes), MC_NO_ERROR);
            ASSERT_GT(connCompFaceIndicesBytes, McSize(0));
            ASSERT_GE(connCompFaceIndicesBytes, McSize(sizeof(McUint32) * 3)); // triangle
            std::vector<McUint32> faceIndices;
            faceIndices.resize(connCompFaceIndicesBytes / sizeof(McUint32));
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE, connCompFaceIndicesBytes, faceIndices.data(), NULL), MC_NO_ERROR);

            for (McInt32 v = 0; v < (McInt32)faceIndices.size(); ++v) {
                ASSERT_GE((McUint32)faceIndices[v], (McUint32)0);
                ASSERT_LT((McUint32)faceIndices[v], numberOfVertices); //  "out of bounds vertex index"
            }

            // face sizes
            McSize connCompFaceSizesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &connCompFaceSizesBytes), MC_NO_ERROR);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(McUint32));
            std::vector<McUint32> faceSizes;
            faceSizes.resize(connCompFaceSizesBytes / sizeof(McUint32));

            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, connCompFaceSizesBytes, faceSizes.data(), NULL), MC_NO_ERROR);
            ASSERT_GT(faceSizes.size(), std::size_t(0)); //  "there has to be at least one face in a connected component"

            for (McInt32 v = 0; v < (McInt32)faceSizes.size(); ++v) {
                ASSERT_GE(faceSizes[v], (McUint32)3); // "3 is the minimum possible number of vertices in a polygon, which is a triangle"
            }

            // edge indices
            McSize connCompEdgesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, 0, NULL, &connCompEdgesBytes), MC_NO_ERROR);
            ASSERT_GE(connCompEdgesBytes, McSize(sizeof(McUint32) * 6)); // triangle

            std::vector<McUint32> edgeIndices;
            edgeIndices.resize(connCompEdgesBytes / sizeof(McUint32));
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, connCompEdgesBytes, edgeIndices.data(), NULL), MC_NO_ERROR);
            ASSERT_GE((McUint32)edgeIndices.size(), (McUint32)6); // "6 is the minimum number of indices in a triangle, which is the simplest polygon"

            EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (McUint32)connectedComponents.size(), connectedComponents.data()), MC_NO_ERROR);
            connectedComponents.clear();
        }
    }
}
