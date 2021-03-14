#include "utest.h"
#include <mcut/mcut.h>

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// libigl dependencies
#include <Eigen/Core>
#include <igl/readOFF.h>

struct Benchmark {
    McContext myContext = MC_NULL_HANDLE;
};

UTEST_F_SETUP(Benchmark)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, MC_NULL_HANDLE), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);
}

UTEST_F_TEARDOWN(Benchmark)
{
    EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
}

struct InputMesh {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // variables for mesh data in a format suited for MCUT
    std::string fpath; // path to mesh file
    std::vector<uint32_t> faceSizesArray; // vertices per face
    std::vector<uint32_t> faceIndicesArray; // face indices
    std::vector<double> vertexCoordsArray; // vertex coords
};

void readOFF(InputMesh& srcMesh)
{
    bool srcMeshLoaded = igl::readOFF(srcMesh.fpath, srcMesh.V, srcMesh.F);

    if (!srcMeshLoaded) {
        std::fprintf(stderr, "error: could not load source mesh --> %s\n", srcMesh.fpath.c_str());
        std::exit(1);
    }

    // copy vertices
    for (int i = 0; i < srcMesh.V.rows(); ++i) {
        const Eigen::Vector3d& v = srcMesh.V.row(i);
        assert(v.size() == 3);
        srcMesh.vertexCoordsArray.push_back(v.x());
        srcMesh.vertexCoordsArray.push_back(v.y());
        srcMesh.vertexCoordsArray.push_back(v.z());
    }

    // copy faces
    for (int i = 0; i < srcMesh.F.rows(); ++i) {
        const Eigen::VectorXi& f = srcMesh.F.row(i);
        for (int j = 0; j < f.rows(); ++j) {
            srcMesh.faceIndicesArray.push_back(f[j]);
        }
        srcMesh.faceSizesArray.push_back(f.rows());
    }
}

#include <iostream>
UTEST_F(Benchmark, benchmarks)
{
    std::vector<std::pair<std::string, std::string>> benchmarkMeshPairs;

    const int NUMBER_OF_BENCHMARKS = 25; //

    for (int i = 0; i < NUMBER_OF_BENCHMARKS; ++i) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(3) << i;
        std::string s = ss.str();
        benchmarkMeshPairs.emplace_back("src-mesh" + s + ".off", "cut-mesh" + s + ".off");
    }

    for (auto& i : benchmarkMeshPairs) {
        const std::string srcMeshName = i.first;
        const std::string cutMeshName = i.second;

        std::cout << srcMeshName << " " << cutMeshName << std::endl;

        InputMesh srcMesh;
        srcMesh.fpath = std::string(MESHES_DIR) + "/benchmarks/" + srcMeshName;
        readOFF(srcMesh);

        ASSERT_GT((int)srcMesh.faceIndicesArray.size(), 2);
        ASSERT_GT((int)srcMesh.faceSizesArray.size(), 0);
        ASSERT_GT((int)srcMesh.vertexCoordsArray.size(), 3);

        InputMesh cutMesh;
        cutMesh.fpath = std::string(MESHES_DIR) + "/benchmarks/" + cutMeshName;
        readOFF(cutMesh);

        ASSERT_GT((int)cutMesh.faceIndicesArray.size(), 2);
        ASSERT_GT((int)cutMesh.faceSizesArray.size(), 0);
        ASSERT_GT((int)cutMesh.vertexCoordsArray.size(), 3);

        // do the cutting
        // --------------
        ASSERT_EQ(mcDispatch(
                      utest_fixture->myContext,
                      MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
                      &srcMesh.vertexCoordsArray[0],
                      &srcMesh.faceIndicesArray[0],
                      &srcMesh.faceSizesArray[0],
                      srcMesh.vertexCoordsArray.size() / 3,
                      srcMesh.faceSizesArray.size(),
                      &cutMesh.vertexCoordsArray[0],
                      &cutMesh.faceIndicesArray[0],
                      &cutMesh.faceSizesArray[0],
                      cutMesh.vertexCoordsArray.size() / 3,
                      cutMesh.faceSizesArray.size()),
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
        for (int i = 0; i < (int)connComps.size(); ++i) {
            McConnectedComponent cc = connComps[i]; // connected compoenent id

            uint64_t vertexCountBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &vertexCountBytes), MC_NO_ERROR);
            ASSERT_EQ(vertexCountBytes, sizeof(uint32_t));

            // we can also directly query the number of vertices since we know the number of bytes, which is a constant 4 bytes.
            uint32_t numberOfVertices = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, vertexCountBytes, &numberOfVertices, NULL), MC_NO_ERROR);
            ASSERT_GT((int)numberOfVertices, 0);

            // vertex array
            uint64_t connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
            ASSERT_GT(connCompVerticesBytes, 0);
            ASSERT_GE(connCompVerticesBytes, sizeof(float) * 9); // triangle
            std::vector<float> vertices;
            uint32_t nfloats = (uint32_t)(connCompVerticesBytes / sizeof(float));
            vertices.resize(nfloats);

            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

            ASSERT_EQ((uint64_t)numberOfVertices, (uint64_t)(connCompVerticesBytes / (sizeof(float) * 3)));

            // face indices
            uint64_t connCompFaceIndicesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &connCompFaceIndicesBytes), MC_NO_ERROR);
            ASSERT_GT(connCompFaceIndicesBytes, 0);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(uint32_t) * 3); // triangle
            std::vector<uint32_t> faceIndices;
            faceIndices.resize(connCompFaceIndicesBytes / sizeof(uint32_t));
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE, connCompFaceIndicesBytes, faceIndices.data(), NULL), MC_NO_ERROR);

            for (int i = 0; i < (int)faceIndices.size(); ++i) {
                ASSERT_GE((uint32_t)faceIndices[i], (uint32_t)0);
                ASSERT_LT((uint32_t)faceIndices[i], numberOfVertices); //  "out of bounds vertex index"
            }

            // face sizes
            uint64_t connCompFaceSizesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &connCompFaceSizesBytes), MC_NO_ERROR);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(uint32_t));
            std::vector<uint32_t> faceSizes;
            faceSizes.resize(connCompFaceSizesBytes / sizeof(uint32_t));

            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, connCompFaceSizesBytes, faceSizes.data(), NULL), MC_NO_ERROR);
            ASSERT_GT(faceSizes.size(), 0); //  "there has to be at least one face in a connected component"

            for (int i = 0; i < (int)faceSizes.size(); ++i) {
                ASSERT_GE(faceSizes[i], (uint32_t)3); // "3 is the minimum possible number of vertices in a polygon, which is a triangle"
            }

            // edge indices
            uint64_t connCompEdgesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, 0, NULL, &connCompEdgesBytes), MC_NO_ERROR);
            ASSERT_GE(connCompEdgesBytes, sizeof(uint32_t) * 6); // triangle

            std::vector<uint32_t> edgeIndices;
            edgeIndices.resize(connCompEdgesBytes / sizeof(uint32_t));
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, connCompEdgesBytes, edgeIndices.data(), NULL), MC_NO_ERROR);
            ASSERT_GE((uint32_t)edgeIndices.size(), (uint32_t)6); // "6 is the minimum number of indices in a triangle, which is the simplest polygon"

            EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (uint32_t)connComps.size(), connComps.data()), MC_NO_ERROR);
            connComps.clear();
        }
    }
}
