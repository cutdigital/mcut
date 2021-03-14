
#include "mcut/mcut.h"
#include "off.h"
#include "gtest/gtest.h"
#include <cstring>

#if defined(_WIN32)

#define _CRT_SECURE_NO_WARNINGS

#endif

namespace {

class BenchmarkInputMeshTest : public testing::Test {
protected:
    void SetUp() override
    {
        // create with no flags (default)
        EXPECT_EQ(mcCreateContext(&context_, MC_DEBUG), MC_NO_ERROR);
        EXPECT_TRUE(context_ != nullptr);
    }

    void runMyTest(const std::string& srcMeshName, const std::string& cutMeshName)
    {
        srcMeshPath_ = std::string(ROOT_DIR) + "/meshes/benchmarks/" + srcMeshName;
        float* pSrcMeshVertices = nullptr;
        uint32_t* pSrcMeshFaceIndices = nullptr;
        uint32_t* pSrcMeshFaceSizes = nullptr;
        uint32_t numSrcMeshVertices = 0;
        uint32_t numSrcMeshFaces = 0;

        readOFF(srcMeshPath_.c_str(), &pSrcMeshVertices, &pSrcMeshFaceIndices, &pSrcMeshFaceSizes, &numSrcMeshVertices, &numSrcMeshFaces);

        ASSERT_TRUE(pSrcMeshVertices != nullptr);
        ASSERT_TRUE(pSrcMeshFaceIndices != nullptr);
        ASSERT_TRUE(pSrcMeshVertices != nullptr);
        ASSERT_GT((int)numSrcMeshVertices, 2);
        ASSERT_GT((int)numSrcMeshFaces, 0);

        cutMeshPath_ = std::string(ROOT_DIR) + "/meshes/benchmarks/" + cutMeshName;
        float* pCutMeshVertices = nullptr;
        uint32_t* pCutMeshFaceIndices = nullptr;
        uint32_t* pCutMeshFaceSizes = nullptr;
        uint32_t numCutMeshVertices = 0;
        uint32_t numCutMeshFaces = 0;

        readOFF(cutMeshPath_.c_str(), &pCutMeshVertices, &pCutMeshFaceIndices, &pCutMeshFaceSizes, &numCutMeshVertices, &numCutMeshFaces);

        ASSERT_TRUE(pCutMeshVertices != nullptr);
        ASSERT_TRUE(pCutMeshFaceIndices != nullptr);
        ASSERT_TRUE(pCutMeshFaceSizes != nullptr);
        ASSERT_GT((int)numCutMeshVertices, 2);
        ASSERT_GT((int)numCutMeshFaces, 0);

        // do the cutting
        // --------------
        ASSERT_EQ(mcDispatch(
                      context_,
                      MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                      pSrcMeshVertices,
                      pSrcMeshFaceIndices,
                      pSrcMeshFaceSizes,
                      numSrcMeshVertices,
                      numSrcMeshFaces,
                      pCutMeshVertices,
                      pCutMeshFaceIndices,
                      pCutMeshFaceSizes,
                      numCutMeshVertices,
                      numCutMeshFaces),
            MC_NO_ERROR);

        uint32_t numConnComps = 0;

        ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

        if (numConnComps == 0) {
            printf("no connected component found\n");
        }

        pConnComps_.resize(numConnComps);

        ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)pConnComps_.size(), pConnComps_.data(), NULL), MC_NO_ERROR);

        //
        // query connected component data
        //
        for (int i = 0; i < (int)pConnComps_.size(); ++i) {
            McConnectedComponent cc = pConnComps_[i]; // connected compoenent id

            uint64_t vertexCountBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &vertexCountBytes), MC_NO_ERROR);
            ASSERT_EQ(vertexCountBytes, sizeof(uint32_t));

            // we can also directly query the number of vertices since we know the number of bytes, which is a constant 4 bytes.
            uint32_t numberOfVertices = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, vertexCountBytes, &numberOfVertices, NULL), MC_NO_ERROR);
            ASSERT_GT((int)numberOfVertices, 0);

            // vertex array
            uint64_t connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes), MC_NO_ERROR);
            ASSERT_GT(connCompVerticesBytes, 0);
            ASSERT_GE(connCompVerticesBytes, sizeof(float) * 9); // triangle
            std::vector<float> vertices;
            uint32_t nfloats = (uint32_t)(connCompVerticesBytes / sizeof(float));
            vertices.resize(nfloats);

            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL), MC_NO_ERROR);

            ASSERT_EQ((uint64_t)numberOfVertices, (uint64_t)(connCompVerticesBytes / (sizeof(float) * 3)));

            // face indices
            uint64_t connCompFaceIndicesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &connCompFaceIndicesBytes), MC_NO_ERROR);
            ASSERT_GT(connCompFaceIndicesBytes, 0);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(uint32_t) * 3); // triangle
            std::vector<uint32_t> faceIndices;
            faceIndices.resize(connCompFaceIndicesBytes / sizeof(uint32_t));
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE, connCompFaceIndicesBytes, faceIndices.data(), NULL), MC_NO_ERROR);

            for (int i = 0; i < (int)faceIndices.size(); ++i) {
                ASSERT_GE((uint32_t)faceIndices[i], (uint32_t)0);
                ASSERT_LT((uint32_t)faceIndices[i], numberOfVertices) << "out of bounds vertex index";
            }

            // face sizes
            uint64_t connCompFaceSizesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &connCompFaceSizesBytes), MC_NO_ERROR);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(uint32_t));
            std::vector<uint32_t> faceSizes;
            faceSizes.resize(connCompFaceSizesBytes / sizeof(uint32_t));

            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, connCompFaceSizesBytes, faceSizes.data(), NULL), MC_NO_ERROR);
            ASSERT_GT(faceSizes.size(), 0) << "there has to be at least one face in a connected component";

            for (int i = 0; i < (int)faceSizes.size(); ++i) {
                ASSERT_GE(faceSizes[i], (uint32_t)3) << "3 is the minimum possible number of vertices in a polygon, which is a triangle";
            }

            // edge indices
            uint64_t connCompEdgesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, 0, NULL, &connCompEdgesBytes), MC_NO_ERROR);
            ASSERT_GE(connCompEdgesBytes, sizeof(uint32_t) * 6); // triangle

            std::vector<uint32_t> edgeIndices;
            edgeIndices.resize(connCompEdgesBytes / sizeof(uint32_t));
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, connCompEdgesBytes, edgeIndices.data(), NULL), MC_NO_ERROR);
            ASSERT_GE((uint32_t)edgeIndices.size(), (uint32_t)6) << "6 is the minimum number of indices in a triangle, which is the simplest polygon";
        }
    }

    virtual void TearDown()
    {
        EXPECT_EQ(mcReleaseConnectedComponents(context_, (uint32_t)pConnComps_.size(), pConnComps_.data()), MC_NO_ERROR);
        EXPECT_EQ(mcReleaseContext(context_), MC_NO_ERROR);
    }

    McContext context_ = MC_NULL_HANDLE;
    std::string srcMeshPath_;
    std::string cutMeshPath_;
    std::vector<McConnectedComponent> pConnComps_;
};

#define ADD_BENCHMARK_TEST(testnum_)                                         \
    TEST_F(BenchmarkInputMeshTest, testnum_)                                 \
    {                                                                        \
        runMyTest("src-mesh" #testnum_ ".off", "cut-mesh" #testnum_ ".off"); \
    }

ADD_BENCHMARK_TEST(000)
ADD_BENCHMARK_TEST(001)
ADD_BENCHMARK_TEST(002)
ADD_BENCHMARK_TEST(003)
ADD_BENCHMARK_TEST(004)
ADD_BENCHMARK_TEST(005)
ADD_BENCHMARK_TEST(006)
ADD_BENCHMARK_TEST(007)
ADD_BENCHMARK_TEST(008)
ADD_BENCHMARK_TEST(009)
ADD_BENCHMARK_TEST(010)
ADD_BENCHMARK_TEST(011)
ADD_BENCHMARK_TEST(012)
ADD_BENCHMARK_TEST(013)
ADD_BENCHMARK_TEST(014)
ADD_BENCHMARK_TEST(015)
ADD_BENCHMARK_TEST(016)
ADD_BENCHMARK_TEST(017)
ADD_BENCHMARK_TEST(018)
ADD_BENCHMARK_TEST(019)
ADD_BENCHMARK_TEST(020)
ADD_BENCHMARK_TEST(021)
ADD_BENCHMARK_TEST(022)
ADD_BENCHMARK_TEST(023)
ADD_BENCHMARK_TEST(024)

} // namespace {
