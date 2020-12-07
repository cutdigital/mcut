
#include "off.h"
#include "mcut/mcut.h"
#include "gtest/gtest.h"
#include <cstring>

namespace {

class BenchmarkInputMeshTest : public testing::Test {
protected:
    void SetUp() override
    {
        // create with no flags (default)
        EXPECT_EQ(mcCreateContext(&context_, 0), MC_NO_ERROR);
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
                      numCutMeshFaces,
                      0, NULL, NULL),
            MC_NO_ERROR);

        uint32_t numConnComps = 0;

        ASSERT_EQ(mcGetConnectedComponents(context_, MC_TRUE, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps, 0, NULL, NULL), MC_NO_ERROR);

        if (numConnComps == 0) {
            printf("no connected component found\n");
        }

        pConnComps_.resize(numConnComps);

        ASSERT_EQ(mcGetConnectedComponents(context_, MC_TRUE, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)pConnComps_.size(), pConnComps_.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

        //
        // query connected component data
        //
        for (int i = 0; i < (int)pConnComps_.size(); ++i) {
            McConnectedComponent cc = pConnComps_[i]; // connected compoenent id

            uint64_t vertexCountBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &vertexCountBytes, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_EQ(vertexCountBytes, sizeof(uint32_t));

            // we can also directly query the number of vertices since we know the number of bytes, which is a constant 4 bytes.
            uint32_t numberOfVertices = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, vertexCountBytes, &numberOfVertices, NULL, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_GT((int)numberOfVertices, 0);

            // vertex array
            uint64_t connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_GT(connCompVerticesBytes, 0);
            ASSERT_GE(connCompVerticesBytes, sizeof(float) * 9); // triangle
            std::vector<float> vertices;
            uint32_t nfloats = (uint32_t)(connCompVerticesBytes / sizeof(float));
            vertices.resize(nfloats);

            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

            ASSERT_EQ((uint64_t)numberOfVertices, (uint64_t)(connCompVerticesBytes / (sizeof(float) * 3)));

            // face indices
            uint64_t connCompFaceIndicesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &connCompFaceIndicesBytes, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_GT(connCompFaceIndicesBytes, 0);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(uint32_t) * 3); // triangle
            std::vector<uint32_t> faceIndices;
            faceIndices.resize(connCompFaceIndicesBytes / sizeof(uint32_t));
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_FACE, connCompFaceIndicesBytes, faceIndices.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

            for (int i = 0; i < (int)faceIndices.size(); ++i) {
                ASSERT_GE((uint32_t)faceIndices[i], (uint32_t)0);
                ASSERT_LT((uint32_t)faceIndices[i], numberOfVertices) << "out of bounds vertex index";
            }

            // face sizes
            uint64_t connCompFaceSizesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &connCompFaceSizesBytes, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_GE(connCompFaceIndicesBytes, sizeof(uint32_t));
            std::vector<uint32_t> faceSizes;
            faceSizes.resize(connCompFaceSizesBytes / sizeof(uint32_t));

            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, connCompFaceSizesBytes, faceSizes.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_GT(faceSizes.size(), 0) << "there has to be at least one face in a connected component";

            for (int i = 0; i < (int)faceSizes.size(); ++i) {
                ASSERT_GE(faceSizes[i], (uint32_t)3) << "3 is the minimum possible number of vertices in a polygon, which is a triangle";
            }

            // edge indices
            uint64_t connCompEdgesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, 0, NULL, &connCompEdgesBytes, 0, NULL, NULL), MC_NO_ERROR);
            ASSERT_GE(connCompEdgesBytes, sizeof(uint32_t) * 6); // triangle

            std::vector<uint32_t> edgeIndices;
            edgeIndices.resize(connCompEdgesBytes / sizeof(uint32_t));
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_EDGE, connCompEdgesBytes, edgeIndices.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);
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

class ExactCoordsInputMeshTest : public testing::Test {
protected:
    void SetUp() override
    {
        // create with no flags (default)
        EXPECT_EQ(mcCreateContext(&context_, 0), MC_NO_ERROR);
        EXPECT_TRUE(context_ != nullptr);
    }

    virtual void TearDown()
    {
        EXPECT_EQ(mcReleaseConnectedComponents(context_, (uint32_t)pConnComps_.size(), pConnComps_.data()), MC_NO_ERROR);
        EXPECT_EQ(mcReleaseContext(context_), MC_NO_ERROR);
    }

    McContext context_ = MC_NULL_HANDLE;
    std::vector<McConnectedComponent> pConnComps_;
};

TEST_F(ExactCoordsInputMeshTest, dispatchExactCoords)
{
    const char pSrcMeshVertices[] = {
        "0 0 0\n"
        "0 10 0\n"
        "10 0 0"
    }; // exact numbers (x y z\nx y z\n x, .....)
    uint32_t pSrcMeshFaceIndices[] = { 0, 2, 1 };
    uint32_t pSrcMeshFaceSizes[] = { 3 };
    uint32_t numSrcMeshVertices = 3;
    uint32_t numSrcMeshFaces = 1;

    ASSERT_TRUE(pSrcMeshVertices != nullptr);
    ASSERT_TRUE(pSrcMeshFaceIndices != nullptr);
    ASSERT_TRUE(pSrcMeshVertices != nullptr);
    ASSERT_GT((int)numSrcMeshVertices, 2);
    ASSERT_GT((int)numSrcMeshFaces, 0);

    const char pCutMeshVertices[] = {
        "-2.5 7.5 2.5\n"
        "-2.5 7.5 -7.5\n"
        "7.5 7.5 2.5"
    }; // exact numbers ...
    uint32_t pCutMeshFaceIndices[] = { 0, 2, 1 };
    uint32_t pCutMeshFaceSizes[] = { 3 };
    uint32_t numCutMeshVertices = 3;
    uint32_t numCutMeshFaces = 1;

    ASSERT_TRUE(pCutMeshVertices != nullptr);
    ASSERT_TRUE(pCutMeshFaceIndices != nullptr);
    ASSERT_TRUE(pCutMeshFaceSizes != nullptr);
    ASSERT_GT((int)numCutMeshVertices, 2);
    ASSERT_GT((int)numCutMeshFaces, 0);

    // do the cutting
    // --------------
    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_EXACT, // vertex array is a string of numbers
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces,
                  0, NULL, NULL),
        MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_TRUE, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps, 0, NULL, NULL), MC_NO_ERROR);

    if (numConnComps == 0) {
        printf("no connected component found\n");
    }

    pConnComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_TRUE, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)pConnComps_.size(), pConnComps_.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

    //
    // query connected component data
    //
    for (int i = 0; i < (int)pConnComps_.size(); ++i) {
        McConnectedComponent cc = pConnComps_[i]; // connected compoenent id

        uint64_t vertexCountBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &vertexCountBytes, 0, NULL, NULL), MC_NO_ERROR);
        ASSERT_EQ(vertexCountBytes, sizeof(uint32_t));

        // we can also directly query the number of vertices since we know the number of bytes, which is a constant 4 bytes.
        uint32_t numberOfVertices = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, vertexCountBytes, &numberOfVertices, NULL, 0, NULL, NULL), MC_NO_ERROR);
        ASSERT_GT((int)numberOfVertices, 0);

        // float
        // -----
        {
            // vertex array
            uint64_t connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &connCompVerticesBytes, 0, NULL, NULL), MC_NO_ERROR);

            std::vector<float> vertices;
            uint32_t nfloats = (uint32_t)(connCompVerticesBytes / sizeof(float));
            vertices.resize(nfloats);

            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, connCompVerticesBytes, (void*)vertices.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

            ASSERT_EQ((uint64_t)numberOfVertices, (uint64_t)(connCompVerticesBytes / (sizeof(float) * 3)));
        }

        // double
        // -----
        {
            // vertex array
            uint64_t connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &connCompVerticesBytes, 0, NULL, NULL), MC_NO_ERROR);

            std::vector<double> vertices;
            uint32_t ndoubles = (uint32_t)(connCompVerticesBytes / sizeof(double));
            vertices.resize(ndoubles);

            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, connCompVerticesBytes, (void*)vertices.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

            ASSERT_EQ((uint64_t)numberOfVertices, (uint64_t)(connCompVerticesBytes / (sizeof(double) * 3)));
        }

        // exact numerical strings
        // ----------------------
        {
            // vertex array
            uint64_t connCompVerticesBytes = 0;
            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_EXACT, 0, NULL, &connCompVerticesBytes, 0, NULL, NULL), MC_NO_ERROR);

            std::vector<char> rawVerticesString;
            rawVerticesString.resize(connCompVerticesBytes);

            ASSERT_EQ(mcGetConnectedComponentData(context_, MC_TRUE, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_EXACT, connCompVerticesBytes, (void*)rawVerticesString.data(), NULL, 0, NULL, NULL), MC_NO_ERROR);

            auto my_isdigit = [&](unsigned char ch) {
                return ch == '.' || ch == '-' || std::isdigit(ch);
            };

            std::vector<char> x;
            std::vector<char> y;
            std::vector<char> z;

            const char* vptr = reinterpret_cast<const char*>(rawVerticesString.data());
            const char* vptr_ = vptr; // shifted

            for (uint32_t i = 0; i < numberOfVertices * 3; ++i) {

                vptr_ = std::strchr(vptr, ' ');
                std::ptrdiff_t diff = vptr_ - vptr;
                uint64_t srcStrLen = diff + 1; // extra byte for null-char
                if (vptr_ == nullptr) {
                    srcStrLen = strlen(vptr) + 1;
                    ASSERT_TRUE(i == ((numberOfVertices * 3) - 1));
                }

                ASSERT_GT(srcStrLen, 0);

                if ((i % 3) == 0) { // x
                    x.resize(srcStrLen);
                    std::sscanf(vptr, "%s", &x[0]);
                    x.back() = '\0';
                    ASSERT_TRUE(my_isdigit(x[0]));
                } else if ((i % 3) - 1 == 0) { // y
                    y.resize(srcStrLen);
                    std::sscanf(vptr, "%s", &y[0]);
                    y.back() = '\0';
                    ASSERT_TRUE(my_isdigit(y[0]));
                } else if ((i % 3) - 2 == 0) { // z
                    z.resize(srcStrLen);
                    std::sscanf(vptr, "%s", &z[0]);
                    z.back() = '\0';
                    ASSERT_TRUE(my_isdigit(z[0]));
                }

                vptr = vptr_ + 1; // offset so that we point to the start of the next number/line
            }
        }
    }
}

} // namespace {
