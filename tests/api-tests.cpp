#include "mcut/mcut.h"
#include "off.h"
#include "gtest/gtest.h"
#include <algorithm>
//
// TODO: test cut path query code & partial cut disable test.
//

MCAPI_ATTR void mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{
    printf("---------------\n");
    printf("Debug message ( %d ): %s ", id, message);

    switch (source) {
    case MC_DEBUG_SOURCE_API:
        printf("Source: API");
        break;
    case MC_DEBUG_SOURCE_KERNEL:
        printf("Source: Kernel");
        break;
    }

    printf("\n");

    switch (type) {
    case MC_DEBUG_TYPE_ERROR:
        printf("Type: Error");
        break;
    case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        printf("Type: Deprecated Behaviour");
        break;
    case MC_DEBUG_TYPE_OTHER:
        printf("Type: Other");
        break;
    }

    printf("\n");

    switch (severity) {
    case MC_DEBUG_SEVERITY_HIGH:
        printf("Severity: high");
        break;
    case MC_DEBUG_SEVERITY_MEDIUM:
        printf("Severity: medium");
        break;
    case MC_DEBUG_SEVERITY_LOW:
        printf("Severity: low");
        break;
    case MC_DEBUG_SEVERITY_NOTIFICATION:
        printf("Severity: notification");
        break;
    }

    printf("\n\n");
}

namespace {

// context creation

TEST(ContextCreationTest, creationWithNoFlags)
{
    McContext context;
    McResult err = mcCreateContext(&context, 0);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

TEST(ContextCreationTest, creationWithDebugFlag)
{
    McContext context;
    McResult err = mcCreateContext(&context, MC_DEBUG);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

// context query

class DebugContextTest : public testing::Test {
protected:
    void SetUp() override
    {
        McResult err = mcCreateContext(&context_, MC_DEBUG);
        EXPECT_TRUE(context_ != nullptr);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    virtual void TearDown()
    {
        McResult err = mcReleaseContext(context_);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    McContext context_;
};

TEST_F(DebugContextTest, queryContextDebugFlag)
{
    uint64_t bytes;
    McFlags flags;
    EXPECT_EQ(mcGetInfo(context_, MC_CONTEXT_FLAGS, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(McFlags));

    EXPECT_EQ(mcGetInfo(context_, MC_CONTEXT_FLAGS, sizeof(McFlags), &flags, nullptr), MC_NO_ERROR);

    EXPECT_EQ(flags & MC_DEBUG, MC_DEBUG);
}

TEST_F(DebugContextTest, setDebugCallback)
{
    EXPECT_EQ(mcDebugMessageCallback(context_, mcDebugOutput, nullptr), MC_NO_ERROR);
}

TEST_F(DebugContextTest, setDebugCallbackMessageControl)
{
    EXPECT_EQ(mcDebugMessageControl(context_, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true), MC_NO_ERROR);
}

class ProfilingContextTest : public testing::Test {
protected: // You should make the members protected s.t. they can be
           // accessed from sub-classes.
    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    void SetUp() override
    {

        McResult err = mcCreateContext(&context_, MC_PROFILING_ENABLE);
        EXPECT_TRUE(context_ != nullptr);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    virtual void TearDown()
    {
        McResult err = mcReleaseContext(context_);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    McContext context_;
};

TEST_F(ProfilingContextTest, queryContextProfilingFlag)
{
    uint64_t bytes;
    McFlags flags;
    EXPECT_EQ(mcGetInfo(context_, MC_CONTEXT_FLAGS, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(McFlags));

    EXPECT_EQ(mcGetInfo(context_, MC_CONTEXT_FLAGS, sizeof(McFlags), &flags, nullptr), MC_NO_ERROR);

    EXPECT_EQ(flags & MC_PROFILING_ENABLE, MC_PROFILING_ENABLE);
}

// General queries via mcGet

TEST(Test_mcGet_Function, queryDefaultAPIValues)
{
    McContext context = MC_NULL_HANDLE;
    EXPECT_EQ(mcCreateContext(&context, 0), MC_NO_ERROR);
    EXPECT_TRUE(context != nullptr);
    uint64_t bytes;

    uint64_t defaultPrec = 0;
    EXPECT_EQ(mcGetInfo(context, MC_DEFAULT_PRECISION, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(context, MC_DEFAULT_PRECISION, bytes, &defaultPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(defaultPrec, 0u);

    uint64_t minPrec = 0;
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MIN, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MIN, bytes, &minPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(minPrec, 0u);

    uint64_t maxPrec = 0;
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MAX, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MAX, bytes, &maxPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(maxPrec, 0u);

    ASSERT_GE(maxPrec, minPrec);

    McFlags defaultRoundingMode = 0;
    EXPECT_EQ(mcGetInfo(context, MC_DEFAULT_ROUNDING_MODE, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(McFlags));
    EXPECT_EQ(mcGetInfo(context, MC_DEFAULT_ROUNDING_MODE, bytes, &defaultRoundingMode, nullptr), MC_NO_ERROR);
    ASSERT_TRUE(
        defaultRoundingMode == MC_ROUNDING_MODE_TO_NEAREST || //
        defaultRoundingMode == MC_ROUNDING_MODE_TOWARD_ZERO || //
        defaultRoundingMode == MC_ROUNDING_MODE_TOWARD_POS_INF || //
        defaultRoundingMode == MC_ROUNDING_MODE_TOWARD_NEG_INF);
}

TEST(PrecisionAndRoundingModes, test_mcSetPrecision_and_mcGetPrecision_functions)
{
    McContext context = MC_NULL_HANDLE;
    EXPECT_EQ(mcCreateContext(&context, 0), MC_NO_ERROR);
    EXPECT_TRUE(context != nullptr);

    // mcGetInfo MC_PRECISION_MIN
    uint64_t minPrec = 0;
    uint64_t bytes = 0;
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MIN, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MIN, bytes, &minPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(minPrec, 0u);

    // mcGetInfo MC_PRECISION_MAX
    uint64_t maxPrec = 0;
    bytes = 0;
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MAX, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(context, MC_PRECISION_MAX, bytes, &maxPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(minPrec, 0u);

    // mcSetPrecision
    uint64_t precValSet = std::max((uint64_t)64ul, minPrec); // bits
    EXPECT_EQ(mcSetPrecision(context, precValSet), MC_NO_ERROR);

    // mcGetPrecision
    uint64_t prec = 0;
    EXPECT_EQ(mcGetPrecision(context, &prec), MC_NO_ERROR);

    ASSERT_EQ(prec, precValSet);

    // rounding modes

    uint32_t roundingModeValSet = MC_ROUNDING_MODE_TO_NEAREST;
    EXPECT_EQ(mcSetRoundingMode(context, roundingModeValSet), MC_NO_ERROR);

    uint32_t roundingMode = 0;
    EXPECT_EQ(mcGetRoundingMode(context, &roundingMode), MC_NO_ERROR);

    ASSERT_EQ(roundingModeValSet, roundingMode);
}

// seamed connected-component query

class SeamedConnComp : public testing::Test {
protected:
    void SetUp() override
    {
        McResult err = mcCreateContext(&context_, 0);
        EXPECT_TRUE(context_ != nullptr);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    void mySetup(const std::string& srcMeshFileName, const std::string& cutMeshFileName)
    {
        const std::string srcMeshPath = std::string(ROOT_DIR) + "/meshes/benchmarks/" + srcMeshFileName;

        readOFF(srcMeshPath.c_str(), &pSrcMeshVertices, &pSrcMeshFaceIndices, &pSrcMeshFaceSizes, &numSrcMeshVertices, &numSrcMeshFaces);

        ASSERT_TRUE(pSrcMeshVertices != nullptr);
        ASSERT_TRUE(pSrcMeshFaceIndices != nullptr);
        ASSERT_TRUE(pSrcMeshVertices != nullptr);
        ASSERT_GT((int)numSrcMeshVertices, 2);
        ASSERT_GT((int)numSrcMeshFaces, 0);

        const std::string cutMeshPath = std::string(ROOT_DIR) + "/meshes/benchmarks/" + cutMeshFileName;

        readOFF(cutMeshPath.c_str(), &pCutMeshVertices, &pCutMeshFaceIndices, &pCutMeshFaceSizes, &numCutMeshVertices, &numCutMeshFaces);

        ASSERT_TRUE(pCutMeshVertices != nullptr);
        ASSERT_TRUE(pCutMeshFaceIndices != nullptr);
        ASSERT_TRUE(pCutMeshFaceSizes != nullptr);
        ASSERT_GT((int)numCutMeshVertices, 2);
        ASSERT_GT((int)numCutMeshFaces, 0);
    }

    virtual void TearDown()
    {
        free(pSrcMeshVertices);
        free(pSrcMeshFaceIndices);
        free(pSrcMeshFaceSizes);
        free(pCutMeshVertices);
        free(pCutMeshFaceIndices);
        free(pCutMeshFaceSizes);
        McResult err = mcReleaseContext(context_);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    std::vector<McConnectedComponent> connComps_;
    McContext context_;

    float* pSrcMeshVertices = nullptr;
    uint32_t* pSrcMeshFaceIndices = nullptr;
    uint32_t* pSrcMeshFaceSizes = nullptr;
    uint32_t numSrcMeshVertices = 0;
    uint32_t numSrcMeshFaces = 0;
    float* pCutMeshVertices = nullptr;
    uint32_t* pCutMeshFaceIndices = nullptr;
    uint32_t* pCutMeshFaceSizes = nullptr;
    uint32_t numCutMeshVertices = 0;
    uint32_t numCutMeshFaces = 0;
};

TEST_F(SeamedConnComp, querySeamedMeshVertices)
{
    // partial cut intersection between a cube and a quad
    mySetup("src-mesh013.off", "cut-mesh013.off");

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

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    // NOTE: there can only be a seamed mesh whose origin/parent is the cut-mesh in this test
    // a seamed conn-comp whose origin is the cut-mesh is guarranteed to exist if the src-mesh is water-tight.
    // More generally, a seamed mesh is guarranteed to exist if and only if discovered seams/cut-paths are either 1) "circular" (loop) or 2) "linear"
    // which means that they sever/partition the respective origin (src-mesh or cut-mesh)
    ASSERT_EQ(numConnComps, 1);

    connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, (uint32_t)connComps_.size(), connComps_.data(), NULL), MC_NO_ERROR);

    for (int i = 0; i < (int)connComps_.size(); ++i) {
        McConnectedComponent cc = connComps_[i]; // connected compoenent id

        uint32_t numberOfVertices = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, sizeof(uint32_t), &numberOfVertices, NULL), MC_NO_ERROR);
        ASSERT_GT((int)numberOfVertices, 0);

        // indices of the vertices which define the seam
        uint64_t connCompSeamVertexIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, 0, NULL, &connCompSeamVertexIndicesBytes), MC_NO_ERROR);

        std::vector<uint32_t> seamVertexIndices;
        seamVertexIndices.resize(connCompSeamVertexIndicesBytes / sizeof(uint32_t));
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, connCompSeamVertexIndicesBytes, seamVertexIndices.data(), NULL), MC_NO_ERROR);

        for (int i = 0; i < (int)seamVertexIndices.size(); ++i) {
            ASSERT_GE((uint32_t)seamVertexIndices[i], (uint32_t)0);
            ASSERT_LT((uint32_t)seamVertexIndices[i], numberOfVertices) << "out of bounds vertex index";
        }

        ASSERT_EQ(seamVertexIndices.size(), 4u); // specifc to benchmark meshes used (see setup function).
    }
}

TEST_F(SeamedConnComp, querySeamedMeshOriginPartialCut)
{
    mySetup("src-mesh013.off", "cut-mesh013.off");

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

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(numConnComps, 1);

    connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, (uint32_t)connComps_.size(), connComps_.data(), NULL), MC_NO_ERROR);

    McConnectedComponent cc = connComps_[0]; // connected compoenent id

    uint32_t numberOfVertices = 0;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, sizeof(uint32_t), &numberOfVertices, NULL), MC_NO_ERROR);
    ASSERT_GT((int)numberOfVertices, 0);

    McSeamOrigin orig = MC_SEAM_ORIGIN_ALL;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

    ASSERT_TRUE(orig == MC_SEAM_ORIGIN_CUTMESH);

    ASSERT_TRUE(numberOfVertices > numCutMeshVertices);
}

TEST_F(SeamedConnComp, querySeamedMeshTypeCompleteCut)
{
    // complete cut: cube and quad with two polygons
    mySetup("src-mesh014.off", "cut-mesh014.off");

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

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(numConnComps, 2u);

    connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, (uint32_t)connComps_.size(), connComps_.data(), NULL), MC_NO_ERROR);

    bool foundSeamedMeshFromSrcMesh = false;
    bool foundSeamedMeshFromCutMesh = false;
    for (int i = 0; i < (int)connComps_.size(); ++i) {
        McConnectedComponent cc = connComps_[i]; // connected compoenent id

        uint32_t numberOfVertices = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, sizeof(uint32_t), &numberOfVertices, NULL), MC_NO_ERROR);
        ASSERT_GT((int)numberOfVertices, 0);

        McSeamOrigin orig = McSeamOrigin::MC_SEAM_ORIGIN_ALL;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

        ASSERT_TRUE(orig == MC_SEAM_ORIGIN_SRCMESH || orig == MC_SEAM_ORIGIN_CUTMESH);

        if (orig == MC_SEAM_ORIGIN_SRCMESH) {
            foundSeamedMeshFromSrcMesh = true;
            ASSERT_TRUE(numberOfVertices > numSrcMeshVertices);

        } else {
            foundSeamedMeshFromCutMesh = true;
            ASSERT_TRUE(numberOfVertices > numCutMeshVertices);
        }
    }

    ASSERT_TRUE(foundSeamedMeshFromCutMesh && foundSeamedMeshFromCutMesh);
}

TEST_F(SeamedConnComp, dispatchRequireSeveringSeamsCompleteCut)
{
    // complete cut: cube and quad with two polygons
    mySetup("src-mesh014.off", "cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_REQUIRE_THROUGH_CUTS,
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

    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(numConnComps, 2u);
}

TEST_F(SeamedConnComp, dispatchRequireSeveringSeamsPartialCut)
{
    mySetup("src-mesh013.off", "cut-mesh013.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_REQUIRE_THROUGH_CUTS,
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
    ASSERT_EQ(numConnComps, 0u) << "there should be no connected components";
}

/////

class FaceAndVertexDataMapsQueryTest : public testing::Test {
protected: // You should make the members protected s.t. they can be
           // accessed from sub-classes.
    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    void SetUp() override
    {

        McResult err = mcCreateContext(&context_, MC_NULL_HANDLE);
        EXPECT_TRUE(context_ != nullptr);
        EXPECT_EQ(err, MC_NO_ERROR);

        // NOTE: using same example mesh as hello world tutorial

        pSrcMeshVertices = {
            // cube vertices
            -5, -5, 5, // 0
            5, -5, 5, // 1
            5, 5, 5, //2
            -5, 5, 5, //3
            -5, -5, -5, //4
            5, -5, -5, //5
            5, 5, -5, //6
            -5, 5, -5 //7
        };
        pSrcMeshFaceIndices = {
            // cube faces
            0, 1, 2, 3, //0
            7, 6, 5, 4, //1
            1, 5, 6, 2, //2
            0, 3, 7, 4, //3
            3, 2, 6, 7, //4
            4, 5, 1, 0 //5
        };
        pSrcMeshFaceSizes = { // cube face sizes
            4, 4, 4, 4, 4, 4
        };

        // the cut mesh
        // ---------
        pCutMeshVertices = {
            // cut mesh vertices
            -20, -4, 0, //0
            0, 20, 20, //1
            20, -4, 0, //2
            0, 20, -20 //3
        };

        pCutMeshFaceIndices = {
            0, 1, 2, //0
            0, 2, 3 //1
        };
        pCutMeshFaceSizes = {
            3, 3
        };
    }

    void mySetup(McFlags dispatchFlags)
    {
        ASSERT_EQ(mcDispatch(
                      context_,
                      MC_DISPATCH_VERTEX_ARRAY_FLOAT | dispatchFlags,
                      pSrcMeshVertices.data(),
                      pSrcMeshFaceIndices.data(),
                      pSrcMeshFaceSizes.data(),
                      pSrcMeshVertices.size() / 3,
                      pSrcMeshFaceIndices.size() / 4,
                      pCutMeshVertices.data(),
                      pCutMeshFaceIndices.data(),
                      pCutMeshFaceSizes.data(),
                      pCutMeshVertices.size() / 3,
                      pCutMeshFaceIndices.size() / 3),
            MC_NO_ERROR);

        uint32_t numConnComps = 0;

        ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

        connComps_.resize(numConnComps);

        ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps_.size(), connComps_.data(), NULL), MC_NO_ERROR);
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    virtual void TearDown()
    {
        pSrcMeshVertices.clear();
        pSrcMeshFaceIndices.clear();
        pSrcMeshFaceSizes.clear();
        pCutMeshVertices.clear();
        pCutMeshFaceIndices.clear();
        pCutMeshFaceSizes.clear();
        EXPECT_EQ(mcReleaseConnectedComponents(context_, connComps_.size(), connComps_.data()), MC_NO_ERROR);
        connComps_.clear();
        EXPECT_EQ(mcReleaseContext(context_), MC_NO_ERROR);
    }

    McContext context_;
    std::vector<McConnectedComponent> connComps_;
    std::vector<float> pSrcMeshVertices;
    std::vector<uint32_t> pSrcMeshFaceIndices;
    std::vector<uint32_t> pSrcMeshFaceSizes;
    uint32_t numSrcMeshVertices = 0;
    uint32_t numSrcMeshFaces = 0;
    std::vector<float> pCutMeshVertices;
    std::vector<uint32_t> pCutMeshFaceIndices;
    std::vector<uint32_t> pCutMeshFaceSizes;
    uint32_t numCutMeshVertices = 0;
    uint32_t numCutMeshFaces = 0;
};

TEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeVertexMap)
{
    mySetup(MC_DISPATCH_INCLUDE_VERTEX_MAP);

    for (int i = 0; i < (int)connComps_.size(); ++i) {
        McConnectedComponent cc = connComps_[i]; // connected compoenent id

        uint32_t numberOfVertices = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, sizeof(uint32_t), &numberOfVertices, NULL), MC_NO_ERROR);
        ASSERT_GT((int)numberOfVertices, 0);

        McConnectedComponentType type = McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McSeamOrigin), &type, NULL), MC_NO_ERROR);

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes), MC_NO_ERROR);
        ASSERT_EQ(numBytes / sizeof(uint32_t), numberOfVertices);

        std::vector<uint32_t> vertexMap;
        vertexMap.resize(numberOfVertices);

        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, vertexMap.size() * sizeof(uint32_t), vertexMap.data(), NULL), MC_NO_ERROR);

        auto testSrcMeshCC = [&]() {
            const int numberOfSrcMeshVertices = pSrcMeshVertices.size() / 3;
            for (int i = 0; i < numberOfVertices; i++) {
                const uint32_t correspondingSrcMeshVertex = vertexMap[i];
                const bool isIntersectionPoint = (correspondingSrcMeshVertex == MC_UNDEFINED_VALUE);
                if (!isIntersectionPoint) {
                    ASSERT_GE(correspondingSrcMeshVertex, 0);
                    ASSERT_LT(correspondingSrcMeshVertex, numberOfSrcMeshVertices);
                }
            }
        };

        auto testPatchCC = [&]() {
            const int numberOfCutMeshVertices = pCutMeshVertices.size() / 3;
            for (int i = 0; i < numberOfVertices; i++) {
                const uint32_t correspondingCutMeshVertex = vertexMap[i];
                const bool isIntersectionPoint = (correspondingCutMeshVertex == MC_UNDEFINED_VALUE);
                if (!isIntersectionPoint) {
                    ASSERT_GE(correspondingCutMeshVertex, 0);
                    ASSERT_LT(correspondingCutMeshVertex, numberOfCutMeshVertices);
                }
            }
        };

        switch (type) {
        case McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT: { // comes from source-mesh
            testSrcMeshCC();
        } break;
        case McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_PATCH: { // comes from cut-mesh
            testPatchCC();
        } break;
        case McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_SEAM: {
            // find out where it comes from (source-mesh or cut-mesh)
            McSeamOrigin orig = MC_SEAM_ORIGIN_ALL;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

            if (orig == MC_SEAM_ORIGIN_SRCMESH) {
                testSrcMeshCC();
            } else {
                ASSERT_TRUE(orig == MC_SEAM_ORIGIN_CUTMESH);
                testPatchCC();
            }
        } break;
        }
    }
}

TEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeFaceMap)
{
    mySetup(MC_DISPATCH_INCLUDE_FACE_MAP);

    for (int i = 0; i < (int)connComps_.size(); ++i) {
        McConnectedComponent cc = connComps_[i]; // connected compoenent id

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes), MC_NO_ERROR);
        uint32_t numberOfFaces = numBytes / sizeof(uint32_t);
        ASSERT_GT((int)numberOfFaces, 0);

        McConnectedComponentType type = McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McSeamOrigin), &type, NULL), MC_NO_ERROR);

        numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes), MC_NO_ERROR);
        ASSERT_EQ(numBytes / sizeof(uint32_t), numberOfFaces);

        std::vector<uint32_t> faceMap;
        faceMap.resize(numberOfFaces);

        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, faceMap.size() * sizeof(uint32_t), faceMap.data(), NULL), MC_NO_ERROR);

        auto testSrcMeshCC = [&]() {
            const int numberOfSrcMeshFaces = pSrcMeshFaceSizes.size();
            for (int i = 0; i < numberOfFaces; i++) {
                const uint32_t correspondingSrcMeshFace = faceMap[i];
                ASSERT_TRUE(correspondingSrcMeshFace != MC_UNDEFINED_VALUE); // all face indices are mapped!
                ASSERT_GE(correspondingSrcMeshFace, 0);
                ASSERT_LT(correspondingSrcMeshFace, numberOfSrcMeshFaces);
            }
        };

        auto testPatchCC = [&]() {
            const int numberOfCutMeshVertices = pCutMeshFaceSizes.size();
            for (int i = 0; i < numberOfFaces; i++) {
                const uint32_t correspondingCutMeshVertex = faceMap[i];
                ASSERT_TRUE(correspondingCutMeshVertex != MC_UNDEFINED_VALUE);
                ASSERT_GE(correspondingCutMeshVertex, 0);
                ASSERT_LT(correspondingCutMeshVertex, numberOfCutMeshVertices);
            }
        };

        switch (type) {
        case McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT: { // comes from source-mesh
            testSrcMeshCC();
        } break;
        case McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_PATCH: { // comes from cut-mesh
            testPatchCC();
        } break;
        case McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_SEAM: {
            // find out where it comes from (source-mesh or cut-mesh)
            McSeamOrigin orig = MC_SEAM_ORIGIN_ALL;
            ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

            if (orig == MC_SEAM_ORIGIN_SRCMESH) {
                testSrcMeshCC();
            } else {
                ASSERT_TRUE(orig == MC_SEAM_ORIGIN_CUTMESH);
                testPatchCC();
            }
        } break;
        }
    }
}

TEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeVertexMapFlagMissing)
{
    mySetup(0);

    for (int i = 0; i < (int)connComps_.size(); ++i) {
        McConnectedComponent cc = connComps_[i]; // connected compoenent id

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes), MC_INVALID_VALUE);
    }
}

TEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeFaceMapFlagMissing)
{
    mySetup(0);

    for (int i = 0; i < (int)connComps_.size(); ++i) {
        McConnectedComponent cc = connComps_[i]; // connected compoenent id

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes), MC_INVALID_VALUE);
    }
}

///

class mcDispatchFILTER : public testing::Test {
protected:
    void SetUp() override
    {
        McResult err = mcCreateContext(&context_, MC_NULL_HANDLE);
        EXPECT_TRUE(context_ != nullptr);
        EXPECT_EQ(err, MC_NO_ERROR);
    }

    void mySetup(const std::string& srcMeshFileName, const std::string& cutMeshFileName)
    {
        const std::string srcMeshPath = std::string(ROOT_DIR) + "/meshes/" + srcMeshFileName;

        readOFF(srcMeshPath.c_str(), &pSrcMeshVertices, &pSrcMeshFaceIndices, &pSrcMeshFaceSizes, &numSrcMeshVertices, &numSrcMeshFaces);

        ASSERT_TRUE(pSrcMeshVertices != nullptr);
        ASSERT_TRUE(pSrcMeshFaceIndices != nullptr);
        ASSERT_TRUE(pSrcMeshVertices != nullptr);
        ASSERT_GT((int)numSrcMeshVertices, 2);
        ASSERT_GT((int)numSrcMeshFaces, 0);

        const std::string cutMeshPath = std::string(ROOT_DIR) + "/meshes/" + cutMeshFileName;

        readOFF(cutMeshPath.c_str(), &pCutMeshVertices, &pCutMeshFaceIndices, &pCutMeshFaceSizes, &numCutMeshVertices, &numCutMeshFaces);

        ASSERT_TRUE(pCutMeshVertices != nullptr);
        ASSERT_TRUE(pCutMeshFaceIndices != nullptr);
        ASSERT_TRUE(pCutMeshFaceSizes != nullptr);
        ASSERT_GT((int)numCutMeshVertices, 2);
        ASSERT_GT((int)numCutMeshFaces, 0);
    }

    virtual void TearDown()
    {
        free(pSrcMeshVertices);
        free(pSrcMeshFaceIndices);
        free(pSrcMeshFaceSizes);
        free(pCutMeshVertices);
        free(pCutMeshFaceIndices);
        free(pCutMeshFaceSizes);

        EXPECT_EQ(mcReleaseConnectedComponents(context_, connComps_.size(), connComps_.data()), MC_NO_ERROR);
        EXPECT_EQ(mcReleaseContext(context_), MC_NO_ERROR);
    }

    std::vector<McConnectedComponent> connComps_;
    McContext context_;

    float* pSrcMeshVertices = nullptr;
    uint32_t* pSrcMeshFaceIndices = nullptr;
    uint32_t* pSrcMeshFaceSizes = nullptr;
    uint32_t numSrcMeshVertices = 0;
    uint32_t numSrcMeshFaces = 0;
    float* pCutMeshVertices = nullptr;
    uint32_t* pCutMeshFaceIndices = nullptr;
    uint32_t* pCutMeshFaceSizes = nullptr;
    uint32_t numCutMeshVertices = 0;
    uint32_t numCutMeshFaces = 0;
};

TEST_F(mcDispatchFILTER, all)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT, // if we dont filter anything, then MCUT computes everything
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 12); // including sealed, partially, unsealed, above, below, patches & seams
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    for (uint32_t i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);
    }
}

TEST_F(mcDispatchFILTER, partialCutWithInsideSealing) // Partial cut (CURRENTLY FAILS)
{
    mySetup("/bunny.off", "/bunnyCuttingPlanePartial.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED | MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one completely filled (from the inside) fragment
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    for (uint32_t i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED", thus a partially cut fragment will be neither above nor below.
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_UNDEFINED);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE", which mean "complete sealed from the inside".
        ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_INSIDE);
    }
}

TEST_F(mcDispatchFILTER, fragmentLocationBelowInside)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one completely filled (from the inside) fragment
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    for (uint32_t i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW" and "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE"
        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW"
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE", which mean "complete sealed from the inside".
        ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_INSIDE);
    }
}

TEST_F(mcDispatchFILTER, fragmentLocationBelowOutside)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one completely filled (from the outside) fragment
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    for (uint32_t i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW" and "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE"
        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW"
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE", which mean "complete sealed from the inside".
        ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_OUTSIDE);
    }
}

TEST_F(mcDispatchFILTER, fragmentLocationBelowOutsidePartial)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE_EXHAUSTIVE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_GE(numConnectedComponents, 1); // one completely filled (from the outside) fragment
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    for (uint32_t i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // When dispatch is called with MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE_EXHAUSTIVE, it means that MCUT will compute both fully sealed and partially sealed
        // fragments.
        ASSERT_TRUE(sealType == McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_PARTIAL || sealType == McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_OUTSIDE);
    }
}

TEST_F(mcDispatchFILTER, fragmentLocationBelowUnsealed)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_NONE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one unsealed fragment
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    McConnectedComponent cc = connComps_[0];
    ASSERT_TRUE(cc != MC_NULL_HANDLE);

    McConnectedComponentType type;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
    ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
    McFragmentLocation location;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
    ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
    McFragmentSealType sealType;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
    ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_NONE);
    McPatchLocation patchLocation;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
    ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_UNDEFINED);
}

// TODO: fragments ABOVE

TEST_F(mcDispatchFILTER, patchInside)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_PATCH_INSIDE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one interior patch
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    McConnectedComponent cc = connComps_[0];
    ASSERT_TRUE(cc != MC_NULL_HANDLE);

    McConnectedComponentType type;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
    ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_PATCH);
    McPatchLocation patchLocation;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
    ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_INSIDE);
}

TEST_F(mcDispatchFILTER, patchOutside)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_PATCH_OUTSIDE, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one interior patch
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    McConnectedComponent cc = connComps_[0];
    ASSERT_TRUE(cc != MC_NULL_HANDLE);

    McConnectedComponentType type;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
    ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_PATCH);
    McPatchLocation patchLocation;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
    ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_OUTSIDE);
}

TEST_F(mcDispatchFILTER, seamFromSrcMesh)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_SEAM_SRCMESH, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one interior patch
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    McConnectedComponent cc = connComps_[0];
    ASSERT_TRUE(cc != MC_NULL_HANDLE);

    McConnectedComponentType type;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
    ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_SEAM);
    McSeamOrigin origin;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &origin, NULL), MC_NO_ERROR);
    ASSERT_EQ(origin, McSeamOrigin::MC_SEAM_ORIGIN_SRCMESH);
}

TEST_F(mcDispatchFILTER, seamFromCutMesh)
{
    // "src-mesh014.off" = cube with four-sided faces
    // "cut-mesh014.off" = square made of two triangles
    // complete cut
    mySetup("/benchmarks/src-mesh014.off", "/benchmarks/cut-mesh014.off");

    ASSERT_EQ(mcDispatch(
                  context_,
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_FILTER_SEAM_CUTMESH, // **
                  // source mesh
                  pSrcMeshVertices,
                  pSrcMeshFaceIndices,
                  pSrcMeshFaceSizes,
                  numSrcMeshVertices,
                  numSrcMeshFaces,
                  // cut mesh
                  pCutMeshVertices,
                  pCutMeshFaceIndices,
                  pCutMeshFaceSizes,
                  numCutMeshVertices,
                  numCutMeshFaces),
        MC_NO_ERROR);

    uint32_t numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, 1); // one interior patch
    connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(context_, MC_CONNECTED_COMPONENT_TYPE_ALL, connComps_.size(), &connComps_[0], NULL), MC_NO_ERROR);

    McConnectedComponent cc = connComps_[0];
    ASSERT_TRUE(cc != MC_NULL_HANDLE);

    McConnectedComponentType type;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
    ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_SEAM);
    McSeamOrigin origin;
    ASSERT_EQ(mcGetConnectedComponentData(context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &origin, NULL), MC_NO_ERROR);
    ASSERT_EQ(origin, McSeamOrigin::MC_SEAM_ORIGIN_SRCMESH);
}

} //  namespace {
