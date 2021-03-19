#include "utest.h"
#include <mcut/mcut.h>
#include <vector>

struct FaceAndVertexDataMapsQueryTest {

    McContext context_;
    std::vector<McConnectedComponent> connComps_;
    std::vector<float> pSrcMeshVertices;
    std::vector<uint32_t> pSrcMeshFaceIndices;
    std::vector<uint32_t> pSrcMeshFaceSizes;
    std::vector<float> pCutMeshVertices;
    std::vector<uint32_t> pCutMeshFaceIndices;
    std::vector<uint32_t> pCutMeshFaceSizes;
};

UTEST_F_SETUP(FaceAndVertexDataMapsQueryTest)
{
    McResult err = mcCreateContext(&utest_fixture->context_, MC_NULL_HANDLE);
    EXPECT_TRUE(utest_fixture->context_ != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    // NOTE: using same example mesh as hello world tutorial

    utest_fixture->pSrcMeshVertices = {
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
    utest_fixture->pSrcMeshFaceIndices = {
        // cube faces
        0, 1, 2, 3, //0
        7, 6, 5, 4, //1
        1, 5, 6, 2, //2
        0, 3, 7, 4, //3
        3, 2, 6, 7, //4
        4, 5, 1, 0 //5
    };
    utest_fixture->pSrcMeshFaceSizes = { // cube face sizes
        4, 4, 4, 4, 4, 4
    };

    // the cut mesh
    // ---------
    utest_fixture->pCutMeshVertices = {
        // cut mesh vertices
        -20, -4, 0, //0
        0, 20, 20, //1
        20, -4, 0, //2
        0, 20, -20 //3
    };

    utest_fixture->pCutMeshFaceIndices = {
        0, 1, 2, //0
        0, 2, 3 //1
    };
    utest_fixture->pCutMeshFaceSizes = {
        3, 3
    };
}

UTEST_F_TEARDOWN(FaceAndVertexDataMapsQueryTest)
{
    utest_fixture->pSrcMeshVertices.clear();
    utest_fixture->pSrcMeshFaceIndices.clear();
    utest_fixture->pSrcMeshFaceSizes.clear();
    utest_fixture->pCutMeshVertices.clear();
    utest_fixture->pCutMeshFaceIndices.clear();
    utest_fixture->pCutMeshFaceSizes.clear();
    EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->context_, utest_fixture->connComps_.size(), utest_fixture->connComps_.data()), MC_NO_ERROR);
    utest_fixture->connComps_.clear();
    EXPECT_EQ(mcReleaseContext(utest_fixture->context_), MC_NO_ERROR);
}

UTEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeVertexMap)
{
    ASSERT_EQ(mcDispatch(
                      utest_fixture->context_,
                      MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_INCLUDE_VERTEX_MAP,
                      utest_fixture->pSrcMeshVertices.data(),
                      utest_fixture->pSrcMeshFaceIndices.data(),
                      utest_fixture->pSrcMeshFaceSizes.data(),
                      utest_fixture->pSrcMeshVertices.size() / 3,
                      utest_fixture->pSrcMeshFaceIndices.size() / 4,
                      utest_fixture->pCutMeshVertices.data(),
                      utest_fixture->pCutMeshFaceIndices.data(),
                      utest_fixture->pCutMeshFaceSizes.data(),
                      utest_fixture->pCutMeshVertices.size() / 3,
                      utest_fixture->pCutMeshFaceIndices.size() / 3),
            MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    for (int i = 0; i < (int)utest_fixture->connComps_.size(); ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i]; // connected compoenent id

        uint32_t numberOfVertices = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, sizeof(uint32_t), &numberOfVertices, NULL), MC_NO_ERROR);
        ASSERT_GT((int)numberOfVertices, 0);

        McConnectedComponentType type = McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McSeamOrigin), &type, NULL), MC_NO_ERROR);

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes), MC_NO_ERROR);
        ASSERT_EQ(numBytes / sizeof(uint32_t), numberOfVertices);

        std::vector<uint32_t> vertexMap;
        vertexMap.resize(numberOfVertices);

        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, vertexMap.size() * sizeof(uint32_t), vertexMap.data(), NULL), MC_NO_ERROR);

        auto testSrcMeshCC = [&]() {
            const int numberOfSrcMeshVertices = utest_fixture->pSrcMeshVertices.size() / 3;
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
            const int numberOfCutMeshVertices = utest_fixture->pCutMeshVertices.size() / 3;
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
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

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

UTEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeFaceMap)
{
    ASSERT_EQ(mcDispatch(
                      utest_fixture->context_,
                      MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_INCLUDE_FACE_MAP,
                      utest_fixture->pSrcMeshVertices.data(),
                      utest_fixture->pSrcMeshFaceIndices.data(),
                      utest_fixture->pSrcMeshFaceSizes.data(),
                      utest_fixture->pSrcMeshVertices.size() / 3,
                      utest_fixture->pSrcMeshFaceIndices.size() / 4,
                      utest_fixture->pCutMeshVertices.data(),
                      utest_fixture->pCutMeshFaceIndices.data(),
                      utest_fixture->pCutMeshFaceSizes.data(),
                      utest_fixture->pCutMeshVertices.size() / 3,
                      utest_fixture->pCutMeshFaceIndices.size() / 3),
            MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    for (int i = 0; i < (int)utest_fixture->connComps_.size(); ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i]; // connected compoenent id

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes), MC_NO_ERROR);
        uint32_t numberOfFaces = numBytes / sizeof(uint32_t);
        ASSERT_GT((int)numberOfFaces, 0);

        McConnectedComponentType type = McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McSeamOrigin), &type, NULL), MC_NO_ERROR);

        numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes), MC_NO_ERROR);
        ASSERT_EQ(numBytes / sizeof(uint32_t), numberOfFaces);

        std::vector<uint32_t> faceMap;
        faceMap.resize(numberOfFaces);

        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, faceMap.size() * sizeof(uint32_t), faceMap.data(), NULL), MC_NO_ERROR);

        auto testSrcMeshCC = [&]() {
            const int numberOfSrcMeshFaces = utest_fixture->pSrcMeshFaceSizes.size();
            for (int i = 0; i < numberOfFaces; i++) {
                const uint32_t correspondingSrcMeshFace = faceMap[i];
                ASSERT_TRUE(correspondingSrcMeshFace != MC_UNDEFINED_VALUE); // all face indices are mapped!
                ASSERT_GE(correspondingSrcMeshFace, 0);
                ASSERT_LT(correspondingSrcMeshFace, numberOfSrcMeshFaces);
            }
        };

        auto testPatchCC = [&]() {
            const int numberOfCutMeshVertices = utest_fixture->pCutMeshFaceSizes.size();
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
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

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

UTEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeVertexMapFlagMissing)
{
    ASSERT_EQ(mcDispatch(
                      utest_fixture->context_,
                      MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                      utest_fixture->pSrcMeshVertices.data(),
                      utest_fixture->pSrcMeshFaceIndices.data(),
                      utest_fixture->pSrcMeshFaceSizes.data(),
                      utest_fixture->pSrcMeshVertices.size() / 3,
                      utest_fixture->pSrcMeshFaceIndices.size() / 4,
                      utest_fixture->pCutMeshVertices.data(),
                      utest_fixture->pCutMeshFaceIndices.data(),
                      utest_fixture->pCutMeshFaceSizes.data(),
                      utest_fixture->pCutMeshVertices.size() / 3,
                      utest_fixture->pCutMeshFaceIndices.size() / 3),
            MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    
    for (int i = 0; i < (int)utest_fixture->connComps_.size(); ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i]; // connected compoenent id

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes), MC_INVALID_VALUE);
    }
}

UTEST_F(FaceAndVertexDataMapsQueryTest, dispatchIncludeFaceMapFlagMissing)
{
    ASSERT_EQ(mcDispatch(
                      utest_fixture->context_,
                      MC_DISPATCH_VERTEX_ARRAY_FLOAT,
                      utest_fixture->pSrcMeshVertices.data(),
                      utest_fixture->pSrcMeshFaceIndices.data(),
                      utest_fixture->pSrcMeshFaceSizes.data(),
                      utest_fixture->pSrcMeshVertices.size() / 3,
                      utest_fixture->pSrcMeshFaceIndices.size() / 4,
                      utest_fixture->pCutMeshVertices.data(),
                      utest_fixture->pCutMeshFaceIndices.data(),
                      utest_fixture->pCutMeshFaceSizes.data(),
                      utest_fixture->pCutMeshVertices.size() / 3,
                      utest_fixture->pCutMeshFaceIndices.size() / 3),
            MC_NO_ERROR);

    uint32_t numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    
    for (int i = 0; i < (int)utest_fixture->connComps_.size(); ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i]; // connected compoenent id

        uint64_t numBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes), MC_INVALID_VALUE);
    }
}