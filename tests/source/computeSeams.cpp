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
#include "mio/mio.h"

#include <string>
#include <vector>


#ifdef _WIN32
#pragma warning(disable : 26812) // Unscoped enums from mcut.h
#endif // _WIN32

struct SeamConnectedComponent {
    std::vector<McConnectedComponent> connComps_ = {};
    McContext context_ = MC_NULL_HANDLE;

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

UTEST_F_SETUP(SeamConnectedComponent)
{
    McResult err = mcCreateContext(&utest_fixture->context_, 0);
    EXPECT_TRUE(utest_fixture->context_ != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F_TEARDOWN(SeamConnectedComponent)
{
    if (utest_fixture->connComps_.size() > 0) {
        EXPECT_EQ(mcReleaseConnectedComponents(
                      utest_fixture->context_,
                      (McUint32)utest_fixture->connComps_.size(),
                      utest_fixture->connComps_.data()),
            MC_NO_ERROR);
    }

    EXPECT_EQ(mcReleaseContext(utest_fixture->context_), MC_NO_ERROR);

    mioFreeMesh(&utest_fixture->srcMesh);
    mioFreeMesh(&utest_fixture->cutMesh);
}

UTEST_F(SeamConnectedComponent, queryVertices)
{
    // partial cut intersection between a cube and a quad

    //
    // read-in the source-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/src-mesh013.off").c_str(),
                &utest_fixture->srcMesh.pVertices,
                &utest_fixture->srcMesh.pFaceVertexIndices,
                &utest_fixture->srcMesh.pFaceSizes,
                &utest_fixture->srcMesh.numVertices,
                &utest_fixture->srcMesh.numFaces);

    //
    // read-in the cut-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/cut-mesh013.off").c_str(),
                &utest_fixture->cutMesh.pVertices,
                &utest_fixture->cutMesh.pFaceVertexIndices,
                &utest_fixture->cutMesh.pFaceSizes,
                &utest_fixture->cutMesh.numVertices,
                &utest_fixture->cutMesh.numFaces);


    ASSERT_EQ(mcDispatch(
                    utest_fixture->context_,
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
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

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    // NOTE: there can only be a seamed mesh whose origin/parent is the cut-mesh in this test
    // a seam conn-comp whose origin is the cut-mesh is guarranteed to exist if the src-mesh is water-tight.
    // More generally, a seamed mesh is guarranteed to exist if and only if discovered seams/cut-paths are either 1) "circular" (loop) or 2) "linear"
    // which means that they sever/partition the respective origin (src-mesh or cut-mesh)
    ASSERT_EQ(numConnComps, McUint32(1));

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, (McUint32)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    for (McInt32 c = 0; c < (McInt32)utest_fixture->connComps_.size(); ++c) {
        McConnectedComponent cc = utest_fixture->connComps_[c]; // connected compoenent id

        // indices of the vertices which define the seam
        McSize connCompSeamVertexIndicesBytes = 0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, 0, NULL, &connCompSeamVertexIndicesBytes), MC_NO_ERROR);

        std::vector<McUint32> seamVertexIndices;
        seamVertexIndices.resize(connCompSeamVertexIndicesBytes / sizeof(McUint32));
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, connCompSeamVertexIndicesBytes, seamVertexIndices.data(), NULL), MC_NO_ERROR);

        for (McInt32 i = 0; i < (McInt32)seamVertexIndices.size(); ++i) {
            ASSERT_GE((McUint32)seamVertexIndices[i], (McUint32)0);
        }

        ASSERT_EQ((McUint32)seamVertexIndices.size(), 4u); // specifc to benchmark meshes used (see setup function).
    }
}

UTEST_F(SeamConnectedComponent, queryOriginPartialCut)
{
    //
    // read-in the source-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/src-mesh013.off").c_str(),
                &utest_fixture->srcMesh.pVertices,
                &utest_fixture->srcMesh.pFaceVertexIndices,
                &utest_fixture->srcMesh.pFaceSizes,
                &utest_fixture->srcMesh.numVertices,
                &utest_fixture->srcMesh.numFaces);

    //
    // read-in the cut-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/cut-mesh013.off").c_str(),
                &utest_fixture->cutMesh.pVertices,
                &utest_fixture->cutMesh.pFaceVertexIndices,
                &utest_fixture->cutMesh.pFaceSizes,
                &utest_fixture->cutMesh.numVertices,
                &utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(mcDispatch(
                    utest_fixture->context_,
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
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

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(McUint32(1), numConnComps);

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, (McUint32)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    McConnectedComponent cc = utest_fixture->connComps_[0]; // connected compoenent id

    McSeamOrigin orig = (McSeamOrigin)(0);
    ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

    ASSERT_TRUE(orig == MC_SEAM_ORIGIN_CUTMESH);
}

UTEST_F(SeamConnectedComponent, queryConnectedComponentType_CompleteCut)
{
    //
    // read-in the source-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/src-mesh014.off").c_str(),
                &utest_fixture->srcMesh.pVertices,
                &utest_fixture->srcMesh.pFaceVertexIndices,
                &utest_fixture->srcMesh.pFaceSizes,
                &utest_fixture->srcMesh.numVertices,
                &utest_fixture->srcMesh.numFaces);

    //
    // read-in the cut-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/cut-mesh014.off").c_str(),
                &utest_fixture->cutMesh.pVertices,
                &utest_fixture->cutMesh.pFaceVertexIndices,
                &utest_fixture->cutMesh.pFaceSizes,
                &utest_fixture->cutMesh.numVertices,
                &utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(mcDispatch(
                utest_fixture->context_,
                MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
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

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(numConnComps, 2u);

    utest_fixture->connComps_.resize(numConnComps);

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, (McUint32)utest_fixture->connComps_.size(), utest_fixture->connComps_.data(), NULL), MC_NO_ERROR);

    bool foundSeamedMeshFromSrcMesh = false;
    bool foundSeamedMeshFromCutMesh = false;
    for (McInt32 i = 0; i < (McInt32)utest_fixture->connComps_.size(); ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i]; // connected compoenent id

        McSeamOrigin orig = McSeamOrigin::MC_SEAM_ORIGIN_ALL;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &orig, NULL), MC_NO_ERROR);

        ASSERT_TRUE(orig == MC_SEAM_ORIGIN_SRCMESH || orig == MC_SEAM_ORIGIN_CUTMESH);

        if (orig == MC_SEAM_ORIGIN_SRCMESH) {
            foundSeamedMeshFromSrcMesh = true;
        } else {
            foundSeamedMeshFromCutMesh = true;
        }
    }

    ASSERT_TRUE(foundSeamedMeshFromSrcMesh || foundSeamedMeshFromCutMesh);
}

UTEST_F(SeamConnectedComponent, dispatchRequireThroughCuts_CompleteCut)
{
    //
    // read-in the source-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/src-mesh014.off").c_str(),
                &utest_fixture->srcMesh.pVertices,
                &utest_fixture->srcMesh.pFaceVertexIndices,
                &utest_fixture->srcMesh.pFaceSizes,
                &utest_fixture->srcMesh.numVertices,
                &utest_fixture->srcMesh.numFaces);

    //
    // read-in the cut-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/cut-mesh014.off").c_str(),
                &utest_fixture->cutMesh.pVertices,
                &utest_fixture->cutMesh.pFaceVertexIndices,
                &utest_fixture->cutMesh.pFaceSizes,
                &utest_fixture->cutMesh.numVertices,
                &utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(mcDispatch(
                    utest_fixture->context_,
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
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

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_SEAM, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(numConnComps, 2u);
}

UTEST_F(SeamConnectedComponent, dispatchRequireThroughCuts_PartialCut)
{
    //
    // read-in the source-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/src-mesh013.off").c_str(),
                &utest_fixture->srcMesh.pVertices,
                &utest_fixture->srcMesh.pFaceVertexIndices,
                &utest_fixture->srcMesh.pFaceSizes,
                &utest_fixture->srcMesh.numVertices,
                &utest_fixture->srcMesh.numFaces);

    //
    // read-in the cut-mesh from file
    //

    mioReadOFF((std::string(MESHES_DIR) + "/benchmarks/cut-mesh013.off").c_str(),
                &utest_fixture->cutMesh.pVertices,
                &utest_fixture->cutMesh.pFaceVertexIndices,
                &utest_fixture->cutMesh.pFaceSizes,
                &utest_fixture->cutMesh.numVertices,
                &utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(mcDispatch(
                    utest_fixture->context_,
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_REQUIRE_THROUGH_CUTS,
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

    McUint32 numConnComps = 0;

    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps), MC_NO_ERROR);
    ASSERT_EQ(numConnComps, 2u); // there should be no connected components besides inputs
}
