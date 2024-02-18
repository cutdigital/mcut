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

struct DispatchFilterFlags {
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

UTEST_F_SETUP(DispatchFilterFlags)
{
    McResult err = mcCreateContext(&utest_fixture->context_, 0);
    EXPECT_TRUE(utest_fixture->context_ != NULL);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F_TEARDOWN(DispatchFilterFlags)
{
    if (utest_fixture->connComps_.size() > 0) {
        EXPECT_EQ(mcReleaseConnectedComponents(
                      utest_fixture->context_,
                      (McInt32)utest_fixture->connComps_.size(),
                      utest_fixture->connComps_.data()),
            MC_NO_ERROR);
    }

    EXPECT_EQ(mcReleaseContext(utest_fixture->context_), MC_NO_ERROR);

    mioFreeMesh(&utest_fixture->srcMesh);
	mioFreeMesh(&utest_fixture->cutMesh);
}

UTEST_F(DispatchFilterFlags, noFiltering)
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

	ASSERT_EQ(mcDispatch(utest_fixture->context_,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(McUint32(12), numConnectedComponents); // including sealed, partially, unsealed, above, below, patches & seams
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McUint32 i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);
    }
}

UTEST_F(DispatchFilterFlags, partialCutWithInsideSealing)
{
    //
	// read-in the source-mesh from file
	//

	mioReadOFF((std::string(MESHES_DIR) + "/bunny.off").c_str(),
			   &utest_fixture->srcMesh.pVertices,
			   &utest_fixture->srcMesh.pFaceVertexIndices,
			   &utest_fixture->srcMesh.pFaceSizes,
			   &utest_fixture->srcMesh.numVertices,
			   &utest_fixture->srcMesh.numFaces);

	//
	// read-in the cut-mesh from file
	//

	mioReadOFF((std::string(MESHES_DIR) + "/bunnyCuttingPlanePartial.off").c_str(),
			   &utest_fixture->cutMesh.pVertices,
			   &utest_fixture->cutMesh.pFaceVertexIndices,
			   &utest_fixture->cutMesh.pFaceSizes,
			   &utest_fixture->cutMesh.numVertices,
			   &utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(mcDispatch(
                    utest_fixture->context_,
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED | MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(4)); // one completely filled (from the inside) fragment plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McUint32 i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type = (McConnectedComponentType)0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);

        if (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            continue;
        }

        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location = (McFragmentLocation)0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED", thus a partially cut fragment will be neither above nor below.
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_UNDEFINED);
        McFragmentSealType sealType = (McFragmentSealType)0;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE", which mean "complete sealed from the inside".
        if (sealType == McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE) {
            McPatchLocation patchLocation = (McPatchLocation)0;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
            ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_INSIDE);
        }
    }
}

UTEST_F(DispatchFilterFlags, fragmentLocationBelowInside)
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
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one completely filled (from the inside) fragment plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McUint32 i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);

        if (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            continue;
        }

        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW" and "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE"
        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW"
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE", which mean "complete sealed from the inside".
        ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_INSIDE);
    }
}

UTEST_F(DispatchFilterFlags, fragmentLocationBelowOutside)
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
                    MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one completely filled (from the outside) fragment plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McUint32 i = 0; i < numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW" and "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE"
        if (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            continue;
        }
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW"
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        // The dispatch function was called with "MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE", which mean "complete sealed from the inside".
        ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_OUTSIDE);
    }
}

UTEST_F(DispatchFilterFlags, fragmentLocationBelowUnsealed)
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
                  MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | MC_DISPATCH_FILTER_FRAGMENT_SEALING_NONE,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one unsealed fragment plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    McConnectedComponent cc = utest_fixture->connComps_[0];
    ASSERT_TRUE(cc != MC_NULL_HANDLE);

    McConnectedComponentType type;
    ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);

    if (type != McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {

        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
        McFragmentLocation location;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &location, NULL), MC_NO_ERROR);
        ASSERT_EQ(location, McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW);
        McFragmentSealType sealType;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sealType, NULL), MC_NO_ERROR);
        ASSERT_EQ(sealType, McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_NONE);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_UNDEFINED);
    }
}

// TODO: fragments ABOVE

UTEST_F(DispatchFilterFlags, patchInside)
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
                  MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_PATCH_INSIDE,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one interior patch plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McInt32 i = 0; i < (McInt32)numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[0];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        if (type != McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_PATCH);
            McPatchLocation patchLocation;
            ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
            ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_INSIDE);
        }
    }
}

UTEST_F(DispatchFilterFlags, patchOutside)
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
                  MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_PATCH_OUTSIDE,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one interior patch plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McInt32 i = 0; i < (McInt32)numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        if (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            continue;
        }

        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_PATCH);
        McPatchLocation patchLocation;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL), MC_NO_ERROR);
        ASSERT_EQ(patchLocation, McPatchLocation::MC_PATCH_LOCATION_OUTSIDE);
    }
}

UTEST_F(DispatchFilterFlags, seamFromSrcMesh)
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
                  MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_SEAM_SRCMESH,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one interior patch plus input
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McInt32 i = 0; i < (McInt32)numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        if (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            continue;
        }
        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_SEAM);
        McSeamOrigin origin;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &origin, NULL), MC_NO_ERROR);
        ASSERT_EQ(origin, McSeamOrigin::MC_SEAM_ORIGIN_SRCMESH);
    }
}

UTEST_F(DispatchFilterFlags, seamFromCutMesh)
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
						 MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_FILTER_SEAM_CUTMESH,
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

    McUint32 numConnectedComponents = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents), MC_NO_ERROR);
    ASSERT_EQ(numConnectedComponents, McUint32(3)); // one interior patch plus inputs
    utest_fixture->connComps_.resize(numConnectedComponents);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->context_, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)utest_fixture->connComps_.size(), &utest_fixture->connComps_[0], NULL), MC_NO_ERROR);

    for (McInt32 i = 0; i < (McInt32)numConnectedComponents; ++i) {
        McConnectedComponent cc = utest_fixture->connComps_[i];
        ASSERT_TRUE(cc != MC_NULL_HANDLE);

        McConnectedComponentType type;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL), MC_NO_ERROR);
        if (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            continue;
        }

        ASSERT_EQ(type, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_SEAM);
        McSeamOrigin origin;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->context_, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamOrigin), &origin, NULL), MC_NO_ERROR);
        ASSERT_EQ(origin, McSeamOrigin::MC_SEAM_ORIGIN_CUTMESH);
    }
}
