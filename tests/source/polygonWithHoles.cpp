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

struct PolygonsWithHoles { 
    McContext context_ {};

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

UTEST_F_SETUP(PolygonsWithHoles)
{
    McResult err = mcCreateContext(&utest_fixture->context_, 0);
    EXPECT_TRUE(utest_fixture->context_ != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F_TEARDOWN(PolygonsWithHoles)
{
    EXPECT_EQ(mcReleaseContext(utest_fixture->context_), MC_NO_ERROR);

    mioFreeMesh(&utest_fixture->srcMesh);
	mioFreeMesh(&utest_fixture->cutMesh);
}

UTEST_F(PolygonsWithHoles, outputWillHaveHoles)
{
    // partial cut intersection between a cube and a quad

    
    //
	// read-in the source-mesh from file
	//

	mioReadOFF((std::string(MESHES_DIR) + "/cube-flattened.off").c_str(),
			   &utest_fixture->srcMesh.pVertices,
			   &utest_fixture->srcMesh.pFaceVertexIndices,
			   &utest_fixture->srcMesh.pFaceSizes,
			   &utest_fixture->srcMesh.numVertices,
			   &utest_fixture->srcMesh.numFaces);

	//
	// read-in the cut-mesh from file
	//

	mioReadOFF((std::string(MESHES_DIR) + "/cube-flattened-with-holes.off").c_str(),
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
        MC_INVALID_OPERATION);
}
