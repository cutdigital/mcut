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

 /*
 This test creates multiple contexts that run in parallel.

 There are as many contexts as half the number of hardware threads.

 Each context is run in its own thread.
 */

#include "utest.h"

#include <mcut/mcut.h>
#include <vector>

struct Mesh {
	std::vector<McDouble> vertices = {};
	std::vector<McUint32> faceIndices = {};
	std::vector<McUint32> faceSizes = {};
    McUint32 numVertices=0;
    McUint32 numFaces=0;
};

struct IntersectionType {
    McContext context = MC_NULL_HANDLE;

    Mesh srcMesh = {};
	Mesh cutMesh = {};
};

UTEST_F_SETUP(IntersectionType)
{
    utest_fixture->context  = MC_NULL_HANDLE;
    ASSERT_EQ(mcCreateContextWithHelpers(&utest_fixture->context , MC_OUT_OF_ORDER_EXEC_MODE_ENABLE, 0), MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->context  != nullptr);
}

UTEST_F_TEARDOWN(IntersectionType)
{
    EXPECT_EQ(mcReleaseContext(utest_fixture->context), MC_NO_ERROR);
}

UTEST_F(IntersectionType, defaultValue)
{
    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    // we have not yet called the dispatch function to the value should be undefined i.e. MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM
    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM); 
}

Mesh makeCube(McDouble halfExtent, McDouble translX, McDouble translY, McDouble translZ)
{
    const McDouble h = halfExtent;

    Mesh m;
    m.numVertices = 8;
    m.numFaces = 6;
    std::vector<McDouble>& verts = m.vertices;

    verts.resize((McSize)m.numVertices*3); 

    // front 

    McInt32 i = 0;
    // v0 - bottom left
    verts[i++] = -h; // x
    verts[i++] = -h; // y
    verts[i++] = h; // z

    // v1 - bottom right
    verts[i++] = h; // x
    verts[i++] = -h; // y
    verts[i++] = h; // z

    // v2 - top right
    verts[i++] = h; // x
    verts[i++] = h; // y
    verts[i++] = h; // z

    // v3 - top left
    verts[i++] = -h; // x
    verts[i++] = h; // y
    verts[i++] = h; // z

    // back

    // v4 - bottom left
    verts[i++] = -h; // x
    verts[i++] = -h; // y
    verts[i++] = -h; // z

    // v5 - bottom right
    verts[i++] = h; // x
    verts[i++] = -h; // y
    verts[i++] = -h; // z

    // v6 - top right
    verts[i++] = h; // x
    verts[i++] = h; // y
    verts[i++] = -h; // z

    // v7 - top left
    verts[i++] = -h; // x
    verts[i++] = h; // y
    verts[i++] = -h; // z

    for (McInt32 j = 0; j < 8; ++j)
    {
		verts[(McSize)j * 3 + 0] += translX;
		verts[(McSize)j * 3 + 1] += translY;
		verts[(McSize)j * 3 + 2] += translZ;
    }

    std::vector<McIndex>& faces = m.faceIndices;
	faces.resize((McSize)m.numFaces * 4);

    i = 0;
    // front
    faces[i++] = 0;
    faces[i++] = 1;
    faces[i++] = 2;
    faces[i++] = 3;

    // back
    faces[i++] = 7;
    faces[i++] = 6;
    faces[i++] = 5;
    faces[i++] = 4;

    // left
    faces[i++] = 4;
    faces[i++] = 0;
    faces[i++] = 3;
    faces[i++] = 7;

    // right
    faces[i++] = 1;
    faces[i++] = 5;
    faces[i++] = 6;
    faces[i++] = 2;

    // top
    faces[i++] = 3;
    faces[i++] = 2;
    faces[i++] = 6;
    faces[i++] = 7;

    // bottom
    faces[i++] = 0;
    faces[i++] = 4;
    faces[i++] = 5;
    faces[i++] = 1;

    m.faceSizes.resize(m.numFaces);
    for (McUint32 j = 0; j < m.numFaces; ++j)
    {
        m.faceSizes[j] = 4;
    }

    return m;
}

Mesh makeQuad_xz(
    McDouble halfExtent, McDouble translX, McDouble translY, McDouble translZ)
{
    const McDouble h = halfExtent;
    Mesh m;
    m.numVertices = 4;
    m.numFaces = 1;
    std::vector<McDouble>& verts = m.vertices;

    verts.resize((McSize)m.numVertices * 3);

    McInt32 i = 0;

    verts[i++] = -h; // x
    verts[i++] = 0; // y
    verts[i++] = h; // z

    verts[i++] = h; // x
    verts[i++] = 0; // y
    verts[i++] = h; // z

    verts[i++] = h; // x
    verts[i++] = 0; // y
    verts[i++] = -h; // z

    verts[i++] = -h; // x
    verts[i++] = 0; // y
    verts[i++] = -h; // z

    for (McInt32 j = 0; j < 4; ++j)
    {
		verts[(McSize)j * 3 + 0] += translX;
		verts[(McSize)j * 3 + 1] += translY;
		verts[(McSize)j * 3 + 2] += translZ;
    }

    std::vector<McIndex>& faces = m.faceIndices;
	faces.resize((McSize)m.numFaces * 4);

    i = 0;
    // front
    faces[i++] = 0;
    faces[i++] = 1;
    faces[i++] = 2;
    faces[i++] = 3;

    m.faceSizes.resize(m.numFaces);
	for(McUint32 j = 0; j < m.numFaces; ++j)
    {
        m.faceSizes[j] = 4;
    }

    return m;
}

Mesh makeQuad_xy(
    McDouble halfExtent, McDouble translX, McDouble translY, McDouble translZ)
{
    const McDouble h = halfExtent;
    Mesh m;
    m.numVertices = 4;
    m.numFaces = 1;
    std::vector<McDouble>& verts = m.vertices;

    verts.resize((McSize)m.numVertices * 3);

    McInt32 i = 0;

    verts[i++] = -h; // x
    verts[i++] = h; // y
    verts[i++] = 0; // z

    verts[i++] = h; // x
    verts[i++] = h; // y
    verts[i++] = 0; // z

    verts[i++] = h; // x
    verts[i++] = -h; // y
    verts[i++] = 0; // z

    verts[i++] = -h; // x
    verts[i++] = -h; // y
    verts[i++] = 0; // z

    for (McInt32 j = 0; j < 4; ++j)
    {
		verts[(McSize)j * 3 + 0] += translX;
		verts[(McSize)j * 3 + 1] += translY;
		verts[(McSize)j * 3 + 2] += translZ;
    }

    std::vector<McIndex>& faces = m.faceIndices;
	faces.resize((McSize)m.numFaces * 4);

    i = 0;
    // front
    faces[i++] = 0;
    faces[i++] = 1;
    faces[i++] = 2;
    faces[i++] = 3;

    m.faceSizes.resize(m.numFaces);
	for(McUint32 j = 0; j < m.numFaces; ++j)
    {
        m.faceSizes[j] = 4;
    }

    return m;
}

// Watertight cut-mesh INSIDE watertight source-mesh
UTEST_F(IntersectionType, watertightCutMeshInsideWatertightSourceMesh)
{
    utest_fixture->srcMesh = makeCube(2.0, 0,0,0);
    utest_fixture->cutMesh = makeCube(1.0, 0, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_INSIDE_SOURCEMESH);
}

// Watertight source-mesh INSIDE watertight cut-mesh
UTEST_F(IntersectionType, watertightSourceMeshInsideWatertightCutMesh)
{
    utest_fixture->srcMesh = makeCube(1.0, 0, 0, 0);
    utest_fixture->cutMesh = makeCube(2.0, 0, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_INSIDE_CUTMESH);
}

// SEPARTATED Watertight source-mesh and watertight cut-mesh
UTEST_F(IntersectionType, separatedWatertightSourceMeshAndWatertightCutMesh)
{
    utest_fixture->srcMesh = makeCube(1.0, -2, 0, 0);
    utest_fixture->cutMesh = makeCube(1.0, 2, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_NONE);
}

// STANDARD INTERSECTION Watertight source-mesh and watertight cut-mesh
UTEST_F(IntersectionType, stdIntersectionWatertightSourceMeshAndWatertightCutMesh)
{
    utest_fixture->srcMesh = makeCube(1.0, 0, 0, 0);
    utest_fixture->cutMesh = makeCube(1.0, 0.5, 0.5, 0.5);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_STANDARD);
}

// STANDARD INTERSECTION Watertight source-mesh and watertight cut-mesh WITH GP ENFORCEMENT
UTEST_F(IntersectionType, stdIntersectionWatertightSourceMeshAndWatertightCutMeshWithGpEnforcement)
{
    utest_fixture->srcMesh = makeCube(1.0, 0, 0, 0); // note: the inputs overlap
    utest_fixture->cutMesh = makeCube(1.0, 0, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_STANDARD);
}

// SMWatertight-CM-not-watertight
//  - standard intersection

// SEPARATED Watertight source-mesh and open cut-mesh
UTEST_F(IntersectionType, separatedWatertightSourceMeshAndOpenCutMesh)
{
    utest_fixture->srcMesh = makeCube(1.0, 0, 0, 0); 
    utest_fixture->cutMesh = makeQuad_xz(2, 0, 2, 0); // above cube

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_NONE);
}

// Open cut-mesh INSIDE Watertight source-mesh 
UTEST_F(IntersectionType, openCutMeshInsideWatertightSourceMesh)
{
    utest_fixture->srcMesh = makeCube(10.0, 0, 0, 0);
    utest_fixture->cutMesh = makeQuad_xz(2, 0, 2, 0); // above cube

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_INSIDE_SOURCEMESH);
}

// Open cut-mesh INTERSECTS Watertight source-mesh 
UTEST_F(IntersectionType, openCutMeshIntersectsWatertightSourceMesh)
{
    utest_fixture->srcMesh = makeCube(1.0, 0, 0, 0);
    utest_fixture->cutMesh = makeQuad_xz(1, 0.5, 0, 0.5); // above cube

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_STANDARD);
}

// Open source-mesh INTERSECTS Watertight cut-mesh 
UTEST_F(IntersectionType, watertightCutMeshIntersectsOpenSourceMesh)
{
    utest_fixture->srcMesh = makeQuad_xz(1, 0.5, 0, 0.5); 
    utest_fixture->cutMesh = makeCube(1.0, 0, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_STANDARD);
}

// Open source-mesh INTERSECTS Watertight cut-mesh 
UTEST_F(IntersectionType, openSourceMeshInsidewatertightCutMesh)
{
    utest_fixture->srcMesh = makeQuad_xz(1, 0, 0, 0);
    utest_fixture->cutMesh = makeCube(10.0, 0, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_INSIDE_CUTMESH);
}

// SEPARATED Open source-mesh and Watertight cut-mesh 
UTEST_F(IntersectionType, separatedOpenSourceMeshAndWatertightCutMesh)
{
    utest_fixture->srcMesh = makeQuad_xz(1, 10, 0, 0);
    utest_fixture->cutMesh = makeCube(1, -10, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_NONE);
}

// SM-not-Watertight-CM-not-watertight
//  - properly outside/separately
//  - standard intersection
//

// SEPARATED Open source-mesh and Watertight cut-mesh 
UTEST_F(IntersectionType, separatedOpenSourceMeshAndOpenCutMesh)
{
    utest_fixture->srcMesh = makeQuad_xz(1, 10, 0, 0);
    utest_fixture->cutMesh = makeQuad_xz(1, -10, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_NONE);
}

// Open source-mesh INTERSECTS Open cut-mesh 
UTEST_F(IntersectionType, openCutMeshIntersectsOpenSourceMesh)
{
    utest_fixture->srcMesh = makeQuad_xz(1, 0, 0, 0);
    utest_fixture->cutMesh = makeQuad_xy(2, 0, 0, 0);

    const McResult result = mcDispatch(
        utest_fixture->context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_INTERSECTION_TYPE,
        &utest_fixture->srcMesh.vertices[0],
        &utest_fixture->srcMesh.faceIndices[0],
        &utest_fixture->srcMesh.faceSizes[0],
        utest_fixture->srcMesh.numVertices,
        utest_fixture->srcMesh.numFaces,
        &utest_fixture->cutMesh.vertices[0],
        &utest_fixture->cutMesh.faceIndices[0],
        &utest_fixture->cutMesh.faceSizes[0],
        utest_fixture->cutMesh.numVertices,
        utest_fixture->cutMesh.numFaces);

    ASSERT_EQ(result, MC_NO_ERROR);

    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value = (McDispatchIntersectionType)0;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_STANDARD);
}