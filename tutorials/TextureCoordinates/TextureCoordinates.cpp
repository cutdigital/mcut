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
 *  TextureCoordinates.cpp
 *
 *  \brief:
 *  This tutorial shows how to propagate per-vertex texture (i.e. uv) coordinates 
 *  from the input meshes and unto the output connected components after cutting.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"
#include "mio/mio.h"

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

#define my_assert(cond)                             \
    if (!(cond))                                    \
    {                                               \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::abort();                               \
    }


// basic comparison of doubles
McBool compare(McDouble x, McDouble y)
{
    return std::fabs(x - y) < 1e-6;
}

std::string resolve_cc_name_string(McContext context,
								   McConnectedComponent cc,
								   McBool& isFragment,
								   McFragmentLocation& fragmentLocation,
								   McBool& isDerivedFromSrcMesh);
// simple structure representing a 2d vector and the operations we will perform with it
struct vec2
{
	union
	{
		struct
		{
			McDouble x, y;
		};
		struct
		{
			McDouble u, v;
		};
	};

	vec2 operator*(const McDouble c) const
	{
		vec2 result = {{0.0, 0.0}};
		result.x = x * c;
		result.y = y * c;
		return result;
	}

	vec2 operator+(const vec2& rhs) const
	{
		vec2 result = {{0.0, 0.0}};
		result.x = x + rhs.x;
		result.y = y + rhs.y;
		return result;
	}

	vec2 operator-(const vec2& rhs) const
	{
		vec2 result = {{0.0, 0.0}};
		result.x = x - rhs.x;
		result.y = y - rhs.y;
		return result;
	}
};


// simple structure representing a 3d vector and the operations we will perform with it
struct vec3
{
	union
	{
		struct
		{
			McDouble x, y, z;
		};
		struct
		{
			McDouble u, v, w;
		};
	};

	vec3 operator*(const McDouble c) const
	{
		vec3 result = {{0.0, 0.0, 0.0}};
		result.x = x * c;
		result.y = y * c;
		result.z = z * c;
		return result;
	}

	vec3 operator+(const vec3& rhs) const
	{
		vec3 result = {{0.0, 0.0, 0.0}};
		result.x = x + rhs.x;
		result.y = y + rhs.y;
		result.z = z + rhs.z;
		return result;
	}

	vec3 operator-(const vec3& rhs) const
	{
		vec3 result = {{0.0, 0.0, 0.0}};
		result.x = x - rhs.x;
		result.y = y - rhs.y;
		result.z = z - rhs.z;
		return result;
	}
};

std::string resolve_cc_name_string(McContext context,
								   McConnectedComponent cc,
								   McBool& isFragment,
								   McFragmentLocation& fragmentLocation);

inline McDouble
getTriangleArea2D(McDouble x1, McDouble y1, McDouble x2, McDouble y2, McDouble x3, McDouble y3)
{
	return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
}

vec3 crossProduct(const vec3& u, const vec3& v)
{
	vec3 out = {{0.0, 0.0, 0.0}};
	out.x = u.y * v.z - u.z * v.y;
	out.y = u.z * v.x - u.x * v.z;
	out.z = u.x * v.y - u.y * v.x;
	return out;
}

// Compute barycentric coordinates (u, v, w) for point p with respect to triangle (a, b, c)
vec3 getBarycentricCoords(const vec3& p, const vec3& a, const vec3& b, const vec3& c)
{
	// Unnormalized triangle normal
	const vec3 m = crossProduct(b - a, c - a);
	// Nominators and one-over-denominator for u and v ratios
	McDouble nu = 0.0, nv = 0.0, ood = 0.0;
	// Absolute components for determining projection plane
	const McDouble x = std::abs(m.x), y = std::abs(m.y), z = std::abs(m.z);

	// Compute areas in plane of largest projection
	if(x >= y && x >= z)
	{
		// x is largest, project to the yz plane
		nu = getTriangleArea2D(p.y, p.z, b.y, b.z, c.y, c.z);
		// Area of PBC in yz plane
		nv = getTriangleArea2D(p.y, p.z, c.y, c.z, a.y, a.z);
		// Area of PCA in yz plane
		ood = 1.0f / m.x; // 1/(2*area of ABC in yz plane
	}
	else if(y >= x && y >= z)
	{
		// y is largest, project to the xz plane
		nu = getTriangleArea2D(p.x, p.z, b.x, b.z, c.x, c.z);
		nv = getTriangleArea2D(p.x, p.z, c.x, c.z, a.x, a.z);
		ood = 1.0f / -m.y;
	}
	else
	{
		// z is largest, project to the xy plane
		nu = getTriangleArea2D(p.x, p.y, b.x, b.y, c.x, c.y);
		nv = getTriangleArea2D(p.x, p.y, c.x, c.y, a.x, a.y);
		ood = 1.0f / m.z;
	}

	vec3 result = {{0.0, 0.0, 0.0}};
	result.u = nu * ood;
	result.v = nv * ood;
	result.w = 1.0f - result.u - result.v;
	return result;
}


McInt32 main()
{
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

    MioMesh cutMesh = srcMesh;

    //
	// read-in the source-mesh from file
	//
	mioReadOBJ(DATA_DIR "/a.obj",
			   &srcMesh.pVertices,
			   &srcMesh.pNormals,
			   &srcMesh.pTexCoords,
			   &srcMesh.pFaceSizes,
			   &srcMesh.pFaceVertexIndices,
			   &srcMesh.pFaceVertexTexCoordIndices,
			   &srcMesh.pFaceVertexNormalIndices,
			   &srcMesh.numVertices,
			   &srcMesh.numNormals,
			   &srcMesh.numTexCoords,
			   &srcMesh.numFaces);

	//
	// read-in the cut-mesh from file
	//

	mioReadOBJ(DATA_DIR "/b.obj",
			   &cutMesh.pVertices,
			   &cutMesh.pNormals,
			   &cutMesh.pTexCoords,
			   &cutMesh.pFaceSizes,
			   &cutMesh.pFaceVertexIndices,
			   &cutMesh.pFaceVertexTexCoordIndices,
			   &cutMesh.pFaceVertexNormalIndices,
			   &cutMesh.numVertices,
			   &cutMesh.numNormals,
			   &cutMesh.numTexCoords,
			   &cutMesh.numFaces);

    //
    //  create a context
    // 
    McContext context = MC_NULL_HANDLE;

    McResult status = mcCreateContext(&context, MC_DEBUG);

    my_assert(status == MC_NO_ERROR);

    //
	//  do the cutting
	//
    status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_VERTEX_MAP | MC_DISPATCH_INCLUDE_FACE_MAP,
		// source mesh
		srcMesh.pVertices,
		srcMesh.pFaceVertexIndices,
		srcMesh.pFaceSizes,
		srcMesh.numVertices,
		srcMesh.numFaces,
		// cut mesh
		cutMesh.pVertices,
		cutMesh.pFaceVertexIndices,
		cutMesh.pFaceSizes,
		cutMesh.numVertices,
		cutMesh.numFaces);

    my_assert(status == MC_NO_ERROR);

    //
	// query the number of available connected components
	//
    McUint32 connectedComponentCount = 0;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);

    my_assert(status == MC_NO_ERROR);

    printf("connected components: %d\n", (McInt32)connectedComponentCount);

    if (connectedComponentCount == 0)
    {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    std::vector<McConnectedComponent> connectedComponents(connectedComponentCount, 0);
    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);
    my_assert(status == MC_NO_ERROR);

    //
	// query the data of each connected component
	//

    for (McInt32 i = 0; i < (McInt32)connectedComponentCount; ++i)
    {
        McConnectedComponent cc = connectedComponents[i]; // connected compoenent id

        McSize numBytes = 0;

        //
		//  vertices
		//

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        
        my_assert(status == MC_NO_ERROR);

        const McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McDouble) * 3));
        std::vector<McDouble> ccVertices((McSize)ccVertexCount * 3u, 0);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void *)ccVertices.data(), NULL);
        
        my_assert(status == MC_NO_ERROR);

        //
		//  faces
		//

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);

        my_assert(status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceIndices(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);

        my_assert(status == MC_NO_ERROR);

        //
		//  face sizes
		//

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);

        my_assert(status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceSizes(numBytes / sizeof(McUint32), 0);
        ccFaceSizes.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);

        my_assert(status == MC_NO_ERROR);

        //
		// vertex map
		//

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes);

        my_assert(status == MC_NO_ERROR);

        std::vector<McUint32> ccVertexMap(numBytes / sizeof(McUint32));
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, numBytes, ccVertexMap.data(), NULL);

        my_assert(status == MC_NO_ERROR);

        //
		//  face map
		//

        const McUint32 ccFaceCount = static_cast<McUint32>(ccFaceSizes.size());
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes);

        my_assert(status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceMap(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, numBytes, ccFaceMap.data(), NULL);

        my_assert(status == MC_NO_ERROR);

        //
		// Here we create a name for the connected component based on its properties
		// and save whether its a fragment, and if so the location of this fragment
		//

		McBool isFragment = MC_FALSE;
		McBool isDerivedFromSrcMesh = MC_FALSE;
		McFragmentLocation fragmentLocation = (McFragmentLocation)0;
		const std::string name =
			resolve_cc_name_string(context, cc, isFragment, fragmentLocation, isDerivedFromSrcMesh);

        McInt32 faceVertexOffsetBase = 0;

        char fnameBuf[64];
		sprintf(fnameBuf, ("OUT_" + name + "%d.obj").c_str(), i);
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

        std::vector<vec2> ccTexCoords; // output texture coordinates
        std::vector<McUint32> ccFaceVertexTexCoordIndices;

        // for each face in CC
        for (McInt32 f = 0; f < (McInt32)ccFaceCount; ++f)
        {
            // input mesh face index (which may be offsetted!)
            const McUint32 imFaceIdxRaw = ccFaceMap.at(f); // source- or cut-mesh
            // input mesh face index (actual index value, accounting for offset)
            McUint32 imFaceIdx = imFaceIdxRaw;
            McBool faceIsFromSrcMesh = (imFaceIdxRaw < (McUint32)srcMesh.numFaces);

            if (!faceIsFromSrcMesh)
            {
				imFaceIdx = imFaceIdxRaw - (McUint32)srcMesh.numFaces; // accounting for offset
            }

            McInt32 faceSize = ccFaceSizes.at(f);

            // for each vertex in face
            for (McInt32 v = 0; v < (McInt32)faceSize; ++v)
            {
                const McInt32 ccVertexIdx = ccFaceIndices.at((McSize)faceVertexOffsetBase + v);
                // input mesh (source mesh or cut mesh) vertex index (which may be offsetted)
                const McUint32 imVertexIdxRaw = ccVertexMap.at(ccVertexIdx);
				const McBool vertexIsFromSrcMesh = (imVertexIdxRaw < srcMesh.numVertices);
                const McBool vertexIsIntersectionPoint = (imVertexIdxRaw == MC_UNDEFINED_VALUE);
                McUint32 imVertexIdx = imVertexIdxRaw; // actual index value, accounting for offset

                if (!vertexIsFromSrcMesh)
                {
					imVertexIdx = (imVertexIdxRaw - (McUint32)srcMesh.numVertices); // account for offset
                }

                const MioMesh *inputMeshPtr = &srcMesh; // assume origin face is from source mesh

                if (!faceIsFromSrcMesh)
                {
                    inputMeshPtr = &cutMesh;
                }

                // the face on which the current cc face came from
				const McIndex* imFaceVertexIndices = inputMeshPtr->pFaceVertexIndices + (imFaceIdx * 3);

                vec2 texCoord = {{0.0, 0.0}};;

                if (vertexIsIntersectionPoint)
                { // texture coords unknown and must be computed

					//
                    // interpolate texture coords from source-mesh values
					//

					// coordinates of current point (whose barycentric coords we want)
					vec3 p = {{0.0, 0.0, 0.0}};
					p.x = (ccVertices[((McSize)ccVertexIdx * 3u) + 0u]);
					p.y = (ccVertices[((McSize)ccVertexIdx * 3u) + 1u]);
					p.z = (ccVertices[((McSize)ccVertexIdx * 3u) + 2u]);

					// vertices of the origin face (i.e. the face from which the current face came from).
					// NOTE: we have assumed triangulated input meshes for simplicity. Otherwise, interpolation
					// will be more complex, which is unnecessary for now.
					vec3 a = {{0.0, 0.0, 0.0}};
					{
						const McDouble* ptr = inputMeshPtr->pVertices + imFaceVertexIndices[0] * 3;
						a.x = ptr[0];
						a.y = ptr[1];
						a.z = ptr[2];
					}
					vec3 b = {{0.0, 0.0, 0.0}};
					{
						const McDouble* ptr = inputMeshPtr->pVertices + imFaceVertexIndices[1] * 3;
						b.x = ptr[0];
						b.y = ptr[1];
						b.z = ptr[2];
					}

					vec3 c = {{0.0, 0.0, 0.0}};
					{
						const McDouble* ptr = inputMeshPtr->pVertices + imFaceVertexIndices[2] * 3;
						c.x = ptr[0];
						c.y = ptr[1];
						c.z = ptr[2];
					}

					const vec3 bary = getBarycentricCoords(p, a, b, c);

                    // compute the texture coordinates of our intersection point by interpolation
                    // --------------------------------------------------------------------------

                    // indices of the 2d texture coords that are used by "imFaceIdx" (3 indices per face)
					const McUint32* imFaceUVIndices = inputMeshPtr->pFaceVertexTexCoordIndices + (imFaceIdx * 3);

                    // texture coordinates of vertices of origin face
					const vec2 ta = {
						inputMeshPtr->pTexCoords[imFaceUVIndices[0]*2 + 0], 
						inputMeshPtr->pTexCoords[imFaceUVIndices[0]* 2 + 1]
					};

					const vec2 tb = {
						inputMeshPtr->pTexCoords[imFaceUVIndices[1] * 2 + 0],
						inputMeshPtr->pTexCoords[imFaceUVIndices[1] * 2 + 1]
					};

					const vec2 tc = {
						inputMeshPtr->pTexCoords[imFaceUVIndices[2] * 2 + 0],
						inputMeshPtr->pTexCoords[imFaceUVIndices[2] * 2 + 1]
					};

                    // interpolate using barycentric coords
                    texCoord = (ta * bary.u) + (tb * bary.v) + (tc * bary.w);
                }
                else
                { // texture coords are known and can therefore be inferred from input mesh

                    McInt32 faceVertexOffset = -1;
                    // for each vertex index in face (assuming 3 vertices)
                    for (McInt32 p = 0; p < (McInt32)3; ++p)
                    {
						if((McInt32)imFaceVertexIndices[p] == (McInt32)imVertexIdx)
                        {
                            faceVertexOffset = p;
                            break;
                        }
                    }

                    my_assert(faceVertexOffset != -1);

                    McUint32 texCoordsIdx = *(inputMeshPtr->pFaceVertexTexCoordIndices + (imFaceIdx * 3) + faceVertexOffset); //UV_F.row(imFaceIdx)(faceVertexOffset);

					texCoord = {
						inputMeshPtr->pTexCoords[texCoordsIdx * 2 + 0],
						inputMeshPtr->pTexCoords[texCoordsIdx * 2 + 1],
					};// UV_V.row(texCoordsIdx);
                }

				//
				// do some book-keeping so that we do not store duplicates
				//

                McInt32 texCoordIndex = -1;

                std::vector<vec2>::const_iterator fiter = std::find_if(
                    ccTexCoords.cbegin(), ccTexCoords.cend(), [&](const vec2& e)
                    { return compare(e.x , texCoord.x ) && compare(e.y , texCoord.y ); });

                if (fiter != ccTexCoords.cend())
                {
                    texCoordIndex = (McInt32)std::distance(ccTexCoords.cbegin(), fiter);
                }

                if (texCoordIndex == -1)
                { // tex coord not yet stored for CC vertex in face
                    texCoordIndex = (McInt32)ccTexCoords.size();
                    ccTexCoords.push_back(texCoord);
                }

                ccFaceVertexTexCoordIndices.push_back(texCoordIndex);
            } // for (McInt32 v = 0; v < faceSize; ++v) {

            faceVertexOffsetBase += faceSize;
        } // for (McInt32 f = 0; f < ccFaceCount; ++f) {

		//
        // Save mesh to (.obj) file with texture coordinates
		// 
		// NOTE: In order to actually view the output mesh with the propagated texture coordinates,
		// the output file must be [manually edited] by adding "mtllib <name>.mtl" as the first (non-comment) 
		// line of the file. The placeholder "<name>" should be replace with either "a" or "b"
		// depending on whether the respective output mesh was derived from the source-mesh ("a.obj") or
		// the cut-mesh ("b.obj"). You can know from-which input mesh an output CC was derived based on 
		// the file name.
        // 

		mioWriteOBJ(fpath.c_str(),
					ccVertices.data(),
					nullptr, //pNormals
					(McDouble*)ccTexCoords.data(),
					ccFaceSizes.data(),
					ccFaceIndices.data(),
					ccFaceVertexTexCoordIndices.data(), // pFaceVertexTexCoordIndices
					nullptr, // pFaceVertexNormalIndices
					ccVertexCount,
					0, // numNormals
					(McUint32)ccTexCoords.size(),
					ccFaceCount);
    }

	//
	// We no longer need the mem of input meshes, so we can free them!
	//
	mioFreeMesh(&srcMesh);
	mioFreeMesh(&cutMesh);

    //
	// free connected component data
	//
	status = mcReleaseConnectedComponents(context, 0, NULL);

	my_assert(status == MC_NO_ERROR);

	//
	// destroy context
	//
	status = mcReleaseContext(context);

	my_assert(status == MC_NO_ERROR);

    return 0;
}



std::string resolve_cc_name_string(McContext context,
								   McConnectedComponent cc,
								   McBool& isFragment,
								   McFragmentLocation& fragmentLocation,
								   McBool& isDerivedFromSrcMesh)
{
	isDerivedFromSrcMesh = MC_FALSE;

	// get type
	McConnectedComponentType ccType = (McConnectedComponentType)0;

	McResult status = mcGetConnectedComponentData(context,
												  cc,
												  MC_CONNECTED_COMPONENT_DATA_TYPE,
												  sizeof(McConnectedComponentType),
												  &ccType,
												  NULL);

	my_assert(status == MC_NO_ERROR);

	std::string name;
	McPatchLocation patchLocation = (McPatchLocation)0;

	if(ccType == MC_CONNECTED_COMPONENT_TYPE_SEAM)
	{
		name += "seam";
	}
	else if(ccType == MC_CONNECTED_COMPONENT_TYPE_INPUT)
	{
		name += "input";
	}
	else
	{
		isFragment = (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
		
		name += isFragment ? "frag" : "patch";

		status = mcGetConnectedComponentData(context,
											 cc,
											 MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION,
											 sizeof(McPatchLocation),
											 &patchLocation,
											 NULL);

		my_assert(status == MC_NO_ERROR);

		name += patchLocation == MC_PATCH_LOCATION_INSIDE
					? ".ploc=in"
					: (patchLocation == MC_PATCH_LOCATION_OUTSIDE ? ".ploc=out" : ".ploc=undef");

		if(isFragment)
		{

			status = mcGetConnectedComponentData(context,
												 cc,
												 MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION,
												 sizeof(McFragmentLocation),
												 &fragmentLocation,
												 NULL);

			my_assert(status == MC_NO_ERROR);

			name += fragmentLocation == MC_FRAGMENT_LOCATION_ABOVE
						? ".floc=abv"
						: ".floc=blw"; // missing loc="undefined" case

			McFragmentSealType sType = (McFragmentSealType)0;

			status = mcGetConnectedComponentData(context,
												 cc,
												 MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE,
												 sizeof(McFragmentSealType),
												 &sType,
												 NULL);

			my_assert(status == MC_NO_ERROR);

			name += sType == MC_FRAGMENT_SEAL_TYPE_COMPLETE ? ".seal=yes" : ".seal=no";
		}
	}

	isDerivedFromSrcMesh = (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);

	// connected-components is not a fragment && it is a seam
	if(!isDerivedFromSrcMesh)
	{
		if(ccType == MC_CONNECTED_COMPONENT_TYPE_SEAM)
		{
			// get origin
			McSeamOrigin ccOrig = (McSeamOrigin)0;

			status = mcGetConnectedComponentData(context,
												 cc,
												 MC_CONNECTED_COMPONENT_DATA_ORIGIN,
												 sizeof(McSeamOrigin),
												 &ccOrig,
												 NULL);

			my_assert(status == MC_NO_ERROR);

			isDerivedFromSrcMesh = (ccOrig == McSeamOrigin::MC_SEAM_ORIGIN_SRCMESH);
			name += isDerivedFromSrcMesh ? ".orig=s" : ".orig=c";
		}
		else if(ccType == MC_CONNECTED_COMPONENT_TYPE_INPUT)
		{
			McInputOrigin ccOrig = (McInputOrigin)0;

			status = mcGetConnectedComponentData(context,
												 cc,
												 MC_CONNECTED_COMPONENT_DATA_ORIGIN,
												 sizeof(McInputOrigin),
												 &ccOrig,
												 NULL);

			my_assert(status == MC_NO_ERROR);

			isDerivedFromSrcMesh = (ccOrig == McInputOrigin::MC_INPUT_ORIGIN_SRCMESH);
			name += isDerivedFromSrcMesh ? ".orig=s" : ".orig=c";
		}
	}

	return name;
}