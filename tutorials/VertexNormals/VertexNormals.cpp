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
 *  VertexNormals.cpp
 *
 *  \brief:
 *  This tutorial shows how to propagate per-vertex normals (e.g. for smooth 
 *  shading) from input meshes and onto the output connected components after 
 *  cutting.
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
#include <string>
#include <cmath> // std::sqrt

#define my_assert(cond)                             \
    if (!(cond))                                    \
    {                                               \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::abort();                               \
    }

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
	vec3 out = {{0.0, 0.0, 0.0}};;
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
	McDouble nu=0.0, nv=0.0, ood=0.0;
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

	vec3 result = {{0.0, 0.0, 0.0}};;
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
	mioReadOBJ(DATA_DIR "/cube.obj",
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

	mioReadOBJ(DATA_DIR "/plane.obj",
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
	// create a context
	//

	McContext context = MC_NULL_HANDLE;

	McResult status = mcCreateContext(&context, MC_DEBUG);

	my_assert(status == MC_NO_ERROR);

    //
	//  do the cutting
	//
	status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_VERTEX_MAP | MC_DISPATCH_INCLUDE_FACE_MAP, // We need vertex and face maps to propagate normals
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

	McUint32 connectedComponentCount;

	status = mcGetConnectedComponents(
		context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);

	my_assert(status == MC_NO_ERROR);

	printf("connected components: %u\n", connectedComponentCount);

	if(connectedComponentCount == 0)
	{
		fprintf(stdout, "no connected components found\n");
		exit(0); // should not happen!
	}

	std::vector<McConnectedComponent> connectedComponents(connectedComponentCount, MC_NULL_HANDLE);

	status = mcGetConnectedComponents(context,
									  MC_CONNECTED_COMPONENT_TYPE_ALL,
									  (McUint32)connectedComponents.size(),
									  connectedComponents.data(),
									  NULL);

	my_assert(status == MC_NO_ERROR);

    //
	// query the data of each connected component
	//

    for(McUint32 ci = 0; ci < (McUint32)connectedComponents.size(); ++ci)
    {
        McConnectedComponent cc = connectedComponents[ci]; // connected component id

        //
		//  vertices
		//

		McSize numBytes = 0;

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

		my_assert(status == MC_NO_ERROR);

		McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McDouble) * 3));
		std::vector<McDouble> ccVertices(ccVertexCount * 3u, 0.0);

		status = mcGetConnectedComponentData(context,
											 cc,
											 MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE,
											 numBytes,
											 (void*)ccVertices.data(),
											 NULL);

		my_assert(status == MC_NO_ERROR);

        //
		//  faces
		//

		numBytes = 0;

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);

		my_assert(status == MC_NO_ERROR);

		std::vector<McUint32> ccFaceIndices(numBytes / sizeof(McUint32), 0u);

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);

		my_assert(status == MC_NO_ERROR);

        //
		//  face sizes
		//

		numBytes = 0;

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);

		my_assert(status == MC_NO_ERROR);

		std::vector<McUint32> ccFaceSizes(numBytes / sizeof(McUint32), 0u);

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);

		my_assert(status == MC_NO_ERROR);

        //
		// vertex map
		//

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes);

		my_assert(status == MC_NO_ERROR);

		std::vector<McUint32> ccVertexMap(numBytes / sizeof(McUint32), 0u);

		status = mcGetConnectedComponentData(context,
											 cc,
											 MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP,
											 numBytes,
											 ccVertexMap.data(),
											 NULL);

		my_assert(status == MC_NO_ERROR);

        //
		//  face map
		//

		const McUint32 ccFaceCount = static_cast<McUint32>(ccFaceSizes.size());

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes);

		my_assert(status == MC_NO_ERROR);

		std::vector<McUint32> ccFaceMap(numBytes / sizeof(McUint32), 0);

		status = mcGetConnectedComponentData(
			context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, numBytes, ccFaceMap.data(), NULL);

		my_assert(status == MC_NO_ERROR);

        //
		// Here we create a name for the connected component based on its properties
		// and save whether its a fragment, and if so the location of this fragment
		//

		McBool isFragment = false;
		McFragmentLocation fragmentLocation = (McFragmentLocation)0;
		const std::string name = resolve_cc_name_string(context, cc, isFragment, fragmentLocation);

        McInt32 faceVertexOffsetBase = 0;

        std::vector<vec3> ccVertexNormals(ccVertexCount, {0., 0., 0.});
		std::vector<McUint32> ccFaceVertexNormalIndices; // each vertex get its own (averaged) normal from neighbouring faces
		
        // intersection points do not have a normal value that can be copied (inferred) from an input
        // mesh, it has to be computed by interpolating normals on the origin/proginator face in the input mesh.
        // We keep a reference count to compute averaged normals per intersection point.
        std::map<McInt32, McInt32> ccSeamVertexToRefCount;
       
        //std::vector<McInt32> ccReversedFaces;

        // for each face in CC
        for (McInt32 f = 0; f < (McInt32)ccFaceCount; ++f)
        {

            // input mesh face index (which may be offsetted!)
			const McUint32 imFaceIdxRaw = ccFaceMap.at(f); // source- or cut-mesh
            // input mesh face index (actual index value, accounting for offset)
			McUint32 imFaceIdx = imFaceIdxRaw;
            const McBool faceIsFromSrcMesh = (imFaceIdxRaw < (McUint32)srcMesh.numFaces);
            McBool flipNormalsOnFace = MC_FALSE; 

            if (!faceIsFromSrcMesh)
            {
                imFaceIdx = imFaceIdxRaw - srcMesh.numFaces; // accounting for offset

				my_assert(imFaceIdx < (srcMesh.numFaces + cutMesh.numFaces)); // in case of underflow

                flipNormalsOnFace = (isFragment == MC_TRUE && fragmentLocation == MC_FRAGMENT_LOCATION_ABOVE);
            }

            const McUint32 faceSize = (McUint32)ccFaceSizes.at(f);

			my_assert(faceSize >= 3);

            // for each vertex in face
			for(McUint32 v = 0; v < faceSize; ++v)
            {
                const McUint32 ccVertexIdx = ccFaceIndices[(McSize)faceVertexOffsetBase + v];
                // input mesh (source mesh or cut mesh) vertex index (which may be offsetted)
                const McUint32 imVertexIdxRaw = ccVertexMap.at(ccVertexIdx);
				const McBool vertexIsFromSrcMesh = (imVertexIdxRaw < srcMesh.numVertices);
				const McBool isSeamVertex = (imVertexIdxRaw == MC_UNDEFINED_VALUE);
                McUint32 imVertexIdx = imVertexIdxRaw; // actual index value, accounting for offset

                if (!vertexIsFromSrcMesh)
                {
                    imVertexIdx = (imVertexIdxRaw - srcMesh.numVertices); // account for offset
					my_assert(imVertexIdx < cutMesh.numVertices || isSeamVertex);
                }

                const MioMesh *inputMeshPtr = &srcMesh; // assume origin face is from source mesh

                if (!faceIsFromSrcMesh) // is cut-mesh face
                {
                    inputMeshPtr = &cutMesh;
                }

				// indices of the normals that are used by "imFaceIdx"
				const McUint32* imFaceNormalIndices = inputMeshPtr->pFaceVertexNormalIndices + (imFaceIdx * 3);

                // the input-mesh face from which the current cc face came
				// NOTE: there is a presumption here that the input mesh is a triangle-mesh!
				const McUint32* imFaceVertexIndices = inputMeshPtr->pFaceVertexIndices + (imFaceIdx*3);

                if (isSeamVertex)
                { // normal is unknown and must be inferred

                    // interpolate normal vector components from input-mesh values

                    // coordinates of current point (whose barycentric coords we want)
					vec3 p = {{0.0, 0.0, 0.0}};;
					p.x = (ccVertices[((McSize)ccVertexIdx * 3u) + 0u]);
					p.y = (ccVertices[((McSize)ccVertexIdx * 3u) + 1u]);
					p.z = (ccVertices[((McSize)ccVertexIdx * 3u) + 2u]);
	 
					// vertices of the origin face (i.e. the face from which the current face came from).
					// NOTE: we have assumed triangulated input meshes for simplicity. Otherwise, interpolation
					// will be more complex, which is unnecessary for now.
					vec3 a = {{0.0, 0.0, 0.0}};;
					{
						const McDouble* ptr = inputMeshPtr->pVertices + imFaceVertexIndices[0] * 3;
						a.x = ptr[0];
						a.y = ptr[1];
						a.z = ptr[2];
					}
					vec3 b = {{0.0, 0.0, 0.0}};;
					{
						const McDouble* ptr = inputMeshPtr->pVertices + imFaceVertexIndices[1] * 3;
						b.x = ptr[0];
						b.y = ptr[1];
						b.z = ptr[2];
					}

					vec3 c = {{0.0, 0.0, 0.0}};;
					{
						const McDouble* ptr = inputMeshPtr->pVertices + imFaceVertexIndices[2] * 3;
						c.x = ptr[0];
						c.y = ptr[1];
						c.z = ptr[2];
					}

					const vec3 bary = getBarycentricCoords(p, a, b, c);

					// normal coordinates of vertices in the origin face

					vec3 normalA = {{0.0, 0.0, 0.0}};;
					{
						const McDouble* ptr = inputMeshPtr->pNormals + (imFaceNormalIndices[0] * 3);
						normalA.x = ptr[0];
						normalA.y = ptr[1];
						normalA.z = ptr[2];
					}
					vec3 normalB = {{0.0, 0.0, 0.0}};;
					{
						const McDouble* ptr = inputMeshPtr->pNormals + (imFaceNormalIndices[1] * 3);
						normalB.x = ptr[0];
						normalB.y = ptr[1];
						normalB.z = ptr[2];
					}
					vec3 normalC = {{0.0, 0.0, 0.0}};;
					{
						const McDouble* ptr = inputMeshPtr->pNormals + (imFaceNormalIndices[2] * 3);
						normalC.x = ptr[0];
						normalC.y = ptr[1];
						normalC.z = ptr[2];
					}

					
					// interpolate at point "p" using barycentric coords
					const vec3 normal = ((normalA * bary.u) + (normalB * bary.v) + (normalC * bary.w)) * (flipNormalsOnFace ? -1.0 : 1.0);

                    ccVertexNormals[ccVertexIdx] = ccVertexNormals[ccVertexIdx] + normal; // accumulate (so that we can later compute the average)

                    if (ccSeamVertexToRefCount.find(ccVertexIdx) == ccSeamVertexToRefCount.cend())
                    {
                        ccSeamVertexToRefCount[ccVertexIdx] = 1;
                    }
                    else
                    {
                        ccSeamVertexToRefCount[ccVertexIdx] += 1;
                    }
                }
                else
                { // normal must be inferred from input mesh

					vec3& normal = ccVertexNormals[ccVertexIdx];

					if(normal.x == McDouble(0.0) && normal.y == McDouble(0.0) &&
					   normal.z == McDouble(0.0)) // not yet copied/defined
                    {
                        McInt32 faceVertexOffset = -1;

                        // for each vertex index in face
						for(McUint32 i = 0; i < 3; ++i)
                        {
                            if ( imFaceVertexIndices[i] == imVertexIdx)
                            {
                                faceVertexOffset = i;
                                break;
                            }
                        }

                        my_assert(faceVertexOffset != -1);

                        const McUint32 imNormalIdx = imFaceNormalIndices[faceVertexOffset];
						const McDouble* imNormal = inputMeshPtr->pNormals + (imNormalIdx * 3);
						normal.x = imNormal[0];
						normal.y = imNormal[1];
						normal.z = imNormal[2];
                    }
                }
				
            } // for (McInt32 v = 0; v < faceSize; ++v) {        
			faceVertexOffsetBase += faceSize;
        } // for (McInt32 f = 0; f < ccFaceCount; ++f) {

		// for each seam-vertex
        for (std::map<McInt32, McInt32>::const_iterator it = ccSeamVertexToRefCount.cbegin(); it != ccSeamVertexToRefCount.cend(); ++it)
        {
            const McInt32 ccSeamVertexIndex = it->first;
            const McInt32 refCount = it->second;
            my_assert(refCount >= 1);
			vec3& normal = ccVertexNormals[ccSeamVertexIndex];
			normal = normal * (1.0/((McDouble) refCount)); // average

			// squared length
			const McDouble len2 = (normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z);
			const McDouble len = std::sqrt(len2); // length

			normal = normal * (1.0 / len); // re-normalize
        }

		ccFaceVertexNormalIndices = ccFaceIndices; // since number of vertices is the same as number of normals

        // save cc mesh to .obj file
        // -------------------------

        char fnameBuf[64];
        sprintf(fnameBuf, ("OUT_" + name + "-%d.obj").c_str(), ci);
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

		mioWriteOBJ(fpath.c_str(),
					ccVertices.data(),
					(McDouble*)ccVertexNormals.data(),
					nullptr, // pTexCoords
					ccFaceSizes.data(),
					ccFaceIndices.data(),
					nullptr, // pFaceVertexTexCoordIndices
					ccFaceVertexNormalIndices.data(),
					ccVertexCount,
					(McUint32)ccVertexNormals.size(),
					0, // numTexCoords
					ccFaceCount);
    }

	//
	// We no longer need the mem of input meshes, so we can free it!
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
								   McFragmentLocation& fragmentLocation)
{
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

	McBool ccIsFromSrcMesh = (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);

	// connected-components is not a fragment && it is a seam
	if(!ccIsFromSrcMesh)
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

			ccIsFromSrcMesh = (ccOrig == McSeamOrigin::MC_SEAM_ORIGIN_SRCMESH);
			name += ccIsFromSrcMesh ? ".orig=s" : ".orig=c";
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

			ccIsFromSrcMesh = (ccOrig == McInputOrigin::MC_INPUT_ORIGIN_SRCMESH);
			name += ccIsFromSrcMesh ? ".orig=s" : ".orig=c";
		}
	}

	return name;
}