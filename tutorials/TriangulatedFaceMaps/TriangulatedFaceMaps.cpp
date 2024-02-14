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
 *  TriangulatedFaceMaps.cpp
 *
 *  \brief:
 *  This tutorial shows how to propagate per-triangulated-face normals (flat shading) 
 * 	from input meshes and onto the output triangulated connected components after cutting.
 * 	It essentially shows you how to query the mapping between the triangulated faces of
 * 	an output connected component and the original/proginator face of an input mesh
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

#define my_assert(cond)                                                                            \
	if(!(cond))                                                                                    \
	{                                                                                              \
		fprintf(stderr, "MCUT error: %s\n", #cond);                                                \
		std::abort();                                                                              \
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

    vec3 operator*(const McDouble c) const {
		vec3 result = {};
		result.x = x * c;
		result.y = y * c;
		result.z = z * c;
		return result;
    }

    vec3 operator+(const vec3& rhs) const {
		vec3 result = {};
		result.x = x + rhs.x;
		result.y = y + rhs.y;
		result.z = z + rhs.z;
		return result;
    }

	vec3 operator-(const vec3& rhs) const
	{
		vec3 result = {};
		result.x = x - rhs.x;
		result.y = y - rhs.y;
		result.z = z - rhs.z;
		return result;
	}
};

inline McDouble
getTriangleArea2D(McDouble x1, McDouble y1, McDouble x2, McDouble y2, McDouble x3, McDouble y3)
{
	return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
}

vec3 crossProduct(const vec3& u, const vec3& v)
{
	vec3 out = {};
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

	vec3 result = {};
	result.u = nu * ood;
	result.v = nv * ood;
	result.w = 1.0f - result.u - result.v;
    return result;
}


// basic comparison of doubles
bool are_equal(const McDouble& x, const McDouble &y)
{
	const McDouble eps = 1e-6;
	const McDouble diff = (x > y) ? (x - y) : (y - x);

	return diff < eps;
}

// assigns a string name to a connected component based on it unique properties
std::string resolve_cc_name_string(McContext context,
								   McConnectedComponent cc,
								   McBool& isFragment,
								   McFragmentLocation& fragmentLocation,
								   McBool& isDerivedFromSrcMesh);



int main()
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

	McResult status = mcCreateContext(&context, MC_NULL_HANDLE);

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

    McUint32 numConnectedComponents;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnectedComponents);
    my_assert(status == MC_NO_ERROR);

    printf("connected components: %d\n", (McInt32)numConnectedComponents);

    if (numConnectedComponents == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    std::vector<McConnectedComponent> connectedComponents(numConnectedComponents, 0);
    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);
    my_assert(status == MC_NO_ERROR);

    //
	// query the data of each connected component
	//

    for (McInt32 i = 0; i < (McInt32)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i]; 

        //
		//  vertices
		//

        McSize numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        my_assert(status == MC_NO_ERROR);

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(double) * 3));
        std::vector<double> ccVertices((size_t)ccVertexCount * 3u, 0.0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);
        my_assert(status == MC_NO_ERROR);

        //
        //  [triangulated] faces
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);
        my_assert(status == MC_NO_ERROR);
        std::vector<McUint32> ccTriangulatedFaceIndices(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes, ccTriangulatedFaceIndices.data(), NULL);
        my_assert(status == MC_NO_ERROR);

        //
        //  vertex map
        // 

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes);
        my_assert(status == MC_NO_ERROR);
        std::vector<McUint32> ccVertexMap(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, numBytes, ccVertexMap.data(), NULL);
        my_assert(status == MC_NO_ERROR);

        //
        //  triangulated face map
        // 

        const McUint32 ccTriangulatedFaceCount = static_cast<McUint32>(ccTriangulatedFaceIndices.size()/3);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION_MAP, 0, NULL, &numBytes);
        
        my_assert(status == MC_NO_ERROR);
        
        std::vector<McUint32> ccTriangleFaceMap(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION_MAP, numBytes, ccTriangleFaceMap.data(), NULL);
        
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

        McUint32 faceVertexOffsetBase = 0; // offset of vertex in face
        // our array normals (what we want). This is the list that will be referred to by
        // the face-vertices of the cc (each face will have vertex indices and normal indices) 
		std::vector<McDouble> ccNormals; 

		std::vector<McUint32> ccFaceVertexNormalIndices; // normal indices reference by each face of cc

        // for each triangle in cc
        for (McInt32 t = 0; t < (McInt32)ccTriangulatedFaceCount; ++t) {

            // get the [origin] input-mesh face index (Note: this index may be offsetted
            // to distinguish between source-mesh and cut-mesh faces).
            const McUint32 imFaceIdxRaw = ccTriangleFaceMap.at(t); // source- or cut-mesh face index (we don't know yet)
            // This is how we infer which mapped indices belong to the source-mesh or the cut-mesh.
			const bool faceIsFromSrcMesh = (imFaceIdxRaw < (McUint32)srcMesh.numFaces);
			bool flipNormalsOnFace = false;
			// Now compute the actual input mesh face index (accounting for offset)
			McUint32 imFaceIdx = imFaceIdxRaw;

			if(!faceIsFromSrcMesh)
			{ // if the current face is from the cut-mesh
				imFaceIdx = (imFaceIdxRaw - (McUint32)srcMesh.numFaces); // accounting for offset
				// check if we need to flip normals on the face
				flipNormalsOnFace = (isFragment && fragmentLocation == MC_FRAGMENT_LOCATION_ABOVE);
			}

			McUint32 faceSize = 3; /// number of vertices in face

			// for each vertex in face
			for(McUint32 v = 0; v < faceSize; ++v)
			{
				const McUint32 ccVertexIdx = ccTriangulatedFaceIndices.at((McSize)faceVertexOffsetBase + v);
				const McUint32 imVertexIdxRaw = ccVertexMap.at(ccVertexIdx);
				bool vertexIsFromSrcMesh = (imVertexIdxRaw < srcMesh.numVertices);
                // i.e. a vertex along the cut-path (an intersection point)
				const bool isSeamVertex = (imVertexIdxRaw == MC_UNDEFINED_VALUE); 
				McUint32 imVertexIdx = imVertexIdxRaw; // actual index value, accounting for offset

				if(!vertexIsFromSrcMesh)
				{
                    // account for offset
					imVertexIdx = (imVertexIdxRaw - (McUint32)srcMesh.numVertices); 
				}

				const MioMesh* inputMeshPtr = faceIsFromSrcMesh ? &srcMesh : &cutMesh;

				// the face from which the current cc face came ("birth face")
				//const std::vector<int>& imFace = inputMeshPtr->F[imFaceIdx];
                // NOTE: there is a presumption here that the input mesh is a triangle-mesh!
                const McUint32* imFace = inputMeshPtr->pFaceVertexIndices + (imFaceIdx * 3);

				vec3 normal; // the normal of the current vertex

				if(isSeamVertex)
				{ // normal completely unknown and must be inferred

                    //
					// interpolate from input-mesh values
					//

					// coordinates of current point (whose barycentric coords we want)
					vec3 p = {0.0, 0.0,0.0};
					p.x = (ccVertices[((McSize)ccVertexIdx * 3u) + 0u]);
					p.y = (ccVertices[((McSize)ccVertexIdx * 3u) + 1u]);
					p.z = (ccVertices[((McSize)ccVertexIdx * 3u) + 2u]);

					// vertices of the origin face (i.e. the face from which the current face came from).
					// NOTE: we have assumed triangulated input meshes for simplicity. Otherwise, interpolation
					// will be more complex, which is unnecessary for now.
					vec3 a = {0.0, 0.0, 0.0};
					{
                        const McDouble* ptr = inputMeshPtr->pVertices + imFace[0]*3;
                        a.x = ptr[0];
                        a.y = ptr[1];
                        a.z = ptr[2];
                    }
					vec3 b = {0.0, 0.0, 0.0};
                    {
                        const McDouble* ptr = inputMeshPtr->pVertices + imFace[1]*3;
                        b.x = ptr[0];
                        b.y = ptr[1];
                        b.z = ptr[2];
                    }

                    vec3 c = {0.0, 0.0, 0.0};
                    {
                        const McDouble* ptr = inputMeshPtr->pVertices + imFace[2]*3;
                        c.x = ptr[0];
                        c.y = ptr[1];
                        c.z = ptr[2];
                    }
					
                    const vec3 bary = getBarycentricCoords(p, a, b, c);

                    //
					// compute the normal of our vertex by interpolation
					// 

					// indices of the normals that are used by the vertices defining the origin face "imFaceIdx"
					const McUint32* imFaceNormalIndices = inputMeshPtr->pFaceVertexNormalIndices + (imFaceIdx*3); //inputMeshPtr->FN[imFaceIdx];

					// normal coordinates of vertices in the origin face
					
                    vec3 normalA = {0.0, 0.0, 0.0};
					{
                        const McDouble* ptr = inputMeshPtr->pNormals + (imFaceNormalIndices[0]*3);
                        normalA.x = ptr[0];
                        normalA.y = ptr[1];
                        normalA.z = ptr[2];
                    }
					vec3 normalB = {0.0, 0.0, 0.0};
					{
                        const McDouble* ptr = inputMeshPtr->pNormals + (imFaceNormalIndices[1]*3);
                        normalB.x = ptr[0];
                        normalB.y = ptr[1];
                        normalB.z = ptr[2];
                    }
					vec3 normalC = {0.0, 0.0, 0.0};
					{
                        const McDouble* ptr = inputMeshPtr->pNormals + (imFaceNormalIndices[2]*3);
                        normalC.x = ptr[0];
                        normalC.y = ptr[1];
                        normalC.z = ptr[2];
                    }

					// interpolate at point "p" using barycentric coords
					normal = (normalA * bary.u) + (normalB * bary.v ) + (normalC * bary.w);

					// NOTE: if all three vertices have the same normal (index) then there is no need for interpolation
					// we'd just copy that value from the respective input mesh.
				}
				else
				{ // the normal is known and can therefore be inferred from the input mesh

					// the index of the mapped-to index in the input mesh
					McIndex imFaceVertexOffset = MC_UNDEFINED_VALUE;

                    // for each vertex in face
					for(McUint32 p = 0; p < inputMeshPtr->pFaceSizes[imFaceIdx]; ++p)
					{
						if(imFace[p] == imVertexIdx)
						{
							imFaceVertexOffset = p;
							break;
						}
					}

					my_assert(imFaceVertexOffset != MC_UNDEFINED_VALUE);

					// get the normal index of the vertex
					McUint32 imNormalIdx = inputMeshPtr->pFaceVertexNormalIndices[imFaceIdx*3 + imFaceVertexOffset];
					// copy the normal value from the input mesh
					const McDouble* ptr = inputMeshPtr->pNormals + (imNormalIdx * 3);
					
					normal.x = ptr[0];
                    normal.y = ptr[1];
                    normal.z = ptr[2];
				}

				// When MCUT seal's holes, it uses polygons directly from the cut mesh. These polygons
				// may require to be flipped sometimes when holes are filled (e.g. when a fragment is
				// "above"). Thus, we cannot just copy/interpolate the normal from the origin mesh in
				// such cases, we must also flip (negate) it.
				
                if(flipNormalsOnFace)
				{
					normal = normal * -1.0;
				}

                //
				// Do some book-keeping to prevent us from duplicating the normals that we write to file.
				//
                
				// has a normal with the same value already been computed?
                // pointer to the location of the existing (i.e. already computed) normal, which matches "normal"
				const McDouble* pNormalIter = nullptr; 

                // for each normal vector (in the list that we have so far)
				for(McUint32 n = 0; n < ccNormals.size() / 3; ++n)
				{
					const McDouble* pNormal = ccNormals.data() + (n * 3);
					const bool haveMatch = are_equal(normal.x, pNormal[0]) &&
										   are_equal(normal.y, pNormal[1]) &&
										   are_equal(normal.z, pNormal[2]);
					if(haveMatch)
					{
						pNormalIter = pNormal;
						break;
					}
				}

                std::ptrdiff_t normalIndex = -1;

				if(pNormalIter != nullptr)
				{
					normalIndex = (pNormalIter - ccNormals.data()) / 3;

                    my_assert(normalIndex >=0);
				}
                else{
                    normalIndex = ccNormals.size()/3;

                    ccNormals.push_back(normal.x);
					ccNormals.push_back(normal.y);
					ccNormals.push_back(normal.z);
                }

				ccFaceVertexNormalIndices.push_back(normalIndex);
			} // for (int v = 0; v < faceSize; ++v) {

			faceVertexOffsetBase += faceSize;

        } // for (McInt32 f = 0; f < ccTriangulatedFaceCount; ++f) {

        //
		// save connected component (mesh) to an .obj file
		// 

		char fnameBuf[64];
		sprintf(fnameBuf, ("OUT_" + name + "-%d.obj").c_str(), i);
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

        // each face has 3 vertices
        std::vector<McUint32> ccFaceSizes((McUint32)ccTriangulatedFaceIndices.size()/3, 3);

        mioWriteOBJ(
            fpath.c_str(), 
            ccVertices.data(), 
            ccNormals.data(), 
            nullptr, // pTexCoords
            ccFaceSizes.data(), 
            ccTriangulatedFaceIndices.data(), 
            nullptr, // pFaceVertexTexCoordIndices
            ccFaceVertexNormalIndices.data(), 
            ccVertexCount, 
            ccNormals.size()/3, 
            0, // numTexCoords
            ccTriangulatedFaceCount);
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