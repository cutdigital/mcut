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
 *  AdjacentFaces.cpp
 *
 *  \brief:
 *  This tutorial shows how to query adjacent faces of any face of a connected 
 *  component.
 * 
 *  The tutorial is presented in the context of merging neighbouring faces of a 
 *  connected component that share some property (e.g. an ID tag), where this 
 *  property is _derived_ from origin/birth faces. 
 * 
 *  A group of faces that share the property and define a connected patch will 
 *  be merged into a single face. This is useful in situations where e.g. one 
 *  has to triangulate the faces of an input mesh before cutting and then 
 *  recover the untriangulated faces afterwards.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Efnterprise Ltd.
 *
 **************************************************************************/


#include "mcut/mcut.h"
#include "mio/mio.h"

#include <cassert>
#include <fstream>
#include <map>
#include <queue>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>

McUint32 getAdjFacesBaseOffset(const McUint32 faceIdx, const McUint32* faceAdjFacesSizes);

McUint32 getFaceIndicesBaseOffset(const McUint32 faceIdx, const McUint32* faceSizes);

void mergeAdjacentMeshFacesByProperty(
    std::vector<McUint32>& meshFaceIndicesOUT,
    std::vector<McUint32>& meshFaceSizesOUT,
    const std::vector<McUint32>& meshFaces,
    const std::vector<McUint32>& meshFaceSizes,
    const std::vector<McUint32>& meshFaceAdjFace,
    const std::vector<McUint32>& meshFaceAdjFaceSizes,
    const std::map<McInt32, std::vector<McUint32>>& tagToMeshFace,
    const std::map<McUint32, McInt32>& meshFaceToTag);

McInt32 main()
{
	MioMesh srcMesh  = {
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
    mioReadOBJ(DATA_DIR "/triangulatedGrid4x4.obj",
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

    // A map denoting the adjacent faces of the source mesh that share some property
    // (a tag/number) that we will use to merge adjacent faces.

    // Faces in each group are merged into one face/polygon.
    // Groups which are adjacent and share a tag are also merged.

    std::map<McUint32, McInt32> srcMeshFaceToTag = {
        // Bottom-right quadrant faces
        // group 0
        { 0, 0 },
        { 1, 0 },
        // group 1
        { 2, 1 },
        { 3, 1 },
        // group 2
        { 8, 2 },
        { 9, 2 },
        // group 3
        { 10, 3 },
        { 11, 3 },

        // Top-left quadrant faces

        // group 4
        // For the sake of demonstration, we use the same tag ("0") for group 4 and group 0.
        // This is fine because the triangles/faces of group 0 are not adjacent with any
        // triangles/faces in group 4, which mean group 0 and 4 result in two separate faces
        // after merging.
        { 20, 0 },
        { 21, 0 },
        { 22, 0 },
        { 23, 0 },
        { 28, 0 },
        { 29, 0 },
        { 30, 0 },
        { 31, 0 },

        // Top-right quadrant

        // group 5 (note: this group is merged with group-2)
        { 16, 2 },
        // group 6
        { 17, 0xBEEF }
    };

    std::map<McInt32, std::vector<McUint32>> tagToSrcMeshFaces;
    for (std::map<McUint32, McInt32>::const_iterator i = srcMeshFaceToTag.cbegin(); i != srcMeshFaceToTag.cend(); ++i) {
        tagToSrcMeshFaces[i->second].push_back(i->first);
    }

    //
	// read-in the cut-mesh from file
	// 

    mioReadOBJ(DATA_DIR "/quad.obj",
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
    
    if (status != MC_NO_ERROR)
    {
		fprintf(stderr, "mcCreateContext failed (%d)\n", (McInt32)status);
		exit(1);
    }

    status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_FACE_MAP,
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

    if(status != MC_NO_ERROR)
    {
	    fprintf(stderr, "mcDispatch failed (%d)\n", (McInt32)status);
	    exit(1);
    }

    //
    // MCUT is not longer using mem of input meshes, so we can free it!
    //
	mioFreeMesh(&srcMesh);
	mioFreeMesh(&cutMesh);

    //
    // query the number of available connected components
    // 

    McUint32 connectedComponentCount = 0;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);
    
    if(status != MC_NO_ERROR)
	{
		fprintf(stderr, "1:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_ALL) failed (%d)\n", (McInt32)status);
		exit(1);
	}

    printf("connected components: %d\n", (McInt32)connectedComponentCount);

    if (connectedComponentCount == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    std::vector<McConnectedComponent> connectedComponents(connectedComponentCount, MC_NULL_HANDLE);
    connectedComponents.resize(connectedComponentCount);
    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);

    if(status != MC_NO_ERROR)
	{
		fprintf(stderr,
				"2:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_ALL) failed (%d)\n",
				(McInt32)status);
		exit(1);
	}

    //
    // query the data of each connected component 
    // 

    for (McInt32 c = 0; c < (McInt32)connectedComponentCount; ++c) {
        
        McConnectedComponent cc = connectedComponents[c];

        //
        // type
        //
		McConnectedComponentType type = (McConnectedComponentType)0;

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &type, NULL);

        if(status != MC_NO_ERROR)
		{
			fprintf(
				stderr,
				"1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_TYPE) failed (%d)\n",
				(McInt32)status);
			exit(1);
		}

        if (!(type == MC_CONNECTED_COMPONENT_TYPE_INPUT || type == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT)) {
            // we only care about the input source mesh, and the "fragment" connected components
            continue;
        }

        if (type == MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            
            McInputOrigin origin = (McInputOrigin)0;
            
            status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McInputOrigin), &origin, NULL);
			
            if(status != MC_NO_ERROR)
			{
				fprintf(stderr,
						"mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_ORIGIN)"
						"failed (%d)\n",
						(McInt32)status);
				exit(1);
			}
            
            if (origin == MC_INPUT_ORIGIN_CUTMESH) {
                continue; // we only care about the source mesh
            }
        }

        //
        // vertices
        // 

        McSize numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (%d)\n", (McInt32)status);
			exit(1);
		}

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McDouble) * 3));
        std::vector<McDouble> ccVertices((McSize)ccVertexCount * 3u, 0);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (McVoid*)ccVertices.data(), NULL);
		
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        //
        // faces
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
		
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        std::vector<McUint32> ccFaceIndices(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);
		
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        //
        // face sizes
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_SIZE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        std::vector<McUint32> ccFaceSizes(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_SIZE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        const McUint32 ccFaceCount = static_cast<McUint32>(ccFaceSizes.size());

        {
            char buf[512];
            sprintf(buf, OUTPUT_DIR "/cc%d.obj", c);
            
            mioWriteOBJ(buf,
						&ccVertices[0],
						nullptr,
						nullptr,
						&ccFaceSizes[0],
						&ccFaceIndices[0],
						nullptr,
						nullptr,
						ccVertexCount,
						0,
						0,
						ccFaceCount);
        }

        //
        // face map
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_MAP) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        std::vector<McUint32> ccFaceMap(numBytes / sizeof(McUint32), 0);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, numBytes, ccFaceMap.data(), NULL);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_MAP) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        //
        // face adjacency info
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE, 0, NULL, &numBytes);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        std::vector<McUint32> ccFaceAdjFaces(numBytes / sizeof(McUint32), 0);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE, numBytes, ccFaceAdjFaces.data(), NULL);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        //
        // face adjacency sizes (number of adjacent faces per face)
        // 
        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE, 0, NULL, &numBytes);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        std::vector<McUint32> ccFaceAdjFacesSizes(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE, numBytes, ccFaceAdjFacesSizes.data(), NULL);
        
        if(status != MC_NO_ERROR)
		{
			fprintf(stderr,
					"2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE"
					"SIZE) "
					"failed (%d)\n",
					(McInt32)status);
			exit(1);
		}

        //
        // resolve mapping between tags and CC faces
        //
        
        // NOTE: only those CC face whose origin face was tagged will themselves be tagged.
        
        std::map<McInt32, std::vector<McUint32>> tagToCcFaces;
        std::map<McUint32, McInt32> ccFaceToTag;

        for (McInt32 ccFaceID = 0; ccFaceID < (McInt32)ccFaceCount; ++ccFaceID) {
            
            McInt32 imFaceID = ccFaceMap[ccFaceID];
            std::map<McUint32, McInt32>::const_iterator srcMeshFaceToTagIter = srcMeshFaceToTag.find(imFaceID);
            bool faceWasTagged = (srcMeshFaceToTagIter != srcMeshFaceToTag.cend());

            if (faceWasTagged) {
                const McInt32 tag = srcMeshFaceToTagIter->second;
                ccFaceToTag[ccFaceID] = tag;
                tagToCcFaces[tag].push_back(ccFaceID);
            }
        }

        for (std::map<McUint32, McInt32>::const_iterator i = srcMeshFaceToTag.cbegin(); i != srcMeshFaceToTag.cend(); ++i) {
            tagToSrcMeshFaces[i->second].push_back(i->first);
        }

        std::vector<McUint32> ccFaceIndicesMerged;
        std::vector<McUint32> ccFaceSizesMerged;

        mergeAdjacentMeshFacesByProperty(
            ccFaceIndicesMerged,
            ccFaceSizesMerged,
            ccFaceIndices,
            ccFaceSizes,
            ccFaceAdjFaces,
            ccFaceAdjFacesSizes,
            tagToCcFaces,
            ccFaceToTag);

        {
            char buf[512];
            sprintf(buf, OUTPUT_DIR "/cc%d-merged.obj", c);
            
            // NOTE: For the sake of simplicity, we keep unreferenced vertices.
            
            mioWriteOBJ(buf,
						&ccVertices[0],
						nullptr,
						nullptr,
						&ccFaceSizesMerged[0],
						&ccFaceIndicesMerged[0],
						nullptr,
						nullptr,
						ccVertexCount,
						0,
						0,
						(McUint32)ccFaceSizesMerged.size());
        }
    }

    //
    // free connected component data
    // 

    status = mcReleaseConnectedComponents(context, (McUint32)connectedComponents.size(), connectedComponents.data());

	if(status != MC_NO_ERROR)
	{
		fprintf(stderr, "mcReleaseConnectedComponents failed (%d)\n", (McInt32)status);
		exit(1);
	}

    //
    // destroy context
    // 
    
    status = mcReleaseContext(context);

    if(status != MC_NO_ERROR)
	{
		fprintf(stderr, "mcReleaseContext failed (%d)\n", (McInt32)status);
		exit(1);
	}

    return 0;
}

McUint32 getAdjFacesBaseOffset(const McUint32 faceIdx, const McUint32* faceAdjFacesSizes)
{
    McUint32 baseOffset = 0;
    for (McUint32 f = 0; f < faceIdx; ++f) {
        baseOffset += faceAdjFacesSizes[f];
    }
    return baseOffset;
}

McUint32 getFaceIndicesBaseOffset(const McUint32 faceIdx, const McUint32* faceSizes)
{
    McUint32 baseOffset = 0;
    for (McUint32 f = 0; f < faceIdx; ++f) {
        baseOffset += faceSizes[f];
    }
    return baseOffset;
};

void mergeAdjacentMeshFacesByProperty(
    std::vector<McUint32>& meshFaceIndicesOUT,
    std::vector<McUint32>& meshFaceSizesOUT,
    const std::vector<McUint32>& meshFaces,
    const std::vector<McUint32>& meshFaceSizes,
    const std::vector<McUint32>& meshFaceAdjFace,
    const std::vector<McUint32>& meshFaceAdjFaceSizes,
    const std::map<McInt32, std::vector<McUint32>>& tagToMeshFace,
    const std::map<McUint32, McInt32>& meshFaceToTag)
{
    // for each tag
    for (std::map<McInt32, std::vector<McUint32>>::const_iterator iter = tagToMeshFace.cbegin(); iter != tagToMeshFace.cend(); ++iter) {

        // NOTE: may contain faces that form disjoint patches i.e. not all faces
        // are merged into one. It is possible to create more than one new face
        // after the merging where the resulting faces after merging are not adjacent.
        std::vector<McUint32> meshFacesWithSameTag = iter->second; // copy!

        // merge the faces that are adjacent
        std::vector<std::vector<McUint32>> adjacentFaceLists; // i.e. each element is a patch/collection of adjacent faces

        do {
            adjacentFaceLists.push_back(std::vector<McUint32>()); // add new patch
            std::vector<McUint32>& curAdjFaceList = adjacentFaceLists.back();

            // queue of adjacent faces
            std::deque<McUint32> adjFaceQueue;
            adjFaceQueue.push_back(meshFacesWithSameTag.back()); // start with any
            meshFacesWithSameTag.pop_back();
            do {
                McUint32 cur = adjFaceQueue.front();
                adjFaceQueue.pop_front();
                const McInt32 numAdjFaces = meshFaceAdjFaceSizes[cur];
                const McInt32 ccFaceAdjFacesBaseOffset = getAdjFacesBaseOffset(cur, meshFaceAdjFaceSizes.data());

                curAdjFaceList.push_back(cur);

                // for each adjacent face of current face
                for (McInt32 i = 0; i < numAdjFaces; ++i) {
                    const McUint32 adjFaceID = meshFaceAdjFace[(size_t)ccFaceAdjFacesBaseOffset + i];

                    std::vector<McUint32>::const_iterator curAdjFaceListIter = std::find(curAdjFaceList.cbegin(), curAdjFaceList.cend(), adjFaceID);
                    bool alreadyAddedToCurAdjFaceList = (curAdjFaceListIter != curAdjFaceList.cend());

                    if (!alreadyAddedToCurAdjFaceList) {

                        // does the adjacent face share a Tag..?
                        std::vector<McUint32>::const_iterator fiter = std::find(iter->second.cbegin(), iter->second.cend(), adjFaceID);
                        bool haveSharedTag = (fiter != iter->second.cend());

                        if (haveSharedTag) {

                            std::deque<McUint32>::const_iterator queueIter = std::find(adjFaceQueue.cbegin(), adjFaceQueue.cend(), adjFaceID);
                            bool alreadyAddedToAdjFaceQueue = (queueIter != adjFaceQueue.end());
                            if (!alreadyAddedToAdjFaceQueue) {
                                adjFaceQueue.push_back(adjFaceID); // add it!

                                std::vector<McUint32>::iterator facesWithSharedTagIter = std::find(meshFacesWithSameTag.begin(), meshFacesWithSameTag.end(), adjFaceID);
                                if (facesWithSharedTagIter != meshFacesWithSameTag.cend()) {
                                    meshFacesWithSameTag.erase(facesWithSharedTagIter); // remove since we have now associated with patch.
                                }
                            }
                        }
                    }
                }

            } while (!adjFaceQueue.empty());
        } while (!meshFacesWithSameTag.empty());

        for (std::vector<std::vector<McUint32>>::const_iterator adjacentFaceListsIter = adjacentFaceLists.cbegin();
             adjacentFaceListsIter != adjacentFaceLists.cend();
             ++adjacentFaceListsIter) {

            // Unordered list of halfedges which define the boundary of our new
            // face
            std::vector<std::pair<McInt32, McInt32>> halfedgePool;

            for (McInt32 f = 0; f < (McInt32)adjacentFaceListsIter->size(); ++f) {

                const McUint32 meshFaceID = adjacentFaceListsIter->at(f);
                const McUint32 meshFaceVertexCount = meshFaceSizes[meshFaceID];
                const McUint32 baseIdx = getFaceIndicesBaseOffset(meshFaceID, meshFaceSizes.data());
                const McInt32 numFaceEdges = (McInt32)meshFaceVertexCount; // NOTE: a polygon has the same number of vertices as its edges.

                // for each edge of face
                for (McInt32 faceEdgeID = 0; faceEdgeID < numFaceEdges; ++faceEdgeID) {

                    const McInt32 srcIdx = faceEdgeID;
                    const McInt32 tgtIdx = (faceEdgeID + 1) % meshFaceVertexCount;
                    const McUint32 srcVertexIdx = meshFaces[(size_t)baseIdx + srcIdx];
                    const McUint32 tgtVertexIdx = meshFaces[(size_t)baseIdx + tgtIdx];

                    std::vector<std::pair<McInt32, McInt32>>::iterator fiter = std::find_if(
                        halfedgePool.begin(),
                        halfedgePool.end(),
                        [&](const std::pair<McInt32, McInt32>& elem) {
                            return ((McUint32)elem.first == srcVertexIdx && (McUint32)elem.second == tgtVertexIdx) || //
                                ((McUint32)elem.second == srcVertexIdx && (McUint32)elem.first == tgtVertexIdx);
                        });

                    const bool opposite_halfedge_exists = (fiter != halfedgePool.cend());

                    if (opposite_halfedge_exists) {
                        halfedgePool.erase(fiter);
                    } else {
                        halfedgePool.emplace_back(srcVertexIdx, tgtVertexIdx);
                    }
                }
            }

            std::map<McInt32, std::vector<McInt32>> vertexToHalfedges;

            for (McInt32 i = 0; i < (McInt32)halfedgePool.size(); ++i) {
                std::pair<McInt32, McInt32> halfedge = halfedgePool[i];
                vertexToHalfedges[halfedge.first].push_back(i);
                vertexToHalfedges[halfedge.second].push_back(i);
            }

            std::vector<McUint32> polygon;
            std::map<McInt32, std::vector<McInt32>>::const_iterator cur;
            std::map<McInt32, std::vector<McInt32>>::const_iterator next = vertexToHalfedges.cbegin(); // could start from any

            do {
                cur = next;
                next = vertexToHalfedges.cend();
                polygon.push_back(cur->first);

                // find next (pick the halfedge whose "source" is the current vertex)
                std::vector<McInt32> halfedges = cur->second;

                for (McInt32 i = 0; i < 2; ++i) {
                    std::pair<McInt32, McInt32> edge = halfedgePool[halfedges[i]];
                    if (edge.first == cur->first && std::find(polygon.cbegin(), polygon.cend(), (McUint32)edge.second) == polygon.cend()) {
                        next = vertexToHalfedges.find(edge.second);
                        assert(next != vertexToHalfedges.cend());
                        break;
                    }
                }

            } while (next != vertexToHalfedges.cend());

            meshFaceIndicesOUT.insert(meshFaceIndicesOUT.end(), polygon.cbegin(), polygon.cend());
            meshFaceSizesOUT.push_back((McUint32)polygon.size());
        }
    }

    // Now we add the untagged faces into the new mesh (the ones which did not need merging)

    for (McInt32 meshFaceID = 0; meshFaceID < (McInt32)meshFaceSizes.size(); ++meshFaceID) {
        bool faceWasMerged = meshFaceToTag.find(meshFaceID) != meshFaceToTag.cend();
        if (!faceWasMerged) {
            const McUint32 baseIdx = getFaceIndicesBaseOffset(meshFaceID, meshFaceSizes.data());
            const McUint32 meshFaceVertexCount = meshFaceSizes[meshFaceID];

            for (McInt32 i = 0; i < (McInt32)meshFaceVertexCount; ++i) {
                meshFaceIndicesOUT.push_back(meshFaces[(size_t)baseIdx + i]);
            }

            meshFaceSizesOUT.push_back(meshFaceVertexCount);
        }
    }
}
