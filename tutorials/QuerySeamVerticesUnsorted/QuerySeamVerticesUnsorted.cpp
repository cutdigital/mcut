/***************************************************************************
 *
 *  Copyright (C) 2023 CutDigital Enterprise Ltd
 *  Licensed under the GNU GPL License, Version 3.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      https://www.gnu.org/licenses/gpl-3.0.html.
 *
 *  For your convenience, a copy of the License has been included in this
 *  repository.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  QuerySeamVerticesUnsorted.cpp
 *
 *  \brief:
 *  This tutorial shows how to query 'seam vertices' of an output connected component.
 *  A seam vertex is one which lies along the intersection contour/path as a result
 *  of a cut. The example code provide only shows how to query the seam vertices in 
 *  UNSORTED order i.e. MCUT just give you back an array containing all the vertices
 *  in a connected component that lie on the intersection contour.
 * 
 *  For an example of how to request a sorted array of seam vertices (which also 
 *  identifies individual contours/paths) refer to the tutorial file called: 
 *  "QuerySeamVerticesSorted".
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"
#include "mio/mio.h"

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>
#include <fstream>

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::exit(1);                               \
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

	mioReadOFF(DATA_DIR "/source-mesh.off",
			   &srcMesh.pVertices,
			   &srcMesh.pFaceVertexIndices,
               &srcMesh.pFaceSizes,
			   &srcMesh.numVertices,
			   &srcMesh.numFaces);

	//
	// read-in the cut-mesh from file
	//

	mioReadOFF(DATA_DIR "/cut-mesh.off",
			   &cutMesh.pVertices,
			   &cutMesh.pFaceVertexIndices,
               &cutMesh.pFaceSizes,
			   &cutMesh.numVertices,
			   &cutMesh.numFaces);

    //
    // create a context
    // 

    McContext context = MC_NULL_HANDLE;

    McResult status = mcCreateContext(&context, MC_DEBUG);

    my_assert (status == MC_NO_ERROR);

    //
    // do the cutting
    // 

    status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
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
	// We no longer need the mem of input meshes, so we can free it!
	//
	mioFreeMesh(&srcMesh);
	mioFreeMesh(&cutMesh);

    //
    // query the number of available connected components after the cut
    // 
    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL, &connectedComponentCount);

    my_assert (status == MC_NO_ERROR);

    if (connectedComponentCount == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    connectedComponents.resize(connectedComponentCount);

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);

    my_assert (status == MC_NO_ERROR);

    //
    //  query the data of each connected component 
    // 

    for (McInt32 i = 0; i < (McInt32)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i]; // connected compoenent id

        McSize numBytes = 0;

        //
        // query the seam ccVertices (indices)
        // 
        
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> seamVertexIndices;
        seamVertexIndices.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, numBytes, seamVertexIndices.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        { // save the seam vertex indices
            char seamFnameBuf[512];
            sprintf(seamFnameBuf, OUTPUT_DIR "/OUT_conncomp%d-seam-vertices.txt", i);
			printf("write:  %s\n", seamFnameBuf);
            std::ofstream f(seamFnameBuf);
            f << "total = " << seamVertexIndices.size() << "\n";
            f << "indices = ";
            
            for(McInt32 i = 0; i < (McInt32)seamVertexIndices.size(); ++i)
            {
                f << seamVertexIndices[i] << " ";
            }
            f << "\n";
        }

        //
        // vertices
        //

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McDouble) * 3));

        std::vector<McDouble> ccVertices(ccVertexCount * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
        //  (triangulated) faces
        //

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        McUint32 numberOfTriangles = (McUint32)(numBytes / (sizeof(McIndex) * 3));

        std::vector<McIndex> triangleIndices(numberOfTriangles * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes, (void*)triangleIndices.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
		// save connected component (mesh) to a .off file
		//
        // NOTE: we save the output file in .off format (rather than .obj)
        // because .off uses relative/zero-based indexing of vertices, which 
        // makes it easier to inspect the dumped seam-vertex text file.
        // 
         
        char fnameBuf[64];
		sprintf(fnameBuf, "OUT_conncomp%d.off", i);
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

        
        mioWriteOFF(fnameBuf,
            ccVertices.data(),
            triangleIndices.data(),
            NULL, // if null then function treats "pFaceVertexIndices" array as made up of triangles
            NULL, // we don't care about writing edges
            ccVertexCount,
            numberOfTriangles,
            0 // zero edges since we don't care about writing edges
        );
    }

    //
    // free memory of _all_ connected components (could also free them individually inside above for-loop)
    // 

    status = mcReleaseConnectedComponents(context, 0, NULL);

    my_assert (status == MC_NO_ERROR);

    //
    // free memory of context
    // 
    
    status = mcReleaseContext(context);

    my_assert (status == MC_NO_ERROR);

    return 0;
}
