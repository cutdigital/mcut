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
 *  QuerySeamVerticesUnsorted.cpp
 *
 *  \brief:
 *  This tutorial shows how to query 'seam vertices' of an output connected component
 *  as a sorted array. A seam vertex is one which lies along the intersection 
 *  contour/path as a result of a cut. The code also shows you how to identify individual 
 *  contours/paths.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"
#include "mio/mio.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);

    my_assert (status == MC_NO_ERROR);

    //
	// do it
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

    my_assert (status == MC_NO_ERROR);

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

    status = mcGetConnectedComponents(
        context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL, &connectedComponentCount);

    my_assert (status == MC_NO_ERROR);

    if (connectedComponentCount == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    connectedComponents.resize(connectedComponentCount);

    status = mcGetConnectedComponents(
        context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, (McUint32)connectedComponents.size(),
        connectedComponents.data(), NULL);

    my_assert (status == MC_NO_ERROR);

    //
    //  query the data of each connected component 
    // 

    for (int i = 0; i < (int)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i]; // connected compoenent id

        McSize numBytes = 0;

        //
        // Query the seam vertices (indices)
        // 

        // The format of this array (of 32-bit unsigned int elements) is as follows:
        // [
        //      <num-total-sequences>, // McIndex/McUint32
        //      <num-vertices-in-1st-sequence>, // McIndex/McUint32
        //      <1st-sequence-is-loop-flag>, // McBool/McIndex/McUint32
        //      <vertex-indices-of-1st-sequence>, // consecutive elements of McIndex
        //      <num-vertices-in-2nd-sequence>,
        //      <2nd-sequence-is-loop-flag>,
        //      <vertex-indices-of-2nd-sequence>,
        //      ... and so on, until last sequence
        // ]

        status = mcGetConnectedComponentData(
            context, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX_SEQUENCE, 0,
            NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> seamVertexSequenceArrayFromMCUT;
        seamVertexSequenceArrayFromMCUT.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(
            context, cc, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX_SEQUENCE,
            numBytes, seamVertexSequenceArrayFromMCUT.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        // We now put each sorted sequence of seam vertices into its own array.
        // This serves two purposes:
        // 1. to make it easier to write the sequence vertex list to file
        // 2. to show users how to access the ordered sequences of vertices per
        // seam/intersection contour

        // list of seam vertex sequences, each with a flag stating whether it is a
        // loop or not
        std::vector<std::pair<std::vector<McUint32>, McBool>> seamVertexSequences;
        McUint32 runningOffset = 0; // a runnning offset that we use to access data
                                    // in "seamVertexSequenceArrayFromMCUT"
        // number of sequences produced by MCUT
        const McUint32 numSeamVertexSequences = seamVertexSequenceArrayFromMCUT[runningOffset++]; // always the first
                                                                                                  // element

        // for each sequence...
        for (McUint32 numSeamVertexSequenceIter = 0;
             numSeamVertexSequenceIter < numSeamVertexSequences;
             ++numSeamVertexSequenceIter) {
            // create entry to store own array and flag
			seamVertexSequences.push_back(std::pair<std::vector<McUint32>, McBool>());
			std::pair<std::vector<McUint32>, McBool>& currentSeamVertexSequenceData = seamVertexSequences.back();

            // Ordered list of vertex indices in CC defining the seam sequence
            std::vector<McUint32>& currentSeamVertexSequenceIndices = currentSeamVertexSequenceData.first; 
            // does it form a loop? (auxilliary piece of info that might be useful to users)
            McBool& isLoop = currentSeamVertexSequenceData.second; 

            // NOTE: The order in which we do things here matters because of how we rely on
            // "runningOffset++". For each sequence we have 
            // 1) the number of vertex indices in that sequence 
            // 2) a flag telling us whether the sequence is a loop, and 
            // 3) the sorted list of vertex indices that for the sequence
            const McUint32 currentSeamVertexSequenceIndicesArraySize = seamVertexSequenceArrayFromMCUT[runningOffset++];
            
            currentSeamVertexSequenceIndices.resize(currentSeamVertexSequenceIndicesArraySize);

            isLoop = seamVertexSequenceArrayFromMCUT[runningOffset++];

            // copy seam vertex indices of current sequence into local array
            memcpy(currentSeamVertexSequenceIndices.data(),
                seamVertexSequenceArrayFromMCUT.data() + runningOffset,
                sizeof(McUint32) * currentSeamVertexSequenceIndicesArraySize);

            runningOffset += currentSeamVertexSequenceIndicesArraySize;
        }

        //
        // We are now going to save the sequences to file. 
        //

        char seamFnameBuf[512];
		sprintf(seamFnameBuf, OUTPUT_DIR "/OUT_conncomp%d-seam-vertices.txt", i);

		printf("write:  %s\n", seamFnameBuf);
		std::ofstream f(seamFnameBuf);
        // for each sequence
        for (McUint32 j = 0; j < (McUint32)seamVertexSequences.size(); ++j) {

            
			f << "sequence = " << j << "\n";
            f << "size = " << (McUint32)seamVertexSequences[j].first.size() << "\n";
            f << "is_loop = " << ((seamVertexSequences[j].second == MC_TRUE) ? "TRUE" : "FALSE") << "\n";
            f << "vertices = ";
            
            for(McInt32 i = 0; i < (McInt32)seamVertexSequences[j].first.size(); ++i)
            {
                f << seamVertexSequences[j].first[i] << " ";
            }

            f << "\n\n";
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

