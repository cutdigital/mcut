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
 *  ReversedConnectedComponentFaces.cpp
 *
 *  \brief:
 *  This tutorial shows how to query faces of output connected components
 *  where the WINDING ORDER IS REVERSED.
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

#include <vector>
#include <string>

#define my_assert(cond)                                                                            \
	if(!(cond))                                                                                    \
	{                                                                                              \
		fprintf(stderr, "MCUT error: %s\n", #cond);                                                \
		std::abort();                                                                              \
	}

McInt32 main()
{
    //
    // Create meshes to intersect
    //

    // the source-mesh (a cube)
    McDouble cubeVertices[] = {
        -1, -1, 1, // 0
        1, -1, 1, // 1
        -1, 1, 1, // 2
        1, 1, 1, // 3
        -1, -1, -1, // 4
        1, -1, -1, // 5
        -1, 1, -1, // 6
        1, 1, -1 // 7
    };
    McUint32 cubeFaces[] = {
        0, 3, 2, // 0
        0, 1, 3, // 1
        1, 7, 3, // 2
        1, 5, 7, // 3
        5, 6, 7, // 4
        5, 4, 6, // 5
        4, 2, 6, // 6
        4, 0, 2, // 7
        2, 7, 6, // 8
        2, 3, 7, // 9
        4, 1, 0, // 10
        4, 5, 1, // 11

    };
    McInt32 numCubeVertices = 8;
    McInt32 numCubeFaces = 12;

    McUint32 cubeFaceSizes[] = {
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
    };

    // the cut mesh (a quad formed of two triangles)

    McDouble cutMeshVertices[] = {
        -1.2, 1.6, 0.994070,
        1.4, -1.3, 0.994070,
        -1.2, 1.6, -1.005929,
        1.4, -1.3, -1.005929
    };

    McUint32 cutMeshFaces[] = {
        1, 2, 0,
        1, 3, 2
    };

    McUint32 numCutMeshVertices = 4;
    McUint32 numCutMeshFaces = 2;

    //
    // create a context
    // 
    McContext context = MC_NULL_HANDLE;
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);

    my_assert (status == MC_NO_ERROR);

    //
    // do the cutting
    // 
    status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
        cubeVertices,
        cubeFaces,
        cubeFaceSizes,
        numCubeVertices,
        numCubeFaces,
        cutMeshVertices,
        cutMeshFaces,
        nullptr, // cutMeshFaceSizes, // no need to give 'ccFaceSizes' parameter since cut-mesh is a triangle mesh
        numCutMeshVertices,
        numCutMeshFaces);

    my_assert (status == MC_NO_ERROR);

    //
    // query the number of available connected components after the cut
    // 
    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);

    my_assert (status == MC_NO_ERROR);

    if (connectedComponentCount == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    connectedComponents.resize(connectedComponentCount);

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);

    my_assert (status == MC_NO_ERROR);

    //
    //  query the data of each connected component 
    // 

    for (McInt32 i = 0; i < (McInt32)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i]; // connected compoenent id

        //
        // vertices
        // 

        McSize numBytes = 0;

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McDouble) * 3));

        std::vector<McDouble> ccVertices(ccVertexCount * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (McVoid*)ccVertices.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        // get connected component type to determine whether we should flip faces
        // ----------------------------------------------------------------------
        McBool flip_faces = MC_TRUE;
        
        {
            McConnectedComponentType type = (McConnectedComponentType)0;

            status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), (McVoid*)&type, NULL);

            my_assert (status == MC_NO_ERROR);

            // in this tutorial we flip the faces of all fragments

            flip_faces = (type == McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);

            McConnectedComponentFaceWindingOrder windingOrder = McConnectedComponentFaceWindingOrder::MC_CONNECTED_COMPONENT_FACE_WINDING_ORDER_AS_GIVEN;

            if (flip_faces) {
                printf("** Will flip/reverse face indices!!\n");
                windingOrder = McConnectedComponentFaceWindingOrder::MC_CONNECTED_COMPONENT_FACE_WINDING_ORDER_REVERSED;
            }

            status = mcBindState(context, MC_CONTEXT_CONNECTED_COMPONENT_FACE_WINDING_ORDER, sizeof(McConnectedComponentFaceWindingOrder), &windingOrder);

            my_assert (status == MC_NO_ERROR);
        }
        
        //
        // faces
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceIndices;
        ccFaceIndices.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);

        my_assert (status == MC_NO_ERROR);
        
        //
        // query the face sizes
        // 

        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        
        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceSizes;
        ccFaceSizes.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
		// save connected component (mesh) to an .obj file
		// 

		char fnameBuf[64];
		sprintf(fnameBuf, "OUT_conncomp%d-%s.obj", i, (flip_faces ? "reversed" : "as-given"));
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

        mioWriteOBJ(
            fpath.c_str(), 
            ccVertices.data(), 
            nullptr, // pNormals
            nullptr, // pTexCoords
            ccFaceSizes.data(), 
            ccFaceIndices.data(), 
            nullptr, // pFaceVertexTexCoordIndices
            nullptr, // pFaceVertexNormalIndices 
            ccVertexCount, 
            0, //numNormals 
            0, // numTexCoords
            (McUint32)ccFaceSizes.size());
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
