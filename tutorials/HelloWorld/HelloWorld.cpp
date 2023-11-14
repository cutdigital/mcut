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
 * HelloWorld.cpp
 *
 * \brief:
 *  Simple "hello world" program using MCUT.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"

#include <stdio.h>
#include <stdlib.h>

#include <vector>

void writeOFF(
    const char* fpath,
    McFloat* pVertices,
    McUint32* pFaceIndices,
    McUint32* pFaceSizes,
    McUint32 numVertices,
    McUint32 numFaces);

McInt32 main()
{
    //
    // Create meshes to intersect
    //

    // the source-mesh (a cube)
    
    McFloat srcMeshVertices[] = {
        -5, -5, 5,  // vertex 0
        5, -5, 5,   // vertex 1
        5, 5, 5,    // vertex 2
        -5, 5, 5,   // vertex 3
        -5, -5, -5, // vertex 4
        5, -5, -5,  // vertex 5
        5, 5, -5,   // vertex 6
        -5, 5, -5   // vertex 7
    };

    McUint32 srcMeshFaces[] = {
        0, 1, 2, 3, // face 0
        7, 6, 5, 4, // face 1
        1, 5, 6, 2, // face 2
        0, 3, 7, 4, // face 3
        3, 2, 6, 7, // face 4
        4, 5, 1, 0  // face 5
    };
    
    McUint32 srcMeshFaceSizes[] = { 4, 4, 4, 4, 4, 4};

    McUint32 srcMeshVertexCount = 8;
    McUint32 srcMeshFaceCount = 6;

    // the cut mesh (a quad formed of two triangles)

    McFloat cutMeshVertices[] = {
        -20, -4, 0, // vertex 0
        0, 20, 20,  // vertex 1
        20, -4, 0,  // vertex 2
        0, 20, -20  // vertex 3
    };

    McUint32 cutMeshFaces[] = {
        0, 1, 2, // face 0
        0, 2, 3  // face 1
    };

    // McUint32 cutMeshFaceSizes[] = { 3, 3};
    
    McUint32 cutMeshVertexCount = 4;
    McUint32 cutMeshFaceCount = 2;

    //
    // create a context
    // 

    McContext context = MC_NULL_HANDLE;
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "could not create context (%d)\n", (McInt32)status);
        exit(EXIT_FAILURE);
    }

    //
    // do the cutting
    // 

    status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
        srcMeshVertices,
        srcMeshFaces,
        srcMeshFaceSizes,
        srcMeshVertexCount,
        srcMeshFaceCount,
        cutMeshVertices,
        cutMeshFaces,
        nullptr, // cutMeshFaceSizes, // no need to give 'cutMeshFaceSizes' parameter since the cut-mesh is a triangle mesh
        cutMeshVertexCount,
        cutMeshFaceCount);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "dispatch call failed (%d)\n", (McInt32)status);
        exit(EXIT_FAILURE);
    }

    //
    // query the number of available connected components after the cut
    // 

    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "1:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_ALL) failed (%d)\n", (McInt32)status);
        exit(EXIT_FAILURE);
    }

    if (connectedComponentCount == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(EXIT_FAILURE);
    }

    connectedComponents.resize(connectedComponentCount); // allocate for the amount we want to get

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "2:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_ALL) failed (%d)\n", (McInt32)status);
        exit(EXIT_FAILURE);
    }

    //
    //  query the data of each connected component 
    // 

    for (McInt32 i = 0; i < (McInt32)connectedComponents.size(); ++i) {
        
        McConnectedComponent cc = connectedComponents[i]; // connected compoenent id
        McSize bytesToAllocate = 0;

        //
        // vertices
        // 

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &bytesToAllocate);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT) failed (%d)\n", (McInt32)status);
            exit(EXIT_FAILURE);
        }

        McUint32 ccVertexCount = (McUint32)(bytesToAllocate / (sizeof(McFloat) * 3ull));
        std::vector<McFloat> ccVertices(ccVertexCount * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, bytesToAllocate, (McVoid*)ccVertices.data(), NULL);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT) failed (%d)\n", (McInt32)status);
            exit(EXIT_FAILURE);
        }

        //
        // faces
        // 

        bytesToAllocate = 0; 

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &bytesToAllocate);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE) failed (%d)\n", (McInt32)status);
            exit(EXIT_FAILURE);
        }

        std::vector<McUint32> faceIndices;
        faceIndices.resize(bytesToAllocate / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, bytesToAllocate, (McVoid*)faceIndices.data(), NULL);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE) failed (%d)\n", (McInt32)status);
            exit(EXIT_FAILURE);
        }

        //
        // face sizes (vertices per face)
        // 

        bytesToAllocate = 0;

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &bytesToAllocate);
        
        if (status != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_SIZE) failed (%d)\n", (McInt32)status);
            exit(EXIT_FAILURE);
        }

        std::vector<McUint32> faceSizes;
        faceSizes.resize(bytesToAllocate / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, bytesToAllocate, (McVoid*)faceSizes.data(), NULL);

        if (status != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_SIZE) failed (%d)\n", (McInt32)status);
            exit(EXIT_FAILURE);
        }

        //
        // save to cc to file (.off)
        // 

        char fnameBuf[32];

        sprintf(fnameBuf, "cc%d.off", i);
        writeOFF(fnameBuf,
            (McFloat*)ccVertices.data(),
            (McUint32*)faceIndices.data(),
            (McUint32*)faceSizes.data(),
            (McUint32)ccVertices.size() / 3u,
            (McUint32)faceSizes.size());
    }

    //
    // free memory of _all_ connected components (could also free them individually inside above for-loop)
    // 

    status = mcReleaseConnectedComponents(context, 0, NULL);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseConnectedComponents failed (%d)\n", (McInt32)status);
        exit(EXIT_FAILURE);
    }

    //
    // free memory of context
    // 
    
    status = mcReleaseContext(context);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseContext failed (%d)\n", (McInt32)status);
        exit(EXIT_FAILURE);
    }

    return 0;
}

// write mesh to .off file
void writeOFF(
    const char* fpath,
    McFloat* pVertices,
    McUint32* pFaceIndices,
    McUint32* pFaceSizes,
    McUint32 numVertices,
    McUint32 numFaces)
{
    fprintf(stdout, "write: %s\n", fpath);

    FILE* file = fopen(fpath, "w");

    if (file == NULL) {
        fprintf(stderr, "error: failed to open `%s`", fpath);
        exit(EXIT_FAILURE);
    }

    fprintf(file, "OFF\n");
    fprintf(file, "%d %d %d\n", numVertices, numFaces, 0 /*numEdges*/);
    McInt32 i;
    for (i = 0; i < (McInt32)numVertices; ++i) {
        McFloat* vptr = pVertices + (i * 3);
        fprintf(file, "%f %f %f\n", vptr[0], vptr[1], vptr[2]);
    }

    McInt32 faceBaseOffset = 0;
    for (i = 0; i < (McInt32)numFaces; ++i) {
        McUint32 faceVertexCount = pFaceSizes[i];
        fprintf(file, "%d", (McInt32)faceVertexCount);
        McInt32 j;
        for (j = 0; j < (McInt32)faceVertexCount; ++j) {
            McUint32* fptr = pFaceIndices + faceBaseOffset + j;
            fprintf(file, " %d", *fptr);
        }
        fprintf(file, "\n");
        faceBaseOffset += faceVertexCount;
    }

    fclose(file);
}
