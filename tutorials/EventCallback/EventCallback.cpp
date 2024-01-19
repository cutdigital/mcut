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
 *  EventCallback.cpp
 *
 *  \brief:
 *  This tutorial shows how to call the asynchronious mcDisptach function and
 *  assign a callback that is invoked when the mcDisptach call is complete.
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

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::exit(1);                               \
    }

struct CallbackData {
    McInt32 a;
    const char* b;
};

// This callback function will be called asynchroniously w.r.t the client application.
// Moreover, it is invoked by MCUT when the associated event has finished execution.
McVoid myCallbackFunction(McEvent event, McVoid* data)
{
    printf("Callback invoked!\n");

    printf("event handle = %p\n", event);

    CallbackData* cd_ptr = (CallbackData*)data;

    printf("callback data ptr: a=%d b=%s\n", cd_ptr->a, cd_ptr->b);
}

McInt32 main()
{
    //
    // Create meshes to intersect
    //

    // the source-mesh (a cube)
    
    McFloat cubeVertices[] = {
        -5, -5, 5, // 0
        5, -5, 5, // 1
        5, 5, 5, // 2
        -5, 5, 5, // 3
        -5, -5, -5, // 4
        5, -5, -5, // 5
        5, 5, -5, // 6
        -5, 5, -5 // 7
    };
    McUint32 cubeFaces[] = {
        0, 1, 2, 3, // 0
        7, 6, 5, 4, // 1
        1, 5, 6, 2, // 2
        0, 3, 7, 4, // 3
        3, 2, 6, 7, // 4
        4, 5, 1, 0 // 5
    };
    McUint32 cubeFaceSizes[] = {
        4, 4, 4, 4, 4, 4
    };
    McUint32 numCubeVertices = 8;
    McUint32 numCubeFaces = 6;

    // the cut mesh (a quad formed of two triangles)
    McFloat cutMeshVertices[] = {
        -20, -4, 0, // 0
        0, 20, 20, // 1
        20, -4, 0, // 2
        0, 20, -20 // 3
    };
    McUint32 cutMeshFaces[] = {
        0, 1, 2, // 0
        0, 2, 3 // 1
    };
    McUint32 cutMeshFaceSizes[] = {
         3, 3};
    McUint32 numCutMeshVertices = 4;
    McUint32 numCutMeshFaces = 2;

    // 2. create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);

    if (status != MC_NO_ERROR) {
        fprintf(stderr, "could not create context (status=%d)\n", (McInt32)status);
        exit(1);
    }

    //
    // do the cutting
    //
    // NOTE: non-blocking/asynchronious dispatch call.
    //
    // We will have to rely on "dispatchEvent" to know when the cutting operation has finished
    //

    McEvent dispatchEvent = MC_NULL_HANDLE;

    status = mcEnqueueDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
        cubeVertices,
        cubeFaces,
        cubeFaceSizes,
        numCubeVertices,
        numCubeFaces,
        cutMeshVertices,
        cutMeshFaces,
		cutMeshFaceSizes, 
        numCutMeshVertices,
        numCutMeshFaces, 
        // new parameters
        0, NULL, &dispatchEvent);

    my_assert (status == MC_NO_ERROR);

    //
    // The following logic can be understood to execute as follows:
    //  Call the function "myCallbackFunction" when "dispatchEvent" has finished
    //
    CallbackData callbackData;
    callbackData.a = 42;
    callbackData.b = "I found the answer to life";

    status = mcSetEventCallback(dispatchEvent, myCallbackFunction, (McVoid*)&callbackData);

    //
    // query the number of available connected components after the cut
    // 
    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

#if 1
    //
    // block until the "dispatchEvent" has finished
    //
    status = mcWaitForEvents(1, &dispatchEvent);
    my_assert (status == MC_NO_ERROR);

    //
    // By waiting for an event (with "mcWaitForEvents"), we are able to probe that event for its execution
    // status to find out if the associated operation (i.e. "mcEnqueueDispatch") run without any errors (i.e.  errors due 
    // to something wrong with the input meshes).
    //
    McResult dispatchRuntimeStatus = MC_NO_ERROR;

    status = mcGetEventInfo(dispatchEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS, sizeof(McResult), &dispatchRuntimeStatus, NULL);
    
    my_assert (status == MC_NO_ERROR);

    if (dispatchRuntimeStatus != MC_NO_ERROR) {
        fprintf(stderr, "mcEnqueueDispatch failed internally (status=%d)\n", (McInt32)dispatchRuntimeStatus);
        exit(1);
    }

    // invoke next operation (blocks until "connectedComponentCount" is set) 
    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);

    my_assert (status == MC_NO_ERROR);

#else
    //
    // Here "mcEnqueueGetConnectedComponents" is also a non-blocking call, which HAPPENS AFTER "dispatchEvent" has finished
    //
    McEvent ccQueryEvent = MC_NULL_HANDLE; 
    
    // Note: An API call that can "emit" an event can also wait on an event, and vice versa
    status = mcEnqueueGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount, 1, &dispatchEvent, &ccQueryEvent);
    
    my_assert (status == MC_NO_ERROR);

    //
    // block until the "ccQueryEvent" has finished
    //
    status = mcWaitForEvents(1, &ccQueryEvent);

    my_assert (status == MC_NO_ERROR);
#endif

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
        McSize numBytes = 0;

        //
        // vertices
        // 

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McFloat) * 3ull));
        std::vector<McFloat> ccVertices(ccVertexCount * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT, numBytes, (McVoid*)ccVertices.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
        // faces
        // 

        numBytes = 0; 

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceIndices;
        ccFaceIndices.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, (McVoid*)ccFaceIndices.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
        // face sizes (vertices per face)
        // 

        numBytes = 0;

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        
        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceSizes;
        ccFaceSizes.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, (McVoid*)ccFaceSizes.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
		// save connected component (mesh) to an .obj file
		// 

		char fnameBuf[64];
		sprintf(fnameBuf, "OUT_conncomp%d.obj", i);
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

        // "mioWriteOBJ" expects a vertex array of doubles. So we temporarilly create one.
        std::vector<McDouble> ccVertices64(ccVertices.size(), McDouble(0.0));

        for(McUint32 v =0; v < (McUint32)ccVertices.size(); ++v)
        {
            ccVertices64[v] = (McDouble)ccVertices[v];
        }

        mioWriteOBJ(
            fpath.c_str(), 
            ccVertices64.data(), 
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

	my_assert(status == MC_NO_ERROR);

	//
	// free memory of context
	//

	status = mcReleaseContext(context);

	my_assert(status == MC_NO_ERROR);

    return 0;
}

// write mesh to .off file
McVoid writeOFF(
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
        exit(1);
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
