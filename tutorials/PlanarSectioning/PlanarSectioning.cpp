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
 *  PlanarSectioning.cpp
 *
 *  \brief:
 *  This tutorial shows how to partition/slice a mesh with a plane using mcut.
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
        std::abort();                               \
    }

void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{
    
    std::string debug_src;
	switch(source)
	{
	case MC_DEBUG_SOURCE_API:
		debug_src = "API";
		break;
	case MC_DEBUG_SOURCE_KERNEL:
		debug_src = "KERNEL";
		break;
	case MC_DEBUG_SOURCE_FRONTEND:
        debug_src = "FRONTEND";
    case MC_DEBUG_SOURCE_ALL:case MC_DEBUG_SOURCE_IGNORE:
        break;
	}
	std::string debug_type;
	switch(type)
	{
	case MC_DEBUG_TYPE_ERROR:
		debug_type = "ERROR";
		break;
	case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
		debug_type = "DEPRECATION";
		break;
	case MC_DEBUG_TYPE_OTHER:
		debug_type = "OTHER";
		break;
	case MC_DEBUG_TYPE_ALL:case MC_DEBUG_TYPE_IGNORE:
		break;
	}


    std::string severity_str;

    switch (severity) {
    case MC_DEBUG_SEVERITY_HIGH:
        severity_str = "HIGH";
        break;
    case MC_DEBUG_SEVERITY_MEDIUM:
        severity_str = "MEDIUM";
        break;
    case MC_DEBUG_SEVERITY_LOW:
        severity_str = "LOW";
        break;
    case MC_DEBUG_SEVERITY_NOTIFICATION:
        severity_str = "NOTIFICATION";
        break;
    case MC_DEBUG_SEVERITY_ALL:
        break;
    }

    printf("MCUT[%d:%p,%s:%s:%s:%zu] %s\n", id, userParam, debug_src.c_str(), debug_type.c_str(),severity_str.c_str(), length, message);
}

int main()
{
    // 1. Create meshes.
    // -----------------

    // the cube
    // --------
    McFloat cubeVertices[] = {
        -5, -5, 5,  // 0
        5, -5, 5,   // 1
        5, 5, 5,    //2
        -5, 5, 5,   //3
        -5, -5, -5, //4
        5, -5, -5,  //5
        5, 5, -5,   //6
        -5, 5, -5   //7
    };
    McUint32 cubeFaces[] = {
        0, 1, 2, 3, //0
        7, 6, 5, 4, //1
        1, 5, 6, 2, //2
        0, 3, 7, 4, //3
        3, 2, 6, 7, //4
        4, 5, 1, 0  //5
    };
    McUint32 cubeFaceSizes[] = {
        4, 4, 4, 4, 4, 4};
    McUint32 numCubeVertices = 8;
    McUint32 numCubeFaces = 6;

    //
    // create a context
    // 
    McContext context = MC_NULL_HANDLE;
    McResult status = mcCreateContext(&context, MC_DEBUG);

   my_assert (status == MC_NO_ERROR);

    //
    // configure debug output
    // 
    McSize numBytes = 0;
    McFlags contextFlags;
    
    status = mcGetInfo(context, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes);
    
    my_assert (status == MC_NO_ERROR);
    
    status = mcGetInfo(context, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr);
    
    my_assert (status == MC_NO_ERROR);

    if (contextFlags & MC_DEBUG) {
        status = mcDebugMessageCallback(context, mcDebugOutput, nullptr);

        my_assert (status == MC_NO_ERROR);

        status = mcDebugMessageControl(context, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, MC_TRUE);

        my_assert (status == MC_NO_ERROR);
    }


    const McDouble normal[] = {0, 1, 1}; // can point in any direction
    const McDouble sectionOffset = 0.45;
    McEvent dispatchEvent = MC_NULL_HANDLE;
    
    //
    // do the magic!
    // 
    status = mcEnqueueDispatchPlanarSection(
        context,
        MC_DISPATCH_VERTEX_ARRAY_FLOAT | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
        cubeVertices,
        cubeFaces,
        cubeFaceSizes,
        numCubeVertices,
        numCubeFaces,
        normal,
        sectionOffset,
        0,
        nullptr,
        &dispatchEvent
    );

    my_assert (status == MC_NO_ERROR);

    status =  mcWaitForEvents(1,&dispatchEvent );

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

    for (int i = 0; i < (int)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i]; // connected compoenent id

        numBytes = 0;

        //
        // vertices
        //

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

        my_assert (status == MC_NO_ERROR);

        McUint32 ccVertexCount = (McUint32)(numBytes / (sizeof(McDouble) * 3));

        std::vector<McDouble> ccVertices(ccVertexCount * 3u);

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);

        my_assert (status == MC_NO_ERROR);
        
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
        // face sizes
        // 
        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        
        my_assert (status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceSizes;
        ccFaceSizes.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);

        my_assert (status == MC_NO_ERROR);

        //
        // save to mesh file (.off)
        // 

        char fnameBuf[64];
		sprintf(fnameBuf, "OUT_conncomp%d.off", i);
		std::string fpath(OUTPUT_DIR "/" + std::string(fnameBuf));

        mioWriteOFF(fpath.c_str(),
            (McDouble*)ccVertices.data(),
            (McUint32*)ccFaceIndices.data(),
            (McUint32*)ccFaceSizes.data(),
             NULL,
            (McUint32)ccVertices.size() / 3,
            (McUint32)ccFaceSizes.size(), 
            (McUint32)0);
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
        exit(1);
    }

    fprintf(file, "OFF\n");
    fprintf(file, "%d %d %d\n", numVertices, numFaces, 0 /*numEdges*/);
    int i;
    for (i = 0; i < (int)numVertices; ++i) {
        McFloat* vptr = pVertices + (i * 3);
        fprintf(file, "%f %f %f\n", vptr[0], vptr[1], vptr[2]);
    }

    int faceBaseOffset = 0;
    for (i = 0; i < (int)numFaces; ++i) {
        McUint32 faceVertexCount = pFaceSizes[i];
        fprintf(file, "%d", (int)faceVertexCount);
        int j;
        for (j = 0; j < (int)faceVertexCount; ++j) {
            McUint32* fptr = pFaceIndices + faceBaseOffset + j;
            fprintf(file, " %d", *fptr);
        }
        fprintf(file, "\n");
        faceBaseOffset += faceVertexCount;
    }

    fclose(file);
}
