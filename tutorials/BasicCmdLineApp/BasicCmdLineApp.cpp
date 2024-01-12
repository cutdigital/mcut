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
 *  BasicCmdLineApp.cpp
 *
 *  \brief:
 *  This tutorial shows how to implement a basic command line application 
 *  with mcut. This simple app accepts two input meshes and computes their 
 *  intersection to then dump the output connected components after cutting. 
 *  The accepted file formats are:
 *  - obj
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"
#include "mio/mio.h"

#include <string>
#include <vector>
#include <iostream>
#include <inttypes.h> // PRId64
#include <stdio.h>
#include <stdlib.h>

#define my_assert(cond)                                                                            \
	if(!(cond))                                                                                    \
	{                                                                                              \
		fprintf(stderr, "MCUT error: %s\n", #cond);                                                \
		std::exit(1);                                                                              \
	}

#define mcCheckError(errCode) mcCheckError_(errCode, __FILE__, __LINE__)

void mcCheckError_(McResult status, const char* file, McUint32 line);

void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    McUint32 id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

McUint32 main(McUint32 argc, char* argv[])
{
    bool help = false;

    for (McUint32 i = 0; help == false && i < argc; ++i) {
        if (!strcmp("--help", argv[i]) || !strcmp("-h", argv[i])) {
            help = true;
        }
    }

    if (help || argc < 3) {
        fprintf(stdout, "<exe> <path/to/source-mesh> <path/to/cut-mesh>\n\nSupported file types: obj\n");
        return 1;
    }

    const char* srcMeshFilePath = argv[1];
    const char* cutMeshFilePath = argv[2];

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
	mioReadOBJ(srcMeshFilePath,
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

	mioReadOBJ(cutMeshFilePath,
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

	#ifdef  NDEBUG
    McResult status = mcCreateContext(&context, MC_NULL_HANDLE);
#else
    McResult status = mcCreateContext(&context, MC_DEBUG);
#endif
    mcCheckError(status);

    //
    // config debug output
    // 

    McSize numBytes = 0;
    McFlags contextFlags;

    status = mcGetInfo(context, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes);

    mcCheckError(status);

    my_assert(sizeof(McFlags) == numBytes);

    status = mcGetInfo(context, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr);

    mcCheckError(status);

    if (contextFlags & MC_DEBUG) { // did the user enable debugging mode?
        mcDebugMessageCallback(context, mcDebugOutput, nullptr);
        mcDebugMessageControl(context, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);
    }

    //
	//  do the cutting
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

    mcCheckError(status);

    //
	// We no longer need the mem of input meshes, so we can free it!
	//
	mioFreeMesh(&srcMesh);
	mioFreeMesh(&cutMesh);

    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    // we want to query all available connected components:
    // --> fragments, patches, seams and inputs
    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);
    
    mcCheckError(status);

    if (connectedComponentCount == 0) {
        printf("no connected components found\n");
        std::exit(0);
    }

    connectedComponents.resize(connectedComponentCount);

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);

    mcCheckError(status);

    //
    // query connected component data
    //
    for (McUint32 i = 0; i < (McUint32)connectedComponents.size(); ++i) {

        McConnectedComponent cc = connectedComponents[i]; // handle

        //
		//  ccVertices
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

#if 1 // triangulated output
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);

        my_assert(status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceIndices(numBytes / sizeof(McUint32), 0);
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes, ccFaceIndices.data(), NULL);

        my_assert(status == MC_NO_ERROR);

        std::vector<McUint32> ccFaceSizes(ccFaceIndices.size()/3, 3);
#else // nontriangulated output
        // face indices
        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
        mcCheckError(status);

        my_assert(numBytes > 0);

        std::vector<McUint32> ccFaceIndices;
        ccFaceIndices.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);
        mcCheckError(status);

        // face sizes
        numBytes = 0;
        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        mcCheckError(status);

        my_assert(numBytes > 0);

        std::vector<McUint32> ccFaceSizes;
        ccFaceSizes.resize(numBytes / sizeof(McUint32));

        status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);
        mcCheckError(status);
#endif

        //
		// save connected component (mesh) to an .obj file
		// 

		char fnameBuf[64];
		sprintf(fnameBuf, "conncomp%d.obj", i);
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

void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    McUint32 id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{
    std::string debug_src;
    switch (source) {
    case MC_DEBUG_SOURCE_API:
        debug_src = "API";
        break;
    case MC_DEBUG_SOURCE_KERNEL:
        debug_src = "KERNEL";
        break;
    case MC_DEBUG_SOURCE_ALL:
        break;
    }

    std::string debug_type;
    switch (type) {
    case MC_DEBUG_TYPE_ERROR:
        debug_type = "ERROR";
        break;
    case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
         debug_type = "DEPRECATION";
        break;
    case MC_DEBUG_TYPE_OTHER:
        //printf("Type: Other");
        debug_type = "OTHER";
        break;
    case MC_DEBUG_TYPE_ALL:
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

void mcCheckError_(McResult status, const char* file, McUint32 line)
{
    std::string error;
    switch (status) {
    case MC_OUT_OF_MEMORY:
        error = "MC_OUT_OF_MEMORY";
        break;
    case MC_INVALID_VALUE:
        error = "MC_INVALID_VALUE";
        break;
    case MC_INVALID_OPERATION:
        error = "MC_INVALID_OPERATION";
        break;
    case MC_NO_ERROR:
        error = "MC_NO_ERROR";
        break;
        case MC_RESULT_MAX_ENUM:
        error = "UNKNOWN";
        break;
    }
    if (status) {
        std::cout << error << " | " << file << " (" << line << ")" << std::endl;
    }
    
}
