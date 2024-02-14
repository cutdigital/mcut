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
 *  InOutQuery.cpp
 *
 *  \brief:
 *  This tutorial shows how to determine the spatial configuration that
 *  mcut found the input meshes to be in. That is, it shows you how to 
 *  know whether e.g. the source-mesh is enclosed inside of the cut-mesh,
 *  which may be a possible reason for mcut producing zero output.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/mcut.h"
#include "mio/mio.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::abort();                               \
    }

void MCAPI_PTR mcDebugOutputCALLBACK(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

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

	mioReadOFF(DATA_DIR "/cube.off",
			   &srcMesh.pVertices,
			   &srcMesh.pFaceVertexIndices,
               &srcMesh.pFaceSizes,
			   &srcMesh.numVertices,
			   &srcMesh.numFaces);

	//
	// read-in the cut-mesh from file
    // NOTE: the icosphere (cut-mesh) provided lies inside the cube (source-mesh)
	//

	mioReadOFF(DATA_DIR "/icosphere.off",
			   &cutMesh.pVertices,
			   &cutMesh.pFaceVertexIndices,
               &cutMesh.pFaceSizes,
			   &cutMesh.numVertices,
			   &cutMesh.numFaces);

    McContext context = MC_NULL_HANDLE;

    McResult status = mcCreateContextWithHelpers(&context, MC_DEBUG, 2);

    if (status != MC_NO_ERROR) {
        printf("mcCreateContext failed (err=%d)", (int)status);
        exit(1);
    }

    //
    // config debug output (optional)
    // 

    McFlags contextFlags = MC_NULL_HANDLE;

    status = mcGetInfo(context, MC_CONTEXT_FLAGS, sizeof(McFlags), &contextFlags, nullptr);

    my_assert (status == MC_NO_ERROR);

    if (contextFlags & MC_DEBUG) {
        status = mcDebugMessageCallback(context, mcDebugOutputCALLBACK, nullptr);

        my_assert (status == MC_NO_ERROR);

        status = mcDebugMessageControl(context, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);

        my_assert (status == MC_NO_ERROR);
    }
    
    //
    // invoke dispatch to do the cutting
    //
    status = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | 
        /* 
            This flag is now required. Otherwise, MCUT will not check for 
            the [Type of Intersection] when the inputs do not intersect 
            (i.e. in order to produce a cut). If this flag is not specified, 
            then querying the intersection type will return either 1) a 
            stale value from some other dispatch call (for which the flag 
            was specified) or 2) "MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM" 
            which is the default value. 
        */
        MC_DISPATCH_INCLUDE_INTERSECTION_TYPE ,
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

    McUint32 connectedComponentCount = 0;

    //
    // Query for all available connected components. 
    // Note: we can also skip this step and immediately request "MC_CONTEXT_DISPATCH_INTERSECTION_TYPE"
    //

    status = mcGetConnectedComponents(
        context,
        MC_CONNECTED_COMPONENT_TYPE_ALL,
        0, NULL,
        &connectedComponentCount);

    my_assert (status == MC_NO_ERROR);

    printf("have %u connected components\n", connectedComponentCount);

    McSize bytes = 0;

    status = mcGetInfo(context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes);

    my_assert (status == MC_NO_ERROR);

    if (bytes != sizeof(McDispatchIntersectionType))
    {
        fprintf(stderr, "bytesize mismatch\n");
    }

    McDispatchIntersectionType intersectionType;

    status =  mcGetInfo(context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &intersectionType, 0);

    my_assert (status == MC_NO_ERROR);

    //
    // Once we know the type of intersection (via "intersectionType"), we can go ahead 
    // and do what ever we want with this information. In this tutorial, we simply print
    // the type of intersection to the terminal then exit.
    // 

    std::string intersectionTypeStr;
    switch (intersectionType)
    {
    case McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_STANDARD:
    {
        intersectionTypeStr = "MC_DISPATCH_INTERSECTION_TYPE_STANDARD"; // input meshes were intersecting to give a cut (as per standard MCUT procedure)
    }break;
    case McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_INSIDE_CUTMESH:
    {
        // the source-mesh was found to lie inside of the cut-mesh, which implies that the cut-mesh is a watertght (i.e. 2-manifold) surface mesh.
        // The source-mesh can be a watertight- or open mesh. 
        intersectionTypeStr = "MC_DISPATCH_INTERSECTION_TYPE_INSIDE_CUTMESH"; 
    }break;
    case McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_INSIDE_SOURCEMESH:
    {
        // the cut-mesh was found to lie inside of the source-mesh, which implies that the source-mesh is a watertght (i.e. 2-manifold) surface mesh.
        // The cut-mesh can be a watertight- or open mesh.
        intersectionTypeStr = "MC_DISPATCH_INTERSECTION_TYPE_INSIDE_SOURCEMESH";
    }break;
    case McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_NONE:
    {
        // The input meshes do not intersect to produce a cut, and neither mesh encloses the other. 
        // That is, they are apart, which means that the set-intersection of the space they enclose is empty.
        intersectionTypeStr = "MC_DISPATCH_INTERSECTION_TYPE_NONE";
    }break;
    default:
        intersectionTypeStr = "UNKNOWN"; // something strange happened (unlikely)
        break;
    }

    printf("Intersection type = %s\n", intersectionTypeStr.c_str());

    // 
    // destroy context
    //
    status = mcReleaseContext(context);

    my_assert (status == MC_NO_ERROR);

    return 0;
}

void MCAPI_PTR mcDebugOutputCALLBACK(McDebugSource source,
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

    printf("MCUT[%d:%p,%s:%s:%s:%zu] %s\n", id, userParam, debug_src.c_str(), debug_type.c_str(), severity_str.c_str(), length, message);
}

void mcCheckError_(McResult err, const char* file, int line)
{
    std::string error;
    switch (err) {
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
    if (err) {
        std::cout << error << " | " << file << " (" << line << ")" << std::endl;
    }
}
