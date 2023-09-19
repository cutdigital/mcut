/**
 * Copyright (c) 2021-2023 Floyd M. Chitalu.
 * All rights reserved.
 *
 * NOTE: This file is licensed under GPL-3.0-or-later (default).
 * A commercial license can be purchased from Floyd M. Chitalu.
 *
 * License details:
 *
 * (A)  GNU General Public License ("GPL"); a copy of which you should have
 *      recieved with this file.
 * 	    - see also: <http://www.gnu.org/licenses/>
 * (B)  Commercial license.
 *      - email: floyd.m.chitalu@gmail.com
 *
 * The commercial license options is for users that wish to use MCUT in
 * their products for comercial purposes but do not wish to release their
 * software products under the GPL license.
 *
 * Author(s)     : Floyd M. Chitalu
 */

#include "mcut/mcut.h"
#include <chrono>
#include <cstring>
#include <inttypes.h> // PRId64
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

void MCAPI_PTR mcDebugOutputCALLBACK(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

void readOFF(
    const char* fpath,
    double** pVertices,
    unsigned int** pFaceIndices,
    unsigned int** pFaceSizes,
    unsigned int* numVertices,
    unsigned int* numFaces);

void writeOFF(
    const char* fpath,
    const double* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces);

int main(int argc, char* argv[])
{
    double* srcMeshVertices = NULL;
    uint32_t* srcMeshFaceIndices = NULL;
    uint32_t* srcMeshFaceSizes = NULL;
    uint32_t srcMeshNumFaces = 0;
    uint32_t srcMeshNumVertices = 0;

    double* cutMeshVertices = NULL;
    uint32_t* cutMeshFaceIndices = NULL;
    uint32_t* cutMeshFaceSizes = NULL;
    uint32_t cutMeshNumFaces = 0;
    uint32_t cutMeshNumVertices = 0;

    McResult api_err = MC_NO_ERROR;
    
    //
    // NOTE: the icosphere (cut-mesh) provided lies inside the cube (source-mesh)
    //

    const char* srcMeshFilePath = DATA_DIR "/cube.off";
    const char* cutMeshFilePath = DATA_DIR "/icosphere.off";

    printf(">> source-mesh file: %s\n", srcMeshFilePath);
    readOFF(srcMeshFilePath, &srcMeshVertices, &srcMeshFaceIndices, &srcMeshFaceSizes, &srcMeshNumVertices, &srcMeshNumFaces);
    printf(">> vertices=%u faces=%u\n", srcMeshNumVertices, srcMeshNumFaces);

    printf(">> cut-mesh file: %s\n", cutMeshFilePath);
    readOFF(cutMeshFilePath, &cutMeshVertices, &cutMeshFaceIndices, &cutMeshFaceSizes, &cutMeshNumVertices, &cutMeshNumFaces);
    printf(">> vertices=%u faces=%u\n", cutMeshNumVertices, cutMeshNumFaces);

    McContext context = MC_NULL_HANDLE;

    api_err = mcCreateContext(&context, MC_DEBUG);

    if (api_err != MC_NO_ERROR) {
        printf("mcCreateContext failed (err=%d)", (int)api_err);
        exit(1);
    }

    //
    // config debug output (optional)
    // 

    McFlags contextFlags = MC_NULL_HANDLE;

    api_err = mcGetInfo(context, MC_CONTEXT_FLAGS, sizeof(McFlags), &contextFlags, nullptr);

    if (api_err != MC_NO_ERROR) {
        printf("1:mcGetInfo(MC_CONTEXT_FLAGS) failed (err=%d)", (int)api_err);
        exit(1);
    }

    if (contextFlags & MC_DEBUG) {
        api_err = mcDebugMessageCallback(context, mcDebugOutputCALLBACK, nullptr);

        if (api_err != MC_NO_ERROR) {
            printf("mcDebugMessageCallback failed (err=%d)", (int)api_err);
            exit(1);
        }

        api_err = mcDebugMessageControl(context, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);

        if (api_err != MC_NO_ERROR) {
            printf("mcDebugMessageControl failed (err=%d)", (int)api_err);
            exit(1);
        }
    }
    
    //
    // invoke dispatch to do the cutting
    //
    api_err = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | 
        /* This flag is now required. Otherwise, MCUT will not check for the [type of intersection] when the inputs do not intersect to produce a cut. 
        If this flag is not specified, then querying the intersection type will return either 1_ a stale value from some other 
        dispatch call (for which the flag was specified) or 2) "MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM" which is the default value. */
        MC_DISPATCH_INCLUDE_INTERSECTION_TYPE ,
        // source mesh
        (const McVoid*)srcMeshVertices,
        (const McUint32*)srcMeshFaceIndices,
        (const McUint32*)srcMeshFaceSizes,
        (McUint32)srcMeshNumVertices,
        (McUint32)srcMeshNumFaces,
        // cut mesh
        (const McVoid*)cutMeshVertices,
        (const McUint32*)cutMeshFaceIndices,
        (const McUint32*)cutMeshFaceSizes,
        (McUint32)cutMeshNumVertices,
        (McUint32)cutMeshNumFaces);

    if (api_err != MC_NO_ERROR) {
        printf("mcEnqueueDispatch failed (err=%d)\n", (int)api_err);
        return api_err;
    }

    McUint32 numConnComps = 0;

    //
    // Query for all available connected components. 
    // Note: we can also skip this step and immediately request "MC_CONTEXT_DISPATCH_INTERSECTION_TYPE"
    //

    api_err = mcGetConnectedComponents(
        context,
        MC_CONNECTED_COMPONENT_TYPE_ALL,
        0, NULL,
        &numConnComps);

    if (api_err != MC_NO_ERROR) {
        printf("mcEnqueueGetConnectedComponents failed (err=%d)\n", (int)api_err);
        return api_err;
    }

    printf("have %u connected components\n", numConnComps);

    McSize bytes = 0;

    api_err = mcGetInfo(context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "mcGetInfo failed (err=%d)\n", (int)api_err);
        exit(1);
    }

    if (bytes != sizeof(McDispatchIntersectionType))
    {
        fprintf(stderr, "bytesize mismatch\n");
    }

    McDispatchIntersectionType intersectionType;

    api_err =  mcGetInfo(context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &intersectionType, 0);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "mcGetInfo failed (err=%d)\n", (int)api_err);
        exit(1);
    }

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
        
    // destroy context
    api_err = mcReleaseContext(context);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseContext failed (err=%d)\n", (int)api_err);
        exit(1);
    }

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

    // printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
    // printf("userParam=%p\n", userParam);

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
        // printf("Type: Other");
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)

// https://stackoverflow.com/questions/735126/are-there-alternate-implementations-of-gnu-getline-interface/735472#735472
/* Modifications, public domain as well, by Antti Haapala, 11/10/17
- Switched to getc on 5/23/19 */
#include <errno.h>
#include <stdint.h>

// if typedef doesn't exist (msvc, blah)
typedef intptr_t ssize_t;

ssize_t getline(char** lineptr, size_t* n, FILE* stream)
{
    size_t pos;
    int c;

    if (lineptr == NULL || stream == NULL || n == NULL) {
        errno = EINVAL;
        return -1;
    }

    c = getc(stream);
    if (c == EOF) {
        return -1;
    }

    if (*lineptr == NULL) {
        *lineptr = (char*)malloc(128);
        if (*lineptr == NULL) {
            return -1;
        }
        *n = 128;
    }

    pos = 0;
    while (c != EOF) {
        if (pos + 1 >= *n) {
            size_t new_size = *n + (*n >> 2);
            if (new_size < 128) {
                new_size = 128;
            }
            char* new_ptr = (char*)realloc(*lineptr, new_size);
            if (new_ptr == NULL) {
                return -1;
            }
            *n = new_size;
            *lineptr = new_ptr;
        }

        ((unsigned char*)(*lineptr))[pos++] = (unsigned char)c;
        if (c == '\n') {
            break;
        }
        c = getc(stream);
    }

    (*lineptr)[pos] = '\0';
    return pos;
}
#endif // #if defined (_WIN32)

bool readLine(FILE* file, char** lineBuf, size_t* len)
{
    while (getline(lineBuf, len, file)) {
        if (strlen(*lineBuf) > 1 && (*lineBuf)[0] != '#') {
            return true;
        }
    }
    return false;
}

void readOFF(
    const char* fpath,
    double** pVertices,
    unsigned int** pFaceIndices,
    unsigned int** pFaceSizes,
    unsigned int* numVertices,
    unsigned int* numFaces)
{
    // using "rb" instead of "r" to prevent linefeed conversion
    // See: https://stackoverflow.com/questions/27530636/read-text-file-in-c-with-fopen-without-linefeed-conversion
    FILE* file = fopen(fpath, "rb");

    if (file == NULL) {
        fprintf(stderr, "error: failed to open `%s`", fpath);
        exit(1);
    }

    char* lineBuf = NULL;
    size_t lineBufLen = 0;
    bool lineOk = true;
    int i = 0;

    // file header
    lineOk = readLine(file, &lineBuf, &lineBufLen);

    if (!lineOk) {
        fprintf(stderr, "error: .off file header not found\n");
        exit(1);
    }

    if (strstr(lineBuf, "OFF") == NULL) {
        fprintf(stderr, "error: unrecognised .off file header\n");
        exit(1);
    }

    // #vertices, #faces, #edges
    lineOk = readLine(file, &lineBuf, &lineBufLen);

    if (!lineOk) {
        fprintf(stderr, "error: .off element count not found\n");
        exit(1);
    }

    int nedges = 0;
    sscanf(lineBuf, "%d %d %d", numVertices, numFaces, &nedges);
    *pVertices = (double*)malloc(sizeof(double) * (*numVertices) * 3);
    *pFaceSizes = (unsigned int*)malloc(sizeof(unsigned int) * (*numFaces));

    // vertices
    for (i = 0; i < (double)(*numVertices); ++i) {
        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off vertex not found\n");
            exit(1);
        }

        double x, y, z;
        sscanf(lineBuf, "%lf %lf %lf", &x, &y, &z);

        (*pVertices)[(i * 3) + 0] = x;
        (*pVertices)[(i * 3) + 1] = y;
        (*pVertices)[(i * 3) + 2] = z;
    }
#if _WIN64
    __int64 facesStartOffset = _ftelli64(file);
#else
    long int facesStartOffset = ftell(file);
#endif
    int numFaceIndices = 0;

    // faces
    for (i = 0; i < (int)(*numFaces); ++i) {
        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off file face not found\n");
            exit(1);
        }

        int n; // number of vertices in face
        sscanf(lineBuf, "%d", &n);

        if (n < 3) {
            fprintf(stderr, "error: invalid vertex count in file %d\n", n);
            exit(1);
        }

        (*pFaceSizes)[i] = n;
        numFaceIndices += n;
    }

    (*pFaceIndices) = (unsigned int*)malloc(sizeof(unsigned int) * numFaceIndices);

#if _WIN64
    int err = _fseeki64(file, facesStartOffset, SEEK_SET);
#else
    int err = fseek(file, facesStartOffset, SEEK_SET);
#endif
    if (err != 0) {
        fprintf(stderr, "error: fseek failed\n");
        exit(1);
    }

    int indexOffset = 0;
    for (i = 0; i < (int)(*numFaces); ++i) {

        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off file face not found\n");
            exit(1);
        }

        int n; // number of vertices in face
        sscanf(lineBuf, "%d", &n);

        char* lineBufShifted = lineBuf;
        int j = 0;

        while (j < n) { // parse remaining numbers on lineBuf
            lineBufShifted = strstr(lineBufShifted, " ") + 1; // start of next number

            int val;
            sscanf(lineBufShifted, "%d", &val);

            (*pFaceIndices)[indexOffset + j] = val;
            j++;
        }

        indexOffset += n;
    }

    free(lineBuf);

    fclose(file);
}

void writeOFF(
    const char* fpath,
    const double* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces)
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
        const double* vptr = pVertices + (i * 3);
        fprintf(file, "%f %f %f\n", vptr[0], vptr[1], vptr[2]);
    }

    int faceBaseOffset = 0;
    for (i = 0; i < (int)numFaces; ++i) {
        uint32_t faceVertexCount = pFaceSizes[i];
        fprintf(file, "%d", (int)faceVertexCount);
        int j;
        for (j = 0; j < (int)faceVertexCount; ++j) {
            const uint32_t* fptr = pFaceIndices + faceBaseOffset + j;
            fprintf(file, " %d", *fptr);
        }
        fprintf(file, "\n");
        faceBaseOffset += faceVertexCount;
    }

    fclose(file);
}