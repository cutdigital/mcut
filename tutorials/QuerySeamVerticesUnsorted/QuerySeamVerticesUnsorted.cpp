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

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::exit(1);                               \
    }

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
    const uint32_t* pEdges,
    const uint32_t numVertices,
    const uint32_t numFaces,
    const uint32_t numEdges);

void MCAPI_PTR mcDebugOutput(McDebugSource source,
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

int main()
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

    const char* srcMeshFilePath = DATA_DIR "/source-mesh.off";
    const char* cutMeshFilePath = DATA_DIR "/cut-mesh.off";

    printf(">> source-mesh file: %s\n", srcMeshFilePath);
    printf(">> cut-mesh file: %s\n", cutMeshFilePath);

    // load meshes
    // -----------

    readOFF(srcMeshFilePath, &srcMeshVertices, &srcMeshFaceIndices, &srcMeshFaceSizes, &srcMeshNumVertices, &srcMeshNumFaces);

    printf(">> src-mesh vertices=%u faces=%u\n", srcMeshNumVertices, srcMeshNumFaces);

    readOFF(cutMeshFilePath, &cutMeshVertices, &cutMeshFaceIndices, &cutMeshFaceSizes, &cutMeshNumVertices, &cutMeshNumFaces);

    printf(">> cut-mesh vertices=%u faces=%u\n", cutMeshNumVertices, cutMeshNumFaces);

    // 2. create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    api_err = mcCreateContext(&context, MC_DEBUG);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "could not create context (api_err=%d)\n", (int)api_err);
        exit(1);
    }

    // config debug output
    // -----------------------
    McSize numBytes = 0;
    McFlags contextFlags;
    api_err = mcGetInfo(context, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes);

    api_err = mcGetInfo(context, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr);

    if (contextFlags & MC_DEBUG) {
        mcDebugMessageCallback(context, mcDebugOutput, nullptr);
        mcDebugMessageControl(context, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);
    }

    // 3. do the magic!
    // ----------------
    api_err = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
        srcMeshVertices,
        srcMeshFaceIndices,
        srcMeshFaceSizes,
        srcMeshNumVertices,
        srcMeshNumFaces,
        cutMeshVertices,
        cutMeshFaceIndices,
        cutMeshFaceSizes,
        cutMeshNumVertices,
        cutMeshNumFaces);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "dispatch call failed (api_err=%d)\n", (int)api_err);
        exit(1);
    }

    // 4. query the number of available connected component (only fragments to keep things simple)
    // ------------------------------------------------------------------------------------------
    uint32_t numConnComps;
    std::vector<McConnectedComponent> connComps;

    api_err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL, &numConnComps);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "1:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) failed (api_err=%d)\n", (int)api_err);
        exit(1);
    }

    if (numConnComps == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    connComps.resize(numConnComps);

    api_err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, (uint32_t)connComps.size(), connComps.data(), NULL);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "2:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) failed (api_err=%d)\n", (int)api_err);
        exit(1);
    }

    // 5. query the data of each connected component from MCUT
    // -------------------------------------------------------

    for (int i = 0; i < (int)connComps.size(); ++i) {
        McConnectedComponent connComp = connComps[i]; // connected compoenent id

        McSize numBytes = 0;

        // query the seam vertices (indices)
        // ---------------------------------

        
        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, 0, NULL, &numBytes);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        std::vector<uint32_t> seamVertexIndices;
        seamVertexIndices.resize(numBytes / sizeof(uint32_t));

        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX, numBytes, seamVertexIndices.data(), NULL);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        uint32_t faceSizesStub = (McUint32)seamVertexIndices.size();

        char seamFnameBuf[40];
        sprintf(seamFnameBuf, "frag-%d-seam-vertices.txt", i);

        // save seam vertices to file (.txt)
        // ------------------------
        writeOFF(seamFnameBuf,
            NULL,
            // We pretend that the list of seam vertices is a face, when in actual
            // fact we are simply using the output file as storage for later inspection
            seamVertexIndices.data(),
            &faceSizesStub,
            NULL,
            0,
            1, // one face
            0);

        // query the vertices (coordinates)
        // --------------------------------
numBytes = 0;
        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        uint32_t numberOfVertices = (uint32_t)(numBytes / (sizeof(double) * 3));

        std::vector<double> vertices(numberOfVertices * 3u);

        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)vertices.data(), NULL);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        // query (triangulated) faces
        // -----------------------------
        numBytes = 0;
        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        uint32_t numberOfTriangles = (uint32_t)(numBytes / (sizeof(McIndex) * 3));

        std::vector<McIndex> triangleIndices(numberOfTriangles * 3u);

        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes, (void*)triangleIndices.data(), NULL);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        // save mesh to file (.off)
        // ------------------------
        char fnameBuf[32];
        sprintf(fnameBuf, "frag-%d.off", i);

        writeOFF(fnameBuf,
            vertices.data(),
            triangleIndices.data(),
            NULL, // if null then function treat faceIndices array as made up of triangles
            NULL, // we don't care about writing edges
            numberOfVertices,
            numberOfTriangles,
            0 // zero edges since we don't care about writing edges
        );
    }

    // 6. free connected component data
    // --------------------------------
    api_err = mcReleaseConnectedComponents(context, 0, NULL);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseConnectedComponents failed (api_err=%d)\n", (int)api_err);
        exit(1);
    }

    // 7. destroy context
    // ------------------
    api_err = mcReleaseContext(context);

    if (api_err != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseContext failed (api_err=%d)\n", (int)api_err);
        exit(1);
    }

    return 0;
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
    int api_err = _fseeki64(file, facesStartOffset, SEEK_SET);
#else
    int api_err = fseek(file, facesStartOffset, SEEK_SET);
#endif
    if (api_err != 0) {
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

// To ignore edges when writing the output just pass pEdges = NULL and numEdges = 0
void writeOFF(
    const char* fpath,
    const double* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t* pEdges,
    const uint32_t numVertices,
    const uint32_t numFaces,
    const uint32_t numEdges)
{
    fprintf(stdout, "write: %s\n", fpath);

    FILE* file = fopen(fpath, "w");

    if (file == NULL) {
        fprintf(stderr, "error: failed to open `%s`", fpath);
        exit(1);
    }

    fprintf(file, "OFF\n");
    fprintf(file, "%d %d %d\n", numVertices, numFaces, numEdges);
    int i;
    for (i = 0; i < (int)numVertices; ++i) {
        const double* vptr = pVertices + (i * 3);
        fprintf(file, "%f %f %f\n", vptr[0], vptr[1], vptr[2]);
    }

    int faceBaseOffset = 0;
    for (i = 0; i < (int)numFaces; ++i) {
        const uint32_t faceVertexCount = (pFaceSizes != NULL) ? pFaceSizes[i] : 3;
        fprintf(file, "%d", (int)faceVertexCount);
        int j;
        for (j = 0; j < (int)faceVertexCount; ++j) {
            const uint32_t* fptr = pFaceIndices + faceBaseOffset + j;
            fprintf(file, " %d", *fptr);
        }
        fprintf(file, "\n");
        faceBaseOffset += faceVertexCount;
    }

    for (i = 0; i < (int)numEdges; ++i) {
        const uint32_t* eptr = pEdges + (i * 2);
        fprintf(file, "%u %u\n", eptr[0], eptr[1]);
    }

    fclose(file);
}