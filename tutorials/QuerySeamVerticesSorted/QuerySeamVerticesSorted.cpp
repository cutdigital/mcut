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

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>

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

    //const char* srcMeshFilePath = DATA_DIR "/source-mesh.off";
    //const char* cutMeshFilePath = DATA_DIR "/cut-mesh.off";
    const char* srcMeshFilePath = DATA_DIR "/brad/source-mesh.off";
    const char* cutMeshFilePath = DATA_DIR "/brad/cut-mesh.off";

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

        // The format of this array (of 32-bit unsigned int elements) is as follows:
        // [
        //      <num-total-sequences>, // McIndex/uint32_t
        //      <num-vertices-in-1st-sequence>, // McIndex/uint32_t
        //      <1st-sequence-is-loop-flag-(McBool)>, // McBool, McIndex/uint32_t
        //      <vertex-indices-of-1st-sequence>, // consecutive elements of McIndex
        //      <num-vertices-in-2nd-sequence>,
        //      <2nd-sequence-is-loop-flag-(McBool)>,
        //      <vertex-indices-of-2nd-sequence>,
        //      ... and so on, until last sequence
        // ]

        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX_SEQUENCE, 0, NULL, &numBytes);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX_SEQUENCE) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        std::vector<uint32_t> seamVertexSequenceArrayFromMCUT;
        seamVertexSequenceArrayFromMCUT.resize(numBytes / sizeof(uint32_t));

        api_err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX_SEQUENCE, numBytes, seamVertexSequenceArrayFromMCUT.data(), NULL);

        if (api_err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX_SEQUENCE) failed (api_err=%d)\n", (int)api_err);
            exit(1);
        }

        // We now put each sorted sequence of seam vertices into its own array.
        // This serves two purposes:
        // 1. to make it easier to write the sequence vertex list to file
        // 2. to show users how to access the ordered sequences of vertices per seam/intersection contour

        // list of seam vertex sequences, each with a flag stating whether it is a loop or not
        std::vector<std::pair<std::vector<McUint32>, McBool>> seamVertexSequences;
        uint32_t runningOffset = 0; // a runnning offset that we use to access data in "seamVertexSequenceArrayFromMCUT"
        // number of sequences produced by MCUT
        const uint32_t numSeamVertexSequences = seamVertexSequenceArrayFromMCUT[runningOffset++]; // always the first element

        // for each sequence...
        for(uint32_t numSeamVertexSequenceIter = 0; numSeamVertexSequenceIter < numSeamVertexSequences; ++numSeamVertexSequenceIter)
        {
            // create entry to store own array and flag
            seamVertexSequences.push_back(std::pair<std::vector<McUint32>, McBool>());
            std::pair<std::vector<McUint32>, McBool>& currentSeamVertexSequenceData = seamVertexSequences.back();
            
            std::vector<McUint32>& currentSeamVertexSequenceIndices = currentSeamVertexSequenceData.first; // Ordered list of vertex indices in CC defining the seam sequence 
            McBool &isLoop = currentSeamVertexSequenceData.second; // does it form a loop? (auxilliary piece of info that might be useful to users)
            
            // NOTE: order in which we do things here matter because of how we rely on "runningOffset++"
            // So here, for each sequence we have 1) the number of vertex indices in that sequence 2)
            // a flag telling us whether the sequence is a loop, and 3) the actually consecutive list of 
            // sorted vertex indices that for the sequence 
            const uint32_t currentSeamVertexSequenceIndicesArraySize = seamVertexSequenceArrayFromMCUT[runningOffset++];
            currentSeamVertexSequenceIndices.resize(currentSeamVertexSequenceIndicesArraySize);

            isLoop = seamVertexSequenceArrayFromMCUT[runningOffset++];

            // copy seam vertex indices of current sequence into local array
            memcpy(
                currentSeamVertexSequenceIndices.data(), 
                seamVertexSequenceArrayFromMCUT.data() + runningOffset, 
                sizeof(McUint32) * currentSeamVertexSequenceIndicesArraySize);

            runningOffset += currentSeamVertexSequenceIndicesArraySize;
        }

        //
        // We are now going to save the sequences to file. To do so, we piggyback
        // on the "writeOFF" function and pretend that we are writing a mesh where 
        // each sequence is a face.
        //

        // stub variables for writing to file
        uint32_t numFacesStub = seamVertexSequences.size();
        std::vector<uint32_t> faceSizesArrayStub(numFacesStub);
        std::vector<uint32_t> faceIndicesArrayStub;
        
        std::string flags_str;

        // for each sequence
        for(uint32_t j =0; j < seamVertexSequences.size(); ++j)
        {
            faceSizesArrayStub[j] = seamVertexSequences[j].first.size();
            faceIndicesArrayStub.insert(
                faceIndicesArrayStub.cend(), 
                seamVertexSequences[j].first.cbegin(), 
                seamVertexSequences[j].first.cend());

            flags_str.append("-id" +std::to_string(j) + ((seamVertexSequences[j].second == MC_TRUE) ? "_isLOOP" : "_isOPEN"));
        }

        char seamFnameBuf[1024];
        sprintf(seamFnameBuf, "frag-%d-seam-vertices%s.txt", i, flags_str.c_str());

        // save seam vertices to file (.txt)
        // ------------------------
        writeOFF(seamFnameBuf,
            NULL,
            // We pretend that the list of seam indices is a list of face indices, when in actual
            // fact we are simply using the output file as storage for later inspection
            faceIndicesArrayStub.data(),
            faceSizesArrayStub.data(),
            NULL,
            0,
            (uint32_t)faceSizesArrayStub.size(), 
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
    printf("read OFF file %s: \n", fpath);

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
    fprintf(stdout, "write OFF file: %s\n", fpath);

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

#if 0
enum ObjFileCmdType
{
    /*
    The v command specifies a vertex by its three cartesian coordinates x, y, and z. 
    The vertex is automatically assigned a name depending on the order in which it 
    is found in the file. The first vertex in the file gets the name ‘1’, the second ‘2’, 
    the third ‘3’, and so on.
    */
    VERTEX,
    /*
    This is the vertex normal command. It specifies the normal vector to the surface. 
    x, y, and z are the components of the normal vector. Note that this normal vector 
    is not yet associated with any vertex point. We will have to associate it with a 
    vertex later with another command called the f command.

    The vertex normal command is typically omitted in files because when we group vertices 
    into polygonal faces with the f command, it will automatically determine the normal 
    vector from the vertex coordinates and the order in which the vertices appear.
    */
    NORMAL,
    /*
    The vertex texture command specifies a point in the texture map, which we covered 
    in an earlier section. u and v are the x and y coordinates in the texture map. These 
    will be floating point numbers between 0 and 1. They really don’t tell you anything 
    by themselves, they must be grouped with a vertex in an f face command, just like 
    the vertex normals.
    */
    TEXCOORD,
    /*
    The face command is probably the most important command. It specifies a polygonal 
    face made from the vertices that came before this line.

    To reference a vertex you follow the implicit numbering system of the vertices. For 
    example’ f 23 24 25 27′ means a polygonal face built from vertices 23, 24, 25, and 
    27 in order.

    For each vertex, you may associate a vn command, which then associates that vertex 
    normal to the corresponding vertex. Similarly, you can associate a vt command with 
    a vertex, which will determine the texture mapping to use at this point.

    If you specify a vt or vn for a vertex in a file, you must specify it for all of them.
    */
    FACE,
    TOTAL,
    UNKNOWN =0xFFFFFFFF
};

// Funcion to read in a .obj file storing a single 3D mesh object (in ASCII format) 
// The double pointers will be allocated inside this function and must be freed by caller.
// Only handles polygonal faces, so "vp" command (which is used to specify control 
// points of the surface or curve ) is ignored if encountered in file.
void read_OBJ_file(
    // path to file
    const char* fpath,
    // pointer to list of vertex coordinates  
    double** pVertices,
    // pointer to list of vertex normals  
    double** pNormals,
    // pointer to list of texture coordinates list  
    double** pTexcoords,
    // pointer to list of face indices 
    unsigned int** pFaceIndices,
    // pointer to list of face sizes (number of vertices in each face)
    unsigned int** pFaceSizes,
    // number of vertices 
    unsigned int* numVertices,
    // number of vertex normals 
    unsigned int* numNormals,
    // number of texture coordinates
    unsigned int* numTexcoords,
    // number of faces
    unsigned int* numFaces,
    // use zero-based indexing when storing face vertex indices in pFaceIndices
    bool useRelativeFaceIndices)
{
    fprintf(stdout, "read .obj file: %s\n", fpath);

    FILE* file = fopen(fpath, "rb");

    if (file == NULL) {
        fprintf(stderr, "error: failed to open file `%s`", fpath);
        exit(1);
    }

    // buffer used to store the contents of a line read from the file.
    char* lineBuf = NULL;
    // current length of the line buffer (in characters read) 
    size_t lineBufLen = 0;
    // flag to indicate that we read the line ok (without any error)
    bool lineOk = true;
    int i = 0; // iterator

    // counts the number of passes we have made over the file to parse contents.
    // this is needed because multiple passes are required e.g. to determine how much
    // memory to allocate before actually copying data into pointers.
    int passIterator =0;

    // these variables are defined after having parsed the full file
    int nVertices = 0; // number of vertex coordinates found in file
    int nNormals = 0; // number of vertex normals found in file
    int nTexCoords = 0; // number of vertex vertex texture coordnates found in file
    int nFaces = 0; // number of faces found in file
    int nFaceIndices = 0; // total number of face indices found in file
    int faceIndicesCounter = 0; // running offset into pFaceIndices (final value will be same as nFaceIndices)

    do{ // each iteration will parse the full file
        
        const size_t nread = getline(&lineBuf, &lineBufLen, file); // read contents of line
        
        while (nread != -1) { // each iteration will parse a line in the file      
            
            const size_t lineLen =  strlen(lineBuf);

            assert(lineLen == nread);

            const bool lineIsEmpty = (lineLen == 0);
            
            if (lineIsEmpty) {
                continue; // .. to next line
            }

            const bool lineIsComment =  lineBuf[0] != '#';
            
            if(lineIsComment)
            {
                continue; // ... to next line
            }

            ObjFileCmdType lineCmdType =  ObjFileCmdType::UNKNOWN;
            
            char tmpBuf[3];
            memcpy(tmpBuf, lineBuf, 2); // copy first two characters in line

            //
            // In the following, we determine the type of "command" in the object file that is
            // contained on the current line.
            //
            const bool lineIsVertexCmd = strcmp("v ", tmpBuf) == 0;
            
            if(lineIsVertexCmd)
            {
                lineCmdType = ObjFileCmdType::VERTEX;
            }
            else{
            
                memcpy(tmpBuf, lineBuf, 3);

                const bool lineIsVertexNormalCmd = strcmp("vn ", tmpBuf) == 0;

                if(lineIsVertexNormalCmd)
                {
                    lineCmdType = ObjFileCmdType::NORMAL;
                }
                else {
                        
                    memcpy(tmpBuf, lineBuf, 3);

                    const bool lineIsVertexTexCoordCmd = strcmp("vt ", tmpBuf) == 0;

                    if (lineIsVertexTexCoordCmd) {
                        lineCmdType = ObjFileCmdType::TEXCOORD;        
                    }
                    else {
                        memcpy(tmpBuf, lineBuf, 2);

                        const bool lineIsFaceCmd = strcmp("f ", tmpBuf) == 0;

                        if (lineIsFaceCmd) {
                            lineCmdType = ObjFileCmdType::FACE;        
                        }
                    }
                }
            }

            if(lineCmdType == ObjFileCmdType::UNKNOWN)
            {
                fprintf(stderr, "warning: unrecognised command in line '%s'\n", lineBuf);
            }

            switch (lineCmdType) {
                case ObjFileCmdType::VERTEX:
                {
                    int vertexId = nVertices++; // incremental vertex count in file
                    
                    if(passIterator == 1)
                    {           
                
                        const char* delimiter = " ";
                        char* pch = strtok(lineBuf+2, delimiter);
                        double coords[3] = {0.0, 0.0, 0.0};
                        int compIter = 0;
                    
                        while(pch != NULL)
                        { 
                            scanf(pch, "%f", coords+compIter);
                            pch = strtok (NULL, delimiter);
                        }
                    
                        if(compIter != 3)
                        {
                            fprintf(stderr, "error: have %d components for position of vertex %d\n", compIter, vertexId);
                            std::abort();
                        }
                    }

                }break;
                case ObjFileCmdType::NORMAL:
                {
                    int normalId = nNormals++; // incremental vertex-normal count in file
                    
                    if(passIterator == 1)
                    {           
                
                        const char* delimiter = " ";
                        char* pch = strtok(lineBuf+2, delimiter);
                        double comps[3] = {0.0, 0.0, 0.0}; // components
                        int compIter = 0;
                    
                        while(pch != NULL)
                        { 
                            scanf(pch, "%f", comps+compIter);
                            pch = strtok (NULL, delimiter);
                        }
                    
                        if(compIter != 3)
                        {
                            fprintf(stderr, "error: have %d components for normal at idx=%d\n", compIter, normalId);
                            std::abort();
                        }
                    }
                }break;
                case ObjFileCmdType::TEXCOORD:
                {
                    int texCoordId = nTexCoords++; // incremental vertex-normal count in file
                    
                    if(passIterator == 1)
                    {           
                
                        const char* delimiter = " ";
                        char* pch = strtok(lineBuf+2, delimiter);
                        double comps[2] = {0.0, 0.0}; // components
                        int compIter = 0;
                    
                        while(pch != NULL)
                        { 
                            scanf(pch, "%f", comps+compIter);
                            pch = strtok (NULL, delimiter);
                        }
                    
                        if(compIter != 3)
                        {
                            fprintf(stderr, "error: have %d components for texcoord at idx=%d\n", compIter, texCoordId);
                            std::abort();
                        }
                    }
                }break;
                case ObjFileCmdType::FACE:
                {
                    int faceId = nFaces++; // incremental face count in file
                    
                    const char* delimiter = " ";
                    
                    if(passIterator == 1)
                    {           
                        //
                        // count the number of vertices in face first
                        //
                        char* pch = strtok(lineBuf+2, delimiter);
                        uint32_t faceVertexCount = 0;
                        
                        while(pch != NULL)
                        { 
                            pch = strtok (NULL, delimiter);
                            faceVertexCount++; // track number of vertices found in face
                        }

                        *pFaceSizes[faceId] = faceVertexCount;

                        nFaceIndices += faceVertexCount;
                    }

                    if(passIterator == 2) // third pass
                    {
                        
                        //
                        // allocate mem for the number of vertices in face 
                        //

                        const uint32_t faceVertexCount = (*pFaceSizes)[faceId];

                        // back to start of line
                        char* pch = strtok(lineBuf+2, " "); 

                        int iter = 0; // incremented per vertex of face
                        
                        while(pch != NULL)
                        { 

                            uint32_t it = 0; // incremented per data item of face-vertex (i.e. position, normal and texcoord))
                                    
                            char* pch_ = strtok(pch, "/"); 
                            while (pch_ != NULL) { // an iteration parses all data of a face vertex
                                
                                int val;
                                scanf(pch_, "%d", &val); // extract face vertex data str
                                pch_ = strtok (NULL, "/");

                                switch (it) {
                                    case 0: // vertex id
                                        (*pFaceIndices)[faceIndicesCounter] = (uint32_t)val;
                                    break;
                                    case 1: // texcooord id
                                        // TODO
                                    break;
                                    case 2: // normal id
                                        // TODO
                                    break;
                                    default:

                                        break;
                                }
                                
                                faceIndicesCounter += 1;
    
                                it++;
                            } 

                            pch = strtok (NULL, delimiter); // next token

                            iter++; // track number of vertices found in face
                        }
                    
                        if(iter != (int)faceVertexCount)
                        {
                            fprintf(stderr, "error: have %d vertices when there should be =%d\n", iter, faceVertexCount);
                            std::abort();
                        }
                    }
                }break;
                default:
                break;
            } // switch (lineCmdType) {  
        } 
        
        if(passIterator ==0) // first pass
        {
            if(nVertices > 0)
            {
                *pVertices = (double*)malloc(nVertices * sizeof(double) *3);
            }

            if(nNormals > 0)
            {
                pNormals = (double*)malloc(nNormals * sizeof(double) * 3);
            }

            if(nTexCoords > 0)
            {
                pTexCoords = (double*)malloc(nTexCoords * sizeof(double) * 3)
            }

            if(nFaces > 0)
            {
                *pFaceSizes = (uint32_t* )malloc(nFaces * sizeof(uint32_t));
                
                if(nFaceIndices == 0)
                {
                    fprintf(stderr, "error: invalid face index count %d\n", nFaceIndices);
                    std::abort();
                }

                *pFaceIndices = (uint32_t* )malloc(nFaceIndices * sizeof(uint32_t));
            }
        }
    

    }while(++passIterator < 3);

    //
    // finish, and free up memory
    //
    if(lineBuf != NULL)
    {
        free(lineBuf);
    }

}


#endif