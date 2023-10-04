/**
 * Copyright (c) 2021-2022 Floyd M. Chitalu.
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
#include <cstring>
#include <inttypes.h> // PRId64
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

// libigl dependencies
#include <Eigen/Core>
#include <igl/list_to_matrix.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/writeOBJ.h>

#define stringize(s) #s
#define XSTR(s) stringize(s)

#define ASSERT(a)                        \
    do {                                 \
        if (0 == (a)) {                  \
            std::fprintf(stderr,         \
                "Assertion failed: %s, " \
                "%d at \'%s\'\n",        \
                __FILE__,                \
                __LINE__,                \
                XSTR(a));                \
            std::abort();                \
        }                                \
    } while (0)

#define mcCheckError(errCode) mcCheckError_(errCode, __FILE__, __LINE__)

void mcCheckError_(McResult err, const char* file, int line);

void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

void readMesh(const std::string& path, std::vector<double>& V, std::vector<uint32_t>& F, std::vector<uint32_t>& Fsizes);
void writeOBJ(
    const std::string& path,
    const float* ccVertices,
    const int ccVertexCount,
    const uint32_t* ccFaceIndices,
    const uint32_t* faceSizes,
    const uint32_t ccFaceCount);

int main(int argc, char* argv[])
{
    bool help = false;

    for (int i = 0; help == false && i < argc; ++i) {
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

    std::cout << "source-mesh file:" << srcMeshFilePath << std::endl;
    std::cout << "cut-mesh file:" << cutMeshFilePath << std::endl;

    // load meshes
    // -----------
    std::vector<double> srcMeshVertices;
    std::vector<uint32_t> srcMeshFaceIndices;
    std::vector<uint32_t> srcMeshFaceSizes;
    readMesh(srcMeshFilePath, srcMeshVertices, srcMeshFaceIndices, srcMeshFaceSizes);

    printf("src-mesh vertices=%d faces=%d\n", (int)srcMeshVertices.size()/3, (int)srcMeshFaceSizes.size());

    std::vector<double> cutMeshVertices;
    std::vector<uint32_t> cutMeshFaceIndices;
    std::vector<uint32_t> cutMeshFaceSizes;
    readMesh(cutMeshFilePath, cutMeshVertices, cutMeshFaceIndices, cutMeshFaceSizes);

    printf("cut-mesh vertices=%d faces=%d\n", (int)cutMeshVertices.size()/3, (int)cutMeshFaceSizes.size());


    // init dispatch context
    // ---------------------
    McContext context;
#ifdef  NDEBUG
    McResult err = mcCreateContext(&context, MC_NULL_HANDLE);
#else
    McResult err = mcCreateContext(&context, MC_DEBUG);
#endif
    mcCheckError(err);

    // config debug output
    // -----------------------
    McSize numBytes = 0;
    McFlags contextFlags;
    err = mcGetInfo(context, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes);
    mcCheckError(err);

    ASSERT(sizeof(McFlags) == numBytes);

    err = mcGetInfo(context, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr);
    mcCheckError(err);

    if (contextFlags & MC_DEBUG) {
        mcDebugMessageCallback(context, mcDebugOutput, nullptr);
        mcDebugMessageControl(context, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);
    }


    // do the cutting
    // --------------
    err = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
        // source mesh
        srcMeshVertices.data(),
        srcMeshFaceIndices.data(),
        srcMeshFaceSizes.data(),
        (uint32_t)srcMeshVertices.size() / 3,
        (uint32_t)srcMeshFaceSizes.size(),
        // cut mesh
        cutMeshVertices.data(),
        cutMeshFaceIndices.data(),
        cutMeshFaceSizes.data(),
        (uint32_t)cutMeshVertices.size() / 3,
        (uint32_t)cutMeshFaceSizes.size());

    mcCheckError(err);

    uint32_t numConnComps;
    std::vector<McConnectedComponent> pConnComps;

    // we want to query all available connected components:
    // --> fragments, patches and seamed
    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps);
    mcCheckError(err);

    if (numConnComps == 0) {
        printf("no connected components found\n");
        std::exit(0);
    }

    pConnComps.resize(numConnComps);

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)pConnComps.size(), pConnComps.data(), NULL);

    mcCheckError(err);

    //
    // query connected component data
    //
    for (int i = 0; i < (int)pConnComps.size(); ++i) {
        McConnectedComponent connCompId = pConnComps[i]; // connected compoenent id

        printf("connected component: %d\n", i);

        // vertex array
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        mcCheckError(err);
        uint32_t numberOfVertices = (McUint32)(numBytes / (sizeof(double)*3));
        ASSERT(numberOfVertices >= 3);
        std::vector<double> vertices((size_t)numberOfVertices * 3u);

        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)vertices.data(), NULL);
        mcCheckError(err);

        printf("vertices: %d\n", (int)vertices.size() / 3);

#if 1 // triangulated output
        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);
        ASSERT(err == MC_NO_ERROR);
        std::vector<uint32_t> faceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes, faceIndices.data(), NULL);
        ASSERT(err == MC_NO_ERROR);

        std::vector<uint32_t> faceSizes(faceIndices.size()/3, 3);
#else // nontriangulated output
        // face indices
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
        mcCheckError(err);

        ASSERT(numBytes > 0);

        std::vector<uint32_t> faceIndices;
        faceIndices.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, faceIndices.data(), NULL);
        mcCheckError(err);

        // face sizes
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        mcCheckError(err);

        ASSERT(numBytes > 0);

        std::vector<uint32_t> faceSizes;
        faceSizes.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL);
        mcCheckError(err);
#endif
        printf("faces: %d\n", (int)faceSizes.size());

        char fnameBuf[512];
        sprintf(fnameBuf, "cc%d.obj", i);
        
        std::vector<float> f;
        for(uint32_t j =0; j < (uint32_t)vertices.size(); ++j)
            f.push_back((float)vertices[j]);
        writeOBJ(fnameBuf,
            (float*)f.data(),
            (uint32_t)vertices.size() / 3,
            (uint32_t*)faceIndices.data(),
            (uint32_t*)faceSizes.data(),
            (uint32_t)faceSizes.size());
    }

    // destroy internal data associated with each connected component
    err = mcReleaseConnectedComponents(context, (uint32_t)pConnComps.size(), pConnComps.data());
    mcCheckError(err);

    // destroy context
    err = mcReleaseContext(context);
    mcCheckError(err);

    return 0;
}

void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{
    
    //printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
    //printf("userParam=%p\n", userParam);

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

// libIGL's "writeOBJ" function fails when dealing with facex with > 4 vertices.
void writeOBJ(
    const std::string& path,
    const float* ccVertices,
    const int ccVertexCount,
    const uint32_t* ccFaceIndices,
    const uint32_t* faceSizes,
    const uint32_t ccFaceCount)
{
    printf("write file: %s\n", path.c_str());

    std::ofstream file(path);

    // write vertices and normals
    for (uint32_t i = 0; i < (uint32_t)ccVertexCount; ++i) {
        double x = ccVertices[(McSize)i * 3 + 0];
        double y = ccVertices[(McSize)i * 3 + 1];
        double z = ccVertices[(McSize)i * 3 + 2];
        file << "v " << x << " " << y << " " << z << std::endl;
    }

    int faceVertexOffsetBase = 0;

    // for each face in CC
    for (uint32_t f = 0; f < ccFaceCount; ++f) {

        int faceSize = faceSizes[f];
        file << "f ";
        // for each vertex in face
        for (int v = 0; (v < faceSize); v++) {
            const int ccVertexIdx = ccFaceIndices[(McSize)faceVertexOffsetBase + v];
            file << (ccVertexIdx + 1) << " ";
        } // for (int v = 0; v < faceSize; ++v) {
        file << std::endl;

        faceVertexOffsetBase += faceSize;
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
    float** pVertices,
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
    *pVertices = (float*)malloc(sizeof(float) * (*numVertices) * 3);
    *pFaceSizes = (unsigned int*)malloc(sizeof(unsigned int) * (*numFaces));

    // vertices
    for (i = 0; i < (float)(*numVertices); ++i) {
        lineOk = readLine(file, &lineBuf, &lineBufLen);

        if (!lineOk) {
            fprintf(stderr, "error: .off vertex not found\n");
            exit(1);
        }

        float x, y, z;
        sscanf(lineBuf, "%f %f %f", &x, &y, &z);

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
void readMesh(const std::string& path, std::vector<double>& V, std::vector<uint32_t>& F, std::vector<uint32_t>& Fsizes)
{
    printf("read: %s\n", path.c_str());
    
    Eigen::MatrixXd Vmat;
    Eigen::MatrixXi Fmat;

    if (path.find(".obj") != std::string::npos) {
        igl::readOBJ(path, Vmat, Fmat);

        for (int i = 0; i < (int)Vmat.rows(); ++i) {
            const Eigen::VectorXd v = Vmat.row(i);
            V.push_back((double)v(0));
            V.push_back((double)v(1));
            V.push_back((double)v(2));
        }

        for (int i = 0; i < (int)Fmat.rows(); ++i) {
            const Eigen::VectorXi f = Fmat.row(i);
            for (int j = 0; j < (int)f.rows(); ++j) {
                F.push_back((uint32_t)f(j));
            }

            Fsizes.push_back((uint32_t)f.rows());
        }
    }
    else if(path.find(".off") != std::string::npos){
        float* pVertices=nullptr;
        unsigned int* pFaceIndices=nullptr;
        unsigned int* pFaceSizes=nullptr;
        unsigned int numVertices=0;
        unsigned int numFaces=0;

        readOFF(path.c_str(), &pVertices, &pFaceIndices,
        &pFaceSizes,
        &numVertices,
        &numFaces);

        for (int i = 0; i < (int)numVertices; ++i) {
            V.push_back(pVertices[i*3+0]);
            V.push_back(pVertices[i*3+1]);
            V.push_back(pVertices[i*3+2]);
        }
        int off = 0;
        for (int i = 0; i < (int)numFaces; ++i) {
            unsigned int faceSize = pFaceSizes[i];
            
            for (int j = 0; j < (int)faceSize; ++j) {
                F.push_back(pFaceIndices[off + j]);
            }
            off +=faceSize;
            Fsizes.push_back(faceSize);
        }

        free(pVertices);
        pVertices=nullptr;
        free(pFaceIndices);
        pFaceIndices=nullptr;
        free(pFaceSizes);
        pFaceSizes=nullptr;
    }
    else{
        printf("error: expected .obj or .off file\n");
        std::exit(1);
    }

    
}
