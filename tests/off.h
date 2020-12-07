#ifndef OFFLOADER_H_
#define OFFLOADER_H_

extern "C" void readOFF(
    const char* fpath,
    float** pVertices,
    unsigned int** pFaceIndices,
    unsigned int** pFaceSizes,
    unsigned int* numVertices,
    unsigned int* numFaces);

extern "C" void writeOFF(
    const char* fpath,
    float* pVertices,
    unsigned int* pFaceIndices,
    unsigned int* pFaceSizes,
    unsigned int* pEdgeIndices,
    unsigned int numVertices,
    unsigned int numFaces,
    unsigned int numEdges);

#endif // #ifndef OFFLOADER_H_