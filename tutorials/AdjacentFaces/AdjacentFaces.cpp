#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS 1

#ifdef _WIN32
#pragma warning(disable : 26812) // Unscoped enums from mcut.h
#endif // _WIN32
#endif

/*
This tutorial shows how to query adjacent faces of any face of a connected component.

The tutorial is presented in the context of merging neighouring faces of a 
connected component that share some property (e.g. some ID number), where 
this property is derived from the faces' origin face.

A group of faces that share the property and belong/define-a-small-patch will be merged 
into a single face.
*/

#include "mcut/mcut.h"

#include <algorithm> // std::sort
#include <cassert>
#include <fstream>
#include <map>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

void saveOBJ(
    const std::string& path,
    const double* ccVertices,
    const int ccVertexCount,
    const uint32_t* ccFaceIndices,
    const uint32_t* faceSizes,
    const uint32_t ccFaceCount)
{
    printf("write file: %s\n", path.c_str());

    std::ofstream file(path);

    // write vertices and normals
    for (uint32_t i = 0; i < ccVertexCount; ++i) {
        double x = ccVertices[(uint64_t)i * 3 + 0];
        double y = ccVertices[(uint64_t)i * 3 + 1];
        double z = ccVertices[(uint64_t)i * 3 + 2];
        file << "v " << x << " " << y << " " << z << std::endl;
    }

    int faceVertexOffsetBase = 0;

    // for each face in CC
    for (uint32_t f = 0; f < ccFaceCount; ++f) {

        int faceSize = faceSizes[f];
        file << "f ";
        // for each vertex in face
        for (int v = 0; (v < faceSize); v++) {
            const int ccVertexIdx = ccFaceIndices[(uint64_t)faceVertexOffsetBase + v];
            file << (ccVertexIdx + 1) << " ";
        } // for (int v = 0; v < faceSize; ++v) {
        file << std::endl;

        faceVertexOffsetBase += faceSize;
    }
}

int main()
{
    // src mesh (square made up of two triangles)
    std::vector<double> srcMeshVertices = {
        0., 0., 0., // 0 bottom left
        1., 0., 0., // 1 bottom right
        1., 1., 0, // 2 top right
        0., 1., 0. // 3 top left
    };

    std::vector<uint32_t> srcMeshFaceIndices = {
        0, 1, 2, // 0
        0, 2, 3 // 1
    };

    std::vector<uint32_t> srcMeshFaceSizes = {
        3, 3
    };

    {
        char buf[512];
        sprintf(buf, OUTPUT_DIR "/srcMesh.obj");
        saveOBJ(buf, &srcMeshVertices[0], srcMeshVertices.size() / 3, &srcMeshFaceIndices[0], &srcMeshFaceSizes[0], srcMeshFaceSizes.size());
    }

    // cut mesh (quad)
    std::vector<double> cutMeshVertices = {
        -2.0, 0.5, 2.0, // 0 front left
        2., 0.5, 2.0, // 1 front right
        2., 0.5, -2., // 2 back right
        -2., 0.5, -2., // 3 back left
    };

    std::vector<uint32_t> cutMeshFaceIndices = {
        0, 1, 2, 3
    };

    std::vector<uint32_t> cutMeshFaceSizes = {
        4
    };

    {
        char buf[512];
        sprintf(buf, OUTPUT_DIR "/cutMesh.obj");
        saveOBJ(buf, &cutMeshVertices[0], cutMeshVertices.size() / 3, &cutMeshFaceIndices[0], &cutMeshFaceSizes[0], cutMeshFaceSizes.size());
    }

    // a map denoting the adjacent faces of the source mesh that share some property
    // (in this tutorial its a tag -> 0xBEEF) that we will use to merge faces
    // in output fragments
    std::map<uint32_t, int> srcMeshFaceToTag = {
        { 0, 0xBEEF },
        { 1, 0xBEEF }
    };

    // create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_NULL_HANDLE);
    assert(err == MC_NO_ERROR);

    //  do the cutting (boolean ops)
    // -------------------------------

    err = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_FACE_MAP,
        // source mesh
        reinterpret_cast<const void*>(srcMeshVertices.data()),
        reinterpret_cast<const uint32_t*>(srcMeshFaceIndices.data()),
        srcMeshFaceSizes.data(),
        static_cast<uint32_t>(srcMeshVertices.size() / 3),
        static_cast<uint32_t>(srcMeshFaceSizes.size()),
        // cut mesh
        reinterpret_cast<const void*>(cutMeshVertices.data()),
        reinterpret_cast<const uint32_t*>(cutMeshFaceIndices.data()),
        cutMeshFaceSizes.data(),
        static_cast<uint32_t>(cutMeshVertices.size() / 3),
        static_cast<uint32_t>(cutMeshFaceSizes.size()));

    assert(err == MC_NO_ERROR);

    // query the number of available connected component
    // --------------------------------------------------
    uint32_t numConnComps;
    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL, &numConnComps);
    assert(err == MC_NO_ERROR);

    printf("connected components: %d\n", (int)numConnComps);

    if (numConnComps == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    assert(numConnComps == 2); // exactly 2 (each has two polygons)

    std::vector<McConnectedComponent> connectedComponents(numConnComps, MC_NULL_HANDLE);
    connectedComponents.resize(numConnComps);
    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, (uint32_t)connectedComponents.size(), connectedComponents.data(), NULL);

    assert(err == MC_NO_ERROR);

    // query the data of each connected component from MCUT
    // -------------------------------------------------------

    for (int c = 0; c < numConnComps; ++c) {
        McConnectedComponent connComp = connectedComponents[c];
        uint64_t numBytes = 0;

        // query the number of vertices
        // --------------------------------

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        uint32_t ccVertexCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, numBytes, &ccVertexCount, NULL);
        assert(err == MC_NO_ERROR);

        // query the vertices
        // ----------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<double> ccVertices((uint64_t)ccVertexCount * 3u, 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);
        assert(err == MC_NO_ERROR);

        // query the faces
        // -------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);
        assert(err == MC_NO_ERROR);

        // query the face sizes
        // ------------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceSizes(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, ccFaceSizes.data(), NULL);
        assert(err == MC_NO_ERROR);

        const uint32_t ccFaceCount = static_cast<uint32_t>(ccFaceSizes.size());

        {
            char buf[512];
            sprintf(buf, OUTPUT_DIR "/cc%d.obj", c);
            saveOBJ(buf, &ccVertices[0], ccVertexCount, &ccFaceIndices[0], &ccFaceSizes[0], ccFaceCount);
        }

        // query the face map
        // ------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceMap(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, numBytes, ccFaceMap.data(), NULL);
        assert(err == MC_NO_ERROR);

        // query the face adjacency
        // ------------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceAdjFaces(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE, numBytes, ccFaceAdjFaces.data(), NULL);
        assert(err == MC_NO_ERROR);

        // query the face adjacency sizes
        // -------------------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceAdjFacesSizes(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE, numBytes, ccFaceAdjFacesSizes.data(), NULL);
        assert(err == MC_NO_ERROR);

        // map tag to faces

        std::map<int, std::vector<uint32_t>> tagToCcFace;

        // for each cc face
        for (int f = 0; f < ccFaceCount; ++f) {
            int imFace = ccFaceMap[f];
            int imFaceTag = srcMeshFaceToTag[imFace];
            tagToCcFace[imFaceTag].push_back(f);
        }

        auto ccFaceGetAdjFacesBaseOffset = [](const uint32_t ccFaceIdx, const uint32_t* ccFaceAdjFacesSizes) -> uint32_t {
            uint32_t baseOffset = 0;
            for (uint32_t f = 0; f < ccFaceIdx; ++f) {
                baseOffset += ccFaceAdjFacesSizes[f];
            }
            return baseOffset;
        };

        auto ccFaceIndicesBaseOffset = [](const uint32_t ccFaceIdx, const uint32_t* ccFaceSizes) -> uint32_t {
            uint32_t baseOffset = 0;
            for (uint32_t f = 0; f < ccFaceIdx; ++f) {
                baseOffset += ccFaceSizes[f];
            }
            return baseOffset;
        };

        std::vector<uint32_t> ccFaceIndicesNew;
        std::vector<uint32_t> ccFaceSizesNew;

        // for each tag
        for (std::map<int, std::vector<uint32_t>>::const_iterator iter = tagToCcFace.cbegin(); iter != tagToCcFace.cend(); ++iter) {

            if (iter->second.size() == 1) {

                const uint32_t ccFaceIdx = iter->second.front();
                const uint32_t baseIdx = ccFaceIndicesBaseOffset(ccFaceIdx, ccFaceSizes.data());
                const uint32_t ccFaceVertexCount = ccFaceSizes[ccFaceIdx];

                for (int i = 0; i < ccFaceVertexCount; ++i) {
                    ccFaceIndicesNew.push_back(ccFaceIndices[baseIdx + i]);
                }
                ccFaceSizesNew.push_back(ccFaceVertexCount);

            } else { // multiple faces in set i.e. they need to be merged

                std::vector<uint32_t> similarlyTaggedFaces = iter->second; // copy!

                // merge the faces that are adjacent
                std::vector<std::vector<uint32_t>> adjFaceLists;
                do {
                    adjFaceLists.push_back(std::vector<uint32_t>());
                    std::vector<uint32_t>& curAdjFaceList = adjFaceLists.back();

                    std::queue<uint32_t> q;

                    std::vector<uint32_t>::const_iterator cur = similarlyTaggedFaces.cend();
                    std::vector<uint32_t>::const_iterator next = similarlyTaggedFaces.cbegin();

                    do {
                        cur = next;

                        const int numAdjFaces = ccFaceAdjFacesSizes[*cur];
                        const int ccFaceAdjFacesBaseOffset = ccFaceGetAdjFacesBaseOffset(*cur, ccFaceAdjFacesSizes.data());

                        curAdjFaceList.push_back(*cur);
                        // for each adjacent face of current face
                        for (int i = 0; i < numAdjFaces; ++i) {
                            const uint32_t adjFace = ccFaceAdjFaces[ccFaceAdjFacesBaseOffset + i];
                            std::vector<uint32_t>::const_iterator fiter = std::find(curAdjFaceList.cbegin(), curAdjFaceList.cend(), adjFace);
                            bool alreadyKnown = (fiter != curAdjFaceList.cend());
                            if (!alreadyKnown) {
                                curAdjFaceList.push_back(*fiter);
                            }
                        }

                        // TODO: populate the queue with the unwalked neighbours of current face

                        next = similarlyTaggedFaces.cend();

                        // TODO: set "next" to any unwalked adjacent face of current as next
                        // i.e. search the queue

                    } while (!similarlyTaggedFaces.empty());
                } while (!similarlyTaggedFaces.empty());

                std::vector<std::pair<int, int>> merged_polygon_halfedges;

                // for each face to be merge
                for (int j = 0; j < iter->second.size(); ++j) {

                    const uint32_t ccFaceIdx = iter->second[j];
                    const uint32_t ccFaceVertexCount = ccFaceSizes[ccFaceIdx];
                    const uint32_t baseIdx = ccFaceIndicesBaseOffset(ccFaceIdx, ccFaceSizes.data());
                    const int numFaceEdges = (int)ccFaceVertexCount;

                    // for each edge of face
                    for (int ccFaceEdgeIdx = 0; ccFaceEdgeIdx < numFaceEdges; ++ccFaceEdgeIdx) {

                        const int srcIdx = ccFaceEdgeIdx;
                        const int tgtIdx = (ccFaceEdgeIdx + 1) % ccFaceVertexCount;
                        const uint32_t srcVertexIdx = ccFaceIndices[baseIdx + srcIdx];
                        const uint32_t tgtVertexIdx = ccFaceIndices[baseIdx + tgtIdx];

                        std::vector<std::pair<int, int>>::const_iterator fiter = std::find_if(
                            merged_polygon_halfedges.cbegin(),
                            merged_polygon_halfedges.cend(),
                            [&](const std::pair<int, int>& elem) {
                                return (elem.first == srcVertexIdx && elem.second == tgtVertexIdx) || (elem.second == srcVertexIdx && elem.first == tgtVertexIdx);
                            });

                        const bool opposite_halfedge_exists = (fiter != merged_polygon_halfedges.cend());

                        if (opposite_halfedge_exists) {
                            merged_polygon_halfedges.erase(fiter);
                        } else {
                            merged_polygon_halfedges.emplace_back(srcVertexIdx, tgtVertexIdx);
                        }
                    }
                }

                // sort the halfedges into a connectable/linked sequence
                // i.e. "target" of "current" should be "source" of "next"
                std::sort(
                    merged_polygon_halfedges.begin(),
                    merged_polygon_halfedges.end(),
                    [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                        return a.second == b.first;
                    });

                for (std::vector<std::pair<int, int>>::const_iterator halfedge_iter = merged_polygon_halfedges.cbegin();
                     halfedge_iter != merged_polygon_halfedges.cend();
                     halfedge_iter++) {
                    const uint32_t srcVertexIdx = halfedge_iter->first;
                    ccFaceIndicesNew.push_back(srcVertexIdx);
                }

                // NOTE: a polygon has the same number of vertices as its edges.
                const int ccFaceVertexCountNew = (int)merged_polygon_halfedges.size();
                ccFaceSizesNew.push_back(ccFaceVertexCountNew);
            }
        }

        // NOTE: For the sake of simplicity, unused vertices are not removed.

        const uint32_t ccFaceCountNew = (uint32_t)ccFaceSizesNew.size();

        {
            char buf[512];
            sprintf(buf, OUTPUT_DIR "/cc%d-new.obj", c);
            saveOBJ(buf, &ccVertices[0], ccVertexCount, &ccFaceIndicesNew[0], &ccFaceSizesNew[0], ccFaceCountNew);
        }
    }

    // free connected component data
    // --------------------------------
    err = mcReleaseConnectedComponents(context, (uint32_t)connectedComponents.size(), connectedComponents.data());
    assert(err == MC_NO_ERROR);

    // destroy context
    // ------------------
    err = mcReleaseContext(context);

    assert(err == MC_NO_ERROR);

    return 0;
}
