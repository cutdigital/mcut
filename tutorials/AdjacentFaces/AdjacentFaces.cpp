#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS 1

#ifdef _WIN32
#pragma warning(disable : 26812) // Unscoped enums from mcut.h
#endif // _WIN32
#endif

/*
This tutorial shows how to query adjacent faces of any face of a connected component.
The tutorial is presented in the context of an example where a user wants to merge 
neighouring faces into one polygon based on some property stored at those 
merged faces (e.g. an integer id number).
*/

#include "mcut/mcut.h"

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

int main()
{
    // TODO
#if 0
    // create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_NULL_HANDLE);
    assert(err == MC_NO_ERROR);

    //  do the cutting (boolean ops)
    // -------------------------------

        err = mcDispatch(
            context,
            MC_DISPATCH_VERTEX_ARRAY_DOUBLE | // vertices are in array of doubles
                MC_DISPATCH_ENFORCE_GENERAL_POSITION | // perturb if necessary
                boolOpFlags, // filter flags which specify the type of output we want
            // source mesh
            reinterpret_cast<const void*>(srcMesh.vertexCoordsArray.data()),
            reinterpret_cast<const uint32_t*>(srcMesh.faceIndicesArray.data()),
            srcMesh.faceSizesArray.data(),
            static_cast<uint32_t>(srcMesh.vertexCoordsArray.size() / 3),
            static_cast<uint32_t>(srcMesh.faceSizesArray.size()),
            // cut mesh
            reinterpret_cast<const void*>(cutMesh.vertexCoordsArray.data()),
            cutMesh.faceIndicesArray.data(),
            cutMesh.faceSizesArray.data(),
            static_cast<uint32_t>(cutMesh.vertexCoordsArray.size() / 3),
            static_cast<uint32_t>(cutMesh.faceSizesArray.size()));

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

        assert(numConnComps == 1); // exactly 1 result (for this example)

        std::vector<McConnectedComponent> connectedComponents(numConnComps, MC_NULL_HANDLE);
        connectedComponents.resize(numConnComps);
        err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, (uint32_t)connectedComponents.size(), connectedComponents.data(), NULL);

        assert(err == MC_NO_ERROR);

        // query the data of each connected component from MCUT
        // -------------------------------------------------------

        McConnectedComponent connComp = connectedComponents[0];
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
        std::vector<uint32_t> faceSizes(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL);
        assert(err == MC_NO_ERROR);

        // query the face map
        // ------------------------
        const uint32_t ccFaceCount = static_cast<uint32_t>(faceSizes.size());

        /// ------------------------------------------------------------------------------------

        // Here we show, how to know when connected components, pertain particular boolean operations.

        McPatchLocation patchLocation = (McPatchLocation)0;

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL);
        assert(err == MC_NO_ERROR);

        McFragmentLocation fragmentLocation = (McFragmentLocation)0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &fragmentLocation, NULL);
        assert(err == MC_NO_ERROR);

        uint32_t ccEdgeCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_EDGE_COUNT, sizeof(uint32_t), &ccEdgeCount, NULL);

        // save cc mesh to .obj file
        // -------------------------

        std::string fpath(DATA_DIR "/" + boolOpName + ".obj");

        printf("write file: %s\n", fpath.c_str());

        std::ofstream file(fpath);

        // write vertices and normals
        for (uint32_t i = 0; i < ccVertexCount; ++i) {
            double x = ccVertices[(uint64_t)i * 3 + 0];
            double y = ccVertices[(uint64_t)i * 3 + 1];
            double z = ccVertices[(uint64_t)i * 3 + 2];
            file << "v " << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << x << " " << y << " " << z << std::endl;
        }

        int faceVertexOffsetBase = 0;

        // for each face in CC
        for (uint32_t f = 0; f < ccFaceCount; ++f) {
            bool reverseWindingOrder = (fragmentLocation == MC_FRAGMENT_LOCATION_BELOW) && (patchLocation == MC_PATCH_LOCATION_OUTSIDE);
            int faceSize = faceSizes.at(f);
            file << "f ";
            // for each vertex in face
            for (int v = (reverseWindingOrder ? (faceSize - 1) : 0);
                 (reverseWindingOrder ? (v >= 0) : (v < faceSize));
                 v += (reverseWindingOrder ? -1 : 1)) {
                const int ccVertexIdx = ccFaceIndices[(uint64_t)faceVertexOffsetBase + v];
                file << (ccVertexIdx + 1) << " ";
            } // for (int v = 0; v < faceSize; ++v) {
            file << std::endl;

            faceVertexOffsetBase += faceSize;
        }

        // 6. free connected component data
        // --------------------------------
        err = mcReleaseConnectedComponents(context, (uint32_t)connectedComponents.size(), connectedComponents.data());
        assert(err == MC_NO_ERROR);
    }

    // 7. destroy context
    // ------------------
    err = mcReleaseContext(context);

    assert(err == MC_NO_ERROR);
#endif
    return 0;
}
