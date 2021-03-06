/*
This tutorial shows how to compute boolean operations using MCUT.
*/

#include "mcut/mcut.h"

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
// libigl dependencies
#include <Eigen/Core>
#include <igl/barycentric_coordinates.h>
#include <igl/barycentric_interpolation.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

struct InputMesh {
    // variables for reading .obj file data with libigl
    std::vector<std::vector<double>> V, TC, N;
    std::vector<std::vector<int>> F, FTC, FN;
    std::vector<std::tuple<std::string, unsigned, unsigned>> FM;

    // variables for mesh data in a format suited for MCUT
    std::string fpath; // path to mesh file
    std::vector<uint32_t> faceSizesArray; // vertices per face
    std::vector<uint32_t> faceIndicesArray; // face indices
    std::vector<double> vertexCoordsArray; // vertex coords
};

int main(int argc, char* argv[])
{
    // 1. load meshes.
    // -----------------
    InputMesh srcMesh;

    srcMesh.fpath = DATA_DIR "/cube.obj";
    bool srcMeshLoaded = igl::readOBJ(srcMesh.fpath, srcMesh.V, srcMesh.TC, srcMesh.N, srcMesh.F, srcMesh.FTC, srcMesh.FN);

    if (!srcMeshLoaded) {
        std::fprintf(stderr, "error: could not load source mesh --> %s\n", srcMesh.fpath.c_str());
        std::exit(1);
    }

    // copy vertices
    for (int i = 0; i < (int)srcMesh.V.size(); ++i) {
        const std::vector<double>& v = srcMesh.V[i];
        assert(v.size() == 3);
        srcMesh.vertexCoordsArray.push_back(v[0]);
        srcMesh.vertexCoordsArray.push_back(v[1]);
        srcMesh.vertexCoordsArray.push_back(v[2]);
    }

    // copy faces
    for (int i = 0; i < srcMesh.F.size(); ++i) {
        const std::vector<int>& f = srcMesh.F[i];
        for (int j = 0; j < f.size(); ++j) {
            srcMesh.faceIndicesArray.push_back(f[j]);
        }

        srcMesh.faceSizesArray.push_back(f.size());
    }

    printf("source mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)srcMesh.V.size(), (int)srcMesh.F.size());

    InputMesh cutMesh;
    cutMesh.fpath = DATA_DIR "/torus.obj";
    bool cutMeshLoaded = igl::readOBJ(cutMesh.fpath, cutMesh.V, cutMesh.TC, cutMesh.N, cutMesh.F, cutMesh.FTC, cutMesh.FN);

    if (!cutMeshLoaded) {
        std::fprintf(stderr, "error: could not load source mesh --> %s\n", cutMesh.fpath.c_str());
        std::exit(1);
    }

    // copy vertices
    for (int i = 0; i < cutMesh.V.size(); ++i) {
        const std::vector<double>& v = cutMesh.V[i];
        assert(v.size() == 3);
        cutMesh.vertexCoordsArray.push_back(v[0]);
        cutMesh.vertexCoordsArray.push_back(v[1]);
        cutMesh.vertexCoordsArray.push_back(v[2]);
    }

    // copy faces
    for (int i = 0; i < cutMesh.F.size(); ++i) {
        const std::vector<int>& f = cutMesh.F[i];
        for (int j = 0; j < f.size(); ++j) {
            cutMesh.faceIndicesArray.push_back(f[j]);
        }

        cutMesh.faceSizesArray.push_back(f.size());
    }

    printf("cut mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)cutMesh.V.size(), (int)cutMesh.F.size());

    // 2. create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_DEBUG);
    assert(err == MC_NO_ERROR);

    // 3. do the cutting (boolean ops)
    // -------------------------------
    printf("\nInputs: \n\tShape A = 'cube.obj'.\n\tShape B = 'torus.obj'\n\n");

    // We can either let MCUT compute all possible meshes (including patches etc.), or we can
    // constrain the library to compute exactly the boolean op mesh we want. This 'constrained' case
    // is done with the following flags.
    const std::map<std::string, McFlags> booleanOps = {
        { "A_NOT_B", MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE },
        { "B_NOT_A", MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW },
        { "UNION", MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE },
        { "INTERSECTION", MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW }
    };

    for (std::map<std::string, McFlags>::const_iterator boolOpIter = booleanOps.cbegin(); boolOpIter != booleanOps.cend(); ++boolOpIter) {
        const McFlags boolOpFlags = boolOpIter->second;
        const std::string boolOpName = boolOpIter->first;

        printf("compute %s\n", boolOpName.c_str());

        err = mcDispatch(
            context,
            MC_DISPATCH_VERTEX_ARRAY_DOUBLE | boolOpFlags,
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

        // 4. query the number of available connected component (all types)
        // -------------------------------------------------------------
        uint32_t numConnComps;
        err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps);
        assert(err == MC_NO_ERROR);

        printf("connected components: %d\n", (int)numConnComps);

        if (numConnComps == 0) {
            fprintf(stdout, "no connected components found\n");
            exit(0);
        }

        assert(numConnComps == 1); // exactly 1 result (for this example)

        std::vector<McConnectedComponent> connectedComponents(numConnComps, MC_NULL_HANDLE);
        connectedComponents.resize(numConnComps);
        err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connectedComponents.size(), connectedComponents.data(), NULL);

        assert(err == MC_NO_ERROR);

        // 5. query the data of each connected component from MCUT
        // -------------------------------------------------------

        McConnectedComponent connComp = connectedComponents[0];
        uint64_t numBytes = 0;

        // 5.1 query the number of vertices
        // --------------------------------

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        uint32_t ccVertexCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, numBytes, &ccVertexCount, NULL);
        assert(err == MC_NO_ERROR);

        // 5.2 query the vertices
        // ----------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<double> ccVertices((uint64_t)ccVertexCount * 3u, 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);
        assert(err == MC_NO_ERROR);

        // 5.3 query the faces
        // -------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);
        assert(err == MC_NO_ERROR);

        // 5.4 query the face sizes
        // ------------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);
        std::vector<uint32_t> faceSizes(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL);
        assert(err == MC_NO_ERROR);

        // 5.5 query the face map
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
        for (int i = 0; i < ccVertexCount; ++i) {
            double x = ccVertices[(uint64_t)i * 3 + 0];
            double y = ccVertices[(uint64_t)i * 3 + 1];
            double z = ccVertices[(uint64_t)i * 3 + 2];
            file << "v " << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << x << " " << y << " " << z << std::endl;
        }

        int faceVertexOffsetBase = 0;

        // for each face in CC
        for (int f = 0; f < ccFaceCount; ++f) {
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
        err = mcReleaseConnectedComponents(context, connectedComponents.size(), connectedComponents.data());
        assert(err == MC_NO_ERROR);
    }

    // 7. destroy context
    // ------------------
    err = mcReleaseContext(context);

    assert(err == MC_NO_ERROR);

    return 0;
}
