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

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS 1

#ifdef _WIN32
#pragma warning(disable : 26812) // Unscoped enums from mcut.h
#endif // _WIN32
#endif

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
        std::exit(1);                               \
    }

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
#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>

struct InputMesh {
    // variables for reading .obj file data with libigl
    std::vector<std::vector<double>> V;
    std::vector<std::vector<int>> F;

    // variables for mesh data in a format suited for MCUT
    std::string fpath; // path to mesh file
    std::vector<uint32_t> faceSizesArray; // vertices per face
    std::vector<uint32_t> faceIndicesArray; // face indices
    std::vector<double> vertexCoordsArray; // vertex coords
};

int main(int argc, const char* argv[])
{
    // load meshes.
    // -----------------
    InputMesh srcMesh;

    const bool user_provided_meshes = argc > 1;
    if (user_provided_meshes && argc < 3) {
        fprintf(stderr, "usage: <exec> <srcmesh/path> <cutmesh/path> <boolOp>\n"
                        "The possible values for the <boolOp> arguments are:\n"
                        "\t-u (for union)\n"
                        "\t-i (for intersection)\n"
                        "\t-ds (for difference i.e. source-mesh NOT cut-mesh)\n"
                        "\t-dc (for difference i.e. cut-mesh NOT source-mesh)\n"
                        "\tleave empty for all ops\n");
        return 1;
    } else if (user_provided_meshes) {
        printf("NOTE: using user provided meshes meshes\n");
    } else {
        printf("NOTE: using default meshes\n");
    }

    srcMesh.fpath = user_provided_meshes ? argv[1] : DATA_DIR "/cube.obj";
    bool srcMeshLoaded = igl::read_triangle_mesh(srcMesh.fpath, srcMesh.V, srcMesh.F);

    if (!srcMeshLoaded) {
        std::fprintf(stderr, "error: could not load source mesh --> %s\n", srcMesh.fpath.c_str());
        std::exit(1);
    }

    // copy vertices
    for (int i = 0; i < (int)srcMesh.V.size(); ++i) {
        const std::vector<double>& v = srcMesh.V[i];
        my_assert(v.size() == 3);
        srcMesh.vertexCoordsArray.push_back(v[0]);
        srcMesh.vertexCoordsArray.push_back(v[1]);
        srcMesh.vertexCoordsArray.push_back(v[2]);
    }

    // copy faces
    for (int i = 0; i < (int)srcMesh.F.size(); ++i) {
        const std::vector<int>& f = srcMesh.F[i];
        for (int j = 0; j < (int)f.size(); ++j) {
            srcMesh.faceIndicesArray.push_back(f[j]);
        }

        srcMesh.faceSizesArray.push_back((uint32_t)f.size());
    }

    printf("source mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)srcMesh.V.size(), (int)srcMesh.F.size());

    InputMesh cutMesh;
    cutMesh.fpath = user_provided_meshes ? argv[2] : DATA_DIR "/torus.obj";
    bool cutMeshLoaded = igl::read_triangle_mesh(cutMesh.fpath, cutMesh.V, cutMesh.F);

    if (!cutMeshLoaded) {
        std::fprintf(stderr, "error: could not load source mesh --> %s\n", cutMesh.fpath.c_str());
        std::exit(1);
    }

    // copy vertices
    for (int i = 0; i < (int)cutMesh.V.size(); ++i) {
        const std::vector<double>& v = cutMesh.V[i];
        my_assert(v.size() == 3);
        cutMesh.vertexCoordsArray.push_back(v[0]);
        cutMesh.vertexCoordsArray.push_back(v[1]);
        cutMesh.vertexCoordsArray.push_back(v[2]);
    }

    // copy faces
    for (int i = 0; i < (int)cutMesh.F.size(); ++i) {
        const std::vector<int>& f = cutMesh.F[i];
        for (int j = 0; j < (int)f.size(); ++j) {
            cutMesh.faceIndicesArray.push_back(f[j]);
        }

        cutMesh.faceSizesArray.push_back((uint32_t)f.size());
    }

    printf("cut mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)cutMesh.V.size(), (int)cutMesh.F.size());

    std::string boolOpStr = "*";

    if (argc == 4) {
        if (strcmp(argv[3], "-u") == 0) {
            boolOpStr = "UNION";
        } else if (strcmp(argv[3], "-i") == 0) {
            boolOpStr = "INTERSECTION";
        } else if (strcmp(argv[3], "-ds") == 0) {
            boolOpStr = "A_NOT_B";
        } else if (strcmp(argv[3], "-dc") == 0) {
            boolOpStr = "B_NOT_A";
        } else {
            fprintf(stderr, "invalid boolOp argument value\n");
            return 1;
        }
    }
    else{
        printf("NOTE: computing all boolean ops.\n");
    }
    // create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_DEBUG);
    my_assert(err == MC_NO_ERROR);

    //  do the cutting (boolean ops)
    // -----------------------------
    printf("\nInputs: \n\tShape A = %s'.\n\tShape B = '%s'\n\n", srcMesh.fpath.c_str(), cutMesh.fpath.c_str());

    // We can either let MCUT compute all possible meshes (including patches etc.), or we can
    // constrain the library to compute exactly the boolean op mesh we want. This 'constrained' case
    // is done with the following flags.
    // NOTE: you can extend these flags by bitwise ORing with additional flags (see `McDispatchFlags' in mcut.h)
    const std::map<std::string, McFlags> booleanOps = {
        { "A_NOT_B", MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE },
        { "B_NOT_A", MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW },
        { "UNION", MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE },
        { "INTERSECTION", MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW }
    };

    for (std::map<std::string, McFlags>::const_iterator boolOpIter = booleanOps.cbegin(); boolOpIter != booleanOps.cend(); ++boolOpIter) {
        if (boolOpIter->first != boolOpStr && boolOpStr != "*") {
            continue;
        }

        const McFlags boolOpFlags = boolOpIter->second;
        const std::string boolOpName = boolOpIter->first;

        printf("compute %s\n", boolOpName.c_str());

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

        my_assert(err == MC_NO_ERROR);

        // query the number of available connected component
        // --------------------------------------------------
        uint32_t numConnComps;
        err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, 0, NULL, &numConnComps);
        my_assert(err == MC_NO_ERROR);

        printf("connected components: %d\n", (int)numConnComps);

        if (numConnComps == 0) {
            fprintf(stdout, "no connected components found\n");
            exit(0);
        }

        // my_assert(numConnComps == 1); // exactly 1 result (for this example)

        std::vector<McConnectedComponent> connectedComponents(numConnComps, MC_NULL_HANDLE);
        connectedComponents.resize(numConnComps);
        err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, (uint32_t)connectedComponents.size(), connectedComponents.data(), NULL);

        my_assert(err == MC_NO_ERROR);

        // query the data of each connected component from MCUT
        // -------------------------------------------------------

        McConnectedComponent connComp = connectedComponents[0];

        // query the vertices
        // ----------------------

        McSize numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);
        my_assert(err == MC_NO_ERROR);
        uint32_t ccVertexCount = (uint32_t)(numBytes / (sizeof(double) * 3));
        std::vector<double> ccVertices((McSize)ccVertexCount * 3u, 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);
        my_assert(err == MC_NO_ERROR);

        // query the faces
        // -------------------
        numBytes = 0;

#if 1 // triangulated faces

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, 0, NULL, &numBytes);
        my_assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, numBytes, ccFaceIndices.data(), NULL);
        my_assert(err == MC_NO_ERROR);

        std::vector<uint32_t> faceSizes(ccFaceIndices.size() / 3, 3);

#else // non-triangulated faces (i.e. N-gons)

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
        my_assert(err == MC_NO_ERROR);
        std::vector<uint32_t> ccFaceIndices(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);
        my_assert(err == MC_NO_ERROR);

        // query the face sizes
        // ------------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        my_assert(err == MC_NO_ERROR);
        std::vector<uint32_t> faceSizes(numBytes / sizeof(uint32_t), 0);
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL);
        my_assert(err == MC_NO_ERROR);
#endif

        const uint32_t ccFaceCount = static_cast<uint32_t>(faceSizes.size());

        /// ------------------------------------------------------------------------------------

        // Here we show, how to know when connected components, pertain particular boolean operations.

        McPatchLocation patchLocation = (McPatchLocation)0;

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &patchLocation, NULL);
        my_assert(err == MC_NO_ERROR);

        McFragmentLocation fragmentLocation = (McFragmentLocation)0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &fragmentLocation, NULL);
        my_assert(err == MC_NO_ERROR);

        // save cc mesh to .obj file
        // -------------------------

        auto extract_fname = [](const std::string& full_path) {
            // get filename
            std::string base_filename = full_path.substr(full_path.find_last_of("/\\") + 1);
            // remove extension from filename
            std::string::size_type const p(base_filename.find_last_of('.'));
            std::string file_without_extension = base_filename.substr(0, p);
            return file_without_extension;
        };

        std::string fpath(OUTPUT_DIR "/" + extract_fname(srcMesh.fpath) + "_" + extract_fname(cutMesh.fpath) + "_" + boolOpName + ".obj");

        printf("write file: %s\n", fpath.c_str());

        std::ofstream file(fpath);

        // write vertices and normals
        for (uint32_t i = 0; i < ccVertexCount; ++i) {
            double x = ccVertices[(McSize)i * 3 + 0];
            double y = ccVertices[(McSize)i * 3 + 1];
            double z = ccVertices[(McSize)i * 3 + 2];
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
                const int ccVertexIdx = ccFaceIndices[(McSize)faceVertexOffsetBase + v];
                file << (ccVertexIdx + 1) << " ";
            } // for (int v = 0; v < faceSize; ++v) {
            file << std::endl;

            faceVertexOffsetBase += faceSize;
        }

        // 6. free connected component data
        // --------------------------------
        err = mcReleaseConnectedComponents(context, (uint32_t)connectedComponents.size(), connectedComponents.data());
        my_assert(err == MC_NO_ERROR);
    }

    // 7. destroy context
    // ------------------
    err = mcReleaseContext(context);

    my_assert(err == MC_NO_ERROR);

    return 0;
}
