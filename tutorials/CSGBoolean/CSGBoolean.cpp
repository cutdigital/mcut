/*
  Tutorial on how to propagate normals from input meshes and onto the output meshes after cutting.

  As we focus on "per-vertex" data, each vertex in the input mesh is assumed to have a normal (akin to 
  smooth shading).
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
    std::vector<std::vector<double>> V // positions
        ,
        TC // texture coordinates (unused)
        ,
        N; // per vertex normals (unused)

    std::vector<std::vector<int>> F // faces which are assumed to be triangles for simplicity
        ,
        FTC // per-face texture coordinate indices (unused)
        ,
        FN; // per-face normal indices (unused)
    std::vector<std::tuple<std::string, unsigned, unsigned>> FM;

    std::string fpath;
    std::vector<uint32_t> faceSizesArray; // number of ccVertices in each face in "F"
    std::vector<uint32_t> faceIndicesArray;
    std::vector<double> vertexCoordsArray;
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

    for (int i = 0; i < srcMesh.V.size(); ++i) {
        const std::vector<double>& v = srcMesh.V[i];
        assert(v.size() == 3);
        srcMesh.vertexCoordsArray.push_back(v[0]);
        srcMesh.vertexCoordsArray.push_back(v[1]);
        srcMesh.vertexCoordsArray.push_back(v[2]);
    }

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

    for (int i = 0; i < cutMesh.V.size(); ++i) {
        const std::vector<double>& v = cutMesh.V[i];
        assert(v.size() == 3);
        cutMesh.vertexCoordsArray.push_back(v[0]);
        cutMesh.vertexCoordsArray.push_back(v[1]);
        cutMesh.vertexCoordsArray.push_back(v[2]);
    }

    for (int i = 0; i < cutMesh.F.size(); ++i) {
        const std::vector<int>& f = cutMesh.F[i];
        assert(f.size() == 3); // tutorial assume's triangle meshes for simplicity
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

    if (err != MC_NO_ERROR) {
        fprintf(stderr, "error: could not create MCUT context (err=%d)\n", (int)err);
        exit(1);
    }

    // 3. do the cutting
    // -----------------
    err = mcDispatch(
        context,
        MC_DISPATCH_VERTEX_ARRAY_DOUBLE | MC_DISPATCH_INCLUDE_VERTEX_MAP | MC_DISPATCH_INCLUDE_FACE_MAP,
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

    if (err != MC_NO_ERROR) {
        fprintf(stderr, "error: dispatch call failed (err=%d)\n", (int)err);
        exit(1);
    }

    // 4. query the number of available connected component (all types)
    // -------------------------------------------------------------
    uint32_t numConnComps;
    std::vector<McConnectedComponent> connComps;

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps);

    if (err != MC_NO_ERROR) {
        fprintf(stderr, "1:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_ALL) failed (err=%d)\n", (int)err);
        exit(1);
    }

    printf("connected components: %d\n", (int)numConnComps);

    if (numConnComps == 0) {
        fprintf(stdout, "no connected components found\n");
        exit(0);
    }

    connComps.resize(numConnComps);

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps.size(), connComps.data(), NULL);

    if (err != MC_NO_ERROR) {
        fprintf(stderr, "2:mcGetConnectedComponents(MC_CONNECTED_COMPONENT_TYPE_ALL) failed (err=%d)\n", (int)err);
        exit(1);
    }

    // 5. query the data of each connected component from MCUT
    // -------------------------------------------------------

    for (int i = 0; i < (int)connComps.size(); ++i) {
        McConnectedComponent connComp = connComps[i]; // connected compoenent id

        uint64_t numBytes = 0;

        // 5.1 query the number of ccVertices
        // --------------------------------

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &numBytes);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT) failed (err=%d)\n", (int)err);
            exit(1);
        }

        uint32_t ccVertexCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, numBytes, &ccVertexCount, NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT) failed (err=%d)\n", (int)err);
            exit(1);
        }

        // 5.2 query the ccVertices
        // ----------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT) failed (err=%d)\n", (int)err);
            exit(1);
        }

        std::vector<double> ccVertices;
        ccVertices.resize(ccVertexCount * 3u);

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT) failed (err=%d)\n", (int)err);
            exit(1);
        }

        // 5.3 query the faces
        // -------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE) failed (err=%d)\n", (int)err);
            exit(1);
        }

        std::vector<uint32_t> ccFaceIndices;
        ccFaceIndices.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE) failed (err=%d)\n", (int)err);
            exit(1);
        }

        // 5.4 query the face sizes
        // ------------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        if (err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_SIZE) failed (err=%d)\n", (int)err);
            exit(1);
        }

        std::vector<uint32_t> faceSizes;
        faceSizes.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_SIZE) failed (err=%d)\n", (int)err);
            exit(1);
        }

        // 5.5 query the face map
        // ------------------------
        const uint32_t ccFaceCount = static_cast<uint32_t>(faceSizes.size());

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_MAP) failed (err=%d)\n", (int)err);
            exit(1);
        }

        std::vector<uint32_t> ccFaceMap;
        ccFaceMap.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, numBytes, ccFaceMap.data(), NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_FACE_MAP) failed (err=%d)\n", (int)err);
            exit(1);
        }

        // get type
        McConnectedComponentType ccType;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &ccType, NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_TYPE) failed (err=%d)\n", (int)err);
            exit(1);
        }

        uint32_t ccEdgeCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_EDGE_COUNT, sizeof(uint32_t), &ccEdgeCount, NULL);

        /// ------------------------------------------------------------------------------------

        // Here we just build the name of each connected component based on its properties
        std::string name;
        McFragmentLocation fragmentLocation = (McFragmentLocation)0;
        McPatchLocation pathLocation = (McPatchLocation)0;
        bool isFragment = false;
        if (ccType == MC_CONNECTED_COMPONENT_TYPE_SEAMED) {
            name += "seam";
        } else {
            isFragment = (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
            name += isFragment ? "frg" : "ptch";

            err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &pathLocation, NULL);
            assert(err == MC_NO_ERROR);
            name += pathLocation == MC_PATCH_LOCATION_INSIDE ? ".ins" : (pathLocation == MC_PATCH_LOCATION_OUTSIDE ? ".out" : ".ndef");

            if (isFragment) {

                err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &fragmentLocation, NULL);
                assert(err == MC_NO_ERROR);
                name += fragmentLocation == MC_FRAGMENT_LOCATION_ABOVE ? ".abv" : ".blw"; // missing loc="undefined" case

                McFragmentSealType sType = (McFragmentSealType)0;
                err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sType, NULL);
                assert(err == MC_NO_ERROR);
                name += sType == MC_FRAGMENT_SEAL_TYPE_COMPLETE ? ".cmplt" : (sType == MC_FRAGMENT_SEAL_TYPE_PARTIAL ? ".prtl" : ".none");
            }
        }

        bool ccIsBirthedFromSrcMesh = (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);

        // its not a fragment && its a seam cc
        if (!ccIsBirthedFromSrcMesh && ccType == MC_CONNECTED_COMPONENT_TYPE_SEAMED) {
            // get origin
            McSeamedConnectedComponentOrigin ccOrig;
            err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamedConnectedComponentOrigin), &ccOrig, NULL);

            if (err != MC_NO_ERROR) {
                fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_ORIGIN) failed (err=%d)\n", (int)err);
                exit(1);
            }

            ccIsBirthedFromSrcMesh = (ccOrig == MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_SRC_MESH);
            name += ccIsBirthedFromSrcMesh ? ".sm" : ".cm";
        }

        // save cc mesh to .obj file
        // -------------------------

        char fnameBuf[32];
        sprintf(fnameBuf, ("OUT_" + name + ".obj").c_str(), i);
        std::string fname(fnameBuf);
        std::string fpath(DATA_DIR "/" + fname);
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

            // input mesh face index (which may be offsetted!)
            const uint32_t imFaceIdxRaw = ccFaceMap.at(f); // source- or cut-mesh
            // input mesh face index (actual index value, accounting for offset)
            uint32_t imFaceIdx = imFaceIdxRaw;
            bool faceIsFromSrcMesh = (imFaceIdxRaw < srcMesh.F.size());

            if (!faceIsFromSrcMesh) {
                imFaceIdx = imFaceIdxRaw - srcMesh.F.size(); // accounting for offset
            }

            assert(err == MC_NO_ERROR);

            // https://en.wikipedia.org/wiki/Euler_characteristic
            //bool isWaterTight = (ccVertexCount + ccFaceCount - ccEdgeCount) == 2;
            bool reverseWindingOrder = (isFragment /*&& isWaterTight*/ && fragmentLocation == MC_FRAGMENT_LOCATION_BELOW && pathLocation == MC_PATCH_LOCATION_OUTSIDE);

            int faceSize = faceSizes.at(f);
            file << "f ";
            // for each vertex in face
            for (int v = (reverseWindingOrder ? (faceSize-1) : 0); 
                (reverseWindingOrder? (v >= 0) : (v < faceSize)); 
                v += (reverseWindingOrder ? -1 : 1)) {
                const int ccVertexIdx = ccFaceIndices[(uint64_t)faceVertexOffsetBase + v];
                file << (ccVertexIdx + 1) << " ";
            } // for (int v = 0; v < faceSize; ++v) {
            file << std::endl;

            faceVertexOffsetBase += faceSize;
        } // for (int f = 0; f < ccFaceCount; ++f) {
    }

    // 6. free connected component data
    // --------------------------------
    err = mcReleaseConnectedComponents(context, 0, NULL);

    if (err != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseConnectedComponents failed (err=%d)\n", (int)err);
        exit(1);
    }

    // 7. destroy context
    // ------------------
    err = mcReleaseContext(context);

    if (err != MC_NO_ERROR) {
        fprintf(stderr, "mcReleaseContext failed (err=%d)\n", (int)err);
        exit(1);
    }

    return 0;
}
