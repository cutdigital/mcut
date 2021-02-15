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
        N; // per vertex normals

    std::vector<std::vector<int>> F // faces which are assumed to be triangles for simplicity
        ,
        FTC // per-face texture coordinate indices (unused)
        ,
        FN; // per-face normal indices
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
    cutMesh.fpath = DATA_DIR "/plane.obj";
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

    printf("source mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)cutMesh.V.size(), (int)cutMesh.F.size());

    // 2. create a context
    // -------------------
    McContext context = MC_NULL_HANDLE;
    McResult err = mcCreateContext(&context, MC_DEBUG);

    assert(err == MC_NO_ERROR);

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

    assert(err == MC_NO_ERROR);

    // 4. query the number of available connected component (all types)
    // -------------------------------------------------------------
    uint32_t numConnComps;
    std::vector<McConnectedComponent> connComps;

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps);

    assert(err == MC_NO_ERROR);

    printf("connected components: %d\n", (int)numConnComps);

    assert(err == MC_NO_ERROR);

    connComps.resize(numConnComps);

    err = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (uint32_t)connComps.size(), connComps.data(), NULL);

    assert(err == MC_NO_ERROR);

    // 5. query the data of each connected component from MCUT
    // -------------------------------------------------------

    for (int i = 0; i < (int)connComps.size(); ++i) {
        McConnectedComponent connComp = connComps[i]; // connected compoenent id

        uint64_t numBytes = 0;

        // 5.1 query the number of ccVertices
        // --------------------------------

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, 0, NULL, &numBytes);

        assert(err == MC_NO_ERROR);

        uint32_t ccVertexCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT, numBytes, &ccVertexCount, NULL);

        assert(err == MC_NO_ERROR);

        // 5.2 query the ccVertices
        // ----------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes);

        assert(err == MC_NO_ERROR);

        std::vector<double> ccVertices;
        ccVertices.resize(ccVertexCount * 3u);

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)ccVertices.data(), NULL);

        assert(err == MC_NO_ERROR);

        // 5.3 query the faces
        // -------------------

        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);

        assert(err == MC_NO_ERROR);

        std::vector<uint32_t> ccFaceIndices;
        ccFaceIndices.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, ccFaceIndices.data(), NULL);

        assert(err == MC_NO_ERROR);

        // 5.4 query the face sizes
        // ------------------------
        numBytes = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);

        std::vector<uint32_t> faceSizes;
        faceSizes.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL);

        assert(err == MC_NO_ERROR);

        // 5.5 query the vertex map
        // ------------------------

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes);
        assert(err == MC_NO_ERROR);

        std::vector<uint32_t> ccVertexMap;
        ccVertexMap.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, numBytes, ccVertexMap.data(), NULL);

        assert(err == MC_NO_ERROR);

        // 5.5 query the face map
        // ------------------------
        const uint32_t ccFaceCount = static_cast<uint32_t>(faceSizes.size());

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, 0, NULL, &numBytes);

        assert(err == MC_NO_ERROR);

        std::vector<uint32_t> ccFaceMap;
        ccFaceMap.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FACE_MAP, numBytes, ccFaceMap.data(), NULL);

        assert(err == MC_NO_ERROR);

        // get type
        McConnectedComponentType ccType;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_TYPE, sizeof(McConnectedComponentType), &ccType, NULL);

        assert(err == MC_NO_ERROR);

        uint32_t ccEdgeCount = 0;
        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_EDGE_COUNT, sizeof(uint32_t), &ccEdgeCount, NULL);

        /// ------------------------------------------------------------------------------------

        // Here we just build the name of each connected component based on its properties
        std::string name;
        McFragmentLocation fragmentLocation = (McFragmentLocation)0;
        McPatchLocation pathLocation = (McPatchLocation)0;
        bool isFragment = false;
        if (ccType == MC_CONNECTED_COMPONENT_TYPE_SEAM) {
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
        if (!ccIsBirthedFromSrcMesh && ccType == MC_CONNECTED_COMPONENT_TYPE_SEAM) {
            // get origin
            McSeamedConnectedComponentOrigin ccOrig;
            err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_ORIGIN, sizeof(McSeamedConnectedComponentOrigin), &ccOrig, NULL);

            assert(err == MC_NO_ERROR);

            ccIsBirthedFromSrcMesh = (ccOrig == MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_SRC_MESH);
            name += ccIsBirthedFromSrcMesh ? ".sm" : ".cm";
        }

        int faceVertexOffsetBase = 0;

        std::vector<Eigen::Vector3d> ccVertexNormals(ccVertexCount, Eigen::Vector3d(0., 0., 0.));
        // intersection points do not have a normal value that can be copied (inferred) from an input
        // mesh, it has to be computed by interpolating normals on the origin face in the input mesh.
        // We keep a reference count to compute averaged normal per intersection point.
        std::map<int, int> ccSeamVertexToRefCount;
        std::vector<uint32_t> ccFaceVertexNormalIndices;
        std::vector<int> ccReversedFaces;

        // for each face in CC
        for (int f = 0; f < ccFaceCount; ++f) {

            // input mesh face index (which may be offsetted!)
            const uint32_t imFaceIdxRaw = ccFaceMap.at(f); // source- or cut-mesh
            // input mesh face index (actual index value, accounting for offset)
            uint32_t imFaceIdx = imFaceIdxRaw;
            bool faceIsFromSrcMesh = (imFaceIdxRaw < srcMesh.F.size());

            bool flipNormalsOnFace = false;

            if (!faceIsFromSrcMesh) {
                imFaceIdx = imFaceIdxRaw - srcMesh.F.size(); // accounting for offset

                // check if we need to flip normals
                // --------------------------------
                flipNormalsOnFace = (isFragment && fragmentLocation == MC_FRAGMENT_LOCATION_ABOVE);
            }

            assert(err == MC_NO_ERROR);

            // https://en.wikipedia.org/wiki/Euler_characteristic
            bool isWaterTight = (ccVertexCount + ccFaceCount - ccEdgeCount) == 2;
            bool reverseWindingOrder = (isFragment && isWaterTight && fragmentLocation == MC_FRAGMENT_LOCATION_BELOW && pathLocation == MC_PATCH_LOCATION_OUTSIDE);

            // Need to explicitely flip all normals in case of boolean-op B\A, where B=cut-mesh and A=source-mesh.
            // This is side-effect of the way in which polygons are stitched/holes filled by MCUT.
            flipNormalsOnFace = flipNormalsOnFace || reverseWindingOrder;

            int faceSize = faceSizes.at(f);

            // for each vertex in face
            for (int v = 0; v < faceSize; ++v) {

                const int ccVertexIdx = ccFaceIndices[(uint64_t)faceVertexOffsetBase + v];
                // input mesh (source mesh or cut mesh) vertex index (which may be offsetted)
                const uint32_t imVertexIdxRaw = ccVertexMap.at(ccVertexIdx);
                bool vertexIsFromSrcMesh = (imVertexIdxRaw < srcMesh.V.size());
                const bool isSeamVertex = (imVertexIdxRaw == MC_UNDEFINED_VALUE);
                uint32_t imVertexIdx = imVertexIdxRaw; // actual index value, accounting for offset

                if (!vertexIsFromSrcMesh) {
                    imVertexIdx = (imVertexIdxRaw - srcMesh.V.size()); // account for offset
                }

                const InputMesh* inputMeshPtr = &srcMesh; // assume origin face is from source mesh

                if (!faceIsFromSrcMesh) {
                    inputMeshPtr = &cutMesh;
                }

                // the face on which the current cc face came from
                const std::vector<int>& imFace = inputMeshPtr->F[imFaceIdx];

                if (isSeamVertex) { // normal is unknown and must be computed

                    // interpolate texture coords from source-mesh values

                    // 1. get the origin face of the current cc face

                    double x(ccVertices[(ccVertexIdx * 3) + 0]);
                    double y(ccVertices[(ccVertexIdx * 3) + 1]);
                    double z(ccVertices[(ccVertexIdx * 3) + 2]);

                    // vertices of the origin face
                    const std::vector<double>& a = inputMeshPtr->V[imFace[0]];
                    const std::vector<double>& b = inputMeshPtr->V[imFace[1]];
                    const std::vector<double>& c = inputMeshPtr->V[imFace[2]];

                    // barycentric coords of our intersection point on the origin face
                    Eigen::MatrixXd P;
                    P.resize(1, 3);
                    P << x, y, z;
                    Eigen::MatrixXd A;
                    A.resize(1, 3);
                    A << a[0], a[1], a[2];
                    Eigen::MatrixXd B;
                    B.resize(1, 3);
                    B << b[0], b[1], b[2];
                    Eigen::MatrixXd C;
                    C.resize(1, 3);
                    C << c[0], c[1], c[2];
                    Eigen::MatrixXd L;

                    igl::barycentric_coordinates(P, A, B, C, L);

                    // compute the texture coordinates of our intersection point by interpolation
                    // --------------------------------------------------------------------------

                    // indices of the normals that are used by "imFaceIdx"
                    const std::vector<int>& imFaceNormalIndices = inputMeshPtr->FN[imFaceIdx];
                    assert(imFaceNormalIndices.size() == 3);

                    // normals of vertices in origin face
                    const std::vector<double>& Na_ = inputMeshPtr->N[imFaceNormalIndices[0]];
                    const std::vector<double>& Nb_ = inputMeshPtr->N[imFaceNormalIndices[1]];
                    const std::vector<double>& Nc_ = inputMeshPtr->N[imFaceNormalIndices[2]];

                    const Eigen::Vector3d Na(Na_[0], Na_[1], Na_[2]);
                    const Eigen::Vector3d Nb(Nb_[0], Nb_[1], Nb_[2]);
                    const Eigen::Vector3d Nc(Nc_[0], Nc_[1], Nc_[2]);
                    const Eigen::Vector3d baryCoords = L.row(0);

                    // interpolate using barycentric coords
                    Eigen::Vector3d normal = (Na * baryCoords.x()) + (Nb * baryCoords.y()) + (Nc * baryCoords.z()) * (flipNormalsOnFace ? -1.0 : 1.0);

                    ccVertexNormals[ccVertexIdx] += normal;

                    if (ccSeamVertexToRefCount.find(ccVertexIdx) == ccSeamVertexToRefCount.cend()) {
                        ccSeamVertexToRefCount[ccVertexIdx] = 1;
                    } else {
                        ccSeamVertexToRefCount[ccVertexIdx] += 1;
                    }

                } else { // normal must be inferred from input mesh

                    if (ccVertexNormals[ccVertexIdx].norm() == 0) {
                        int faceVertexOffset = -1;
                        // for each vertex index in face
                        for (Eigen::Index i = 0; i < imFace.size(); ++i) {
                            if (imFace[i] == imVertexIdx) {
                                faceVertexOffset = i;
                                break;
                            }
                        }

                        assert(faceVertexOffset != -1);

                        int imNormalIdx = inputMeshPtr->FN[imFaceIdx][faceVertexOffset];
                        const std::vector<double>& n = inputMeshPtr->N[imNormalIdx];
                        assert(n.size() == 3);
                        Eigen::Vector3d normal = Eigen::Vector3d(n[0], n[1], n[2]) * (flipNormalsOnFace ? -1.0 : 1.0);

                        ccVertexNormals[ccVertexIdx] = normal;
                    }
                }
            } // for (int v = 0; v < faceSize; ++v) {

            faceVertexOffsetBase += faceSize;
        } // for (int f = 0; f < ccFaceCount; ++f) {

        for (std::map<int, int>::const_iterator it = ccSeamVertexToRefCount.cbegin(); it != ccSeamVertexToRefCount.cend(); ++it) {
            const int ccSeamVertexIndex = it->first;
            const int refCount = it->second;
            assert(refCount >= 1);
            ccVertexNormals[ccSeamVertexIndex] /= (double)refCount; // average
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

            Eigen::Vector3d n = ccVertexNormals[i];
            file << "vn " << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << n.x() << " " << n.y() << " " << n.z() << std::endl;
        }

        // write faces (with normal indices)

        faceVertexOffsetBase = 0;
        for (int i = 0; i < ccFaceCount; ++i) {
            int faceSize = faceSizes.at(i);

            file << "f ";
            for (int j = 0; j < faceSize; ++j) {
                const int idx = faceVertexOffsetBase + j;
                const int ccVertexIdx = ccFaceIndices[idx];
                file << (ccVertexIdx + 1) << "//" << (ccVertexIdx + 1) << " ";
            }
            file << std::endl;

            faceVertexOffsetBase += faceSize;
        }
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
