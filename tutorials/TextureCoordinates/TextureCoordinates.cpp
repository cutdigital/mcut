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
    Eigen::MatrixXd corner_normals;
    Eigen::MatrixXi fNormIndices;

    Eigen::MatrixXd UV_V;
    Eigen::MatrixXi UV_F;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    std::vector<std::tuple<std::string, unsigned, unsigned>> FM;

    std::string fpath;
    std::vector<uint32_t> faceSizesArray; // number of ccVertices in each face in "F"
    std::vector<uint32_t> faceIndicesArray;
    std::vector<double> vertexCoordsArray;
};

int main(int argc, char* argv[])
{
    if (argc == 1) {
        std::fprintf(stderr, "usage: <program-path> <source-mesh-path> <cut-mesh-path>");
        std::exit(1);
    }

    // 1. load meshes.
    // -----------------
    InputMesh srcMesh;

    srcMesh.fpath = DATA_DIR "/bunny.obj";
    bool srcMeshLoaded = igl::readOBJ(srcMesh.fpath, srcMesh.V, srcMesh.UV_V, srcMesh.corner_normals, srcMesh.F, srcMesh.UV_F, srcMesh.fNormIndices);

    if (!srcMeshLoaded) {
        std::fprintf(stderr, "error: could not load source mesh --> %s\n", srcMesh.fpath);
        std::exit(1);
    }

    for (int i = 0; i < srcMesh.V.rows(); ++i) {
        const Eigen::Vector3d& v = srcMesh.V.row(i);
        srcMesh.vertexCoordsArray.push_back(v.x());
        srcMesh.vertexCoordsArray.push_back(v.y());
        srcMesh.vertexCoordsArray.push_back(v.z());
    }

    for (int i = 0; i < srcMesh.F.rows(); ++i) {
        const Eigen::VectorXi& f = srcMesh.F.row(i);
        srcMesh.faceIndicesArray.push_back(f.x());
        srcMesh.faceIndicesArray.push_back(f.y());
        srcMesh.faceIndicesArray.push_back(f.z());
        srcMesh.faceSizesArray.push_back(f.rows());
    }

    printf("source mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)srcMesh.V.rows(), (int)srcMesh.F.rows());

    InputMesh cutMesh;
    cutMesh.fpath = DATA_DIR "/b.obj";
    bool cutMeshLoaded = igl::readOBJ(cutMesh.fpath, cutMesh.V, cutMesh.UV_V, cutMesh.corner_normals, cutMesh.F, cutMesh.UV_F, cutMesh.fNormIndices);

    if (!cutMeshLoaded) {
        std::fprintf(stderr, "error: could not load cut mesh --> %s\n", cutMesh.fpath);
        std::exit(1);
    }

    for (int i = 0; i < cutMesh.V.rows(); ++i) {
        const Eigen::Vector3d& v = cutMesh.V.row(i);
        cutMesh.vertexCoordsArray.push_back(v.x());
        cutMesh.vertexCoordsArray.push_back(v.y());
        cutMesh.vertexCoordsArray.push_back(v.z());
    }

    for (int i = 0; i < cutMesh.F.rows(); ++i) {
        const Eigen::VectorXi& f = cutMesh.F.row(i);
        cutMesh.faceIndicesArray.push_back(f.x());
        cutMesh.faceIndicesArray.push_back(f.y());
        cutMesh.faceIndicesArray.push_back(f.z());
        cutMesh.faceSizesArray.push_back(f.rows());
    }

    printf("cut mesh:\n\tvertices=%d\n\tfaces=%d\n", (int)cutMesh.V.rows(), (int)cutMesh.F.rows());

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

        // 5.5 query the vertex map
        // ------------------------

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, 0, NULL, &numBytes);
        if (err != MC_NO_ERROR) {
            fprintf(stderr, "1:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP) failed (err=%d)\n", (int)err);
            exit(1);
        }

        std::vector<uint32_t> ccVertexMap;
        ccVertexMap.resize(numBytes / sizeof(uint32_t));

        err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP, numBytes, ccVertexMap.data(), NULL);

        if (err != MC_NO_ERROR) {
            fprintf(stderr, "2:mcGetConnectedComponentData(MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP) failed (err=%d)\n", (int)err);
            exit(1);
        }

        // 5.5 query the face map
        // ------------------------
        const uint32_t numberOfFaces = static_cast<uint32_t>(faceSizes.size());

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

        /// ------------------------------------------------------------------------------------

        std::string name;
        if (ccType == MC_CONNECTED_COMPONENT_TYPE_SEAMED) {
            name += "seam";
        } else {
            bool isFragment = (ccType == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT);
            name += isFragment ? "frg" : "ptch";
            if (isFragment) {
                McFragmentLocation floc = (McFragmentLocation)0;
                err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, sizeof(McFragmentLocation), &floc, NULL);
                assert(err == MC_NO_ERROR);
                name += floc == MC_FRAGMENT_LOCATION_ABOVE ? ".abv" : ".blw";

                McPatchLocation ploc = (McPatchLocation)0;
                err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McFragmentLocation), &ploc, NULL);
                assert(err == MC_NO_ERROR);
                name += ploc == MC_PATCH_LOCATION_INSIDE ? ".in" : (ploc == MC_PATCH_LOCATION_OUTSIDE ? ".out" : ".ndef");

                McFragmentSealType sType = (McFragmentSealType)0;
                err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE, sizeof(McFragmentSealType), &sType, NULL);
                assert(err == MC_NO_ERROR);
                name += sType == MC_FRAGMENT_SEAL_TYPE_COMPLETE ? ".cmplt" : (sType == MC_FRAGMENT_SEAL_TYPE_PARTIAL ? ".prtl" : ".none");
            } else { // patch
                McPatchLocation ploc = (McPatchLocation)0;
                err = mcGetConnectedComponentData(context, connComp, MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, sizeof(McPatchLocation), &ploc, NULL);
                assert(err == MC_NO_ERROR);
                name += ploc == MC_PATCH_LOCATION_INSIDE ? ".in" : (ploc == MC_PATCH_LOCATION_OUTSIDE ? ".out" : ".ndef");
            }
        }
        // otherwise is from cut mesh
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

        std::vector<std::vector<Eigen::Vector2d>> ccFaceToInputMeshTexCoords(numberOfFaces, std::vector<Eigen::Vector2d>());
        int faceVertexOffsetBase = 0;

        // for each face in cc
        for (int f = 0; f < numberOfFaces; ++f) {

            // input mesh face index (which may be offsetted)
            const uint32_t imFaceIdxRaw = ccFaceMap.at(f); // source- or cut-mesh

            // input mesh face index (actual index value, accounting for offset)
            uint32_t imFaceIdx = imFaceIdxRaw;
            bool faceIsBirthedFromSrcMesh = (imFaceIdxRaw < srcMesh.F.rows());

            if (!faceIsBirthedFromSrcMesh) {
                imFaceIdx = imFaceIdxRaw - srcMesh.F.rows(); // accounting for offset
            }

            int faceSize = faceSizes.at(f);

            // for each vertex in face
            for (int v = 0; v < faceSize; ++v) {

                const int ccVertexIdx = ccFaceIndices.at(static_cast<size_t>(faceVertexOffsetBase + v));
                // input mesh vertex index (which may be offsetted)
                const uint32_t imVertexIdxRaw = ccVertexMap.at(ccVertexIdx); // vertex index in source mesh or cut mesh
                bool vertexIsBirthedFromSrcMesh = (imVertexIdxRaw < srcMesh.V.rows());
                const bool isIntersectionPoint = (imVertexIdxRaw == MC_UNDEFINED_VALUE);
                // input mesh vertex index (actual index value, accounting for offset)
                uint32_t imVertexIdx = imVertexIdxRaw;

                if (!vertexIsBirthedFromSrcMesh) {
                    imVertexIdx = (imVertexIdxRaw - srcMesh.V.rows()); // account for offset
                }

                const InputMesh* inputMeshPtr = &srcMesh; // assume origin face is from source mesh

                if (!faceIsBirthedFromSrcMesh) {
                    inputMeshPtr = &cutMesh;
                }

                // the face on which the current cc face came from
                const Eigen::Vector3i& imFace = inputMeshPtr->F.row(imFaceIdx);

                if (isIntersectionPoint) { // texture coords unknown and must be computed

                    //if (ccVertexToInputMeshTexCoord[ccVertexIdx] == Eigen::Vector2d(-1, -1)) { // we haven't already computed the texcoords

                    // interpolate texture coords from source-mesh values

                    // 1. get the origin face of the current cc face

                    double x(ccVertices[(ccVertexIdx * 3) + 0]);
                    double y(ccVertices[(ccVertexIdx * 3) + 1]);
                    double z(ccVertices[(ccVertexIdx * 3) + 2]);

                    // vertices of the origin face
                    const Eigen::Vector3d& a = inputMeshPtr->V.row(imFace.x());
                    const Eigen::Vector3d& b = inputMeshPtr->V.row(imFace.y());
                    const Eigen::Vector3d& c = inputMeshPtr->V.row(imFace.z());

                    // barycentric coords of our intersection point on the origin face
                    Eigen::MatrixXd P;
                    P.resize(1, 3);
                    P << x, y, z;
                    Eigen::MatrixXd A;
                    A.resize(1, 3);
                    A << a.x(), a.y(), a.z();
                    Eigen::MatrixXd B;
                    B.resize(1, 3);
                    B << b.x(), b.y(), b.z();
                    Eigen::MatrixXd C;
                    C.resize(1, 3);
                    C << c.x(), c.y(), c.z();
                    Eigen::MatrixXd L;

                    igl::barycentric_coordinates(P, A, B, C, L);

                    // compute the texture coordinates of our intersection point by interpolation
                    // --------------------------------------------------------------------------

                    // indices of the texture coords that are used by "imFaceIdx"
                    const Eigen::VectorXi& imFaceUVIndices = inputMeshPtr->UV_F.row(imFaceIdx);

                    // texture coordinates of vertices of origin face
                    const Eigen::Vector2d& TCa = inputMeshPtr->UV_V.row(imFaceUVIndices(0));
                    const Eigen::Vector2d& TCb = inputMeshPtr->UV_V.row(imFaceUVIndices(1));
                    const Eigen::Vector2d& TCc = inputMeshPtr->UV_V.row(imFaceUVIndices(2));
                    const Eigen::Vector3d& baryCoords = L.row(0);

                    // interpolate using barycentric coords
                    const Eigen::Vector2d& interpolatedTexCoords = (TCa * baryCoords.x()) + (TCb * baryCoords.y()) + (TCc * baryCoords.z());

                    // save
                    ccFaceToInputMeshTexCoords[f].push_back(interpolatedTexCoords);
                    //}
                } else {
                    // set texture coordinates from the values in the input mesh
                    // ---------------------------------------------------------

                    // the current cc is a fragment, and the current face came from the cut mesh
                    //bool isCutMeshFaceOnFragment = !faceIsBirthedFromSrcMesh && ccIsBirthedFromSrcMesh;

                    // Cut-mesh faces on fragments have--by definition--undefined texture coordinates.
                    // This is because a single mesh is associated with one texture, and that texture
                    // will be the source-mesh texture for fragments. The implication here is that cut
                    // mesh faces on fragments are mapped to texture coordinates which referred to
                    // another texture, which is the texture of the cut mesh.
                    //
                    // Here I just set their coords to a constant 1/2 per texcoord
                    Eigen::Vector2d imTexCoords(0.5, 0.5);

                    //if (!isCutMeshFaceOnFragment) {
                    int faceVertexOffset = -1;
                    // for each vertex index in face
                    for (Eigen::Index i = 0; i < imFace.rows(); ++i) {
                        if (imFace(i) == imVertexIdx) {
                            faceVertexOffset = i;
                            break;
                        }
                    }
                    assert(faceVertexOffset != -1);
                    int texCoordsIdx = inputMeshPtr->UV_F.row(imFaceIdx)(faceVertexOffset);
                    imTexCoords = inputMeshPtr->UV_V.row(texCoordsIdx);
                    // }

                    ccFaceToInputMeshTexCoords[f].push_back(imTexCoords);
                }
            } // for (int v = 0; v < faceSize; ++v) {

            faceVertexOffsetBase += faceSize;
        } // for (int f = 0; f < numberOfFaces; ++f) {

        // save cc mesh to .obj file
        // -------------------------

        char fnameBuf[32];
        sprintf(fnameBuf, (name + "_%d.obj").c_str(), i);
        std::string fname(fnameBuf);
        std::string fpath(OUTPUT_DIR "/" + fname);
        printf("write file: %s\n", fpath.c_str());
        std::ofstream file(fpath);

        file << (ccIsBirthedFromSrcMesh ? "mtllib a.mtl" : "mtllib b.mtl") << std::endl;

        // write vertices
        for (int i = 0; i < ccVertexCount; ++i) {
            double x = ccVertices[i * 3 + 0];
            double y = ccVertices[i * 3 + 1];
            double z = ccVertices[i * 3 + 2];
            file << "v " << x << " " << y << " " << z << std::endl;
        }

        // write tex coords (including duplicates i.e. per face texture coords)
        for (int i = 0; i < numberOfFaces; ++i) {
            int faceSize = faceSizes.at(i);
            const std::vector<Eigen::Vector2d>& faceTexCoords = ccFaceToInputMeshTexCoords[i];

            for (int j = 0; j < faceSize; ++j) {
                Eigen::Vector2d uv = faceTexCoords[j];
                file << "vt " << uv.x() << " " << uv.y() << std::endl; // texcoords have same index as positions
            }
        }

        // write faces
        faceVertexOffsetBase = 0;
        for (int i = 0; i < numberOfFaces; ++i) {
            int faceSize = faceSizes.at(i);

            file << "f ";
            for (int j = 0; j < faceSize; ++j) {
                const int idx = faceVertexOffsetBase + j;
                const int ccVertexIdx = ccFaceIndices.at(static_cast<size_t>(idx));
                file << (ccVertexIdx + 1) << "/" << (idx + 1) << " "; // texcoords have [global] index (we duplicate data for simplicity)
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
