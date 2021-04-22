/**
 * Copyright (c) 2020-2021 CutDigital Ltd.
 * All rights reserved.
 * 
 * NOTE: This file is licensed under GPL-3.0-or-later (default). 
 * A commercial license can be purchased from CutDigital Ltd. 
 *  
 * License details:
 * 
 * (A)  GNU General Public License ("GPL"); a copy of which you should have 
 *      recieved with this file.
 * 	    - see also: <http://www.gnu.org/licenses/>
 * (B)  Commercial license.
 *      - email: contact@cut-digital.com
 * 
 * The commercial license options is for users that wish to use MCUT in 
 * their products for comercial purposes but do not wish to release their 
 * software products under the GPL license. 
 * 
 * Author(s)     : Floyd M. Chitalu
 */

#include "mcut/mcut.h"

#include "mcut/internal/geom.h"
#include "mcut/internal/kernel.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
#include <cfenv>
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

#include <algorithm>
#include <fstream>
#include <memory>
#include <random> // perturbation
#include <stdio.h>
#include <string.h>

#if defined(MCUT_BUILD_WINDOWS)
#pragma warning(disable : 26812)
#endif

#include "mcut/internal/bvh.h"
#include "mcut/internal/geom.h"

// If the inputs are found to not be in general position, then we perturb the
// cut-mesh by this constant (scaled by bbox diag times a random variable [0.1-1.0]).
const mcut::math::real_number_t GENERAL_POSITION_ENFORCMENT_CONSTANT = 1e-6;

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
McRoundingModeFlags convertRoundingMode(int rm)
{
    McRoundingModeFlags rmf = (McRoundingModeFlags)MC_ROUNDING_MODE_TO_NEAREST;
    switch (rm) {
    case FE_TONEAREST:
        rmf = MC_ROUNDING_MODE_TO_NEAREST;
        break;
    case FE_TOWARDZERO:
        rmf = MC_ROUNDING_MODE_TOWARD_ZERO;
        break;
    case FE_UPWARD:
        rmf = MC_ROUNDING_MODE_TOWARD_POS_INF;
        break;
    case FE_DOWNWARD:
        rmf = MC_ROUNDING_MODE_TOWARD_NEG_INF;
        break;
    default:
#if defined(MCUT_DEBUG_BUILD)
        fprintf(stderr, "[MCUT]: conversion error (McRoundingModeFlags)\n");
#endif
        break;
    }
    return rmf;
}

int convertRoundingMode(McRoundingModeFlags rm)
{
    int f = FE_TONEAREST;
    switch (rm) {
    case MC_ROUNDING_MODE_TO_NEAREST:
        f = FE_TONEAREST;
        break;
    case MC_ROUNDING_MODE_TOWARD_ZERO:
        f = FE_TOWARDZERO;
        break;
    case MC_ROUNDING_MODE_TOWARD_POS_INF:
        f = FE_UPWARD;
        break;
    case MC_ROUNDING_MODE_TOWARD_NEG_INF:
        f = FE_DOWNWARD;
        break;
    default:
#if defined(MCUT_DEBUG_BUILD)
        fprintf(stderr, "[MCUT]: conversion error (McRoundingModeFlags)\n");
#endif
        break;
    }
    return f;
}
#else
McRoundingModeFlags convertRoundingMode(mp_rnd_t rm)
{
    McRoundingModeFlags rmf = MC_ROUNDING_MODE_TO_NEAREST;
    switch (rm) {
    case MPFR_RNDN:
        rmf = MC_ROUNDING_MODE_TO_NEAREST;
        break;
    case MPFR_RNDZ:
        rmf = MC_ROUNDING_MODE_TOWARD_ZERO;
        break;
    case MPFR_RNDU:
        rmf = MC_ROUNDING_MODE_TOWARD_POS_INF;
        break;
    case MPFR_RNDD:
        rmf = MC_ROUNDING_MODE_TOWARD_NEG_INF;
        break;
    default:
#if defined(MCUT_DEBUG_BUILD)
        fprintf(stderr, "[MCUT]: conversion error (McRoundingModeFlags)\n");
#endif
    }
    return rmf;
}

mp_rnd_t convertRoundingMode(McRoundingModeFlags rm)
{
    mp_rnd_t f = MPFR_RNDN;
    switch (rm) {
    case MC_ROUNDING_MODE_TO_NEAREST:
        f = MPFR_RNDN;
        break;
    case MC_ROUNDING_MODE_TOWARD_ZERO:
        f = MPFR_RNDZ;
        break;
    case MC_ROUNDING_MODE_TOWARD_POS_INF:
        f = MPFR_RNDU;
        break;
    case MC_ROUNDING_MODE_TOWARD_NEG_INF:
        f = MPFR_RNDD;
        break;
    default:
#if defined(MCUT_DEBUG_BUILD)
        fprintf(stderr, "[MCUT]: conversion error (McRoundingModeFlags)\n");
#endif
        break;
    }
    return f;
}
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

struct IndexArrayMesh {
    IndexArrayMesh() { }
    ~IndexArrayMesh()
    {
        pVertices.release();
        pFaceIndices.release();
        pFaceSizes.release();
        pEdges.release();
    }

    std::unique_ptr<mcut::math::real_number_t[]> pVertices;
    std::unique_ptr<uint32_t[]> pSeamVertexIndices;
    std::unique_ptr<uint32_t[]> pVertexMapIndices; // descriptor/index in original mesh (source/cut-mesh), each vertex has an entry
    std::unique_ptr<uint32_t[]> pFaceIndices;
    std::unique_ptr<uint32_t[]> pFaceMapIndices; // descriptor/index in original mesh (source/cut-mesh), each face has an entry
    std::unique_ptr<uint32_t[]> pFaceSizes;
    std::unique_ptr<uint32_t[]> pEdges;

    uint32_t numVertices = 0;
    uint32_t numSeamVertexIndices = 0;
    uint32_t numFaces = 0;
    uint32_t numFaceIndices = 0;
    uint32_t numEdgeIndices = 0;
};

struct McConnCompBase {
    virtual ~McConnCompBase() {};
    McConnectedComponentType type = (McConnectedComponentType)0;
    IndexArrayMesh indexArrayMesh;
};

struct McFragmentConnComp : public McConnCompBase {
    McFragmentLocation fragmentLocation = (McFragmentLocation)0;
    McFragmentSealType srcMeshSealType = (McFragmentSealType)0;
    McPatchLocation patchLocation = (McPatchLocation)0;
};

struct McPatchConnComp : public McConnCompBase {
    McPatchLocation patchLocation = (McPatchLocation)0;
    ;
};

struct McSeamConnComp : public McConnCompBase {
    McSeamOrigin origin = (McSeamOrigin)0;
};

template <typename Derived>
void ccDeletorFunc(McConnCompBase* p)
{
    delete static_cast<Derived*>(p);
}

struct McDispatchContextInternal {
    std::map<McConnectedComponent, std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)>> connComps = {};

    // state & dispatch flags
    // -----
    McFlags flags = (McFlags)0;
    McFlags dispatchFlags = (McFlags)0;
    std::map<mcut::fd_t, mcut::fd_t> fpPartitionChildFaceToInputSrcMeshFace;
    std::map<mcut::fd_t, mcut::fd_t> fpPartitionChildFaceToInputCutMeshFace;

    // debugging
    // ---------
    pfn_mcDebugOutput_CALLBACK debugCallback = nullptr;
    const void* debugCallbackUserParam = nullptr;
    McFlags debugSource = 0;
    McFlags debugType = 0;
    McFlags debugSeverity = 0;
    std::string lastLoggedDebugDetail = "";

    void log(McDebugSource source,
        McDebugType type,
        unsigned int id,
        McDebugSeverity severity,
        const std::string& message)
    {
        if (debugCallback != nullptr) {
            (*debugCallback)(source, type, id, severity, message.length(), message.c_str(), debugCallbackUserParam);
        }
    }

    // numerical configs
    // -----------------

    // defaults
    static McRoundingModeFlags defaultRoundingMode;
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    static uint64_t defaultPrecision;
    static const uint64_t minPrecision;
    static const uint64_t maxPrecision;
    uint64_t precision = defaultPrecision;
#else
    static mpfr_prec_t defaultPrecision;
    static const mpfr_prec_t minPrecision;
    static const mpfr_prec_t maxPrecision;
    mpfr_prec_t precision = defaultPrecision;
#endif

    // user values

    McRoundingModeFlags roundingMode = defaultRoundingMode;

    void applyPrecisionAndRoundingModeSettings()
    {
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        if (roundingMode != defaultRoundingMode) {
            std::fesetround(convertRoundingMode(roundingMode));
        }

        if (precision != defaultPrecision) {
            log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, "redundant precision change");
            // no-op ("mcut::math::real_number_t" is just "long double" so we cannot change precision - its fixed)
        }
#else
        // MPFR uses global state which could be potentially polluted other libraries/apps using MCUT
        //if (roundingMode != defaultRoundingMode) {
        mcut::math::arbitrary_precision_number_t::set_default_rounding_mode(convertRoundingMode(roundingMode));
        //}

        //if (precision != defaultPrecision) {
        mcut::math::arbitrary_precision_number_t::set_default_precision(precision);
        //}
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    }

    void revertPrecisionAndRoundingModeSettings()
    {
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        if (roundingMode != defaultRoundingMode) {
            std::fesetround(convertRoundingMode(defaultRoundingMode));
        }

        if (precision != defaultPrecision) {
            log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, "redundant precision change");
            // no-op ("mcut::math::real_number_t" is just "long double" so we cannot change precision - its fixed)
        }
#else
        // if (roundingMode != defaultRoundingMode) {
        mcut::math::arbitrary_precision_number_t::set_default_rounding_mode(convertRoundingMode(defaultRoundingMode));
        //}

        // if (precision != defaultPrecision) {
        mcut::math::arbitrary_precision_number_t::set_default_precision(defaultPrecision);
        // }
#endif
    }
};

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
McRoundingModeFlags McDispatchContextInternal::defaultRoundingMode = convertRoundingMode(std::fegetround());
uint64_t McDispatchContextInternal::defaultPrecision = sizeof(mcut::math::real_number_t) * 8;
const uint64_t McDispatchContextInternal::minPrecision = McDispatchContextInternal::defaultPrecision;
const uint64_t McDispatchContextInternal::maxPrecision = McDispatchContextInternal::defaultPrecision;
#else
McRoundingModeFlags McDispatchContextInternal::defaultRoundingMode = convertRoundingMode(mcut::math::arbitrary_precision_number_t::get_default_rounding_mode());
mpfr_prec_t McDispatchContextInternal::defaultPrecision = mcut::math::arbitrary_precision_number_t::get_default_precision();
const mpfr_prec_t McDispatchContextInternal::minPrecision = MPFR_PREC_MIN;
const mpfr_prec_t McDispatchContextInternal::maxPrecision = MPFR_PREC_MAX;
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

std::map<McContext, std::unique_ptr<McDispatchContextInternal>> gDispatchContexts;

McResult indexArrayMeshToHalfedgeMesh(
    std::unique_ptr<McDispatchContextInternal>& ctxtPtr,
    mcut::mesh_t& halfedgeMesh,
    mcut::math::real_number_t& bboxDiagonal,
    const void* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces,
    const mcut::math::vec3* perturbation = NULL)
{
    ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION, "construct halfedge mesh");

    McResult result = McResult::MC_NO_ERROR;
    std::map<uint32_t, mcut::vd_t> vmap;

    if (ctxtPtr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_FLOAT) {
        const float* vptr = reinterpret_cast<const float*>(pVertices);
        for (uint32_t i = 0; i < numVertices; ++i) {
            const float& x = vptr[(i * 3) + 0];
            const float& y = vptr[(i * 3) + 1];
            const float& z = vptr[(i * 3) + 2];
            vmap[i] = halfedgeMesh.add_vertex(
                x + (perturbation != NULL ? (*perturbation).x() : 0.f),
                y + (perturbation != NULL ? (*perturbation).y() : 0.f),
                z + (perturbation != NULL ? (*perturbation).z() : 0.f));
        }
    } else if (ctxtPtr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_DOUBLE) {
        const double* vptr = reinterpret_cast<const double*>(pVertices);
        for (uint32_t i = 0; i < numVertices; ++i) {
            const double& x = vptr[(i * 3) + 0];
            const double& y = vptr[(i * 3) + 1];
            const double& z = vptr[(i * 3) + 2];
            vmap[i] = halfedgeMesh.add_vertex(
                x + (perturbation != NULL ? (*perturbation).x() : 0.f),
                y + (perturbation != NULL ? (*perturbation).y() : 0.f),
                z + (perturbation != NULL ? (*perturbation).z() : 0.f));
        }
    } else if (ctxtPtr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_EXACT) {
#pragma warning("TODO: perturbation code")
        const char* vptr = reinterpret_cast<const char*>(pVertices);
        const char* vptr_ = vptr; // shifted

        // storage for digits and decimal point (if any)
        std::vector<char> x;
        std::vector<char> y;
        std::vector<char> z;

        // for each number
        for (uint32_t i = 0; i < numVertices * 3; ++i) {

            vptr_ = std::strchr(vptr, ' ');
            std::ptrdiff_t diff = vptr_ - vptr;
            uint64_t srcStrLen = diff + 1; // extra byte for null-char
            if (vptr_ == nullptr) {
                srcStrLen = strlen(vptr) + 1;
            }

            if ((i % 3) == 0) { // x
                x.resize(srcStrLen);
                std::sscanf(vptr, "%s", &x[0]);
                x.back() = '\0';
            } else if ((i % 3) - 1 == 0) { // y
                y.resize(srcStrLen);
                std::sscanf(vptr, "%s", &y[0]);
                y.back() = '\0';
            } else if ((i % 3) - 2 == 0) { // z
                z.resize(srcStrLen);
                std::sscanf(vptr, "%s", &z[0]);
                z.back() = '\0';

                vmap[i / 3u] = halfedgeMesh.add_vertex(x.data(), y.data(), z.data()); // add vertex with exact number from parsed string

                vptr_ = std::strchr(vptr, '\n'); // .. find newline char (start of next vertex)

                if (vptr_ == nullptr && i != (numVertices * 3) - 1) { // its not the last entry
                    result = McResult::MC_INVALID_VALUE;

                    if (result != McResult::MC_NO_ERROR) {
                        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid numerical string format");

                        return result;
                    }
                }
            }

            vptr = vptr_ + 1; // offset so that we point to the start of the next number/line
        }
    } else {
        result = McResult::MC_INVALID_VALUE;

        if (result != McResult::MC_NO_ERROR) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid bit precision flag");

            return result;
        }
    }

    mcut::math::vec3 bboxMin(1e10);
    mcut::math::vec3 bboxMax(-1e10);
    for (mcut::mesh_t::vertex_iterator_t i = halfedgeMesh.vertices_begin(); i != halfedgeMesh.vertices_end(); ++i) {
        const mcut::math::vec3& coords = halfedgeMesh.vertex(*i);
        bboxMin = mcut::math::compwise_min(bboxMin, coords);
        bboxMax = mcut::math::compwise_max(bboxMax, coords);
    }
    bboxDiagonal = mcut::math::length(bboxMax - bboxMin);

    int faceSizeOffset = 0;

    for (uint32_t i = 0; i < numFaces; ++i) {

        std::vector<mcut::vd_t> faceVertices;
        int numFaceVertices = 3; // triangle

        numFaceVertices = ((uint32_t*)pFaceSizes)[i];

        if (numFaceVertices < 3) {
            result = McResult::MC_INVALID_VALUE;

            if (result != McResult::MC_NO_ERROR) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid face-size for face - " + std::to_string(i) + " (size = " + std::to_string(numFaceVertices) + ")");

                return result;
            }
        }

        faceVertices.reserve(numFaceVertices);

        for (int j = 0; j < numFaceVertices; ++j) {

            uint32_t idx = ((uint32_t*)pFaceIndices)[faceSizeOffset + j];
            std::map<uint32_t, mcut::vd_t>::const_iterator fIter = vmap.find(idx);

            if (fIter == vmap.cend()) {

                result = McResult::MC_INVALID_VALUE;

                if (result != McResult::MC_NO_ERROR) {
                    ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid vertex index - " + std::to_string(idx));

                    return result;
                }
            }

            const mcut::vertex_descriptor_t descr = fIter->second; //vmap[*fIter.first];

            const bool isDuplicate = std::find(faceVertices.cbegin(), faceVertices.cend(), descr) != faceVertices.cend();

            if (isDuplicate) {
                result = McResult::MC_INVALID_VALUE;

                if (result != McResult::MC_NO_ERROR) {
                    ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "found duplicate vertex in face - " + std::to_string(i));

                    return result;
                }
            }

            faceVertices.push_back(descr);
        }

        mcut::fd_t fd = halfedgeMesh.add_face(faceVertices);

        if (fd == mcut::mesh_t::null_face()) {

            result = McResult::MC_INVALID_VALUE;
            if (result != McResult::MC_NO_ERROR) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid vertices on face - " + std::to_string(i));

                return result;
            }
        }

        faceSizeOffset += numFaceVertices;
    }
    return result;
}

McResult convert(const mcut::status_t& v)
{
    McResult result = McResult::MC_RESULT_MAX_ENUM;
    switch (v) {
    case mcut::status_t::SUCCESS:
        result = McResult::MC_NO_ERROR;
        break;
    case mcut::status_t::GENERAL_POSITION_VIOLATION:
    case mcut::status_t::INVALID_MESH_INTERSECTION:
        result = McResult::MC_INVALID_OPERATION;
        break;
    //case mcut::status_t::INVALID_CUT_MESH:
    //    result = McResult::MC_INVALID_CUT_MESH;
    //    break;
    //case mcut::status_t::INVALID_MESH_INTERSECTION:
    //case mcut::status_t::INVALID_BVH_INTERSECTION:
    //    result = McResult::MC_INVALID_OPERATION;
    //    break;
    //case mcut::status_t::EDGE_EDGE_INTERSECTION:
    //    result = McResult::MC_EDGE_EDGE_INTERSECTION;
    //   break;
    //case mcut::status_t::FACE_VERTEX_INTERSECTION:
    //    result = McResult::MC_FACE_VERTEX_INTERSECTION;
    //    break;
    default:
        std::fprintf(stderr, "[MCUT]: warning - conversion error (McResult)\n");
    }
    return result;
}

McPatchLocation convert(const mcut::cut_surface_patch_location_t& v)
{
    McPatchLocation result = McPatchLocation::MC_PATCH_LOCATION_ALL;
    switch (v) {
    case mcut::cut_surface_patch_location_t::INSIDE:
        result = McPatchLocation::MC_PATCH_LOCATION_INSIDE;
        break;
    case mcut::cut_surface_patch_location_t::OUTSIDE:
        result = McPatchLocation::MC_PATCH_LOCATION_OUTSIDE;
        break;
    case mcut::cut_surface_patch_location_t::UNDEFINED:
        result = McPatchLocation::MC_PATCH_LOCATION_UNDEFINED;
        break;
    default:
        std::fprintf(stderr, "[MCUT]: warning - conversion error (McPatchLocation)\n");
    }
    return result;
}

McFragmentLocation convert(const mcut::connected_component_location_t& v)
{
    McFragmentLocation result = McFragmentLocation::MC_FRAGMENT_LOCATION_ALL;
    switch (v) {
    case mcut::connected_component_location_t::ABOVE:
        result = McFragmentLocation::MC_FRAGMENT_LOCATION_ABOVE;
        break;
    case mcut::connected_component_location_t::BELOW:
        result = McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW;
        break;
    case mcut::connected_component_location_t::UNDEFINED:
        result = McFragmentLocation::MC_FRAGMENT_LOCATION_UNDEFINED;
        break;
    default:
        std::fprintf(stderr, "[MCUT]: warning - conversion error (McFragmentLocation)\n");
    }
    return result;
}

McResult halfedgeMeshToIndexArrayMesh(
    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr,
    IndexArrayMesh& indexArrayMesh,
    const mcut::output_mesh_info_t& halfedgeMeshInfo)
{
    McResult result = McResult::MC_NO_ERROR;

    std::map<mcut::vd_t, uint32_t> vmap;
    //
    // vertices
    //
    // create the vertices

    // number of vertices is the same irrespective of whether we are dealing with a
    // triangulated mesh instance or not. Thus, only one set of vertices is stored

    indexArrayMesh.numVertices = halfedgeMeshInfo.mesh.number_of_vertices();

    MCUT_ASSERT(indexArrayMesh.numVertices >= 3);

    indexArrayMesh.pVertices = std::unique_ptr<mcut::math::real_number_t[]>(new mcut::math::real_number_t[(size_t)indexArrayMesh.numVertices * 3u]);
    if (!halfedgeMeshInfo.data_maps.vertex_map.empty()) {
        indexArrayMesh.pVertexMapIndices = std::unique_ptr<uint32_t[]>(new uint32_t[(size_t)indexArrayMesh.numVertices]);
    }

    for (uint32_t i = 0; i < indexArrayMesh.numVertices; ++i) {

        mcut::mesh_t::vertex_iterator_t vIter = halfedgeMeshInfo.mesh.vertices_begin();
        std::advance(vIter, i);
        const mcut::math::vec3& point = halfedgeMeshInfo.mesh.vertex(*vIter);

        indexArrayMesh.pVertices[((size_t)i * 3u) + 0u] = point.x();
        indexArrayMesh.pVertices[((size_t)i * 3u) + 1u] = point.y();
        indexArrayMesh.pVertices[((size_t)i * 3u) + 2u] = point.z();

        //std::cout << indexArrayMesh.pVertices[(i * 3u) + 0u] << " " << indexArrayMesh.pVertices[(i * 3u) + 1u] << " " << indexArrayMesh.pVertices[(i * 3u) + 2u] << std::endl;

        vmap[*vIter] = i;

        if (!halfedgeMeshInfo.data_maps.vertex_map.empty()) {
            MCUT_ASSERT(halfedgeMeshInfo.data_maps.vertex_map.count(*vIter) == 1);
            // TODO: set vertices that were added as a result of discovering floating polygons to same value as intersection points i.e. uint_max
            indexArrayMesh.pVertexMapIndices[i] = halfedgeMeshInfo.data_maps.vertex_map.at(*vIter); // intersection points at set to uint_max
        }
    }

    MCUT_ASSERT(!vmap.empty());

    // create array of seam vertices

    uint32_t numSeamVertexIndices = (uint32_t)halfedgeMeshInfo.seam_vertices.size();
    indexArrayMesh.numSeamVertexIndices = numSeamVertexIndices;
    MCUT_ASSERT(indexArrayMesh.numSeamVertexIndices > 0u);

    indexArrayMesh.pSeamVertexIndices = std::unique_ptr<uint32_t[]>(new uint32_t[numSeamVertexIndices]);
    for (uint32_t i = 0; i < numSeamVertexIndices; ++i) {
        indexArrayMesh.pSeamVertexIndices[i] = halfedgeMeshInfo.seam_vertices[i];
    }

    //
    // faces
    //

    // NOTE: faces can be zero if mesh is a cut-path mesh
    indexArrayMesh.numFaces = halfedgeMeshInfo.mesh.number_of_faces();

    MCUT_ASSERT(indexArrayMesh.numFaces > 0);

    indexArrayMesh.pFaceSizes = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaces]);
    if (!halfedgeMeshInfo.data_maps.face_map.empty()) {
        indexArrayMesh.pFaceMapIndices = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaces]);
    }
    indexArrayMesh.numFaceIndices = 0;

    std::vector<std::vector<mcut::vd_t>> gatheredFaces;
    for (mcut::mesh_t::face_iterator_t i = halfedgeMeshInfo.mesh.faces_begin(); i != halfedgeMeshInfo.mesh.faces_end(); ++i) {

        std::vector<mcut::vd_t> face;
        std::vector<mcut::vd_t> vertices_around_face = halfedgeMeshInfo.mesh.get_vertices_around_face(*i);

        for (std::vector<mcut::vd_t>::const_iterator iter = vertices_around_face.cbegin();
             iter != vertices_around_face.cend();
             ++iter) {
            MCUT_ASSERT(*iter != mcut::mesh_t::null_vertex());

            face.push_back(*iter);
        }

        gatheredFaces.emplace_back(face);

        indexArrayMesh.numFaceIndices += (int)face.size();

        if (!halfedgeMeshInfo.data_maps.face_map.empty()) {
            MCUT_ASSERT(halfedgeMeshInfo.data_maps.face_map.count(*i) == 1);
            const size_t idx = std::distance(halfedgeMeshInfo.mesh.faces_begin(), i);
            indexArrayMesh.pFaceMapIndices[idx] = static_cast<uint32_t>(halfedgeMeshInfo.data_maps.face_map.at(*i));
        }
    }

    // sanity check

    MCUT_ASSERT(gatheredFaces.size() == indexArrayMesh.numFaces);

    indexArrayMesh.pFaceIndices = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaceIndices]);

    int faceOffset = 0;

    for (int i = 0; i < (int)gatheredFaces.size(); ++i) {
        const std::vector<mcut::vd_t>& face = gatheredFaces[i];
        if (static_cast<std::size_t>(std::numeric_limits<uint32_t>::max()) < face.size()) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
                std::string("number of vertices in face (") + std::to_string(face.size()) + ") exceeds maximum (" + std::to_string(std::numeric_limits<uint32_t>::max()) + ")");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }

        indexArrayMesh.pFaceSizes[i] = static_cast<uint32_t>(face.size());

        for (int j = 0; j < (int)face.size(); ++j) {
            const mcut::vd_t vd = face[j];

            indexArrayMesh.pFaceIndices[(size_t)faceOffset + j] = vmap[vd];
        }
        faceOffset += static_cast<int>(face.size());
    }
    gatheredFaces.clear();

    //
    // edges
    //

    indexArrayMesh.numEdgeIndices = halfedgeMeshInfo.mesh.number_of_edges() * 2;

    MCUT_ASSERT(indexArrayMesh.numEdgeIndices > 0);

    std::vector<std::pair<mcut::vd_t, mcut::vd_t>> gatheredEdges;

    for (mcut::mesh_t::edge_iterator_t i = halfedgeMeshInfo.mesh.edges_begin(); i != halfedgeMeshInfo.mesh.edges_end(); ++i) {

        mcut::vd_t v0 = halfedgeMeshInfo.mesh.vertex(*i, 0);
        mcut::vd_t v1 = halfedgeMeshInfo.mesh.vertex(*i, 1);

        gatheredEdges.emplace_back(v0, v1);
    }

    // sanity check

    MCUT_ASSERT(gatheredEdges.size() == indexArrayMesh.numEdgeIndices / 2);

    indexArrayMesh.pEdges = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numEdgeIndices]);

    for (uint32_t i = 0; i < (uint32_t)gatheredEdges.size(); ++i) {
        const std::pair<mcut::vd_t, mcut::vd_t>& edge = gatheredEdges[i];
        mcut::vd_t v0 = edge.first;
        mcut::vd_t v1 = edge.second;

        MCUT_ASSERT(vmap.find(v0) != vmap.cend());
        indexArrayMesh.pEdges[((size_t)i * 2u) + 0u] = vmap[v0];
        MCUT_ASSERT(vmap.find(v1) != vmap.cend());
        indexArrayMesh.pEdges[((size_t)i * 2u) + 1u] = vmap[v1];
    }

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcSetRoundingMode(McContext context, McFlags rmode)
{
    McResult result = MC_NO_ERROR;

    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "error: context undefined");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    McRoundingModeFlags f = static_cast<McRoundingModeFlags>(rmode);
    bool isvalid = f == MC_ROUNDING_MODE_TO_NEAREST || //
        f == MC_ROUNDING_MODE_TOWARD_ZERO || //
        f == MC_ROUNDING_MODE_TOWARD_POS_INF || //
        f == MC_ROUNDING_MODE_TOWARD_NEG_INF;
    if (!isvalid) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, "invalid rounding mode");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    ctxtPtr->roundingMode = f;
    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcGetRoundingMode(McContext context, McFlags* rmode)
{
    McResult result = MC_NO_ERROR;

    if (context == nullptr) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    *rmode = ctxtPtr->roundingMode;

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcSetPrecision(McContext context, uint64_t prec)
{
    McResult result = MC_NO_ERROR;

    if (context == nullptr) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if (prec < McDispatchContextInternal::minPrecision || prec > McDispatchContextInternal::maxPrecision) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, "out of range precision");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    ctxtPtr->precision = prec;

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcGetPrecision(McContext context, uint64_t* prec)
{
    McResult result = MC_NO_ERROR;

    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;
    *prec = ctxtPtr->precision;

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcCreateContext(McContext* pContext, McFlags flags)
{
    McResult result = McResult::MC_NO_ERROR;

    if (pContext == nullptr) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    std::unique_ptr<McDispatchContextInternal> ctxt = std::unique_ptr<McDispatchContextInternal>(new McDispatchContextInternal);
    ctxt->flags = flags;
    McContext handle = reinterpret_cast<McContext>(ctxt.get());
    auto ret = gDispatchContexts.emplace(handle, std::move(ctxt));
    if (ret.second == false) {
        std::fprintf(stderr, "err: failed to create context\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }
    *pContext = ret.first->first;

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcDebugMessageCallback(McContext pContext, pfn_mcDebugOutput_CALLBACK cb, const void* userParam)
{
    McResult result = McResult::MC_NO_ERROR;

    if (cb == nullptr) {
        std::fprintf(stderr, "err: null callback parameter");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    auto ctxtIter = gDispatchContexts.find(pContext);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if (cb == nullptr) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, "callback parameter NULL");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    ctxtPtr->debugCallback = cb;
    ctxtPtr->debugCallbackUserParam = userParam;

    return result;
}

// find the number of trailing zeros in v
// http://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightLinear
int trailing_zeroes(unsigned int v)
{
    int r; // the result goes here
#ifdef _WIN32
#pragma warning(disable : 4146) // "unary minus operator applied to unsigned type, result still unsigned"
#endif // #ifdef _WIN32
    float f = (float)(v & -v); // cast the least significant bit in v to a float
#ifdef _WIN32
#pragma warning(default : 4146)
#endif // #ifdef _WIN32

// dereferencing type-punned pointer will break strict-aliasing rules
#if __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif

    r = (*(uint32_t*)&f >> 23) - 0x7f;

#if __linux__
#pragma GCC diagnostic pop
#endif
    return r;
}

// https://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit
int set_bit(unsigned int v, unsigned int pos)
{
    v |= 1U << pos;
    return v;
}

int clear_bit(unsigned int v, unsigned int pos)
{
    v &= ~(1UL << pos);
    return v;
}

MCAPI_ATTR McResult MCAPI_CALL mcDebugMessageControl(McContext pContext, McDebugSource source, McDebugType type, McDebugSeverity severity, bool enabled)
{
    McResult result = McResult::MC_NO_ERROR;

    auto ctxtIter = gDispatchContexts.find(pContext);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    // check source parameter
    bool sourceParamValid = source == MC_DEBUG_SOURCE_API || //
        source == MC_DEBUG_SOURCE_KERNEL || //
        source == MC_DEBUG_SOURCE_ALL;

    if (!sourceParamValid) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_MEDIUM, "Invalid source parameter value");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    for (auto i : { McDebugSource::MC_DEBUG_SOURCE_API, McDebugSource::MC_DEBUG_SOURCE_KERNEL }) {
        if ((source & i) && enabled) {
            int n = trailing_zeroes(McDebugSource::MC_DEBUG_SOURCE_ALL & i);
            ctxtPtr->debugSource = set_bit(ctxtPtr->debugSource, n);
        }
    }

    // check debug type parameter
    bool typeParamValid = type == MC_DEBUG_TYPE_ERROR || //
        type == MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR || //
        type == MC_DEBUG_TYPE_OTHER || //
        type == MC_DEBUG_TYPE_ALL;

    if (!typeParamValid) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_MEDIUM, "Invalid debug type parameter value");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    ctxtPtr->debugType = 0;

    for (auto i : { McDebugType::MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR, McDebugType::MC_DEBUG_TYPE_ERROR, McDebugType::MC_DEBUG_TYPE_OTHER }) {
        if ((type & i) && enabled) {
            int n = trailing_zeroes(McDebugType::MC_DEBUG_TYPE_ALL & i);
            ctxtPtr->debugType = set_bit(ctxtPtr->debugType, n);
        }
    }

    // check debug severity parameter
    bool severityParamValid = severity == MC_DEBUG_SEVERITY_HIGH || //
        severity == MC_DEBUG_SEVERITY_MEDIUM || //
        severity == MC_DEBUG_SEVERITY_LOW || //
        severity == MC_DEBUG_SEVERITY_NOTIFICATION || //
        severity == MC_DEBUG_SEVERITY_ALL;

    if (!severityParamValid) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_MEDIUM, "Invalid debug severity parameter value");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    ctxtPtr->debugSeverity = 0;

    for (auto i : { McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, McDebugSeverity::MC_DEBUG_SEVERITY_MEDIUM, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION }) {
        if ((severity & i) && enabled) {
            int n = trailing_zeroes(McDebugSeverity::MC_DEBUG_SEVERITY_ALL & i);
            ctxtPtr->debugSeverity = set_bit(ctxtPtr->debugSeverity, n);
        }
    }

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcGetInfo(const McContext context, McFlags info, uint64_t bytes, void* pMem, uint64_t* pNumBytes)
{
    McResult result = McResult::MC_NO_ERROR;

    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: invalid context\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if (bytes != 0 && pMem == nullptr) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "null parameter");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    switch (info) {
    case MC_CONTEXT_FLAGS:
        if (pMem == nullptr) {
            *pNumBytes = sizeof(ctxtPtr->flags);
        } else {
            if (bytes > sizeof(ctxtPtr->flags)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(&ctxtPtr->flags), bytes);
        }
        break;
    case MC_DEFAULT_PRECISION:
        if (pMem == nullptr) {
            *pNumBytes = sizeof(ctxtPtr->defaultPrecision);
        } else {
            if (bytes > sizeof(ctxtPtr->defaultPrecision)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(&ctxtPtr->defaultPrecision), bytes);
        }
        break;
    case MC_DEFAULT_ROUNDING_MODE:
        if (pMem == nullptr) {
            *pNumBytes = sizeof(ctxtPtr->defaultRoundingMode);
        } else {
            if (bytes > sizeof(ctxtPtr->defaultRoundingMode)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(&ctxtPtr->defaultRoundingMode), bytes);
        }
        break;
    case MC_PRECISION_MAX:
        if (pMem == nullptr) {
            *pNumBytes = sizeof(McDispatchContextInternal::maxPrecision);
        } else {
            if (bytes > sizeof(McDispatchContextInternal::maxPrecision)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<const void*>(&McDispatchContextInternal::maxPrecision), bytes);
        }
        break;
    case MC_PRECISION_MIN:
        if (pMem == nullptr) {
            *pNumBytes = sizeof(McDispatchContextInternal::minPrecision);
        } else {
            if (bytes > sizeof(McDispatchContextInternal::minPrecision)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<const void*>(&McDispatchContextInternal::minPrecision), bytes);
        }
        break;
    case MC_DEBUG_KERNEL_TRACE:
        if (pMem == nullptr) {
            *pNumBytes = ctxtPtr->lastLoggedDebugDetail.length();
        } else {
            if (bytes == 0 || bytes > ctxtPtr->lastLoggedDebugDetail.length()) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<const void*>(ctxtPtr->lastLoggedDebugDetail.data()), bytes);
        }
        break;
        break;
    default:
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_LOW, "unknown info parameter");
        result = McResult::MC_INVALID_VALUE;
        break;
    }

    return result;
}

bool checkFrontendMesh(
    std::unique_ptr<McDispatchContextInternal>& ctxtPtr,
    const void* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces)
{
    bool result = true;
    std::string errmsg;
    if (pVertices == nullptr) {
        errmsg = ("undefined vertices");
        result = false;
    } else if (numVertices < 3) {
        errmsg = "invalid vertex count";
        result = false;
    } else if (pFaceIndices == nullptr) {
        errmsg = "undefined faces";
        result = false;
    } else if (pFaceSizes == nullptr) {
        errmsg = "undefined faces";
        result = false;
    } else if (numFaces == 0) {
        errmsg = "undefined face count";
        result = false;
    }

    if (!result) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, errmsg);
    }
    return result;
}

#if 0
McResult checkMeshPlacement(std::unique_ptr<McDispatchContextInternal>& ctxtPtr, const mcut::mesh_t& srcMesh, const mcut::mesh_t& cutMesh)
{
    MCUT_ASSERT(srcMesh.number_of_vertices() >= 3);
    MCUT_ASSERT(cutMesh.number_of_vertices() >= 3);

    McResult result = McResult::MC_NO_ERROR;
    for (mcut::mesh_t::vertex_iterator_t i = srcMesh.vertices_begin(); i != srcMesh.vertices_end(); ++i) {
        const mcut::math::vec3& srcMeshVertex = srcMesh.vertex(*i);
        for (mcut::mesh_t::vertex_iterator_t j = cutMesh.vertices_begin(); j != cutMesh.vertices_end(); ++j) {
            const mcut::math::vec3& cutMeshVertex = cutMesh.vertex(*j);
            if (srcMeshVertex.x() == cutMeshVertex.x() && srcMeshVertex.y() == cutMeshVertex.y() && srcMeshVertex.z() == cutMeshVertex.z()) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
                    "source-mesh vertex " + std::to_string(*i) + " is the same as cut-mesh vertex " + std::to_string(*j) + "\n");
                result = McResult::MC_INVALID_MESH_PLACEMENT;
                break;
            }
        }
        if (result != McResult::MC_NO_ERROR) {
            break;
        }
    }

    return result;
}
#endif

#include <algorithm> // std::sort
#include <queue> // std::queue for traversal

void constructOIBVH(
    const mcut::mesh_t& mesh,
    std::vector<mcut::geom::bounding_box_t<mcut::math::fast_vec3>>& bvhAABBs,
    std::vector<mcut::fd_t>& bvhLeafNodeFaces)
{
    const int meshFaceCount = mesh.number_of_faces();
    const int bvhNodeCount = mcut::bvh::get_ostensibly_implicit_bvh_size(meshFaceCount);

    // compute mesh-face bounding boxes and their centers
    // ::::::::::::::::::::::::::::::::::::::::::::::::::

    std::vector<mcut::geom::bounding_box_t<mcut::math::fast_vec3>> face_bboxes(meshFaceCount, mcut::geom::bounding_box_t<mcut::math::fast_vec3>());
    std::vector<mcut::math::fast_vec3> face_bbox_centers(meshFaceCount, mcut::math::fast_vec3());

    // for each face in mesh
    for (mcut::mesh_t::face_iterator_t f = mesh.faces_begin(); f != mesh.faces_end(); ++f) {
        const int faceIdx = static_cast<int>(*f);
        const std::vector<mcut::vd_t> vertices_on_face = mesh.get_vertices_around_face(*f);

        // for each vertex on face
        for (std::vector<mcut::vd_t>::const_iterator v = vertices_on_face.cbegin(); v != vertices_on_face.cend(); ++v) {
            const mcut::math::fast_vec3 coords = mesh.vertex(*v);
            face_bboxes[faceIdx].expand(coords);
        }

        const mcut::geom::bounding_box_t<mcut::math::fast_vec3>& bbox = face_bboxes[faceIdx];

        // calculate bbox center
        face_bbox_centers[*f] = (bbox.minimum() + bbox.maximum()) / 2;
    }

    // compute mesh bounding box
    // :::::::::::::::::::::::::

    bvhAABBs.resize(bvhNodeCount);
    mcut::geom::bounding_box_t<mcut::math::fast_vec3>& meshBbox = bvhAABBs.front(); // root bounding box

    // for each vertex in mesh
    for (mcut::mesh_t::vertex_iterator_t v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        const mcut::math::vec3& coords = mesh.vertex(*v);
        meshBbox.expand(coords);
    }

    // compute morton codes
    // ::::::::::::::::::::

    std::vector<std::pair<mcut::fd_t, uint32_t>> bvhLeafNodeDescriptors(meshFaceCount, std::pair<mcut::fd_t, uint32_t>());

    for (mcut::mesh_t::face_iterator_t f = mesh.faces_begin(); f != mesh.faces_end(); ++f) {
        const uint32_t faceIdx = static_cast<uint32_t>(*f);

        const mcut::math::fast_vec3& face_aabb_centre = face_bbox_centers.at(faceIdx);
        const mcut::math::fast_vec3 offset = face_aabb_centre - meshBbox.minimum();
        const mcut::math::fast_vec3 dims = meshBbox.maximum() - meshBbox.minimum();

        const unsigned int mortion_code = mcut::bvh::morton3D(
            static_cast<float>(offset.x() / dims.x()),
            static_cast<float>(offset.y() / dims.y()),
            static_cast<float>(offset.z() / dims.z()));

        const uint32_t idx = (uint32_t)std::distance(mesh.faces_begin(), f);
        bvhLeafNodeDescriptors[idx].first = *f;
        bvhLeafNodeDescriptors[idx].second = mortion_code;
    }

    // sort faces according to morton codes

    std::sort(
        bvhLeafNodeDescriptors.begin(),
        bvhLeafNodeDescriptors.end(),
        [](const std::pair<mcut::fd_t, uint32_t>& a, const std::pair<mcut::fd_t, uint32_t>& b) {
            return a.second < b.second;
        });

    bvhLeafNodeFaces.resize(meshFaceCount);

    const int leaf_level_index = mcut::bvh::get_leaf_level_from_real_leaf_count(meshFaceCount);
    const int leftmost_real_node_on_leaf_level = mcut::bvh::get_level_leftmost_node(leaf_level_index);
    const int rightmost_real_leaf = mcut::bvh::get_rightmost_real_leaf(leaf_level_index, meshFaceCount);
    const int rightmost_real_node_on_leaf_level = mcut::bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, leaf_level_index);

    // save sorted leaf node bvhAABBs and their corrresponding face id
    for (std::vector<std::pair<mcut::fd_t, uint32_t>>::const_iterator it = bvhLeafNodeDescriptors.cbegin(); it != bvhLeafNodeDescriptors.cend(); ++it) {
        const uint32_t index_on_leaf_level = (uint32_t)std::distance(bvhLeafNodeDescriptors.cbegin(), it);

        bvhLeafNodeFaces[index_on_leaf_level] = it->first;

        const int implicit_idx = leftmost_real_node_on_leaf_level + index_on_leaf_level;
        const int memory_idx = mcut::bvh::get_node_mem_index(
            implicit_idx,
            leftmost_real_node_on_leaf_level,
            0,
            rightmost_real_node_on_leaf_level);

        const mcut::geom::bounding_box_t<mcut::math::fast_vec3>& face_bbox = face_bboxes[(uint32_t)it->first];
        bvhAABBs[memory_idx] = face_bbox;
    }

    // construct internal-node bounding boxes
    // ::::::::::::::::::::::::::::::::::::::

    // for each level in the oi-bvh tree (starting from the penultimate level)
    for (int level_index = leaf_level_index - 1; level_index >= 0; --level_index) {

        const int rightmost_real_node_on_level = mcut::bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, level_index);
        const int leftmost_real_node_on_level = mcut::bvh::get_level_leftmost_node(level_index);
        const int number_of_real_nodes_on_level = (rightmost_real_node_on_level - leftmost_real_node_on_level) + 1;

        // for each node on the current level
        for (int level_node_idx_iter = 0; level_node_idx_iter < number_of_real_nodes_on_level; ++level_node_idx_iter) {

            const int node_implicit_idx = leftmost_real_node_on_level + level_node_idx_iter;
            const int left_child_implicit_idx = (node_implicit_idx * 2) + 1;
            const int right_child_implicit_idx = (node_implicit_idx * 2) + 2;
            const bool is_penultimate_level = (level_index == (leaf_level_index - 1));
            const int rightmost_real_node_on_child_level = mcut::bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, level_index + 1);
            const int leftmost_real_node_on_child_level = mcut::bvh::get_level_leftmost_node(level_index + 1);
            const bool right_child_exists = (right_child_implicit_idx <= rightmost_real_node_on_child_level);

            mcut::geom::bounding_box_t<mcut::math::fast_vec3> node_bbox;

            if (is_penultimate_level) { // both children are leaves

                const int left_child_index_on_level = left_child_implicit_idx - leftmost_real_node_on_child_level;
                const mcut::fd_t& left_child_face = bvhLeafNodeFaces.at(left_child_index_on_level);
                const mcut::geom::bounding_box_t<mcut::math::fast_vec3>& left_child_bbox = face_bboxes.at(left_child_face);

                node_bbox.expand(left_child_bbox);

                if (right_child_exists) {
                    const int right_child_index_on_level = right_child_implicit_idx - leftmost_real_node_on_child_level;
                    const mcut::fd_t& right_child_face = bvhLeafNodeFaces.at(right_child_index_on_level);
                    const mcut::geom::bounding_box_t<mcut::math::fast_vec3>& right_child_bbox = face_bboxes.at(right_child_face);
                    node_bbox.expand(right_child_bbox);
                }
            } else { // remaining internal node levels

                const int left_child_memory_idx = mcut::bvh::get_node_mem_index(
                    left_child_implicit_idx,
                    leftmost_real_node_on_child_level,
                    0,
                    rightmost_real_node_on_child_level);
                const mcut::geom::bounding_box_t<mcut::math::fast_vec3>& left_child_bbox = bvhAABBs.at(left_child_memory_idx);

                node_bbox.expand(left_child_bbox);

                if (right_child_exists) {
                    const int right_child_memory_idx = mcut::bvh::get_node_mem_index(
                        right_child_implicit_idx,
                        leftmost_real_node_on_child_level,
                        0,
                        rightmost_real_node_on_child_level);
                    const mcut::geom::bounding_box_t<mcut::math::fast_vec3>& right_child_bbox = bvhAABBs.at(right_child_memory_idx);
                    node_bbox.expand(right_child_bbox);
                }
            }

            const int node_memory_idx = mcut::bvh::get_node_mem_index(
                node_implicit_idx,
                leftmost_real_node_on_level,
                0,
                rightmost_real_node_on_level);

            bvhAABBs.at(node_memory_idx) = node_bbox;
        } // for each real node on level
    } // for each internal level
}

void intersectOIBVHs(
    std::vector<std::pair<mcut::fd_t, mcut::fd_t>>& intersecting_sm_cm_face_pairs,
    const std::vector<mcut::geom::bounding_box_t<mcut::math::fast_vec3>>& srcMeshBvhAABBs,
    const std::vector<mcut::fd_t>& srcMeshBvhLeafNodeFaces,
    const std::vector<mcut::geom::bounding_box_t<mcut::math::fast_vec3>>& cutMeshBvhAABBs,
    const std::vector<mcut::fd_t>& cutMeshBvhLeafNodeFaces)
{
    // simultaneuosly traverse both BVHs to find intersecting pairs
    std::queue<mcut::bvh::node_pair_t> traversalQueue;
    traversalQueue.push({ 0, 0 }); // left = sm BVH; right = cm BVH

    const int numSrcMeshFaces = (int)srcMeshBvhLeafNodeFaces.size();
    MCUT_ASSERT(numSrcMeshFaces >= 1);
    const int numCutMeshFaces = (int)cutMeshBvhLeafNodeFaces.size();
    MCUT_ASSERT(numCutMeshFaces >= 1);

    const int sm_bvh_leaf_level_idx = mcut::bvh::get_leaf_level_from_real_leaf_count(numSrcMeshFaces);
    const int cs_bvh_leaf_level_idx = mcut::bvh::get_leaf_level_from_real_leaf_count(numCutMeshFaces);

    const int sm_bvh_rightmost_real_leaf = mcut::bvh::get_rightmost_real_leaf(sm_bvh_leaf_level_idx, numSrcMeshFaces);
    const int cs_bvh_rightmost_real_leaf = mcut::bvh::get_rightmost_real_leaf(cs_bvh_leaf_level_idx, numCutMeshFaces);

    do {
        mcut::bvh::node_pair_t ct_front_node = traversalQueue.front();

        mcut::geom::bounding_box_t<mcut::math::fast_vec3> sm_bvh_node_bbox;
        mcut::geom::bounding_box_t<mcut::math::fast_vec3> cs_bvh_node_bbox;

        // sm
        const int sm_bvh_node_implicit_idx = ct_front_node.m_left;
        const int sm_bvh_node_level_idx = mcut::bvh::get_level_from_implicit_idx(sm_bvh_node_implicit_idx);
        const bool sm_bvh_node_is_leaf = sm_bvh_node_level_idx == sm_bvh_leaf_level_idx;
        const int sm_bvh_node_level_leftmost_node = mcut::bvh::get_level_leftmost_node(sm_bvh_node_level_idx);
        mcut::fd_t sm_node_face = mcut::mesh_t::null_face();
        const int sm_bvh_node_level_rightmost_node = mcut::bvh::get_level_rightmost_real_node(sm_bvh_rightmost_real_leaf, sm_bvh_leaf_level_idx, sm_bvh_node_level_idx);
        const int sm_bvh_node_mem_idx = mcut::bvh::get_node_mem_index(
            sm_bvh_node_implicit_idx,
            sm_bvh_node_level_leftmost_node,
            0,
            sm_bvh_node_level_rightmost_node);
        sm_bvh_node_bbox = srcMeshBvhAABBs.at(sm_bvh_node_mem_idx);

        if (sm_bvh_node_is_leaf) {
            const int sm_bvh_node_idx_on_level = sm_bvh_node_implicit_idx - sm_bvh_node_level_leftmost_node;
            sm_node_face = srcMeshBvhLeafNodeFaces.at(sm_bvh_node_idx_on_level);
        }

        // cs
        const int cs_bvh_node_implicit_idx = ct_front_node.m_right;
        const int cs_bvh_node_level_idx = mcut::bvh::get_level_from_implicit_idx(cs_bvh_node_implicit_idx);
        const int cs_bvh_node_level_leftmost_node = mcut::bvh::get_level_leftmost_node(cs_bvh_node_level_idx);
        const bool cs_bvh_node_is_leaf = cs_bvh_node_level_idx == cs_bvh_leaf_level_idx;
        mcut::fd_t cs_node_face = mcut::mesh_t::null_face();
        const int cs_bvh_node_level_rightmost_node = mcut::bvh::get_level_rightmost_real_node(cs_bvh_rightmost_real_leaf, cs_bvh_leaf_level_idx, cs_bvh_node_level_idx);
        const int cs_bvh_node_mem_idx = mcut::bvh::get_node_mem_index(
            cs_bvh_node_implicit_idx,
            cs_bvh_node_level_leftmost_node,
            0,
            cs_bvh_node_level_rightmost_node);
        cs_bvh_node_bbox = cutMeshBvhAABBs.at(cs_bvh_node_mem_idx);

        if (cs_bvh_node_is_leaf) {
            const int cs_bvh_node_idx_on_level = cs_bvh_node_implicit_idx - cs_bvh_node_level_leftmost_node;
            cs_node_face = cutMeshBvhLeafNodeFaces.at(cs_bvh_node_idx_on_level);
        }

        const bool haveOverlap = intersect_bounding_boxes(sm_bvh_node_bbox, cs_bvh_node_bbox);

        if (haveOverlap) {

            if (cs_bvh_node_is_leaf && sm_bvh_node_is_leaf) {
                MCUT_ASSERT(cs_node_face != mcut::mesh_t::null_face());
                MCUT_ASSERT(sm_node_face != mcut::mesh_t::null_face());

                intersecting_sm_cm_face_pairs.emplace_back(sm_node_face, cs_node_face);

            } else if (sm_bvh_node_is_leaf && !cs_bvh_node_is_leaf) {
                MCUT_ASSERT(cs_node_face == mcut::mesh_t::null_face());
                MCUT_ASSERT(sm_node_face != mcut::mesh_t::null_face());

                const int cs_bvh_node_left_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 1;
                const int cs_bvh_node_right_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 2;

                const int rightmost_real_node_on_child_level = mcut::bvh::get_level_rightmost_real_node(cs_bvh_rightmost_real_leaf, cs_bvh_leaf_level_idx, cs_bvh_node_level_idx + 1);
                const bool right_child_is_real = cs_bvh_node_right_child_implicit_idx <= rightmost_real_node_on_child_level;

                traversalQueue.push({ sm_bvh_node_implicit_idx, cs_bvh_node_left_child_implicit_idx });

                if (right_child_is_real) {
                    traversalQueue.push({ sm_bvh_node_implicit_idx, cs_bvh_node_right_child_implicit_idx });
                }
            } else if (!sm_bvh_node_is_leaf && cs_bvh_node_is_leaf) {

                MCUT_ASSERT(cs_node_face != mcut::mesh_t::null_face());
                MCUT_ASSERT(sm_node_face == mcut::mesh_t::null_face());

                const int sm_bvh_node_left_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 1;
                const int sm_bvh_node_right_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 2;

                const int rightmost_real_node_on_child_level = mcut::bvh::get_level_rightmost_real_node(sm_bvh_rightmost_real_leaf, sm_bvh_leaf_level_idx, sm_bvh_node_level_idx + 1);
                const bool right_child_is_real = sm_bvh_node_right_child_implicit_idx <= rightmost_real_node_on_child_level;

                traversalQueue.push({ sm_bvh_node_left_child_implicit_idx, cs_bvh_node_implicit_idx });

                if (right_child_is_real) {
                    traversalQueue.push({ sm_bvh_node_right_child_implicit_idx, cs_bvh_node_implicit_idx });
                }

            } else { // both nodes are internal
                MCUT_ASSERT(cs_node_face == mcut::mesh_t::null_face());
                MCUT_ASSERT(sm_node_face == mcut::mesh_t::null_face());

                const int sm_bvh_node_left_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 1;
                const int sm_bvh_node_right_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 2;

                const int cs_bvh_node_left_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 1;
                const int cs_bvh_node_right_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 2;

                const int sm_rightmost_real_node_on_child_level = mcut::bvh::get_level_rightmost_real_node(sm_bvh_rightmost_real_leaf, sm_bvh_leaf_level_idx, sm_bvh_node_level_idx + 1);
                const bool sm_right_child_is_real = sm_bvh_node_right_child_implicit_idx <= sm_rightmost_real_node_on_child_level;

                const int cs_rightmost_real_node_on_child_level = mcut::bvh::get_level_rightmost_real_node(cs_bvh_rightmost_real_leaf, cs_bvh_leaf_level_idx, cs_bvh_node_level_idx + 1);
                const bool cs_right_child_is_real = cs_bvh_node_right_child_implicit_idx <= cs_rightmost_real_node_on_child_level;

                traversalQueue.push({ sm_bvh_node_left_child_implicit_idx, cs_bvh_node_left_child_implicit_idx });

                if (cs_right_child_is_real) {
                    traversalQueue.push({ sm_bvh_node_left_child_implicit_idx, cs_bvh_node_right_child_implicit_idx });
                }

                if (sm_right_child_is_real) {
                    traversalQueue.push({ sm_bvh_node_right_child_implicit_idx, cs_bvh_node_left_child_implicit_idx });

                    if (cs_right_child_is_real) {
                        traversalQueue.push({ sm_bvh_node_right_child_implicit_idx, cs_bvh_node_right_child_implicit_idx });
                    }
                }
            }
        }

        traversalQueue.pop(); // rm ct_front_node
    } while (!traversalQueue.empty());
}

MCAPI_ATTR McResult MCAPI_CALL mcDispatch(
    const McContext context,
    McFlags dispatchFlags,
    const void* pSrcMeshVertices,
    const uint32_t* pSrcMeshFaceIndices,
    const uint32_t* pSrcMeshFaceSizes,
    uint32_t numSrcMeshVertices,
    uint32_t numSrcMeshFaces,
    const void* pCutMeshVertices,
    const uint32_t* pCutMeshFaceIndices,
    const uint32_t* pCutMeshFaceSizes,
    uint32_t numCutMeshVertices,
    uint32_t numCutMeshFaces)
{

    McResult result = McResult::MC_NO_ERROR;
    std::map<McContext, std::unique_ptr<McDispatchContextInternal>>::iterator ctxtIter = gDispatchContexts.find(context);

    // check context found
    if (ctxtIter == gDispatchContexts.cend()) {
        fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if ((dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_FLOAT) == 0 && (dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_DOUBLE) == 0 && (dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_EXACT) == 0) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "dispatch floating-point type unspecified");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    ctxtPtr->dispatchFlags = dispatchFlags;

    bool srcMeshOk = checkFrontendMesh(
        ctxtPtr,
        pSrcMeshVertices,
        pSrcMeshFaceIndices,
        pSrcMeshFaceSizes,
        numSrcMeshVertices,
        numSrcMeshFaces);

    if (!srcMeshOk) {
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    bool cutMeshOk = checkFrontendMesh(
        ctxtPtr,
        pCutMeshVertices,
        pCutMeshFaceIndices,
        pCutMeshFaceSizes,
        numCutMeshVertices,
        numCutMeshFaces);

    if (!cutMeshOk) {
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    mcut::mesh_t srcMeshInternal;
    mcut::math::real_number_t srcMeshBboxDiagonal(0.0);
    result = indexArrayMeshToHalfedgeMesh(
        ctxtPtr,
        srcMeshInternal,
        srcMeshBboxDiagonal,
        pSrcMeshVertices,
        pSrcMeshFaceIndices,
        pSrcMeshFaceSizes,
        numSrcMeshVertices,
        numSrcMeshFaces);

    if (result != McResult::MC_NO_ERROR) {
        return result;
    }

    // check here to ensure that vertex coordinates of one mesh are not colocated with any coordinates in the other mesh
    //result = checkMeshPlacement(ctxtPtr, srcMeshInternal, cutMeshInternal);
    //if (result != McResult::MC_NO_ERROR) {
    //    return result;
    //}

    mcut::input_t backendInput;
    backendInput.src_mesh = &srcMeshInternal;

    backendInput.verbose = false;
    backendInput.require_looped_cutpaths = false;

    backendInput.verbose = static_cast<bool>((ctxtPtr->flags & MC_DEBUG) && (ctxtPtr->debugType & McDebugSource::MC_DEBUG_SOURCE_KERNEL));
    backendInput.require_looped_cutpaths = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_REQUIRE_THROUGH_CUTS);
    backendInput.populate_vertex_maps = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_INCLUDE_VERTEX_MAP);
    backendInput.populate_face_maps = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_INCLUDE_FACE_MAP);

    if ((ctxtPtr->dispatchFlags & MC_DISPATCH_REQUIRE_THROUGH_CUTS) && //
        (ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED)) {
        // The user states that she does not want a partial cut but yet also states that she
        // wants to keep fragments with partial cuts. These two options are mutually exclusive!
        ctxtPtr->log(
            McDebugSource::MC_DEBUG_SOURCE_KERNEL,
            McDebugType::MC_DEBUG_TYPE_ERROR,
            0,
            McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
            "use of mutually-exclusive flags: MC_DISPATCH_REQUIRE_THROUGH_CUTS & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED");
        return McResult::MC_INVALID_VALUE;
    }

    uint32_t filterFlagsAll = ( //
        MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE | //
        MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | //
        MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE_EXHAUSTIVE | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE_EXHAUSTIVE | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_NONE | //
        MC_DISPATCH_FILTER_PATCH_INSIDE | //
        MC_DISPATCH_FILTER_PATCH_OUTSIDE | //
        MC_DISPATCH_FILTER_SEAM_SRCMESH | //
        MC_DISPATCH_FILTER_SEAM_CUTMESH);

    const bool dispatchFilteringEnabled = static_cast<bool>(ctxtPtr->dispatchFlags & filterFlagsAll); // any

    if (dispatchFilteringEnabled) { // user only wants [some] output connected components
        backendInput.keep_fragments_below_cutmesh = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW);
        backendInput.keep_fragments_above_cutmesh = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE);
        backendInput.keep_fragments_sealed_outside = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE);
        backendInput.keep_fragments_sealed_inside = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE);
        backendInput.keep_unsealed_fragments = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_NONE);
        backendInput.keep_fragments_partially_cut = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED);
        backendInput.keep_fragments_sealed_outside_exhaustive = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE_EXHAUSTIVE);
        backendInput.keep_fragments_sealed_inside_exhaustive = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE_EXHAUSTIVE);
        backendInput.keep_inside_patches = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_PATCH_INSIDE);
        backendInput.keep_outside_patches = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_PATCH_OUTSIDE);
        backendInput.keep_srcmesh_seam = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_SEAM_SRCMESH);
        backendInput.keep_cutmesh_seam = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_SEAM_CUTMESH);

    } else { // compute all possible types of connected components
        backendInput.keep_fragments_below_cutmesh = true;
        backendInput.keep_fragments_above_cutmesh = true;
        backendInput.keep_fragments_partially_cut = true;
        backendInput.keep_unsealed_fragments = true;
        backendInput.keep_fragments_sealed_outside = true; // mutually exclusive with exhaustive case
        backendInput.keep_fragments_sealed_inside = true;
        backendInput.keep_fragments_sealed_outside_exhaustive = false;
        backendInput.keep_fragments_sealed_inside_exhaustive = false;
        backendInput.keep_inside_patches = true;
        backendInput.keep_outside_patches = true;
        backendInput.keep_srcmesh_seam = true;
        backendInput.keep_cutmesh_seam = true;
    }

    if ((backendInput.keep_fragments_sealed_outside && backendInput.keep_fragments_sealed_outside_exhaustive)) {
        ctxtPtr->log(
            McDebugSource::MC_DEBUG_SOURCE_API,
            McDebugType::MC_DEBUG_TYPE_ERROR,
            0,
            McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
            "use of mutually exclusive flags MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE_EXHAUSTIVE and MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE");
        return MC_INVALID_VALUE;
    }

    if ((backendInput.keep_fragments_sealed_inside && backendInput.keep_fragments_sealed_inside_exhaustive)) {
        ctxtPtr->log(
            McDebugSource::MC_DEBUG_SOURCE_API,
            McDebugType::MC_DEBUG_TYPE_ERROR,
            0,
            McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
            "use of mutually exclusive flags MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE_EXHAUSTIVE and MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE");
        return MC_INVALID_VALUE;
    }

    if (ctxtPtr->dispatchFlags & MC_DISPATCH_ENFORCE_GENERAL_POSITION) {
        backendInput.enforce_general_position = true;
    }

    // Construct BVHs
    // ::::::::::::::

    ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION, "Build source-mesh BVH");
    std::vector<mcut::geom::bounding_box_t<mcut::math::fast_vec3>> srcMeshBvhAABBs;
    std::vector<mcut::fd_t> srcMeshBvhLeafNodeFaces;

    constructOIBVH(srcMeshInternal, srcMeshBvhAABBs, srcMeshBvhLeafNodeFaces);

    ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION, "Build cut-mesh BVH");

    ctxtPtr->fpPartitionChildFaceToInputSrcMeshFace.clear();
    ctxtPtr->fpPartitionChildFaceToInputCutMeshFace.clear();
    int numSourceMeshFacesInLastDispatchCall = numSrcMeshFaces;

    mcut::output_t backendOutput;

    mcut::mesh_t cutMeshInternal;
    mcut::math::real_number_t cutMeshBboxDiagonal(0.0);
    std::vector<mcut::geom::bounding_box_t<mcut::math::fast_vec3>> cutMeshBvhAABBs;
    std::vector<mcut::fd_t> cutMeshBvhLeafNodeFaces;

    int perturbationIters = 0;
    int kernelDispatchCallCounter = -1;
    do {
        kernelDispatchCallCounter++;

        bool general_position_assumption_was_violated = (/*perturbationIters != -1 &&*/ (backendOutput.status == mcut::status_t::GENERAL_POSITION_VIOLATION));
        bool floating_polygon_was_detected = backendOutput.status == mcut::status_t::DETECTED_FLOATING_POLYGON;
        // ::::::::::::::::::::::::::::::::::::::::::::::::::::
        backendOutput.status = mcut::status_t::SUCCESS;

        mcut::math::vec3 perturbation;

        if (general_position_assumption_was_violated) {
            MCUT_ASSERT(floating_polygon_was_detected == false); // cannot occur at same time!
            perturbationIters++;

            if (perturbationIters > 0) {
                // use by the kernel track if the most-recent perturbation causes the cut-mesh and src-mesh to
                // not intersect at all, which means we need to perturb again.
                backendInput.general_position_enforcement_count = perturbationIters;
                const mcut::math::real_number_t scalar = cutMeshBboxDiagonal * GENERAL_POSITION_ENFORCMENT_CONSTANT;
                std::default_random_engine rd(perturbationIters);
                std::mt19937 mt(rd());
                std::uniform_real_distribution<double> dist(-1.0, 1.0);
                perturbation = mcut::math::vec3(
                    dist(mt) * scalar,
                    dist(mt) * scalar,
                    dist(mt) * scalar);
            }
        } // if (general_position_assumption_was_violated) {

        if ((perturbationIters == 0 /*no perturbs required*/ || general_position_assumption_was_violated) && floating_polygon_was_detected == false) {
            cutMeshInternal.remove_elements();

            result = indexArrayMeshToHalfedgeMesh(
                ctxtPtr,
                cutMeshInternal,
                cutMeshBboxDiagonal,
                pCutMeshVertices,
                pCutMeshFaceIndices,
                pCutMeshFaceSizes,
                numCutMeshVertices,
                numCutMeshFaces,
                ((perturbationIters == 0) ? NULL : &perturbation));

            if (result != McResult::MC_NO_ERROR) {
                return result;
            }

            backendInput.cut_mesh = &cutMeshInternal;

            constructOIBVH(cutMeshInternal, cutMeshBvhAABBs, cutMeshBvhLeafNodeFaces);
        }

        if (floating_polygon_was_detected) {
            MCUT_ASSERT(general_position_assumption_was_violated == false); // cannot occur at same time!

            bool floating_polygon_was_on_srcmesh = false;
            bool floating_polygon_was_on_cutmesh = false;

            for (std::map<mcut::fd_t, std::vector<mcut::floating_polygon_info_t>>::const_iterator detected_floating_polygons_iter = backendOutput.detected_floating_polygons.cbegin();
                 detected_floating_polygons_iter != backendOutput.detected_floating_polygons.cend();
                 ++detected_floating_polygons_iter) {

                // get the [origin] input-mesh face index (Note: this index may be offsetted
                // to distinguish between source-mesh and cut-mesh faces).
                const mcut::fd_t ps_face = detected_floating_polygons_iter->first;

                // NOTE: this boolean needs to be evaluated with "numSourceMeshFacesInLastDispatchCall" since the number of
                // src-mesh faces might change as we add more polygons due to partitioning.
                bool on_srcmesh = ((uint32_t)ps_face < (uint32_t)numSourceMeshFacesInLastDispatchCall);
                floating_polygon_was_on_srcmesh = floating_polygon_was_on_srcmesh || on_srcmesh;
                floating_polygon_was_on_cutmesh = floating_polygon_was_on_cutmesh || !on_srcmesh;

                // This data structure maps the new faces in the modified input mesh, to the original partitioned face in the [user-provided] input mesh.
                std::map<mcut::fd_t, mcut::fd_t>& childFaceToBirthFace = (on_srcmesh ? ctxtPtr->fpPartitionChildFaceToInputSrcMeshFace : ctxtPtr->fpPartitionChildFaceToInputCutMeshFace);

                // Now compute the actual input mesh face index (accounting for offset)
                const mcut::fd_t origin_face_initial = on_srcmesh ? ps_face : mcut::fd_t((uint32_t)ps_face - (uint32_t)numSourceMeshFacesInLastDispatchCall); // accounting for offset (NOTE: must updated "srcMeshInternal" state)

                // pointer to input mesh with face containing floating polygon
                // Note: this mesh will be modified as we add new faces.
                mcut::mesh_t* origin_input_mesh = (on_srcmesh ? &srcMeshInternal : &cutMeshInternal);

                MCUT_ASSERT(static_cast<uint32_t>(origin_face_initial) < (uint32_t)origin_input_mesh->number_of_faces());

                // for each floating polygon detected on current ps-face
                for (std::vector<mcut::floating_polygon_info_t>::const_iterator psFaceFloatingPolyIter = detected_floating_polygons_iter->second.cbegin();
                     psFaceFloatingPolyIter != detected_floating_polygons_iter->second.cend();
                     ++psFaceFloatingPolyIter) {

                    const mcut::floating_polygon_info_t& fpi = *psFaceFloatingPolyIter;

                    // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // Here we now need to partition "origin_face" in "origin_input_mesh"
                    // by adding a new edge which is guarranteed to pass through the area
                    // spanned by the floating polygon.

                    // gather vertices of floating polygon (just a list of 3d coords provided by the kernel)

                    const size_t fpVertexCount = fpi.floating_polygon_vertex_positions.size();
                    MCUT_ASSERT(fpVertexCount >= 3);
                    const size_t fpEdgeCount = fpVertexCount; // num edges is same as num verts

                    // project the floating polygon " to 2D

                    std::vector<mcut::math::vec2> fpVertexCoords2D;

                    mcut::geom::project2D(
                        fpVertexCoords2D,
                        fpi.floating_polygon_vertex_positions.data(),
                        (int)fpi.floating_polygon_vertex_positions.size(),
                        fpi.origin_face_normal_largest_comp);

                    // face to be (potentially) partitioned
                    mcut::fd_t origin_face = origin_face_initial;

                    std::map<mcut::fd_t, mcut::fd_t>::const_iterator originFaceToBirthFaceIter = childFaceToBirthFace.find(origin_face);
                    // This is true if "ps_face" had more than one floating polygon.
                    // We need this to handle the case where another partition of "ps_face" (from one of the other floating polys)
                    // produced a new edge (i.e. the one partitioning "ps_face") that passes through the current floating poly. If this
                    // variable is true, we will need to
                    // 1) find all faces in "childFaceToBirthFace" that are mapped to same birth-face as that of "ps_face" (i.e. search by value)
                    // 2) for each such face check to see if any one of its edges intersect the current floating polygon
                    // This is necessary to ensure a minimal set of partitions. See below for details.
                    bool birth_face_partitioned_atleast_once = (originFaceToBirthFaceIter != childFaceToBirthFace.cend());
                    mcut::fd_t birth_face = mcut::mesh_t::null_face();

                    bool mustPartitionCurrentFace = true;
                    // check if we still need to partition origin_face.
                    // If a partitions has already been made that added an edge into "origin_input_mesh" which passes through the current
                    // floating poly, then we will not need to partition "ps_face".
                    // NOTE: there is no guarrantee that the previously added edge that partitions "ps_face" will not violate general-position w.r.t the current floating poly.
                    // Thus, general position might potentially be violated such that we would have to resort to numerical perturbation in the next mcut::dispatch(...) call.
                    if (birth_face_partitioned_atleast_once) {
                        birth_face = originFaceToBirthFaceIter->second;
                        MCUT_ASSERT(origin_face == originFaceToBirthFaceIter->first);

                        // the child face that we create by partitioning "birth_face" (possibly over multiple dispatch calls
                        // in the case that GP is violated by an added edge)
                        std::vector<mcut::fd_t> faces_from_partitioned_birth_face;

                        // for all other faces that share "birth_face"
                        for (std::map<mcut::fd_t, mcut::fd_t>::const_iterator it = childFaceToBirthFace.cbegin();
                             it != childFaceToBirthFace.cend();
                             ++it) {
                            if (it->second == birth_face) { // matching birth face ?
                                faces_from_partitioned_birth_face.push_back(it->first);
                            }
                        }

                        bool haveFaceIntersectingFP = false;
                        // Should it be the case that we must proceed to make [another] partition of the
                        // birth-face, then "faceContainingFP" represent the existing face (a child of the birth face)
                        // in which the current floating polygon lies.
                        mcut::fd_t faceContainingFP = mcut::mesh_t::null_face();

                        // for each face sharing a birth face with origin_face
                        for (std::vector<mcut::fd_t>::const_iterator it = faces_from_partitioned_birth_face.cbegin();
                             it != faces_from_partitioned_birth_face.cend();
                             ++it) {

                            mcut::fd_t face = *it;

                            // ::::::::::::::::::::::
                            // get face vertex coords
                            const std::vector<mcut::vd_t> faceVertexDescriptors = origin_input_mesh->get_vertices_around_face(face);
                            std::vector<mcut::math::vec3> faceVertexCoords3D(faceVertexDescriptors.size());

                            for (std::vector<mcut::vd_t>::const_iterator i = faceVertexDescriptors.cbegin(); i != faceVertexDescriptors.cend(); ++i) {
                                const size_t idx = std::distance(faceVertexDescriptors.cbegin(), i);
                                const mcut::math::vec3& coords = origin_input_mesh->vertex(*i);
                                faceVertexCoords3D[idx] = coords;
                            }

                            // :::::::::::::::::::::::::
                            // project face coords to 2D
                            std::vector<mcut::math::vec2> faceVertexCoords2D;

                            mcut::geom::project2D(faceVertexCoords2D, faceVertexCoords3D.data(), (int)faceVertexCoords3D.size(), fpi.origin_face_normal_largest_comp);

                            const int numFaceEdges = (int)faceVertexDescriptors.size(); // num edges == num verts
                            const int numFaceVertices = numFaceEdges;

                            // for each edge of face
                            for (int edgeIter = 0; edgeIter < numFaceEdges; ++edgeIter) {

                                const mcut::math::vec2& faceEdgeV0 = faceVertexCoords2D.at(((size_t)edgeIter) + 0);
                                const mcut::math::vec2& faceEdgeV1 = faceVertexCoords2D.at((((size_t)edgeIter) + 1) % numFaceVertices);

                                // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                                // Does the current edge of "face" intersect/pass through the area of
                                // the current floating polygon?

                                bool haveEdgeIntersectingFP = false;

                                // for each edge of current floating poly
                                for (int fpEdgeIter = 0; fpEdgeIter < fpEdgeCount; ++fpEdgeIter) {

                                    const mcut::math::vec2& fpEdgeV0 = fpVertexCoords2D.at(((size_t)fpEdgeIter) + 0);
                                    const mcut::math::vec2& fpEdgeV1 = fpVertexCoords2D.at((((size_t)fpEdgeIter) + 1) % fpVertexCount);

                                    // placeholders
                                    mcut::math::real_number_t _1; // unused
                                    mcut::math::real_number_t _2; // unused
                                    mcut::math::vec2 _3; // unused

                                    const char result = mcut::geom::compute_segment_intersection(faceEdgeV0, faceEdgeV1, fpEdgeV0, fpEdgeV1, _3, _1, _2);

                                    if (result == '1') { // implies a propery segment-segment intersection
                                        haveEdgeIntersectingFP = true;
                                        break;
                                    }
                                }

                                if (haveEdgeIntersectingFP == false && faceContainingFP == mcut::mesh_t::null_face()) {
                                    // here we also do a test to find if the current face actually contains
                                    // the floating polygon in its area. We will need this information in order to
                                    // know the correct birth-face child-face that will be further partitioned
                                    // so as to prevent the current floating polygon from coming up again in the
                                    // next dispatch call.

                                    // for each floating polygon vertex ...
                                    for (int fpVertIter = 0; fpVertIter < (int)fpVertexCoords2D.size(); ++fpVertIter) {
                                        const char ret = mcut::geom::compute_point_in_polygon_test(fpVertexCoords2D.at(fpVertIter), faceVertexCoords2D.data(), (int)faceVertexCoords2D.size());
                                        if (ret == 'i') { // check if strictly interior
                                            faceContainingFP = *it;
                                            break;
                                        }
                                    }
                                }

                                if (haveEdgeIntersectingFP) {
                                    haveFaceIntersectingFP = true;
                                    break;
                                }
                            } // for (std::vector<mcut::hd_t>::const_iterator hIt = halfedges.cbegin(); ...

                            if (haveFaceIntersectingFP) {
                                break; // done
                            }

                        } // for (std::vector<mcut::fd_t>::const_iterator it = faces_from_partitioned_birth_face.cbegin(); ...

                        // i.e. there exists no partitioning-edge which passes through the current floating polygon
                        mustPartitionCurrentFace = (haveFaceIntersectingFP == false);

                        if (mustPartitionCurrentFace) {
                            // update which face we treat as "origin_face" i.e. the one that we will partition
                            MCUT_ASSERT(faceContainingFP != mcut::mesh_t::null_face());
                            origin_face = faceContainingFP;
                        }

                    } // if (birth_face_partitioned_atleast_once) {
                    else {
                        birth_face = origin_face;
                    }

                    if (!mustPartitionCurrentFace) {
                        // skip current floating polygon no need to partition "origin_face" this time
                        // because an already-added edge into "origin_input_mesh" will prevent the current
                        // floating polygon from arising
                        continue; // got to next floating polygon
                    }

                    // gather vertices of "origin_face" (descriptors and 3d coords)

                    //std::vector<mcut::vd_t> originFaceVertexDescriptors = origin_input_mesh->get_vertices_around_face(origin_face);
                    std::vector<mcut::math::vec3> originFaceVertices3d;
                    // get information about each edge (used by "origin_face") that needs to be split along the respective intersection point
                    const std::vector<mcut::hd_t>& origFaceHalfedges = origin_input_mesh->get_halfedges_around_face(origin_face);

                    for (std::vector<mcut::hd_t>::const_iterator i = origFaceHalfedges.cbegin(); i != origFaceHalfedges.cend(); ++i) {
                        const mcut::vd_t src = origin_input_mesh->source(*i); // NOTE: we use source so that edge iterators/indices match with internal mesh storage
                        originFaceVertices3d.push_back(origin_input_mesh->vertex(src));
                    }

                    MCUT_ASSERT(fpi.origin_face_normal_largest_comp != -1); // should be defined when we identify the floating polygon in the kernel

                    // project the "origin_face" to 2D
                    // Since the geometry operations we are concerned about are inherently in 2d, here we project
                    // our coords from 3D to 2D. We project by eliminating the component corresponding
                    // to the "origin_face"'s normal vector's largest component. ("origin_face" and our
                    // floating polygon have the same normal!)
                    //

                    std::vector<mcut::math::vec2> originFaceVertexCoords2D;
                    mcut::geom::project2D(originFaceVertexCoords2D, originFaceVertices3d.data(), (int)originFaceVertices3d.size(), fpi.origin_face_normal_largest_comp);

                    // ROUGH STEPS TO COMPUTE THE LINE THAT WILL BE USED TO PARTITION origin_face
                    // 1. pick two edges in the floating polygon
                    // 2. compute their mid-points
                    // 3. construct a [segment] with these two mid-points
                    // 4. if any vertex of the floating-poly is on the [line] defined by the segment OR
                    //  ... if any vertex of the origin_face on the [line] defined by the segment:
                    //  --> GOTO step 1 and select another pair of edges in the floating poly
                    // 5. construct a ray with the segment whose origin lies outside origin_face
                    // 6. intersect the ray with all edges of origin_face, and keep the intersection points [on the boundary] of origin_face
                    // 7. compute mid-point of our segment (from the two mid-points in step 3)
                    // 8. Get the two closest intersection points to this mid-point of our segment
                    // 9. Partition origin_face using the two closest intersection points this mid-point
                    // 10. Likewise update the connectivity of neighbouring faces of origin_face
                    // --> Neighbours to update are inferred from the halfedges that are partitioned at the two intersection points
                    // 11. remove "origin_face" from "origin_input_mesh"
                    // 12. remove neighbours of "origin_face" from "origin_input_mesh" that shared the edge on which the two intersection points lie.
                    // 13. add the child_polygons of "origin_face" and the re-traced neighbours into "origin_input_mesh"
                    // 14.  store a mapping from newly traced polygons to the original (user provided) input mesh elements
                    // --> This will also be used client vertex- and face-data mapping.

                    auto fpGetEdgeVertexCoords = [&](const int fpEdgeIdx, mcut::math::vec2& fpEdgeV0, mcut::math::vec2& fpEdgeV1) {
                        const int fpFirstEdgeV0Idx = (((size_t)fpEdgeIdx) + 0);
                        fpEdgeV0 = fpVertexCoords2D.at(fpFirstEdgeV0Idx);
                        const int fpFirstEdgeV1Idx = (((size_t)fpEdgeIdx) + 1) % fpVertexCount;
                        fpEdgeV1 = fpVertexCoords2D.at(fpFirstEdgeV1Idx);
                    };

                    auto fpGetEdgeMidpoint = [&](int edgeIdx) {
                        mcut::math::vec2 edgeV0;
                        mcut::math::vec2 edgeV1;
                        fpGetEdgeVertexCoords(edgeIdx, edgeV0, edgeV1);

                        const mcut::math::vec2 midPoint(
                            (edgeV0.x() + edgeV1.x()) * .5, //
                            (edgeV0.y() + edgeV1.y()) * .5);

                        return midPoint;
                    };

                    auto fpGetMidpointDistance = [&](std::pair<int, int> edgePair) {
                        const mcut::math::vec2 edge0MidPoint = fpGetEdgeMidpoint(edgePair.first);
                        const mcut::math::vec2 edge1MidPoint = fpGetEdgeMidpoint(edgePair.second);
                        const mcut::math::real_number_t dist = mcut::math::squared_length(edge1MidPoint - edge0MidPoint);
                        return dist;
                    };

                    // NOTE: using max (i.e. < operator) lead to floating point precision issues on
                    // test 40. The only solution to which is exact arithmetic. However, since we still
                    // want MCUT to work even if the user only has fixed precision numbers.
                    // We pick edges based on this which are closest. No worries about colinear edges
                    // because they will be detected later and skipped!
                    auto fpMaxDistancePredicate = [&](std::pair<int, int> edgePairA, std::pair<int, int> edgePairB) -> bool {
                        const mcut::math::real_number_t aDist = fpGetMidpointDistance(edgePairA);
                        const mcut::math::real_number_t bDist = fpGetMidpointDistance(edgePairB);
                        return aDist < bDist;
                    };

                    std::priority_queue<
                        std::pair<int, int>, //
                        std::vector<std::pair<int, int>>, //
                        decltype(fpMaxDistancePredicate)>
                        fpEdgePairQueue(fpMaxDistancePredicate);

                    // populate queue with [unique] pairs of edges from the floating polygon
                    // priority is given to those pairs with the farthest distance between then
                    for (int i = 0; i < fpEdgeCount; ++i) {
                        for (int j = i + 1; j < fpEdgeCount; ++j) {
                            fpEdgePairQueue.push(std::make_pair(i, j));
                        }
                    }

                    MCUT_ASSERT(fpEdgePairQueue.size() >= 3); // we can have at least 3 pairs for the simplest polygon (triangle) i.e. assuming it is not generate

                    // In the next while loop, each iteration will attempt to contruct a line [passing through
                    // our floating polygon] that will be used partition "origin_face" .
                    // NOTE: the reason we have a while loop is because it allows us to test several possible lines
                    // with-which "origin_face" can be partitioned. Some lines may not usable because they pass through
                    // a vertex of the floating polygon or a vertex the "origin_face" - in which case GP will be
                    // violated.
                    //

                    bool haveSegmentOnFP = false; // the current pair of floating polygon edges worked!

                    // the line segment constructed from midpoints of two edges of the
                    // floating polygon
                    std::pair<mcut::math::vec2, mcut::math::vec2> fpSegment;

                    while (!fpEdgePairQueue.empty() && !haveSegmentOnFP) {

                        const std::pair<int, int> fpEdgePairCur = fpEdgePairQueue.top();
                        fpEdgePairQueue.pop();

                        const mcut::math::vec2 fpEdge0Midpoint = fpGetEdgeMidpoint(fpEdgePairCur.first);
                        const mcut::math::vec2 fpEdge1Midpoint = fpGetEdgeMidpoint(fpEdgePairCur.second);

                        // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                        // if the line intersects/passes through a vertex in "origin_face" or a vertex in
                        // the floating polygon then try another edge pair.

                        auto anyPointIsOnLine = [&](
                                                    const mcut::math::vec2& segStart,
                                                    const mcut::math::vec2& segEnd,
                                                    const std::vector<mcut::math::vec2>& polyVerts) -> bool {
                            for (std::vector<mcut::math::vec2>::const_iterator it = polyVerts.cbegin(); it != polyVerts.cend(); ++it) {
                                if (mcut::geom::collinear(segStart, segEnd, (*it))) {
                                    return true;
                                }
                            }
                            return false;
                        }; // end lambda

                        // do we have general position? i.e. line segment does not pass through a vertex of the
                        // floating polygon and "origin_face"
                        bool haveGPOnFP = !anyPointIsOnLine(fpEdge0Midpoint, fpEdge1Midpoint, fpVertexCoords2D);
                        bool haveGPOnOriginFace = !anyPointIsOnLine(fpEdge0Midpoint, fpEdge1Midpoint, originFaceVertexCoords2D);
                        bool haveGP = haveGPOnFP && haveGPOnOriginFace;

                        if (haveGP) {
                            haveSegmentOnFP = true;
                            fpSegment.first = fpEdge1Midpoint;
                            fpSegment.second = fpEdge0Midpoint;
                        }

                    } // while (fpEdgePairQueue.size() > 0 && successivelyPartitionedOriginFaceWithCurrentEdgePair == false) {

                    if (!haveSegmentOnFP) {
                        // OH OH!
                        // You have encountered an extremely rare problem case.
                        // Email the developers (there is a solution but it requires numerical perturbation on "fpSegment").
                        result = McResult::MC_INVALID_OPERATION;

                        ctxtPtr->log(
                            McDebugSource::MC_DEBUG_SOURCE_KERNEL,
                            McDebugType::MC_DEBUG_TYPE_ERROR,
                            0,
                            McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
                            "Floating-polygon partitioning step could not find a usable fpSegment");

                        return result;
                    }

                    // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // At this point we have a valid line segment with which we can proceed to
                    // partition the "origin_mesh".

                    // Now we compute intersection points between every edge in "origin_face" and
                    // our segment (treating "fpSegment" as an infinitely-long line)

                    const int originFaceVertexCount = (int)originFaceVertices3d.size();
                    MCUT_ASSERT(originFaceVertexCount >= 3);
                    const int originFaceEdgeCount = originFaceVertexCount;

                    // this maps stores the intersection points between our line segment and the
                    // edges of "origin_face"
                    std::vector<
                        // information about "origin_face" edge that is to be split
                        std::pair<
                            // index of an edge (i.e. index in the halfedge-list, where this list is defined w.r.t the
                            // order of halfedges_around_face(origin_face)
                            int,
                            std::pair<
                                mcut::math::vec2, // intersection point coords
                                mcut::math::real_number_t //  parameter value (t) of intersection point along our edge (used to recover 3D coords)
                                >>>
                        originFaceIntersectedEdgeInfo;

                    // *************************************************************************************************************
                    // NOTE: "origFaceEdgeIter==0" corresponds to the second halfedge in the list returned by
                    // "get_halfedges_around_face(origin_face)".
                    // This is because "get_halfedges_around_face" builds the list of vertices by storing the target (not source) of
                    // each halfedge of a given face.
                    // *************************************************************************************************************

                    // for each edge in "origin_face"
                    for (int origFaceEdgeIter = 0; origFaceEdgeIter < originFaceEdgeCount; ++origFaceEdgeIter) {

                        const mcut::math::vec2& origFaceEdgeV0 = originFaceVertexCoords2D.at(((size_t)origFaceEdgeIter) + 0);
                        const mcut::math::vec2& origFaceEdgeV1 = originFaceVertexCoords2D.at(((origFaceEdgeIter) + 1) % originFaceVertexCount);

                        const mcut::math::real_number_t garbageVal(0xdeadbeef);
                        mcut::math::vec2 intersectionPoint(garbageVal);

                        mcut::math::real_number_t origFaceEdgeParam;
                        mcut::math::real_number_t fpEdgeParam;

                        char intersectionResult = mcut::geom::compute_segment_intersection(
                            origFaceEdgeV0, origFaceEdgeV1, fpSegment.first, fpSegment.second, intersectionPoint, origFaceEdgeParam, fpEdgeParam);

                        // These assertion must hold since, by construction, "fpSegment" (computed from two edges
                        // of the floating polygon) partitions the floating polygon which lies inside the area
                        // of "origin_face".
                        // Thus "fpSegment" can never intersect any half|edge/segment of "origin_face". It is the
                        // infinite-line represented by the "fpSegment" that can intersect edges of "origin_face".
                        MCUT_ASSERT(intersectionResult != '1'); // implies segment-segment intersection
                        MCUT_ASSERT(intersectionResult != 'v'); // implies that at-least one vertex of one segment touches the other
                        MCUT_ASSERT(intersectionResult != 'e'); // implies that segments collinearly overlap

                        if (
                            // intersection point was successively computed i.e. the infinite-line of "fpSegment" intersected the edge of "origin_face" (somewhere including outside of "origin_face")
                            (intersectionPoint.x() != garbageVal && intersectionPoint.y() != garbageVal) &&
                            // no actual segment-segment intersection exists, which is what we want
                            intersectionResult == '0') {
                            originFaceIntersectedEdgeInfo.push_back(std::make_pair(origFaceEdgeIter, std::make_pair(intersectionPoint, origFaceEdgeParam)));
                        }
                    } // for (int origFaceEdgeIter = 0; origFaceEdgeIter < originFaceEdgeCount; ++origFaceEdgeIter) {

                    // compute mid-point of "fpSegment", which we will used to find closest intersection points

                    const mcut::math::vec2 fpSegmentMidPoint(
                        (fpSegment.first.x() + fpSegment.second.x()) * 0.5, //
                        (fpSegment.first.y() + fpSegment.second.y()) * 0.5);

                    // Get the two closest [valid] intersection points to "fpSegmentMidPoint".
                    // We do this by sorting elements of "originFaceIntersectedEdgeInfo" by the distance
                    // of their respective intersection point from "fpSegmentMidPoint". We skip intersection
                    // points that do not lie on an edge of "origin_face" because they introduce ambiguities
                    // and that they are technically not usable (i.e. they are outside "origin_face").

                    std::sort(originFaceIntersectedEdgeInfo.begin(), originFaceIntersectedEdgeInfo.end(),
                        [&](std::pair<int, std::pair<mcut::math::vec2, mcut::math::real_number_t>>& a, //
                            std::pair<int, std::pair<mcut::math::vec2, mcut::math::real_number_t>>& b) {
                            mcut::math::real_number_t aDist(std::numeric_limits<double>::max()); // bias toward points inside polygon
                            //char aOnEdge = mcut::geom::compute_point_in_polygon_test(
                            //    a.second.first,
                            //    originFaceVertexCoords2D.data(),
                            //    (int)originFaceVertexCoords2D.size());

                            bool aOnEdge = false;
                            for (int i = 0; i < (int)originFaceVertexCoords2D.size(); ++i) {
                                int i0 = i;
                                int i1 = (i0 + 1) % (int)originFaceVertexCoords2D.size();
                                if (mcut::geom::collinear(originFaceVertexCoords2D[i0], originFaceVertexCoords2D[i1], a.second.first)) {
                                    aOnEdge = true;
                                    break;
                                }
                            }

                            if (aOnEdge) {
                                const mcut::math::vec2 aVec = a.second.first - fpSegmentMidPoint;
                                aDist = mcut::math::squared_length(aVec);
                            }

                            mcut::math::real_number_t bDist(std::numeric_limits<double>::max());
                            //char bOnEdge = mcut::geom::compute_point_in_polygon_test(
                            //    b.second.first,
                            //    originFaceVertexCoords2D.data(),
                            //    (int)originFaceVertexCoords2D.size());
                            bool bOnEdge = false;
                            for (int i = 0; i < (int)originFaceVertexCoords2D.size(); ++i) {
                                int i0 = i;
                                int i1 = (i0 + 1) % (int)originFaceVertexCoords2D.size();
                                if (mcut::geom::collinear(originFaceVertexCoords2D[i0], originFaceVertexCoords2D[i1], b.second.first)) {
                                    bOnEdge = true;
                                    break;
                                }
                            }

                            if (bOnEdge) {
                                const mcut::math::vec2 bVec = b.second.first - fpSegmentMidPoint;
                                bDist = mcut::math::squared_length(bVec);
                            }

                            return aDist > bDist;
                        });

                    // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // At this point we have all information necessary to partition "origin_face" using
                    // the two closest intersection points to "fpSegmentMidPoint".
                    //

                    // this std::vector stores the faces that use an edge that will be partitioned
                    std::vector<mcut::fd_t> replaced_input_mesh_faces = { origin_face };

                    MCUT_ASSERT(originFaceIntersectedEdgeInfo.size() >= 2); // we partition atleast two edges of origin_face [always!]

                    // origFaceEdge0: This is the first edge in the list after sorting.
                    // ---------------------------------------------------------------

                    const std::pair<int, std::pair<mcut::math::vec2, mcut::math::real_number_t>>& originFaceIntersectedEdge0Info = originFaceIntersectedEdgeInfo[0]; // first elem
                    const int origFaceEdge0Idx = originFaceIntersectedEdge0Info.first;
                    const mcut::math::real_number_t& origFaceEdge0IntPointEqnParam = originFaceIntersectedEdge0Info.second.second;

                    // NOTE: minus-1 since "get_vertices_around_face(origin_face)" builds a list using halfedge target vertices
                    // See the starred note above
                    int halfedgeIdx = origFaceEdge0Idx; // mcut::wrap_integer(origFaceEdge0Idx - 1, 0, (int)originFaceEdgeCount - 1); //(origFaceEdge0Idx + 1) % originFaceEdgeCount;
                    const mcut::hd_t origFaceEdge0Halfedge = origFaceHalfedges.at(halfedgeIdx);
                    MCUT_ASSERT(origin_face == origin_input_mesh->face(origFaceEdge0Halfedge));
                    const mcut::ed_t origFaceEdge0Descr = origin_input_mesh->edge(origFaceEdge0Halfedge);
                    const mcut::vd_t origFaceEdge0HalfedgeSrcDescr = origin_input_mesh->source(origFaceEdge0Halfedge);
                    const mcut::vd_t origFaceEdge0HalfedgeTgtDescr = origin_input_mesh->target(origFaceEdge0Halfedge);

                    // query src and tgt coords and build edge vector (i.e. "tgt - src"), which is in 3d
                    const mcut::math::vec3& origFaceEdge0HalfedgeSrc = origin_input_mesh->vertex(origFaceEdge0HalfedgeSrcDescr);
                    const mcut::math::vec3& origFaceEdge0HalfedgeTgt = origin_input_mesh->vertex(origFaceEdge0HalfedgeTgtDescr);

                    // infer 3D intersection point along edge using "origFaceEdge0IntPointEqnParam"
                    const mcut::math::vec3 origFaceEdge0Vec = (origFaceEdge0HalfedgeTgt - origFaceEdge0HalfedgeSrc);
                    const mcut::math::vec3 origFaceEdge0IntPoint3d = origFaceEdge0HalfedgeSrc + (origFaceEdge0Vec * origFaceEdge0IntPointEqnParam);

                    const mcut::hd_t origFaceEdge0HalfedgeOpp = origin_input_mesh->opposite(origFaceEdge0Halfedge);
                    const mcut::fd_t origFaceEdge0HalfedgeOppFace = origin_input_mesh->face(origFaceEdge0HalfedgeOpp);

                    if (origFaceEdge0HalfedgeOppFace != mcut::mesh_t::null_face()) { // exists
                        // this check is needed in the case that both partitioned edges in "origin_face"
                        // are incident to the same two faces
                        const bool contained = std::find(replaced_input_mesh_faces.cbegin(), replaced_input_mesh_faces.cend(), origFaceEdge0HalfedgeOppFace) != replaced_input_mesh_faces.cend();
                        if (!contained) {
                            replaced_input_mesh_faces.push_back(origFaceEdge0HalfedgeOppFace);
                        }
                    }

                    // origFaceEdge1: This is the second edge in the list after sorting.
                    // ---------------------------------------------------------------

                    const std::pair<int, std::pair<mcut::math::vec2, mcut::math::real_number_t>>& originFaceIntersectedEdge1Info = originFaceIntersectedEdgeInfo[1]; // second elem
                    const int origFaceEdge1Idx = originFaceIntersectedEdge1Info.first;
                    const mcut::math::real_number_t& origFaceEdge1IntPointEqnParam = originFaceIntersectedEdge1Info.second.second;

                    halfedgeIdx = origFaceEdge1Idx; /// mcut::wrap_integer(origFaceEdge1Idx - 1, 0, (int)originFaceEdgeCount - 1); // (origFaceEdge1Idx + 1) % originFaceEdgeCount;
                    const mcut::hd_t origFaceEdge1Halfedge = origFaceHalfedges.at(halfedgeIdx);
                    MCUT_ASSERT(origin_face == origin_input_mesh->face(origFaceEdge1Halfedge));
                    const mcut::ed_t origFaceEdge1Descr = origin_input_mesh->edge(origFaceEdge1Halfedge);
                    const mcut::vd_t origFaceEdge1HalfedgeSrcDescr = origin_input_mesh->source(origFaceEdge1Halfedge);
                    const mcut::vd_t origFaceEdge1HalfedgeTgtDescr = origin_input_mesh->target(origFaceEdge1Halfedge);

                    // query src and tgt positions and build vector tgt - src
                    const mcut::math::vec3& origFaceEdge1HalfedgeSrc = origin_input_mesh->vertex(origFaceEdge1HalfedgeSrcDescr);
                    const mcut::math::vec3& origFaceEdge1HalfedgeTgt = origin_input_mesh->vertex(origFaceEdge1HalfedgeTgtDescr);

                    // infer intersection point in 3d using "origFaceEdge0IntPointEqnParam"
                    const mcut::math::vec3 origFaceEdge1Vec = (origFaceEdge1HalfedgeTgt - origFaceEdge1HalfedgeSrc);
                    const mcut::math::vec3 origFaceEdge1IntPoint3d = origFaceEdge1HalfedgeSrc + (origFaceEdge1Vec * origFaceEdge1IntPointEqnParam);

                    const mcut::hd_t origFaceEdge1HalfedgeOpp = origin_input_mesh->opposite(origFaceEdge1Halfedge);
                    const mcut::fd_t origFaceEdge1HalfedgeOppFace = origin_input_mesh->face(origFaceEdge1HalfedgeOpp);

                    if (origFaceEdge1HalfedgeOppFace != mcut::mesh_t::null_face()) { // exists
                        const bool contained = std::find(replaced_input_mesh_faces.cbegin(), replaced_input_mesh_faces.cend(), origFaceEdge1HalfedgeOppFace) != replaced_input_mesh_faces.cend();
                        if (!contained) {
                            replaced_input_mesh_faces.push_back(origFaceEdge1HalfedgeOppFace);
                        }
                    }

                    // gather halfedges of each neighbouring face of "origin_face" that is to be replaced
                    std::map<mcut::fd_t, std::vector<mcut::hd_t>> replacedOrigFaceNeighbourToOldHalfedges;

                    for (std::vector<mcut::fd_t>::const_iterator it = replaced_input_mesh_faces.cbegin(); it != replaced_input_mesh_faces.cend(); ++it) {
                        if (*it == origin_face) {
                            continue;
                        }
                        replacedOrigFaceNeighbourToOldHalfedges[*it] = origin_input_mesh->get_halfedges_around_face(*it);
                    }

                    // :::::::::::::::::::::::::::::::::::::::::::::::::::::
                    //** add new intersection points into origin_input_mesh
                    const mcut::vd_t origFaceEdge0IntPoint3dDescr = origin_input_mesh->add_vertex(origFaceEdge0IntPoint3d);
                    const mcut::vd_t origFaceEdge1IntPoint3dDescr = origin_input_mesh->add_vertex(origFaceEdge1IntPoint3d);

                    // :::::::::::
                    //** add edges

                    // halfedge between the intersection points
                    const mcut::hd_t intPointHalfedgeDescr = origin_input_mesh->add_edge(origFaceEdge0IntPoint3dDescr, origFaceEdge1IntPoint3dDescr);

                    // partitioning edges for origFaceEdge0
                    const mcut::hd_t origFaceEdge0FirstNewHalfedgeDescr = origin_input_mesh->add_edge(origFaceEdge0HalfedgeSrcDescr, origFaceEdge0IntPoint3dDescr); // o --> x
                    const mcut::hd_t origFaceEdge0SecondNewHalfedgeDescr = origin_input_mesh->add_edge(origFaceEdge0IntPoint3dDescr, origFaceEdge0HalfedgeTgtDescr); // x --> o

                    // partitioning edges for origFaceEdge1
                    const mcut::hd_t origFaceEdge1FirstNewHalfedgeDescr = origin_input_mesh->add_edge(origFaceEdge1HalfedgeSrcDescr, origFaceEdge1IntPoint3dDescr); // o--> x
                    const mcut::hd_t origFaceEdge1SecondNewHalfedgeDescr = origin_input_mesh->add_edge(origFaceEdge1IntPoint3dDescr, origFaceEdge1HalfedgeTgtDescr); // x --> o

                    // We will now re-trace the face that are incident to the partitioned edges to create
                    // new faces.
                    std::map<mcut::fd_t, std::vector<mcut::hd_t>> replacedOrigFaceNeighbourToNewHalfedges;

                    // NOTE: first we retrace the neighbouring polygons that shared a partitioned edge with "origin_face".
                    // These are somewhat easier to deal with first because a fixed set of steps can be followed with a simple for-loop.

                    // for each neighbouring face (w.r.t. "origin_face") to be replaced
                    for (std::map<mcut::fd_t, std::vector<mcut::hd_t>>::const_iterator i = replacedOrigFaceNeighbourToOldHalfedges.cbegin();
                         i != replacedOrigFaceNeighbourToOldHalfedges.cend();
                         ++i) {

                        mcut::fd_t face = i->first;
                        MCUT_ASSERT(face != origin_face); // avoid complex case here, where we need to partition the polygon in two. We'll handle that later.

                        const std::vector<mcut::hd_t>& oldHalfedges = i->second;

                        // for each halfedge of face
                        for (std::vector<mcut::hd_t>::const_iterator j = oldHalfedges.cbegin(); j != oldHalfedges.cend(); ++j) {

                            const mcut::hd_t oldHalfedge = *j;
                            mcut::hd_t newHalfedge = mcut::mesh_t::null_halfedge();
                            const mcut::ed_t oldHalfedgeEdge = origin_input_mesh->edge(oldHalfedge);

                            // is the halfedge part of an edge that is to be partitioned...?

                            if (oldHalfedgeEdge == origFaceEdge0Descr) {
                                mcut::hd_t firstNewHalfedge = origin_input_mesh->opposite(origFaceEdge0SecondNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(firstNewHalfedge);
                                mcut::hd_t secondNewHalfedge = origin_input_mesh->opposite(origFaceEdge0FirstNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(secondNewHalfedge);

                            } else if (oldHalfedgeEdge == origFaceEdge1Descr) {
                                mcut::hd_t firstNewHalfedge = origin_input_mesh->opposite(origFaceEdge1SecondNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(firstNewHalfedge);
                                mcut::hd_t secondNewHalfedge = origin_input_mesh->opposite(origFaceEdge1FirstNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(secondNewHalfedge);
                            } else {
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(oldHalfedge); // maintain unpartitioned halfedge
                            }
                        } // for (std::vector<mcut::hd_t>::const_iterator j = oldHalfedges.cbegin(); j != oldHalfedges.cend(); ++j) {

                        // remove neighbour face
                        origin_input_mesh->remove_face(i->first);

                        // immediately add the updated tracing of the neighbour face so that it maintains the same desciptor!
                        std::vector<mcut::vd_t> faceVertices;
                        for (std::vector<mcut::hd_t>::const_iterator it = replacedOrigFaceNeighbourToNewHalfedges[face].cbegin();
                             it != replacedOrigFaceNeighbourToNewHalfedges[face].cend(); ++it) {
                            const mcut::vd_t tgt = origin_input_mesh->target(*it);
                            faceVertices.push_back(tgt);
                        }

                        const mcut::fd_t fdescr = origin_input_mesh->add_face(faceVertices);
                        MCUT_ASSERT(fdescr == i->first);

#if 0
                        std::map<mcut::fd_t, mcut::fd_t>::const_iterator fiter = childFaceToBirthFace.find(fdescr);

                        bool descrIsMapped = (fiter != childFaceToBirthFace.cend());

                        if (!descrIsMapped) {
                            childFaceToBirthFace[fdescr] = birth_face;
                        }
#endif
                    } // for (std::map<mcut::fd_t, std::vector<mcut::hd_t>>::const_iterator i = replacedOrigFaceNeighbourToOldHalfedges.cbegin(); i != replacedOrigFaceNeighbourToOldHalfedges.cend(); ++i) {

                    // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // Here we now handle the complex case where we need to partition
                    // "origin_face" in two new faces.

                    origin_input_mesh->remove_face(origin_face); // one free slot

                    // This queue contains the halfegdes that we'll start to trace our new faces from
                    // (those connected to our new intersection points)
                    std::queue<mcut::hd_t> origFaceiHalfedges;
                    origFaceiHalfedges.push(intPointHalfedgeDescr);
                    origFaceiHalfedges.push(origin_input_mesh->opposite(intPointHalfedgeDescr));

                    // this list containing all halfedges along the boundary of "origin_face"
                    std::vector<mcut::hd_t> origFaceBoundaryHalfdges = { // first add the new boundary-edge partitioning halfedges, since we already know them
                        origFaceEdge0FirstNewHalfedgeDescr,
                        origFaceEdge0SecondNewHalfedgeDescr,
                        origFaceEdge1FirstNewHalfedgeDescr,
                        origFaceEdge1SecondNewHalfedgeDescr
                    };

                    // .... now we add the remaining boundary halfedges of "origin_face" i.e. those not partitioneds
                    for (std::vector<mcut::hd_t>::const_iterator it = origFaceHalfedges.cbegin(); it != origFaceHalfedges.cend(); ++it) {
                        if (*it != origFaceEdge0Halfedge && *it != origFaceEdge1Halfedge) { // if its not one of the replaced/partitioned halfedges
                            origFaceBoundaryHalfdges.push_back(*it);
                        }
                    }

                    // here we will store the tracing of the two child polygons that result from partitioning "origin_face"
                    std::vector<std::vector<mcut::hd_t>> origFaceChildPolygons;

                    do { // each iteration will trace a child polygon
                        mcut::hd_t childPolyHE_cur = mcut::mesh_t::null_halfedge();
                        mcut::hd_t childPolyHE_next = origFaceiHalfedges.front(); // start
                        origFaceiHalfedges.pop();

                        origFaceChildPolygons.push_back(std::vector<mcut::hd_t>());
                        std::vector<mcut::hd_t>& origFaceChildPoly = origFaceChildPolygons.back();

                        const mcut::hd_t firstHalfedge = childPolyHE_next;
                        const mcut::vd_t firstHalfedgeSrc = origin_input_mesh->source(firstHalfedge);

                        do {
                            childPolyHE_cur = childPolyHE_next;
                            origFaceChildPoly.push_back(childPolyHE_cur);
                            const mcut::vd_t childPolyHE_curTgt = origin_input_mesh->target(childPolyHE_cur);
                            childPolyHE_cur = mcut::mesh_t::null_halfedge();
                            childPolyHE_next = mcut::mesh_t::null_halfedge();

                            if (childPolyHE_curTgt != firstHalfedgeSrc) {
                                // find next halfedge to continue building the current child polygon
                                std::vector<mcut::hd_t>::const_iterator fiter = std::find_if(origFaceBoundaryHalfdges.cbegin(), origFaceBoundaryHalfdges.cend(),
                                    [&](const mcut::hd_t h) { // find a boundary halfedge that can be connected to the current halfedge
                                        const mcut::vd_t src = origin_input_mesh->source(h);
                                        return src == childPolyHE_curTgt;
                                    });

                                MCUT_ASSERT(fiter != origFaceBoundaryHalfdges.cend());

                                childPolyHE_next = *fiter;
                            }

                        } while (childPolyHE_next != mcut::mesh_t::null_halfedge());

                        MCUT_ASSERT(origFaceChildPoly.size() >= 3); // minimum size of valid polygon (triangle)

                        // Add child face into mesh
                        std::vector<mcut::vd_t> origFaceChildPolyVertices;

                        for (std::vector<mcut::hd_t>::const_iterator hIt = origFaceChildPoly.cbegin(); hIt != origFaceChildPoly.cend(); ++hIt) {
                            const mcut::vd_t tgt = origin_input_mesh->target(*hIt);
                            origFaceChildPolyVertices.push_back(tgt);
                        }

                        const mcut::fd_t fdescr = origin_input_mesh->add_face(origFaceChildPolyVertices);
                        MCUT_ASSERT(fdescr != mcut::mesh_t::null_face());

                        if (origFaceChildPolygons.size() == 1) {
                            // the first child face will re-use the descriptor of "origin_face".
                            MCUT_ASSERT(fdescr == origin_face);
                        }

                        childFaceToBirthFace[fdescr] = birth_face;

                    } while (origFaceiHalfedges.empty() == false);

                    MCUT_ASSERT(origFaceChildPolygons.size() == 2); // "origin_face" shall only ever be partition into two child polygons

                    // remove the partitioned/'splitted' edges
                    origin_input_mesh->remove_edge(origFaceEdge0Descr);
                    origin_input_mesh->remove_edge(origFaceEdge1Descr);

                } // for (std::vector<mcut::floating_polygon_info_t>::const_iterator psFaceFloatingPolyIter = detected_floating_polygons_iter->second.cbegin(); ...
            } // for (std::vector<mcut::floating_polygon_info_t>::const_iterator detected_floating_polygons_iter = backendOutput.detected_floating_polygons.cbegin(); ...

            // ::::::::::::::::::::::::::::::::::::::::::::
            // rebuild the BVH of "origin_input_mesh" again

            if (floating_polygon_was_on_srcmesh) {
                srcMeshBvhAABBs.clear();
                srcMeshBvhLeafNodeFaces.clear();
                constructOIBVH(srcMeshInternal, srcMeshBvhAABBs, srcMeshBvhLeafNodeFaces);
            }
            if (floating_polygon_was_on_cutmesh) {
                cutMeshBvhAABBs.clear();
                cutMeshBvhLeafNodeFaces.clear();
                constructOIBVH(cutMeshInternal, cutMeshBvhAABBs, cutMeshBvhLeafNodeFaces);
            }

            backendOutput.detected_floating_polygons.clear();
        } // if (floating_polygon_was_detected) {

#if defined(MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE)
        ///////////////////////////////////////////////////////////////////////////
        // generate BVH meshes
        ///////////////////////////////////////////////////////////////////////////

        if (input.verbose) {
            lg << "create BVH meshes\n";

            for (std::map<std::string, std::vector<geom::bounding_box_t<math::fast_vec3>>>::iterator mesh_bvh_iter = bvh_internal_nodes_array.begin();
                 mesh_bvh_iter != bvh_internal_nodes_array.end();
                 ++mesh_bvh_iter) {

                const std::string mesh_name = mesh_bvh_iter->first;
                const std::vector<geom::bounding_box_t<math::fast_vec3>>& internal_nodes_array = mesh_bvh_iter->second;
                const std::vector<std::pair<fd_t, unsigned int>>& leaf_nodes_array = bvh_leaf_nodes_array.at(mesh_name);
                const int real_leaf_node_count = (int)leaf_nodes_array.size();
                //const int bvh_real_node_count = bvh::get_ostensibly_implicit_bvh_size(real_leaf_node_count);
                const int leaf_level_index = bvh::get_leaf_level_from_real_leaf_count(real_leaf_node_count);

                mesh_t bvh_mesh;

                // internal levels
                for (int level_idx = 0; level_idx <= leaf_level_index; ++level_idx) {

                    const int rightmost_real_leaf = bvh::get_rightmost_real_leaf(leaf_level_index, real_leaf_node_count);
                    const int rightmost_real_node_on_level = bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, level_idx);
                    const int leftmost_real_node_on_level = bvh::get_level_leftmost_node(level_idx);
                    const int number_of_real_nodes_on_level = (rightmost_real_node_on_level - leftmost_real_node_on_level) + 1;

                    for (int level_node_idx_iter = 0; level_node_idx_iter < number_of_real_nodes_on_level; ++level_node_idx_iter) {

                        geom::bounding_box_t<math::fast_vec3> node_bbox;
                        const bool is_leaf_level = (level_idx == leaf_level_index);

                        if (is_leaf_level) {
                            const fd_t leaf_node_face = leaf_nodes_array.at(level_node_idx_iter).first;
                            //node_bbox = ps_face_bboxes.at(leaf_node_face);
                            node_bbox.expand(ps_face_bboxes.at(leaf_node_face));
                        } else {
                            const int node_implicit_idx = leftmost_real_node_on_level + level_node_idx_iter;
                            const int node_memory_idx = bvh::get_node_mem_index(
                                node_implicit_idx,
                                leftmost_real_node_on_level,
                                0,
                                rightmost_real_node_on_level);
                            node_bbox.expand(internal_nodes_array.at(node_memory_idx));
                        }

                        std::vector<vd_t> node_bbox_vertices = insert_bounding_box_mesh(bvh_mesh, node_bbox);
                    }
                }

                dump_mesh(bvh_mesh, (mesh_name + ".bvh").c_str());
            }

        } // if (input.verbose)
#endif // #if defined(MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE)

        // Evaluate BVHs to find polygon pairs that will be tested for intersection
        // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION, "Find potentially-intersecting polygons");

        std::vector<std::pair<mcut::fd_t, mcut::fd_t>> intersecting_sm_cm_face_pairs;

        intersectOIBVHs(intersecting_sm_cm_face_pairs, srcMeshBvhAABBs, srcMeshBvhLeafNodeFaces, cutMeshBvhAABBs, cutMeshBvhLeafNodeFaces);

        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION, "Polygon-pairs found = " + std::to_string(intersecting_sm_cm_face_pairs.size()));

        if (intersecting_sm_cm_face_pairs.empty()) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_OTHER, 0, McDebugSeverity::MC_DEBUG_SEVERITY_NOTIFICATION, "Mesh BVHs do not overlap.");
            return result;
        }

        backendInput.intersecting_sm_cm_face_pairs = &intersecting_sm_cm_face_pairs;

        // cut!
        // ----

        numSourceMeshFacesInLastDispatchCall = srcMeshInternal.number_of_faces();

        try {
            ctxtPtr->applyPrecisionAndRoundingModeSettings();
            mcut::dispatch(backendOutput, backendInput);
            ctxtPtr->revertPrecisionAndRoundingModeSettings();
        } catch (const std::exception* e) {
            fprintf(stderr, "fatal: exception caught : %s\n", e->what());
            result = McResult::MC_RESULT_MAX_ENUM;
        }
    } while (
        // general position voliation
        (backendOutput.status == mcut::status_t::GENERAL_POSITION_VIOLATION && backendInput.enforce_general_position) || //
        // kernel detected a floating polygon and we now need to re-partition the origin polygon (in src mesh or cut-mesh) and then restart the cut
        backendOutput.status == mcut::status_t::DETECTED_FLOATING_POLYGON);

    result = convert(backendOutput.status);

    if (result != McResult::MC_NO_ERROR) {

        ctxtPtr->log(
            McDebugSource::MC_DEBUG_SOURCE_KERNEL,
            McDebugType::MC_DEBUG_TYPE_ERROR,
            0,
            McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
            mcut::to_string(backendOutput.status) + " : " + backendOutput.logger.get_reason_for_failure());

        ctxtPtr->lastLoggedDebugDetail = backendOutput.logger.get_log_string();

        return result;
    }

    //
    // sealed (partially or completely) fragment connected components
    //

    for (std::map<mcut::connected_component_location_t, std::map<mcut::cut_surface_patch_location_t, std::vector<mcut::output_mesh_info_t>>>::const_iterator i = backendOutput.connected_components.cbegin();
         i != backendOutput.connected_components.cend();
         ++i) {

        for (std::map<mcut::cut_surface_patch_location_t, std::vector<mcut::output_mesh_info_t>>::const_iterator j = i->second.cbegin();
             j != i->second.cend();
             ++j) {

            //const mcut::cut_surface_patch_location_t& cut_surface_patch_location = j->first;
            const std::string cs_patch_loc_str = mcut::to_string(j->first);

            // The first connected compoenent may or may not have been sealed with atleast one polygon.
            // One example case is when [completely] cutting a source-mesh like the tetrahedron into two pieces using just one cut-mesh polygon.
            // In this case, some fragment(s) will have exactly two output instances i.e. "unsealed" and "completely sealed".
            // This is because there will only be one interior patch polygon, which when stitched leads to a complete seal, thus there are not partial stages of sealing.
            //
            bool first_cc_is_completely_sealed = j->second.size() == 1;
            std::vector<mcut::output_mesh_info_t>::const_iterator start_off = first_cc_is_completely_sealed ? j->second.cbegin() : j->second.cbegin() + 1;
            for (std::vector<mcut::output_mesh_info_t>::const_iterator k = start_off; k != j->second.cend(); ++k) {

                std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> frag = std::unique_ptr<McFragmentConnComp, void (*)(McConnCompBase*)>(new McFragmentConnComp, ccDeletorFunc<McFragmentConnComp>);
                McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(frag.get());
                ctxtPtr->connComps.emplace(clientHandle, std::move(frag));
                McFragmentConnComp* asFragPtr = dynamic_cast<McFragmentConnComp*>(ctxtPtr->connComps.at(clientHandle).get());
                asFragPtr->type = MC_CONNECTED_COMPONENT_TYPE_FRAGMENT;
                asFragPtr->fragmentLocation = convert(i->first);
                asFragPtr->patchLocation = convert(j->first);

                MCUT_ASSERT(asFragPtr->patchLocation != MC_PATCH_LOCATION_UNDEFINED);
                asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE;
                // Note: the last CC is always guarranteed to be fully sealed (see: "include_fragment_sealed_partial" in kernel)!
                //bool is_last_cc = (size_t)std::distance(j->second.cbegin(), k) == (size_t)(j->second.size() - 1);

                //if (is_last_cc) {

                //} else { // did the user tell us to keep partially sealed fragments??
                // MCUT_ASSERT(backendInput.keep_fragments_sealed_inside_exhaustive || backendInput.keep_fragments_sealed_outside_exhaustive);
                // asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_PARTIAL;
                //}

                // Personal note: Do not be tempted to just deal with unsealed fragments here.
                // Its not guarranteed that the first element of "j->second" is always completely unsealed! Only sometimes.
                // Thus, for simplicity we deal with unsealed fragments specifically below (next for-loop)

                halfedgeMeshToIndexArrayMesh(ctxtPtr, asFragPtr->indexArrayMesh, *k);
            }
        }
    }

    //
    // unsealed connected components
    //
    for (std::map<mcut::connected_component_location_t, std::vector<mcut::output_mesh_info_t>>::const_iterator i = backendOutput.unsealed_cc.cbegin();
         i != backendOutput.unsealed_cc.cend();
         ++i) { // for each cc location flag (above/below/undefined)

        for (std::vector<mcut::output_mesh_info_t>::const_iterator j = i->second.cbegin(); j != i->second.cend(); ++j) { // for each mesh

            std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> unsealedFrag = std::unique_ptr<McFragmentConnComp, void (*)(McConnCompBase*)>(new McFragmentConnComp, ccDeletorFunc<McFragmentConnComp>);
            McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(unsealedFrag.get());
            ctxtPtr->connComps.emplace(clientHandle, std::move(unsealedFrag));
            McFragmentConnComp* asFragPtr = dynamic_cast<McFragmentConnComp*>(ctxtPtr->connComps.at(clientHandle).get());
            asFragPtr->type = MC_CONNECTED_COMPONENT_TYPE_FRAGMENT;
            asFragPtr->fragmentLocation = convert(i->first);
            asFragPtr->patchLocation = McPatchLocation::MC_PATCH_LOCATION_UNDEFINED;
            asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_NONE;

            halfedgeMeshToIndexArrayMesh(ctxtPtr, asFragPtr->indexArrayMesh, *j);
        }
    }

    // inside patches
    const std::vector<mcut::output_mesh_info_t>& insidePatches = backendOutput.inside_patches[mcut::cut_surface_patch_winding_order_t::DEFAULT];

    for (std::vector<mcut::output_mesh_info_t>::const_iterator it = insidePatches.cbegin();
         it != insidePatches.cend();
         ++it) {

        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> patchConnComp = std::unique_ptr<McPatchConnComp, void (*)(McConnCompBase*)>(new McPatchConnComp, ccDeletorFunc<McPatchConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(patchConnComp.get());
        ctxtPtr->connComps.emplace(clientHandle, std::move(patchConnComp));
        McPatchConnComp* asPatchPtr = dynamic_cast<McPatchConnComp*>(ctxtPtr->connComps.at(clientHandle).get());
        asPatchPtr->type = MC_CONNECTED_COMPONENT_TYPE_PATCH;
        asPatchPtr->patchLocation = MC_PATCH_LOCATION_INSIDE;

        halfedgeMeshToIndexArrayMesh(ctxtPtr, asPatchPtr->indexArrayMesh, *it);
    }

    // outside patches
    const std::vector<mcut::output_mesh_info_t>& outsidePatches = backendOutput.outside_patches[mcut::cut_surface_patch_winding_order_t::DEFAULT];

    for (std::vector<mcut::output_mesh_info_t>::const_iterator it = outsidePatches.cbegin(); it != outsidePatches.cend(); ++it) {

        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> patchConnComp = std::unique_ptr<McPatchConnComp, void (*)(McConnCompBase*)>(new McPatchConnComp, ccDeletorFunc<McPatchConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(patchConnComp.get());
        ctxtPtr->connComps.emplace(clientHandle, std::move(patchConnComp));
        McPatchConnComp* asPatchPtr = dynamic_cast<McPatchConnComp*>(ctxtPtr->connComps.at(clientHandle).get());
        asPatchPtr->type = MC_CONNECTED_COMPONENT_TYPE_PATCH;
        asPatchPtr->patchLocation = MC_PATCH_LOCATION_OUTSIDE;

        halfedgeMeshToIndexArrayMesh(ctxtPtr, asPatchPtr->indexArrayMesh, *it);
    }

    // seams
    // -----------

    // NOTE: seamed meshes are available if there was no partial cut intersection (due to constraints imposed by halfedge construction rules).

    //  src mesh

    if (backendOutput.seamed_src_mesh.mesh.number_of_faces() > 0) {
        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> srcMeshSeam = std::unique_ptr<McSeamConnComp, void (*)(McConnCompBase*)>(new McSeamConnComp, ccDeletorFunc<McSeamConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(srcMeshSeam.get());
        ctxtPtr->connComps.emplace(clientHandle, std::move(srcMeshSeam));
        McSeamConnComp* asSrcMeshSeamPtr = dynamic_cast<McSeamConnComp*>(ctxtPtr->connComps.at(clientHandle).get());
        asSrcMeshSeamPtr->type = MC_CONNECTED_COMPONENT_TYPE_SEAM;
        asSrcMeshSeamPtr->origin = MC_SEAM_ORIGIN_SRCMESH;
        halfedgeMeshToIndexArrayMesh(ctxtPtr, asSrcMeshSeamPtr->indexArrayMesh, backendOutput.seamed_src_mesh);
    }

    //  cut mesh

    if (backendOutput.seamed_cut_mesh.mesh.number_of_faces() > 0) {
        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> cutMeshSeam = std::unique_ptr<McSeamConnComp, void (*)(McConnCompBase*)>(new McSeamConnComp, ccDeletorFunc<McSeamConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(cutMeshSeam.get());
        ctxtPtr->connComps.emplace(clientHandle, std::move(cutMeshSeam));
        McSeamConnComp* asCutMeshSeamPtr = dynamic_cast<McSeamConnComp*>(ctxtPtr->connComps.at(clientHandle).get());
        asCutMeshSeamPtr->type = MC_CONNECTED_COMPONENT_TYPE_SEAM;
        asCutMeshSeamPtr->origin = MC_SEAM_ORIGIN_CUTMESH;

        halfedgeMeshToIndexArrayMesh(ctxtPtr, asCutMeshSeamPtr->indexArrayMesh, backendOutput.seamed_cut_mesh);
    }

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    // for the caches and pools, in all threads where MPFR is potentially used
    mpfr_mp_memory_cleanup();
#endif

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcGetConnectedComponents(
    const McContext context,
    const McConnectedComponentType connectedComponentType,
    const uint32_t numEntries,
    McConnectedComponent* pConnComps,
    uint32_t* numConnComps)
{
    McResult result = McResult::MC_NO_ERROR;
    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }
    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if (connectedComponentType == 0) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid type-parameter");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    if (numConnComps == nullptr && pConnComps == nullptr) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "null parameter");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    if (numConnComps != nullptr) {
        (*numConnComps) = 0;
    }

    uint32_t gatheredConnCompCounter = 0;

    for (std::map<McConnectedComponent, std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)>>::const_iterator i = ctxtPtr->connComps.cbegin();
         i != ctxtPtr->connComps.cend();
         ++i) {

        //connectedComponentType const auto& connCompHandle = i.first;
        //if ((i->second->type & connectedComponentType)) {

        bool includeConnComp = (i->second->type & connectedComponentType) != 0;

        if (includeConnComp) {
            if (pConnComps == nullptr) // query number
            {
                (*numConnComps)++;
            } else // populate pConnComps
            {
                pConnComps[gatheredConnCompCounter] = i->first;
                gatheredConnCompCounter += 1;
                if (gatheredConnCompCounter == numEntries) {
                    break;
                }
            }
        }
        //}
    }

    return result;
}

McResult MCAPI_CALL mcGetConnectedComponentData(
    const McContext context,
    const McConnectedComponent connCompId,
    McFlags queryFlags,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes)
{
    McResult result = McResult::MC_NO_ERROR;

    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if (bytes != 0 && pMem == nullptr) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "null parameter");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    auto ccRef = ctxtPtr->connComps.find(connCompId);

    if (ccRef == ctxtPtr->connComps.cend()) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid connected component id");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    auto& ccData = ccRef->second;

    switch (queryFlags) {
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT: {
        if (pMem == nullptr) {
            *pNumBytes = sizeof(uint32_t);
        } else {
            if (bytes > sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(&ccData->indexArrayMesh.numVertices), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT: {
        const uint64_t allocatedBytes = ccData->indexArrayMesh.numVertices * sizeof(float) * 3;
        if (pMem == nullptr) {
            *pNumBytes = allocatedBytes;
        } else { // copy mem to client ptr

            if (bytes > allocatedBytes) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            } // if

            uint64_t off = 0;
            float* outPtr = reinterpret_cast<float*>(pMem);
            uint64_t nelems = (uint64_t)(bytes / sizeof(float));

            if (nelems % 3 != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            for (uint32_t i = 0; i < nelems; ++i) {
                const mcut::math::real_number_t& val = ccData->indexArrayMesh.pVertices[i];
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
                const float val_ = static_cast<float>(val);
#else
                const float val_ = static_cast<float>(val.to_double());
#endif
                memcpy(outPtr + off, reinterpret_cast<const void*>(&val_), sizeof(float));
                off += 1;
            }

            MCUT_ASSERT((off * sizeof(float)) == allocatedBytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE: {
        const uint64_t allocatedBytes = ccData->indexArrayMesh.numVertices * sizeof(double) * 3;
        if (pMem == nullptr) {
            *pNumBytes = allocatedBytes;
        } else { // copy mem to client ptr

            if (bytes > allocatedBytes) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            } // if

            uint64_t byteOffset = 0;
            double* outPtr = reinterpret_cast<double*>(pMem);

            uint64_t nelems = (uint64_t)(bytes / sizeof(double));

            if (nelems % 3 != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            //uint32_t verticesToCopy = (uint32_t)(nelems / 3);

            for (uint32_t i = 0; i < nelems; ++i) {
                const mcut::math::real_number_t& val = ccData->indexArrayMesh.pVertices[i];
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
                const double val_ = static_cast<double>(val);
#else
                const double val_ = static_cast<double>(val.to_double());
#endif
                memcpy(outPtr + byteOffset, reinterpret_cast<const void*>(&val_), sizeof(double));
                byteOffset += 1;
            }

            MCUT_ASSERT((byteOffset * sizeof(double)) == allocatedBytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_EXACT: { // arbitrary prec float string

        uint64_t allocatedBytes = 0;
        for (uint32_t i = 0; i < ccData->indexArrayMesh.numVertices * 3; ++i) {
            const mcut::math::real_number_t& val = ccData->indexArrayMesh.pVertices[i];

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
            allocatedBytes += (std::to_string(val) + " ").length(); // NOTE: we could cache results of tmp strings
#else
            allocatedBytes += (val.to_string() + " ").length();
#endif
        }

        MCUT_ASSERT(allocatedBytes > 0);

        if (pMem == nullptr) {
            *pNumBytes = allocatedBytes;
        } else {

            if (bytes > allocatedBytes) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            uint64_t byteOffset = 0;
            int8_t* outPtr = reinterpret_cast<int8_t*>(pMem);

            for (uint32_t i = 0; i < ccData->indexArrayMesh.numVertices * 3; ++i) {
                const mcut::math::real_number_t& val = ccData->indexArrayMesh.pVertices[i];

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
                const std::string val_ = std::to_string(val) + " ";
#else
                const std::string val_ = val.to_string() + " ";
#endif
                if (byteOffset + val_.length() > bytes) {
                    ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                    result = McResult::MC_INVALID_VALUE;
                    return result;
                }
                memcpy(outPtr + byteOffset, reinterpret_cast<const void*>(val_.c_str()), val_.length());
                byteOffset += val_.length();
            }

            MCUT_ASSERT(byteOffset == allocatedBytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_COUNT: {
        if (pMem == nullptr) {
            *pNumBytes = sizeof(uint32_t);
        } else {
            if (bytes > sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(uint32_t) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(&ccData->indexArrayMesh.numFaces), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE: {
        if (pMem == nullptr) {
            MCUT_ASSERT(ccData->indexArrayMesh.numFaceIndices > 0);
            *pNumBytes = ccData->indexArrayMesh.numFaceIndices * sizeof(uint32_t);
        } else {
            if (bytes > ccData->indexArrayMesh.numFaceIndices * sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(uint32_t) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            memcpy(pMem, reinterpret_cast<void*>(ccData->indexArrayMesh.pFaceIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_SIZE: { // non-triangulated only (don't want to store redundant information)
        if (pMem == nullptr) {
            *pNumBytes = ccData->indexArrayMesh.numFaces * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > ccData->indexArrayMesh.numFaces * sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(uint32_t) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            memcpy(pMem, reinterpret_cast<void*>(ccData->indexArrayMesh.pFaceSizes.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_EDGE_COUNT: {
        if (pMem == nullptr) {
            *pNumBytes = sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(uint32_t) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            uint32_t numEdges = ccData->indexArrayMesh.numEdgeIndices / 2;
            memcpy(pMem, reinterpret_cast<void*>(&numEdges), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_EDGE: {
        if (pMem == nullptr) {
            *pNumBytes = ccData->indexArrayMesh.numEdgeIndices * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > ccData->indexArrayMesh.numEdgeIndices * sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % (sizeof(uint32_t) * 2) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(ccData->indexArrayMesh.pEdges.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_TYPE: {
        if (pMem == nullptr) {
            *pNumBytes = sizeof(McConnectedComponentType);
        } else {
            if (bytes > sizeof(McConnectedComponentType)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            if (bytes % sizeof(McConnectedComponentType) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(&ccData->type), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION: {
        if (ccData->type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid client pointer type");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McFragmentLocation);
        } else {

            if (bytes > sizeof(McFragmentLocation)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(McFragmentLocation) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            McFragmentConnComp* fragPtr = dynamic_cast<McFragmentConnComp*>(ccData.get());
            memcpy(pMem, reinterpret_cast<void*>(&fragPtr->fragmentLocation), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION: {

        if (ccData->type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT && ccData->type != MC_CONNECTED_COMPONENT_TYPE_PATCH) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "connected component must be a patch or a fragment");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McPatchLocation);
        } else {
            if (bytes > sizeof(McPatchLocation)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(McPatchLocation) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            const void* src = nullptr;
            if (ccData->type == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
                src = reinterpret_cast<const void*>(&dynamic_cast<McFragmentConnComp*>(ccData.get())->patchLocation);
            } else {
                MCUT_ASSERT(ccData->type == MC_CONNECTED_COMPONENT_TYPE_PATCH);
                src = reinterpret_cast<const void*>(&dynamic_cast<McPatchConnComp*>(ccData.get())->patchLocation);
            }
            memcpy(pMem, src, bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE: {
        if (ccData->type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid client pointer type");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McFragmentSealType);
        } else {
            if (bytes > sizeof(McFragmentSealType)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(McFragmentSealType) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            McFragmentConnComp* fragPtr = dynamic_cast<McFragmentConnComp*>(ccData.get());
            memcpy(pMem, reinterpret_cast<void*>(&fragPtr->srcMeshSealType), bytes);
        }
    } break;
        //
    case MC_CONNECTED_COMPONENT_DATA_ORIGIN: {
        if (ccData->type != MC_CONNECTED_COMPONENT_TYPE_SEAM) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid connected component type");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McSeamOrigin);
        } else {
            if (bytes > sizeof(McSeamOrigin)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % sizeof(McSeamOrigin) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            McSeamConnComp* ptr = dynamic_cast<McSeamConnComp*>(ccData.get());
            memcpy(pMem, reinterpret_cast<void*>(&ptr->origin), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX: {
        if (pMem == nullptr) {
            *pNumBytes = ccData->indexArrayMesh.numSeamVertexIndices * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > ccData->indexArrayMesh.numSeamVertexIndices * sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(ccData->indexArrayMesh.pSeamVertexIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP: {
        if ((ctxtPtr->dispatchFlags & MC_DISPATCH_INCLUDE_VERTEX_MAP) == 0) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_MEDIUM, "dispatch flags not set");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }
        if (pMem == nullptr) {
            *pNumBytes = ccData->indexArrayMesh.numVertices * sizeof(uint32_t); // each each vertex has a map value (intersection point == uint_max)
        } else {
            if (bytes > ccData->indexArrayMesh.numVertices * sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(ccData->indexArrayMesh.pVertexMapIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_MAP: {
        if ((ctxtPtr->dispatchFlags & MC_DISPATCH_INCLUDE_FACE_MAP) == 0) {
            ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_MEDIUM, "dispatch flags not set");
            result = McResult::MC_INVALID_VALUE;
            return result;
        }

        if (pMem == nullptr) {
            *pNumBytes = ccData->indexArrayMesh.numFaces * sizeof(uint32_t); // each each vertex has a map value (intersection point == uint_max)
        } else {
            if (bytes > ccData->indexArrayMesh.numFaces * sizeof(uint32_t)) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "out of bounds memory access");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of bytes");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }
            memcpy(pMem, reinterpret_cast<void*>(ccData->indexArrayMesh.pFaceMapIndices.get()), bytes);
        }
    } break;
    default:
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid enum flag");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }
    return result;
}

McResult MCAPI_CALL mcReleaseConnectedComponents(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps)
{
    McResult result = McResult::MC_NO_ERROR;
    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    const std::unique_ptr<McDispatchContextInternal>& ctxtPtr = ctxtIter->second;

    if (numConnComps > (uint32_t)ctxtPtr->connComps.size()) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid number of connected components");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    if (numConnComps == 0 && pConnComps != NULL) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "number of connected components not set");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    if (numConnComps > 0 && pConnComps == NULL) {
        ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid pointer to connected components");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    bool freeAll = numConnComps == 0 && pConnComps == NULL;

    if (freeAll) {
        ctxtPtr->connComps.clear();
    } else {
        for (int i = 0; i < (int)numConnComps; ++i) {
            McConnectedComponent connCompId = pConnComps[i];
            auto ccRef = ctxtPtr->connComps.find(connCompId);
            if (ccRef == ctxtPtr->connComps.cend()) {
                ctxtPtr->log(McDebugSource::MC_DEBUG_SOURCE_API, McDebugType::MC_DEBUG_TYPE_ERROR, 0, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, "invalid connected component id");
                result = McResult::MC_INVALID_VALUE;
                return result;
            }

            ctxtPtr->connComps.erase(ccRef);
        }
    }

    return result;
}

MCAPI_ATTR McResult MCAPI_CALL mcReleaseContext(const McContext context)
{
    McResult result = McResult::MC_NO_ERROR;
    auto ctxtIter = gDispatchContexts.find(context);

    if (ctxtIter == gDispatchContexts.cend()) {
        std::fprintf(stderr, "err: context undefined\n");
        result = McResult::MC_INVALID_VALUE;
        return result;
    }

    gDispatchContexts.erase(ctxtIter);
    return result;
}
