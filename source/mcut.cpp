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

#include "mcut/internal/kernel.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
#include <cfenv>
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

#include <fstream>
#include <memory>
#include <stdio.h>
#include <string.h>

#if defined(MCUT_BUILD_WINDOWS)
#pragma warning(disable : 26812)
#endif

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
    const void* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces)
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
            vmap[i] = halfedgeMesh.add_vertex(x, y, z);
        }
    } else if (ctxtPtr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_DOUBLE) {
        const double* vptr = reinterpret_cast<const double*>(pVertices);
        for (uint32_t i = 0; i < numVertices; ++i) {
            const double& x = vptr[(i * 3) + 0];
            const double& y = vptr[(i * 3) + 1];
            const double& z = vptr[(i * 3) + 2];
            vmap[i] = halfedgeMesh.add_vertex(x, y, z);
        }
    } else if (ctxtPtr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_EXACT) {
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

            mcut::vd_t descr = fIter->second; //vmap[*fIter.first];

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
    McResult result = McResult::MC_NO_ERROR;
    switch (v) {
    case mcut::status_t::SUCCESS:
        result = McResult::MC_NO_ERROR;
        break;
    case mcut::status_t::INVALID_SRC_MESH:
        result = McResult::MC_INVALID_SRC_MESH;
        break;
    case mcut::status_t::INVALID_CUT_MESH:
        result = McResult::MC_INVALID_CUT_MESH;
        break;
    case mcut::status_t::INVALID_MESH_INTERSECTION:
    case mcut::status_t::INVALID_BVH_INTERSECTION:
        result = McResult::MC_INVALID_OPERATION;
        break;
    case mcut::status_t::EDGE_EDGE_INTERSECTION:
        result = McResult::MC_EDGE_EDGE_INTERSECTION;
        break;
    case mcut::status_t::FACE_VERTEX_INTERSECTION:
        result = McResult::MC_FACE_VERTEX_INTERSECTION;
        break;
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
        result = McResult::MC_INVALID_SRC_MESH;
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
        result = McResult::MC_INVALID_CUT_MESH;
        return result;
    }

    mcut::mesh_t srcMeshInternal;
    result = indexArrayMeshToHalfedgeMesh(
        ctxtPtr,
        srcMeshInternal,
        pSrcMeshVertices,
        pSrcMeshFaceIndices,
        pSrcMeshFaceSizes,
        numSrcMeshVertices,
        numSrcMeshFaces);

    if (result != McResult::MC_NO_ERROR) {
        return result;
    }

    mcut::mesh_t cutMeshInternal;
    result = indexArrayMeshToHalfedgeMesh(
        ctxtPtr,
        cutMeshInternal,
        pCutMeshVertices,
        pCutMeshFaceIndices,
        pCutMeshFaceSizes,
        numCutMeshVertices,
        numCutMeshFaces);

    if (result != McResult::MC_NO_ERROR) {
        return result;
    }

    // check here to ensure that vertex coordinates of one mesh are not colocated with any coordinates in the other mesh
    result = checkMeshPlacement(ctxtPtr, srcMeshInternal, cutMeshInternal);
    if (result != McResult::MC_NO_ERROR) {
        return result;
    }

    mcut::input_t backendInput;
    backendInput.src_mesh = &srcMeshInternal;
    backendInput.cut_mesh = &cutMeshInternal;
    backendInput.verbose = false;
    backendInput.require_looped_cutpaths = false;

    backendInput.verbose = static_cast<bool>((ctxtPtr->flags & MC_DEBUG) && (ctxtPtr->debugType & McDebugSource::MC_DEBUG_SOURCE_KERNEL));
    backendInput.require_looped_cutpaths = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_REQUIRE_THROUGH_CUTS);
    backendInput.populate_vertex_maps = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_INCLUDE_VERTEX_MAP);
    backendInput.populate_face_maps = static_cast<bool>(ctxtPtr->dispatchFlags & MC_DISPATCH_INCLUDE_FACE_MAP);

    if ((ctxtPtr->dispatchFlags & MC_DISPATCH_REQUIRE_THROUGH_CUTS) && //
        (ctxtPtr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED)) {
        // The user states that she does not want a partial cut but yet also states that she
        // wants to keep fragments with partial cuts. These two options are mutually exclusive.
        ctxtPtr->log(
            McDebugSource::MC_DEBUG_SOURCE_KERNEL,
            McDebugType::MC_DEBUG_TYPE_ERROR,
            0,
            McDebugSeverity::MC_DEBUG_SEVERITY_HIGH,
            "use of mutually exclusive dispatch flags: MC_DISPATCH_REQUIRE_THROUGH_CUTS and MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED");
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

    mcut::output_t backendOutput;

    // cut!
    // ----
    {
        /*
            "Writers of libraries using MPFR should be aware that the application and/or another library
            used by the application may also use MPFR, so that changing the exponent range, the default
            precision, or the default rounding mode may have an effect on this other use of MPFR since
            these data are not duplicated (unless they are in a different thread). Therefore any such value
            changed in a library function should be restored before the function returns (unless the purpose
            of the function is to do such a change). Writers of software using MPFR should also be careful
            when changing such a value if they use a library using MPFR (directly or indirectly), in order
            to make sure that such a change is compatible with the library."

        */
        try {
            ctxtPtr->applyPrecisionAndRoundingModeSettings();
            mcut::dispatch(backendOutput, backendInput);
        } catch (...) {
            fprintf(stderr, "fatal: exception caught\n");
        }
        ctxtPtr->revertPrecisionAndRoundingModeSettings();
    }

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

                // Note: the last CC is always guarranteed to be fully sealed (see: "include_fragment_sealed_partial" in kernel)!
                bool is_last_cc = (size_t)std::distance(j->second.cbegin(), k) == (size_t)(j->second.size() - 1);

                if (is_last_cc) {
                    asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE;
                } else { // did the user tell us to keep partially sealed fragments??
                    MCUT_ASSERT(backendInput.keep_fragments_sealed_inside_exhaustive || backendInput.keep_fragments_sealed_outside_exhaustive);
                    asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_PARTIAL;
                }

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
