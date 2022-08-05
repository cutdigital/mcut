#include "mcut/internal/frontend.h"
#include "mcut/internal/preproc.h"

#include "mcut/internal/hmesh.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"

#include <algorithm>
#include <array>
#include <fstream>

#include <memory>

#include <stdio.h>
#include <string.h>
#include <unordered_map>

#include "mcut/internal/cdt/CDT.h"
#include "mcut/internal/cdt/VerifyTopology.h"

#if defined(MCUT_MULTI_THREADED)
#include "mcut/internal/tpool.h"
std::atomic_bool thread_pool_terminate(false);
#endif

#if defined(PROFILING_BUILD)
std::stack<std::unique_ptr<mini_timer>> g_timestack = std::stack<std::unique_ptr<mini_timer>>();
#endif

std::map<McContext, std::unique_ptr<context_t>> g_contexts = {};

void create_context_impl(McContext* pOutContext, McFlags flags)
{
    MCUT_ASSERT(pOutContext != nullptr);

    // allocate internal context object (including associated threadpool etc.)
    std::unique_ptr<context_t> context_uptr = std::unique_ptr<context_t>(new context_t());

    // copy context configuration flags
    context_uptr->flags = flags;

    // create handle (ptr) which will be returned and used by client to access rest of API
    const McContext handle = reinterpret_cast<McContext>(context_uptr.get());

    const std::pair<std::map<McContext, std::unique_ptr<context_t>>::iterator, bool> insertion_result = g_contexts.emplace(handle, std::move(context_uptr));

    const bool context_inserted_ok = insertion_result.second;

    if (!context_inserted_ok) {
        throw std::runtime_error("failed to create context");
    }

    const std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = insertion_result.first;

    MCUT_ASSERT(handle == context_entry_iter->first);

    *pOutContext = context_entry_iter->first;
}

void debug_message_callback_impl(
    McContext contextHandle,
    pfn_mcDebugOutput_CALLBACK cb,
    const void* userParam)
{
    MCUT_ASSERT(contextHandle != nullptr);
    MCUT_ASSERT(cb != nullptr);

    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(contextHandle);

    if (context_entry_iter == g_contexts.end()) {
        // "contextHandle" may not be NULL but that does not mean it maps to
        // a valid object in "g_contexts"
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    // set callback function ptr, and user pointer
    context_uptr->debugCallback = cb;
    context_uptr->debugCallbackUserParam = userParam;
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

void debug_message_control_impl(
    McContext contextHandle,
    McDebugSource source,
    McDebugType type,
    McDebugSeverity severity,
    bool enabled)
{
    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(contextHandle);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    // reset
    context_uptr->debugSource = 0;

    for (auto i : { MC_DEBUG_SOURCE_API, MC_DEBUG_SOURCE_KERNEL }) {
        if ((source & i) && enabled) {

            int n = trailing_zeroes(MC_DEBUG_SOURCE_ALL & i);

            context_uptr->debugSource = set_bit(context_uptr->debugSource, n);
        }
    }

    // reset
    context_uptr->debugType = 0;

    for (auto i : { MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR, MC_DEBUG_TYPE_ERROR, MC_DEBUG_TYPE_OTHER }) {
        if ((type & i) && enabled) {

            int n = trailing_zeroes(MC_DEBUG_TYPE_ALL & i);

            context_uptr->debugType = set_bit(context_uptr->debugType, n);
        }
    }

    // reset
    context_uptr->debugSeverity = 0;

    for (auto i : { MC_DEBUG_SEVERITY_HIGH, MC_DEBUG_SEVERITY_LOW, MC_DEBUG_SEVERITY_MEDIUM, MC_DEBUG_SEVERITY_NOTIFICATION }) {
        if ((severity & i) && enabled) {

            int n = trailing_zeroes(MC_DEBUG_SEVERITY_ALL & i);

            context_uptr->debugSeverity = set_bit(context_uptr->debugSeverity, n);
        }
    }
}

void get_info_impl(
    const McContext context,
    McFlags info,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes)
{
    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(context);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    switch (info) {
    case MC_CONTEXT_FLAGS:
        if (pMem == nullptr) {
            *pNumBytes = sizeof(context_uptr->flags);
        } else {
            memcpy(pMem, reinterpret_cast<void*>(&context_uptr->flags), bytes);
        }
        break;
    default:
        throw std::invalid_argument("unknown info parameter");
        break;
    }
}

void dispatch_impl(
    McContext context,
    McFlags flags,
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
    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(context);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    context_uptr->dispatchFlags = flags;

    preproc(
        context_uptr,
        pSrcMeshVertices,
        pSrcMeshFaceIndices,
        pSrcMeshFaceSizes,
        numSrcMeshVertices,
        numSrcMeshFaces,
        pCutMeshVertices,
        pCutMeshFaceIndices,
        pCutMeshFaceSizes,
        numCutMeshVertices,
        numCutMeshFaces);
}

void get_connected_components_impl(
    const McContext context,
    const McConnectedComponentType connectedComponentType,
    const uint32_t numEntries,
    McConnectedComponent* pConnComps,
    uint32_t* numConnComps)
{
    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(context);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    if (numConnComps != nullptr) {
        (*numConnComps) = 0; // reset
    }

    uint32_t gatheredConnCompCounter = 0;

    for (std::map<McConnectedComponent, std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>>::const_iterator i = context_uptr->connected_components.cbegin();
         i != context_uptr->connected_components.cend();
         ++i) {

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
    }
}

void get_connected_component_data_impl(
    const McContext context,
    const McConnectedComponent connCompId,
    McFlags flags,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes)
{

    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(context);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    std::map<McConnectedComponent, std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>>::const_iterator cc_entry_iter = context_uptr->connected_components.find(connCompId);

    if (cc_entry_iter == context_uptr->connected_components.cend()) {
        throw std::invalid_argument("invalid connected component");
    }

    auto& cc_uptr = cc_entry_iter->second;

    switch (flags) {

    case MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT: {
        const uint64_t allocatedBytes = cc_uptr->indexArrayMesh.numVertices * sizeof(float) * 3;
        if (pMem == nullptr) {
            *pNumBytes = allocatedBytes;
        } else { // copy mem to client ptr

            if (bytes > allocatedBytes) {
                throw std::invalid_argument("out of bounds memory access");
            } // if

            uint64_t off = 0;
            float* outPtr = reinterpret_cast<float*>(pMem);
            uint64_t nelems = (uint64_t)(bytes / sizeof(float));

            if (nelems % 3 != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            for (uint32_t i = 0; i < nelems; ++i) {
                const double& val = cc_uptr->indexArrayMesh.pVertices[i];

                const float val_ = static_cast<float>(val);

                memcpy(outPtr + off, reinterpret_cast<const void*>(&val_), sizeof(float));
                off += 1;
            }

            MCUT_ASSERT((off * sizeof(float)) == allocatedBytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE: {
        const uint64_t allocatedBytes = cc_uptr->indexArrayMesh.numVertices * sizeof(double) * 3;
        if (pMem == nullptr) {
            *pNumBytes = allocatedBytes;
        } else { // copy mem to client ptr

            if (bytes > allocatedBytes) {
                throw std::invalid_argument("out of bounds memory access");
            } // if

            uint64_t byteOffset = 0;
            double* outPtr = reinterpret_cast<double*>(pMem);

            uint64_t nelems = (uint64_t)(bytes / sizeof(double));

            if (nelems % 3 != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            // uint32_t verticesToCopy = (uint32_t)(nelems / 3);

            for (uint32_t i = 0; i < nelems; ++i) {
                const double& val = cc_uptr->indexArrayMesh.pVertices[i];

                const double val_ = static_cast<double>(val);

                memcpy(outPtr + byteOffset, reinterpret_cast<const void*>(&val_), sizeof(double));
                byteOffset += 1;
            }

            MCUT_ASSERT((byteOffset * sizeof(double)) == allocatedBytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE: {
        if (pMem == nullptr) {
            MCUT_ASSERT(cc_uptr->indexArrayMesh.numFaceIndices > 0);
            *pNumBytes = cc_uptr->indexArrayMesh.numFaceIndices * sizeof(uint32_t);
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numFaceIndices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pFaceIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_SIZE: { // non-triangulated only (don't want to store redundant information)
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->indexArrayMesh.numFaces * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numFaces * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pFaceSizes.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE: {
        if (pMem == nullptr) {
            MCUT_ASSERT(cc_uptr->indexArrayMesh.numFaceAdjFaceIndices > 0);
            *pNumBytes = cc_uptr->indexArrayMesh.numFaceAdjFaceIndices * sizeof(uint32_t);
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numFaceAdjFaceIndices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pFaceAdjFaces.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE: {
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->indexArrayMesh.numFaces * sizeof(uint32_t); // each face has a size (num adjacent faces)
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numFaces * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pFaceAdjFacesSizes.get()), bytes);
        }
    } break;

    case MC_CONNECTED_COMPONENT_DATA_EDGE: {
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->indexArrayMesh.numEdgeIndices * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numEdgeIndices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t) * 2) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }
            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pEdges.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_TYPE: {
        if (pMem == nullptr) {
            *pNumBytes = sizeof(McConnectedComponentType);
        } else {
            if (bytes > sizeof(McConnectedComponentType)) {
                throw std::invalid_argument("out of bounds memory access");
            }
            if (bytes % sizeof(McConnectedComponentType) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }
            memcpy(pMem, reinterpret_cast<void*>(&cc_uptr->type), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION: {
        if (cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
            throw std::invalid_argument("invalid client pointer type");
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McFragmentLocation);
        } else {

            if (bytes > sizeof(McFragmentLocation)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(McFragmentLocation) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            fragment_cc_t* fragPtr = dynamic_cast<fragment_cc_t*>(cc_uptr.get());
            memcpy(pMem, reinterpret_cast<void*>(&fragPtr->fragmentLocation), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION: {
        if (cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT && cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_PATCH) {
            throw std::invalid_argument("connected component must be a patch or a fragment");
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McPatchLocation);
        } else {
            if (bytes > sizeof(McPatchLocation)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(McPatchLocation) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const void* src = nullptr;
            if (cc_uptr->type == MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
                src = reinterpret_cast<const void*>(&dynamic_cast<fragment_cc_t*>(cc_uptr.get())->patchLocation);
            } else {
                MCUT_ASSERT(cc_uptr->type == MC_CONNECTED_COMPONENT_TYPE_PATCH);
                src = reinterpret_cast<const void*>(&dynamic_cast<patch_cc_t*>(cc_uptr.get())->patchLocation);
            }
            memcpy(pMem, src, bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE: {
        if (cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) {
            throw std::invalid_argument("invalid client pointer type");
        }

        if (pMem == nullptr) {
            *pNumBytes = sizeof(McFragmentSealType);
        } else {
            if (bytes > sizeof(McFragmentSealType)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(McFragmentSealType) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }
            fragment_cc_t* fragPtr = dynamic_cast<fragment_cc_t*>(cc_uptr.get());
            memcpy(pMem, reinterpret_cast<void*>(&fragPtr->srcMeshSealType), bytes);
        }
    } break;
        //
    case MC_CONNECTED_COMPONENT_DATA_ORIGIN: {
        if (cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_SEAM && cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            throw std::invalid_argument("invalid connected component type");
        }

        size_t nbytes = (cc_uptr->type != MC_CONNECTED_COMPONENT_TYPE_SEAM ? sizeof(McSeamOrigin) : sizeof(McInputOrigin));

        if (pMem == nullptr) {
            *pNumBytes = nbytes;
        } else {
            if (bytes > nbytes) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if ((bytes % nbytes) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            if (cc_uptr->type == MC_CONNECTED_COMPONENT_TYPE_SEAM) {
                seam_cc_t* ptr = dynamic_cast<seam_cc_t*>(cc_uptr.get());
                memcpy(pMem, reinterpret_cast<void*>(&ptr->origin), bytes);
            } else {
                input_cc_t* ptr = dynamic_cast<input_cc_t*>(cc_uptr.get());
                memcpy(pMem, reinterpret_cast<void*>(&ptr->origin), bytes);
            }
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX: {
        if (cc_uptr->type == MC_CONNECTED_COMPONENT_TYPE_INPUT) {
            throw std::invalid_argument("cannot query seam vertices on input connected component");
        }

        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->indexArrayMesh.numSeamVertexIndices * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numSeamVertexIndices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }
            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pSeamVertexIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP: {
        if ((context_uptr->dispatchFlags & MC_DISPATCH_INCLUDE_VERTEX_MAP) == 0) {
            throw std::invalid_argument("dispatch flags not set");
        }
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->indexArrayMesh.numVertices * sizeof(uint32_t); // each each vertex has a map value (intersection point == uint_max)
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numVertices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }
            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pVertexMapIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_MAP: {
        if ((context_uptr->dispatchFlags & MC_DISPATCH_INCLUDE_FACE_MAP) == 0) {
            throw std::invalid_argument("dispatch flags not set");
        }

        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->indexArrayMesh.numFaces * sizeof(uint32_t); // each each vertex has a map value (intersection point == uint_max)
        } else {
            if (bytes > cc_uptr->indexArrayMesh.numFaces * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }
            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pFaceMapIndices.get()), bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION: {
        if (cc_uptr->indexArrayMesh.numTriangleIndices == 0) // compute triangulation if not yet available
        {
            uint32_t face_indices_offset = 0;
            std::vector<uint32_t> tri_face_indices;
            tri_face_indices.reserve(cc_uptr->indexArrayMesh.numFaces);

            const uint32_t nontri_cc_face_count = cc_uptr->indexArrayMesh.numFaces;

            // -----
            std::vector<vec3> face_vertex_coords_3d;
            std::vector<vec2> face_vertex_coords_2d;
            // temp halfedge data structure whose in-built unctionality helps us ensure that
            // the winding order that is computed by the constrained delaunay triangulator
            // is consistent with that of the connected component we are triangulating.
            hmesh_t winding_order_enforcer;

            // add the face which contain the opposite winding order of the
            // current face to be triangulated (we will use this to prevent inserting
            // flipped triangles). This is guarranteed to work because all triangles
            // are adjacent to the border for a constrained triangulation.
            // This is not true for a conforming triangulation.
            std::vector<vd_t> face_reversed;

            std::vector<CDT::V2d<double>> face_polygon_vertices;
            std::vector<CDT::Edge> face_polygon_edges;
            // list of indices which define all triangles that result from the CDT
            std::vector<uint32_t> face_triangulation_indices;
            // used to check that all indices where used in the triangulation. if not, then there will be a hole
            std::vector<bool> vertex_is_used;
            // -----

            // for each face (TODO: make parallel)
            for (uint32_t f = 0; f < nontri_cc_face_count; ++f) {

                const uint32_t face_vertex_count = cc_uptr->indexArrayMesh.pFaceSizes[f];
                const bool face_is_triangle = (face_vertex_count == 3);
                const bool is_adjacent_to_intersection_curve = false; // TODO: need to compute this info in kernel (or check if any vertex of face is an intpt)

                if (face_is_triangle) {

                    for (uint32_t v = 0; v < face_vertex_count; ++v) {
                        const uint32_t face_vertex_idx = cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)face_indices_offset + v];
                        tri_face_indices.push_back(face_vertex_idx);
                    }

                } else if (!face_is_triangle && is_adjacent_to_intersection_curve) {
                    // TODO: This is when we should actually do triangulation
                    //
                    // NOTE: the feature of "performing triangulation only for faces
                    // next to the intersection curve" or "triangulating all non-tri faces"
                    // should be controlled via a flag e.g. MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION_LOCAL and MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION_GLOBAL
                } else {

                    //
                    // init vars
                    //
                    face_vertex_coords_3d.resize(face_vertex_count);
                    face_vertex_coords_2d.clear(); // resized by project2D(...)
                    winding_order_enforcer.reset();
                    face_reversed.resize(face_vertex_count);
                    face_polygon_edges.clear(); //.resize(face_vertex_count); // |edges| == |vertices|
                    face_polygon_vertices.resize(face_vertex_count);
                    face_triangulation_indices.clear();

                    // copy/get face vertices and save mapping
                    // =======================================

                    for (uint32_t v = 0; v < face_vertex_count; ++v) {

                        const uint32_t face_vertex_idx = cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)face_indices_offset + v];

                        const double* const vptr = cc_uptr->indexArrayMesh.pVertices.get() + ((std::size_t)face_vertex_idx * 3);

                        vec3& coord_3d = face_vertex_coords_3d[v];
                        coord_3d[0] = vptr[0];
                        coord_3d[1] = vptr[1];
                        coord_3d[2] = vptr[2];
                    }

                    // project face vertices to 2D (NOTE: area is unchanged)
                    // =====================================================

                    vec3 face_normal;
                    double param_d;
                    int face_normal_largest_component = compute_polygon_plane_coefficients(
                        face_normal,
                        param_d,
                        face_vertex_coords_3d.data(),
                        (int)face_vertex_count);

                    project2D(face_vertex_coords_2d, face_vertex_coords_3d, face_normal, face_normal_largest_component);

                    CDT::Triangulation<double> constrained_delaunay_triangulator(CDT::VertexInsertionOrder::AsProvided); // memory is alway reallocated for this

                    // convert face vertex format & compute revered face
                    // =================================================

                    // the vertices are in counter-clockwise order (user provided order)
                    for (uint32_t i = 0; i < face_vertex_count; ++i) {

                        const vec2& coords = face_vertex_coords_2d[i];

                        face_polygon_vertices[i] = CDT::V2d<double>::make(coords[0], coords[1]);

                        winding_order_enforcer.add_vertex(vec3(coords[0], coords[1], 0.0 /*dont care since polygon is 2D*/)); // .. in fact even the coordinates dont matter for the purposes of hmesh_t here

                        face_reversed[i] = vd_t(face_vertex_count - 1 - i);
                    }

                    // save reversed face
                    // ==================

                    fd_t fd = winding_order_enforcer.add_face(face_reversed);
                    MCUT_ASSERT(fd != hmesh_t::null_face());

                    // create edges (constraints)
                    // ==========================

                    for (uint32_t i = 0; i < face_vertex_count; ++i) {
                        face_polygon_edges.emplace_back(CDT::Edge(i, (i + 1) % face_vertex_count));
                    }

                    // prepare and do constrained delaunay triangulation
                    // =================================================

                    const CDT::DuplicatesInfo duplInfo = CDT::RemoveDuplicates(face_polygon_vertices);
                    if (!duplInfo.duplicates.empty()) {
                        fprintf(stderr, "triangulation has duplicates\n");
                        // TODO: do stuff with "duplInfo"
                    }

                    constrained_delaunay_triangulator.insertVertices(face_polygon_vertices);

                    constrained_delaunay_triangulator.insertEdges(face_polygon_edges);

                    // triangulation done here!
                    // NOTE: it seems that the triangulation can be either CW or CCW.
                    constrained_delaunay_triangulator.eraseOuterTrianglesAndHoles();

                    const CDT::unordered_map<CDT::Edge, CDT::EdgeVec> tmp = CDT::EdgeToPiecesMapping(constrained_delaunay_triangulator.pieceToOriginals);
                    const CDT::unordered_map<CDT::Edge, std::vector<CDT::VertInd>>
                        edgeToSplitVerts = CDT::EdgeToSplitVertices(tmp, constrained_delaunay_triangulator.vertices);

                    if (!CDT::verifyTopology(constrained_delaunay_triangulator)) {

                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "Triangulation on face " + std::to_string(f) + "has wrong topology");
                    }

                    if (constrained_delaunay_triangulator.triangles.empty()) {
                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "Triangulation on face " + std::to_string(f) + "produced zero faces");
                    }

                    // save the triangulation
                    // ======================

                    const uint32_t face_resulting_triangle_count = (uint32_t)constrained_delaunay_triangulator.triangles.size();

                    for (uint32_t i = 0; i < face_resulting_triangle_count; ++i) {

                        // a triangle computed from CDT
                        const CDT::Triangle& triangle = constrained_delaunay_triangulator.triangles[i];

                        // convert to local descriptors
                        std::vector<vd_t> triangle_descriptors = {
                            vd_t(triangle.vertices[0]),
                            vd_t(triangle.vertices[1]),
                            vd_t(triangle.vertices[2])
                        };

                        face_triangulation_indices.emplace_back(triangle.vertices[0]);
                        face_triangulation_indices.emplace_back(triangle.vertices[1]);
                        face_triangulation_indices.emplace_back(triangle.vertices[2]);

                        // check that the winding order matches the triangulated face's order
                        const bool is_insertible = winding_order_enforcer.is_insertable(triangle_descriptors);

                        if (!is_insertible) {
                            // flip the winding order
                            uint32_t a = triangle_descriptors[0];
                            uint32_t c = triangle_descriptors[2];
                            std::swap(a, c);
                            triangle_descriptors[0] = vd_t(a);
                            triangle_descriptors[2] = vd_t(c);
                            const size_t N = face_triangulation_indices.size();
                            std::swap(face_triangulation_indices[N - 1], face_triangulation_indices[N - 3]); // reverse last added triangle's indices
                        }

                        // add face into our WO enforcer
                        fd = winding_order_enforcer.add_face(triangle_descriptors); // keep track of added faces

                        // if this fails then CDT gave us a strange triangulation e.g.
                        // duplicate triangles with opposite winding order
                        if (fd == hmesh_t::null_face()) {
// simply remove/ignore the offending triangle
#if 0
                            uint32_t a = triangle_descriptors[0];
                            uint32_t b = triangle_descriptors[1];
                            uint32_t c = triangle_descriptors[2];
                            printf("inserted face: %u %u %u\n", a, b, c);
                            
                            std::ofstream f("triangulation.obj");

                            for (int i = 0; i < constrained_delaunay_triangulator.vertices.size(); ++i) {
                                auto vert = constrained_delaunay_triangulator.vertices[i];
                                f << "v " << vert.x << " " << vert.y << " " << 0.0 << "\n";
                            }

                            for (uint32_t i = 0; i < face_resulting_triangle_count; ++i) {
                                // a triangle computed from CDT
                                const CDT::Triangle& triangle = constrained_delaunay_triangulator.triangles[i];
                                f << "f " << triangle.vertices[0]+1 << " " << triangle.vertices[1]+1 << " " << triangle.vertices[2]+1 << "\n";
                            }

                            f.close();

                            std::ofstream g("polygon.off");
                            g << "OFF\n";
                            g << face_vertex_count << " 1 0\n"; 
                            for (int i = 0; i < face_vertex_count; ++i) {
                                auto vert = face_vertex_coords_3d[i];
                                g << vert[0] << " " << vert[1] << " " << vert[2] << "\n";
                            }
                            g << face_vertex_count << " "; 
                            for (uint32_t i = 0; i < face_vertex_count; ++i) {
                                // a triangle computed from CDT
                                
                                g << i << " ";
                            }
                            g << "\n";

                            g.close();

                            std::ofstream h("polygon2d.txt");
                            h << face_vertex_count << " " << face_vertex_count << "\n"; 
                            for (int i = 0; i < face_vertex_count; ++i) {
                                auto vert = face_vertex_coords_2d[i];
                                h << vert[0] << " " << vert[1] << "\n";
                            }
                            for (uint32_t i = 0; i < face_vertex_count; ++i) {
                                // a triangle computed from CDT
                                
                                h << i << " " << (i+1) % face_vertex_count <<"\n";
                            }

                            h.close();


                            printf("is_insertible=%d\n", (int)is_insertible);
                            fprintf(stderr, "error: could not insert triangle %d\n", i);
                            std::exit(1);
#endif

                            face_triangulation_indices.pop_back();
                            face_triangulation_indices.pop_back();
                            face_triangulation_indices.pop_back();
                        }
                    }

                    // swap local triangle indices to global index values (in CC) and save
                    // ===================================================================

                    const uint32_t face_triangulation_indices_count = (uint32_t)face_triangulation_indices.size();

                    for (uint32_t i = 0; i < face_triangulation_indices_count; ++i) {
                        const uint32_t local_idx = face_triangulation_indices[i]; // id local within the current face that we are triangulating
                        const uint32_t global_idx = cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)face_indices_offset + local_idx];

                        face_triangulation_indices[(std::size_t)i] = global_idx; // id in the connected component (mesh)
                    }

                    tri_face_indices.insert(tri_face_indices.end(), face_triangulation_indices.begin(), face_triangulation_indices.end());

                } //  if (face_vertex_count == 3)

                face_indices_offset += face_vertex_count;
            }

            MCUT_ASSERT(tri_face_indices.size() >= 3);

            cc_uptr->indexArrayMesh.numTriangleIndices = (uint32_t)tri_face_indices.size();
            cc_uptr->indexArrayMesh.pTriangleIndices = std::unique_ptr<uint32_t[]>(new uint32_t[cc_uptr->indexArrayMesh.numTriangleIndices]);

            memcpy(reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pTriangleIndices.get()), tri_face_indices.data(), tri_face_indices.size() * sizeof(uint32_t));
        } // if(cc_uptr->indexArrayMesh.numTriangleIndices == 0)

        if (pMem == nullptr) // client pointer is null (asking for size)
        {
            MCUT_ASSERT(cc_uptr->indexArrayMesh.pTriangleIndices.get() != nullptr);
            *pNumBytes = cc_uptr->indexArrayMesh.numTriangleIndices * sizeof(uint32_t); // each each vertex has a map value (intersection point == uint_max)
        } else {
            MCUT_ASSERT(cc_uptr->indexArrayMesh.numTriangleIndices >= 3);
            if (bytes > cc_uptr->indexArrayMesh.numTriangleIndices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t)) != 0 || (bytes / sizeof(uint32_t)) % 3 != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pTriangleIndices.get()), bytes);
        }
    } break;
    default:
        throw std::invalid_argument("invalid enum flag");
    }
}

void release_connected_components_impl(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps)
{
    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(context);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    if (numConnComps > (uint32_t)context_uptr->connected_components.size()) {
        throw std::invalid_argument("invalid connected component count");
    }

    bool freeAll = numConnComps == 0 && pConnComps == NULL;

    if (freeAll) {
        context_uptr->connected_components.clear();
    } else {
        for (int i = 0; i < (int)numConnComps; ++i) {
            McConnectedComponent connCompId = pConnComps[i];

            std::map<McConnectedComponent, std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>>::const_iterator cc_entry_iter = context_uptr->connected_components.find(connCompId);

            if (cc_entry_iter == context_uptr->connected_components.cend()) {
                throw std::invalid_argument("invalid connected component id");
            }

            context_uptr->connected_components.erase(cc_entry_iter);
        }
    }
}

void release_context_impl(
    McContext context)
{
    std::map<McContext, std::unique_ptr<context_t>>::iterator context_entry_iter = g_contexts.find(context);

    if (context_entry_iter == g_contexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    g_contexts.erase(context_entry_iter);
}