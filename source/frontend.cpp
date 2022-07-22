#include "mcut/internal/frontend.h"
#include "mcut/internal/frontend_intersect.h"

#include "mcut/internal/geom.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"

#include <algorithm>
#include <array>
#include <fstream>

#include <memory>

#include <stdio.h>
#include <string.h>
#include <unordered_map>

#include "mcut/internal/tri/tri.h"

#if defined(MCUT_MULTI_THREADED)
#include "mcut/internal/scheduler.h"

std::atomic_bool mcut::thread_pool_terminate(false);
#endif

#if defined(PROFILING_BUILD)
std::stack<std::unique_ptr<mcut::mini_timer>> g_timestack = std::stack<std::unique_ptr<mcut::mini_timer>>();
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

    intersect(
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
            uint32_t faceOffset = 0;
            std::vector<uint32_t> ccTriangleIndices;
            ccTriangleIndices.reserve(cc_uptr->indexArrayMesh.numFaces);

            // for each face (TODO: make parallel)
            for (uint32_t f = 0; f < cc_uptr->indexArrayMesh.numFaces; ++f) {
                const uint32_t faceSize = cc_uptr->indexArrayMesh.pFaceSizes[f];
                if (faceSize == 3) {
                    for (uint32_t v = 0; v < faceSize; ++v) {
                        const uint32_t vertexId = cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)faceOffset + v];
                        ccTriangleIndices.push_back(vertexId);
                    }
                } else {
                    // get face vertices (their coordinates) and compute
                    // local (face) to global (cc) vertex index mapping
                    // --------------------------------------------------
                    std::vector<uint32_t> faceVertexIndices(faceSize);
                    std::vector<mcut::vec3> faceVertexCoords3d(faceSize);
                    std::unordered_map<uint32_t, uint32_t> faceLocalToGlobleVertexMap;

                    for (uint32_t v = 0; v < faceSize; ++v) {
                        const uint32_t vertexId = cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)faceOffset + v];
                        faceVertexIndices[v] = vertexId;
                        const double* const vptr = cc_uptr->indexArrayMesh.pVertices.get() + ((std::size_t)vertexId * 3);
                        faceVertexCoords3d[v] = mcut::vec3(vptr[0], vptr[1], vptr[2]);
                        faceLocalToGlobleVertexMap[v] = vertexId;
                    }

                    // project vertices to 2D
                    // ----------------------
                    std::vector<mcut::vec2> faceVertexCoords2d;
                    {
                        mcut::vec3 faceNormal;
                        double param_d;
                        int largestNormalComp = mcut::compute_polygon_plane_coefficients(
                            faceNormal,
                            param_d,
                            faceVertexCoords3d.data(),
                            (int)faceVertexCoords3d.size());

                        mcut::project2D(faceVertexCoords2d, faceVertexCoords3d, faceNormal, largestNormalComp);
                    }

                    std::vector<std::vector<std::array<double, 2>>> polygon(1);
                    std::vector<std::array<double, 2>>& faceVertexCoords2d_ec = polygon.back();
                    // convert 2d vertices into format acceptable by earcut
                    faceVertexCoords2d_ec.resize(faceVertexCoords2d.size());
                    {
                        for (int i = 0; i < (int)faceVertexCoords2d.size(); ++i) {
                            const mcut::vec2& v = faceVertexCoords2d[i];
                            std::array<double, 2>& a = faceVertexCoords2d_ec[i];
                            a[0] = static_cast<double>(v[0]);
                            a[1] = static_cast<double>(v[1]);
                        }
                    }

                    // triangulate face
                    std::vector<uint32_t> faceTriangleIndices = mapbox::earcut(polygon);

                    if (faceTriangleIndices.empty()) {
                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "cannot triangulate face " + std::to_string(f));
                    }

                    // The winding order after triangulation is not consistent.
                    // So need to to check whether to reverse "faceTriangleIndices"
                    // or not. There is a simple rule to know whether to reverse:
                    // Whether or not an input polygon (to earcut) is CW or CCW, the local
                    // indexing of the input points follows 0, 1, 2 ... etc. (because that is how
                    // they are recieved from the mesh, even if mesh has CCW faces)
                    // Thus, to check whether we need to reverse the list, we just need to
                    // check if any index in "faceTriangleIndices" has a larger value the next element,
                    // where (faceTriangleIndices[i] - faceTriangleIndices[i+1 % N]) == 1
                    bool is_ccw_triangulation = true;
                    const int index_count = (int)faceTriangleIndices.size();

                    for (int i = 0; i < index_count; ++i) {
                        const int cur = faceTriangleIndices[i];
                        const int nxt = faceTriangleIndices[(i + 1) % index_count];
                        const bool is_consecutive = std::abs(nxt - cur) == 1; // implies along border/edge of triangulatd polygon
                        if (is_consecutive) {
                            const bool is_reversed = cur > nxt;
                            if (is_reversed) {
                                is_ccw_triangulation = false;
                                break;
                            }
                        }
                    }

                    if (!is_ccw_triangulation) {
                        // Output triangles are clockwise, so we reverse the list
                        std::reverse(faceTriangleIndices.begin(), faceTriangleIndices.end());
                    }

                    // used to check that all indices where used in the triangulation. if not, then there will be a hole
                    std::vector<bool> usedVertexIndicators(faceLocalToGlobleVertexMap.size(), false);
                    // remap local triangle indices to global values and save
                    for (int i = 0; i < (int)faceTriangleIndices.size(); ++i) {
                        const uint32_t triangleLocalVertexIndex = faceTriangleIndices[i]; // id local within the current face that we are triangulating
                        const uint32_t triangleGlobalVertexIndex = faceLocalToGlobleVertexMap.at(triangleLocalVertexIndex);
                        faceTriangleIndices[i] = triangleGlobalVertexIndex; // id in the mesh
                        usedVertexIndicators[triangleLocalVertexIndex] = true;
                    }

                    ccTriangleIndices.insert(ccTriangleIndices.end(), faceTriangleIndices.begin(), faceTriangleIndices.end());

                    for (int i = 0; i < (int)usedVertexIndicators.size(); ++i) {
                        if (usedVertexIndicators[i] == false) {
                            context_uptr->log(
                                MC_DEBUG_SOURCE_KERNEL,
                                MC_DEBUG_TYPE_OTHER, 0,
                                MC_DEBUG_SEVERITY_NOTIFICATION, "Found unused vertex on face " + std::to_string(f) + " for triangulation");
                            break;
                        }
                    }

                } //  if (faceSize == 3)

                faceOffset += faceSize;
            }

            MCUT_ASSERT(ccTriangleIndices.size() >= 3);

            cc_uptr->indexArrayMesh.numTriangleIndices = (uint32_t)ccTriangleIndices.size();
            cc_uptr->indexArrayMesh.pTriangleIndices = std::unique_ptr<uint32_t[]>(new uint32_t[cc_uptr->indexArrayMesh.numTriangleIndices]);

            memcpy(reinterpret_cast<void*>(cc_uptr->indexArrayMesh.pTriangleIndices.get()), ccTriangleIndices.data(), ccTriangleIndices.size() * sizeof(uint32_t));
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