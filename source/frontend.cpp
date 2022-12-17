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

#include "mcut/internal/cdt/cdt.h"

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

    uint32_t valid_cc_counter = 0;

    for (std::map<McConnectedComponent, std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>>::const_iterator i = context_uptr->connected_components.cbegin();
         i != context_uptr->connected_components.cend();
         ++i) {

        const bool is_valid = (i->second->type & connectedComponentType) != 0;

        if (is_valid) {
            if (pConnComps == nullptr) // query number
            {
                (*numConnComps)++;
            } else // populate pConnComps
            {
                pConnComps[valid_cc_counter] = i->first;
                valid_cc_counter += 1;
                if (valid_cc_counter == numEntries) {
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

    const std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>& cc_uptr = cc_entry_iter->second;

    switch (flags) {

    case MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT: {
        const uint64_t allocated_bytes = cc_uptr->kernel_hmesh_data.mesh.number_of_vertices() * sizeof(float) * 3ul; // cc_uptr->indexArrayMesh.numVertices * sizeof(float) * 3;

        if (pMem == nullptr) {
            *pNumBytes = allocated_bytes;
        } else { // copy mem to client ptr

            if (bytes > allocated_bytes) {
                throw std::invalid_argument("out of bounds memory access");
            } // if

            // an element is a component
            const uint64_t nelems = (uint64_t)(bytes / sizeof(float));

            if (nelems % 3 != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const uint64_t num_vertices_to_copy = (nelems / 3);
            uint64_t elem_offset = 0;
            float* casted_ptr = reinterpret_cast<float*>(pMem);

            for (vertex_array_iterator_t viter = cc_uptr->kernel_hmesh_data.mesh.vertices_begin(); viter != cc_uptr->kernel_hmesh_data.mesh.vertices_end(); ++viter) {
                const vec3& coords = cc_uptr->kernel_hmesh_data.mesh.vertex(*viter);

                for (int i = 0; i < 3; ++i) {
                    const float val = static_cast<float>(coords[i]);
                    *(casted_ptr + elem_offset) = val;
                    elem_offset += 1;
                }

                if ((elem_offset / 3) == num_vertices_to_copy) {
                    break;
                }
            }

            MCUT_ASSERT((elem_offset * sizeof(float)) <= allocated_bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE: {
        const uint64_t allocated_bytes = cc_uptr->kernel_hmesh_data.mesh.number_of_vertices() * sizeof(double) * 3ul; // cc_uptr->indexArrayMesh.numVertices * sizeof(float) * 3;

        if (pMem == nullptr) {
            *pNumBytes = allocated_bytes;
        } else { // copy mem to client ptr

            if (bytes > allocated_bytes) {
                throw std::invalid_argument("out of bounds memory access");
            } // if

            // an element is a component
            const int64_t nelems = (uint64_t)(bytes / sizeof(double));

            if (nelems % 3 != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const uint64_t num_vertices_to_copy = (nelems / 3);
            uint64_t elem_offset = 0;
            double* casted_ptr = reinterpret_cast<double*>(pMem);

            for (vertex_array_iterator_t viter = cc_uptr->kernel_hmesh_data.mesh.vertices_begin(); viter != cc_uptr->kernel_hmesh_data.mesh.vertices_end(); ++viter) {

                const vec3& coords = cc_uptr->kernel_hmesh_data.mesh.vertex(*viter);

                for (int i = 0; i < 3; ++i) {
                    *(casted_ptr + elem_offset) = coords[i];
                    elem_offset += 1;
                }

                if ((elem_offset / 3) == num_vertices_to_copy) {
                    break;
                }
            }

            MCUT_ASSERT((elem_offset * sizeof(float)) <= allocated_bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE: {
        if (pMem == nullptr) {
            uint32_t num_indices = 0;

            // TODO: make parallel
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_vertices_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_vertices_around_face(*fiter);

                MCUT_ASSERT(num_vertices_around_face >= 3);

                num_indices += num_vertices_around_face;
            }

            MCUT_ASSERT(num_indices >= 3); // min is a triangle

            *pNumBytes = num_indices * sizeof(uint32_t);
        } else {
            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint32_t num_indices = 0;

            std::vector<vd_t> vertices_around_face;

            uint64_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {

                vertices_around_face.clear();
                cc_uptr->kernel_hmesh_data.mesh.get_vertices_around_face(vertices_around_face, *fiter);
                const uint32_t num_vertices_around_face = (uint32_t)vertices_around_face.size();

                MCUT_ASSERT(num_vertices_around_face >= 3u);

                for (uint32_t i = 0; i < num_vertices_around_face; ++i) {
                    const uint32_t vertex_idx = (uint32_t)vertices_around_face[i];
                    *(casted_ptr + elem_offset) = vertex_idx;
                    ++elem_offset;
                }

                num_indices += num_vertices_around_face;
            }
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_SIZE: { // non-triangulated only (don't want to store redundant information)
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->kernel_hmesh_data.mesh.number_of_faces() * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > cc_uptr->kernel_hmesh_data.mesh.number_of_faces() * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint64_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_vertices_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_vertices_around_face(*fiter);

                MCUT_ASSERT(num_vertices_around_face >= 3);

                *(casted_ptr + elem_offset) = num_vertices_around_face;
                ++elem_offset;
            }
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE: {
        if (pMem == nullptr) {

            MCUT_ASSERT(pNumBytes != nullptr);

            uint32_t num_face_adjacent_face_indices = 0;

            // TODO: make parallel
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_faces_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_faces_around_face(*fiter, nullptr);
                num_face_adjacent_face_indices += num_faces_around_face;
            }

            *pNumBytes = num_face_adjacent_face_indices * sizeof(uint32_t);
        } else {

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint64_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            std::vector<fd_t> faces_around_face;

            // TODO: make parallel
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {

                faces_around_face.clear();
                cc_uptr->kernel_hmesh_data.mesh.get_faces_around_face(faces_around_face, *fiter, nullptr);

                if (!faces_around_face.empty()) {
                    for (uint32_t i = 0; i < (uint32_t)faces_around_face.size(); ++i) {
                        *(casted_ptr + elem_offset) = (uint32_t)faces_around_face[i];
                        elem_offset++;
                    }
                }
            }

            MCUT_ASSERT((elem_offset * sizeof(uint32_t)) <= bytes);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE: {
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->kernel_hmesh_data.mesh.number_of_faces() * sizeof(uint32_t); // each face has a size value (num adjacent faces)
        } else {
            if (bytes > cc_uptr->kernel_hmesh_data.mesh.number_of_faces() * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint64_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_faces_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_faces_around_face(*fiter, nullptr);
                *(casted_ptr + elem_offset) = num_faces_around_face;
                elem_offset++;
            }

            MCUT_ASSERT((elem_offset * sizeof(uint32_t)) <= bytes);
        }
    } break;

    case MC_CONNECTED_COMPONENT_DATA_EDGE: {
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->kernel_hmesh_data.mesh.number_of_edges() * 2 * sizeof(uint32_t); // each edge has two indices
        } else {
            if (bytes > cc_uptr->kernel_hmesh_data.mesh.number_of_edges() * 2 * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t) * 2) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint64_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (edge_array_iterator_t eiter = cc_uptr->kernel_hmesh_data.mesh.edges_begin(); eiter != cc_uptr->kernel_hmesh_data.mesh.edges_end(); ++eiter) {
                const vertex_descriptor_t v0 = cc_uptr->kernel_hmesh_data.mesh.vertex(*eiter, 0);
                *(casted_ptr + elem_offset) = (uint32_t)v0;
                elem_offset++;

                const vertex_descriptor_t v1 = cc_uptr->kernel_hmesh_data.mesh.vertex(*eiter, 1);
                *(casted_ptr + elem_offset) = (uint32_t)v1;
                elem_offset++;
            }

            MCUT_ASSERT((elem_offset * sizeof(uint32_t)) <= bytes);
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

        const uint32_t seam_vertex_count = (uint32_t)cc_uptr->kernel_hmesh_data.seam_vertices.size();

        if (pMem == nullptr) {
            *pNumBytes = seam_vertex_count * sizeof(uint32_t);
        } else {
            if (bytes > (seam_vertex_count * sizeof(uint32_t))) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if ((bytes % (sizeof(uint32_t))) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const uint32_t elems_to_copy = bytes / sizeof(uint32_t);
            uint32_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (uint32_t i = 0; i < elems_to_copy; ++i) {
                const uint32_t seam_vertex_idx = cc_uptr->kernel_hmesh_data.seam_vertices[i];
                *(casted_ptr + elem_offset) = seam_vertex_idx;
                elem_offset++;
            }

            MCUT_ASSERT(elem_offset <= seam_vertex_count);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_VERTEX_MAP: {

        const uint32_t vertex_map_size = cc_uptr->kernel_hmesh_data.data_maps.vertex_map.size();

        if (vertex_map_size == 0) {
            throw std::invalid_argument("vertex map not available"); // user probably forgot to set the dispatch flag
        }

        MCUT_ASSERT(vertex_map_size == (uint32_t)cc_uptr->kernel_hmesh_data.mesh.number_of_vertices());

        if (pMem == nullptr) {
            *pNumBytes = (vertex_map_size * sizeof(uint32_t)); // each each vertex has a map value (intersection point == uint_max)
        } else {
            if (bytes > (vertex_map_size * sizeof(uint32_t))) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t)) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const uint32_t elems_to_copy = (bytes / sizeof(uint32_t));

            MCUT_ASSERT(elems_to_copy <= vertex_map_size);

            uint32_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (uint32_t i = 0; i < elems_to_copy; ++i) // ... for each vertex in CC
            {
                // Here we use whatever index value was assigned to the current vertex by the kernel, where the
                // the kernel does not necessarilly know that the input meshes it was given where modified by
                // the frontend (in this case via polygon partitioning)
                // Vertices that are polygon intersection points have a value of uint_max i.e. null_vertex().

                uint32_t internal_input_mesh_vertex_idx = cc_uptr->kernel_hmesh_data.data_maps.vertex_map[i];
                // We use the same default value as that used by the kernel for intersection
                // points (intersection points at mapped to uint_max i.e. null_vertex())
                uint32_t client_input_mesh_vertex_idx = UINT32_MAX;
                // This is true only for polygon intersection points computed by the kernel
                const bool internal_input_mesh_vertex_is_intersection_point = (internal_input_mesh_vertex_idx == UINT32_MAX);

                if (!internal_input_mesh_vertex_is_intersection_point) { // i.e. a client-mesh vertex or vertex that is added due to face-partitioning
                    // NOTE: The kernel will assign/map a 'proper' index value to vertices that exist due to face partitioning.
                    // 'proper' here means that the kernel treats these vertices as 'original vertices' from a client-provided input
                    // mesh. In reality, the frontend added such vertices in order to partition a face. i.e. the kernel is not aware
                    // that a given input mesh it is working with is modified by the frontend (it assumes that the meshes is exactly as was
                    // provided by the client).
                    // So, here we have to fix that mapping information to correctly state that "any vertex added due to face
                    // partitioning was not in the user provided input mesh" and should therefore be treated/labelled as an intersection
                    // point i.e. it should map to UINT32_MAX because it does not map to any vertex in the client-provided input mesh.
                    bool vertex_exists_due_to_face_partitioning = false;
                    // this flag tells us whether the current vertex maps to one in the internal version of the source mesh
                    // i.e. it does not map to the internal version cut-mesh
                    const bool internal_input_mesh_vertex_is_for_source_mesh = (internal_input_mesh_vertex_idx < cc_uptr->internal_sourcemesh_vertex_count);

                    if (internal_input_mesh_vertex_is_for_source_mesh) {
                        const std::unordered_map<vd_t, vec3>::const_iterator fiter = cc_uptr->source_hmesh_new_poly_partition_vertices->find(vd_t(internal_input_mesh_vertex_idx));
                        vertex_exists_due_to_face_partitioning = (fiter != cc_uptr->source_hmesh_new_poly_partition_vertices->cend());
                    } else // i.e. internal_input_mesh_vertex_is_for_cut_mesh
                    {
                        std::unordered_map<vd_t, vec3>::const_iterator fiter = cc_uptr->cut_hmesh_new_poly_partition_vertices->find(vd_t(internal_input_mesh_vertex_idx));
                        vertex_exists_due_to_face_partitioning = (fiter != cc_uptr->cut_hmesh_new_poly_partition_vertices->cend());
                    }

                    if (!vertex_exists_due_to_face_partitioning) { // i.e. is a client-mesh vertex (an original vertex)

                        MCUT_ASSERT(cc_uptr->internal_sourcemesh_vertex_count > 0);

                        if (!internal_input_mesh_vertex_is_for_source_mesh) // is it a cut-mesh vertex discriptor ..?
                        {
                            // vertices added due to face-partitioning will have an offsetted index/descriptor that is >= client_sourcemesh_vertex_count
                            const uint32_t internal_input_mesh_vertex_idx_without_offset = (internal_input_mesh_vertex_idx - cc_uptr->internal_sourcemesh_vertex_count);
                            client_input_mesh_vertex_idx = (internal_input_mesh_vertex_idx_without_offset + cc_uptr->client_sourcemesh_vertex_count); // ensure that we offset using number of [user-provided mesh] vertices
                        } else {
                            client_input_mesh_vertex_idx = internal_input_mesh_vertex_idx; // src-mesh vertices have no offset unlike cut-mesh vertices
                        }
                    }
                }

                *(casted_ptr + elem_offset) = client_input_mesh_vertex_idx;
                elem_offset++;
            }

            MCUT_ASSERT(elem_offset <= vertex_map_size);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_MAP: {

        const uint32_t face_map_size = cc_uptr->kernel_hmesh_data.data_maps.face_map.size();

        if (face_map_size == 0) {
            throw std::invalid_argument("face map not available"); // user probably forgot to set the dispatch flag
        }

        MCUT_ASSERT(face_map_size == (uint32_t)cc_uptr->kernel_hmesh_data.mesh.number_of_faces());

        if (pMem == nullptr) {
            *pNumBytes = face_map_size * sizeof(uint32_t); // each face has a map value (intersection point == uint_max)
        } else {
            if (bytes > (face_map_size * sizeof(uint32_t))) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if ((bytes % sizeof(uint32_t)) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const uint32_t elems_to_copy = (bytes / sizeof(uint32_t));
            uint32_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

            // TODO: make parallel
            for (uint32_t i = 0; i < elems_to_copy; ++i) // ... for each vertex (to copy) in CC
            {
                uint32_t internal_inputmesh_face_idx = (uint32_t)cc_uptr->kernel_hmesh_data.data_maps.face_map[i];
                uint32_t client_input_mesh_face_idx = INT32_MAX;
                const bool internal_input_mesh_face_idx_is_for_src_mesh = (internal_inputmesh_face_idx < cc_uptr->internal_sourcemesh_face_count);

                if (internal_input_mesh_face_idx_is_for_src_mesh) {

                    std::unordered_map<fd_t, fd_t>::const_iterator fiter = cc_uptr->source_hmesh_child_to_usermesh_birth_face->find(fd_t(internal_inputmesh_face_idx));

                    if (fiter != cc_uptr->source_hmesh_child_to_usermesh_birth_face->cend()) {
                        client_input_mesh_face_idx = fiter->second;
                    } else {
                        client_input_mesh_face_idx = internal_inputmesh_face_idx;
                    }
                    MCUT_ASSERT(client_input_mesh_face_idx < cc_uptr->client_sourcemesh_face_count);
                } else // internalInputMeshVertexDescrIsForCutMesh
                {
                    std::unordered_map<fd_t, fd_t>::const_iterator fiter = cc_uptr->cut_hmesh_child_to_usermesh_birth_face->find(fd_t(internal_inputmesh_face_idx));

                    if (fiter != cc_uptr->cut_hmesh_child_to_usermesh_birth_face->cend()) {
                        uint32_t unoffsettedDescr = (fiter->second - cc_uptr->internal_sourcemesh_face_count);
                        client_input_mesh_face_idx = unoffsettedDescr + cc_uptr->client_sourcemesh_face_count;
                    } else {
                        uint32_t unoffsettedDescr = (internal_inputmesh_face_idx - cc_uptr->internal_sourcemesh_face_count);
                        client_input_mesh_face_idx = unoffsettedDescr + cc_uptr->client_sourcemesh_face_count;
                    }
                }

                MCUT_ASSERT(client_input_mesh_face_idx != INT32_MAX);

                *(casted_ptr + elem_offset) = client_input_mesh_face_idx;
                elem_offset++;
            }

            MCUT_ASSERT(elem_offset <= face_map_size);
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION: {

        if (cc_uptr->constrained_delaunay_triangulation_indices.empty()) // compute triangulation if not yet available
        {
            uint32_t face_indices_offset = 0;
            cc_uptr->constrained_delaunay_triangulation_indices.reserve(cc_uptr->kernel_hmesh_data.mesh.number_of_faces());

            // -----
            std::vector<vec3> face_vertex_coords_3d;
            std::vector<vec2> face_vertex_coords_2d;

            // add the face which contains the opposite winding order of the
            // connected component face face we are triangulating (we will use this to prevent inserting
            // reversed triangles). This is guarranteed to work because all triangles
            // are adjacent to the border for a constrained delaunay triangulation.
            // This is not true for a conforming delaunay triangulation.
            std::vector<vd_t> face_reversed;

            std::vector<vec2> face_polygon_vertices;
            std::vector<cdt::edge_t> face_polygon_edges;
            //  list of indices which define all triangles that result from the CDT
            std::vector<uint32_t> face_triangulation_indices;
            // used to check that all indices where used in the triangulation. if not, then there will be a hole
            std::vector<bool> vertex_is_used;
            // -----

            // for each face (TODO: make parallel)
            std::vector<vertex_descriptor_t> vertices_around_face;

            for (face_array_iterator_t f = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); f != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++f) {

                cc_uptr->kernel_hmesh_data.mesh.get_vertices_around_face(vertices_around_face, *f);

                const uint32_t face_vertex_count = (uint32_t)vertices_around_face.size(); //->indexArrayMesh.pFaceSizes[f];

                MCUT_ASSERT(face_vertex_count >= 3);

                const bool face_is_triangle = (face_vertex_count == 3);
                // const bool is_adjacent_to_intersection_curve = false; // TODO: need to compute this info in kernel (or check if any vertex of face is an intpt)

                if (face_is_triangle) {

                    for (uint32_t v = 0; v < face_vertex_count; ++v) {
                        const uint32_t face_vertex_idx = (uint32_t)vertices_around_face[v]; // cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)face_indices_offset + v];
                        cc_uptr->constrained_delaunay_triangulation_indices.push_back(face_vertex_idx);
                    }

                } else {

                    //
                    // init vars
                    //
                    face_vertex_coords_3d.resize(face_vertex_count);
                    face_vertex_coords_2d.clear(); // resized by project2D(...)
                    face_reversed.resize(face_vertex_count);
                    face_polygon_edges.clear(); //.resize(face_vertex_count); // |edges| == |vertices|
                    face_polygon_vertices.resize(face_vertex_count);
                    face_triangulation_indices.clear();
                    vertex_is_used.resize(face_vertex_count);

                    for (uint32_t v = 0; v < face_vertex_count; ++v) {

                        const vertex_descriptor_t cc_face_vertex_descr = vertices_around_face[v];

                        face_vertex_coords_3d[v] = cc_uptr->kernel_hmesh_data.mesh.vertex(cc_face_vertex_descr);

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

                    cdt::triangulator_t<double> constrained_cdt(cdt::vertex_insertion_order_t::AS_GIVEN); // memory is alway reallocated for this

                    // convert face vertex format & compute revered face
                    // =================================================

                    // the vertices are in counter-clockwise order (user provided order)
                    for (uint32_t i = 0; i < face_vertex_count; ++i) {

                        const vec2& coords = face_vertex_coords_2d[i];

                        face_polygon_vertices[i] = coords; // vec2_<double>::make(coords[0], coords[1]);
                    }
                    
#if 0
                    std::string fname= "face-" + std::to_string(*f) + ".off";
                    std::cout << "dump: " << fname << std::endl;

                    std::ofstream g(fname);
                    g << "OFF\n";
                    g << face_vertex_count << " 1 0\n";
                    for (int i = 0; i < face_vertex_count; ++i) {
                        auto vert = face_polygon_vertices[i];
                        g << vert[0] << " " << vert[1] << " " << 0 << "\n";
                    }
                    g << face_vertex_count << " ";
                    for (uint32_t i = 0; i < face_vertex_count; ++i) {
                        // a triangle computed from CDT

                        g << i << " ";
                    }
                    g << "\n";

                    g.close();
#endif
                    // Temp halfedge data structure whose in-built functionality helps us ensure that
                    // the winding order that is computed by the constrained delaunay triangulator
                    // is consistent with that of the connected component face we are triangulating.
                    //
                    // It stores the neighbours of the current face (it is the connectivity info that
                    // we will used to check for proper winding).
                    hmesh_t wo_enforcer;
                    std::map<vertex_descriptor_t, vertex_descriptor_t> enforcer_to_cc_vmap;
                    std::map<vertex_descriptor_t, vertex_descriptor_t> cc_to_enforcer_vmap;
                    // The halfedge with-which we will identify the first CDT triangle to insert into the
                    // triangulated topology array of the connected component.
                    // We need this to ensure that the first CDT triangle to be inserted is inserted with the
                    // correct winding order. This caters to the scenario where "wo_enforcer" does not
                    // contain enough information to be able to reject the winding order with which we
                    // attempt to insert _the first_ CDT triangle (i.e. its reversed).
                    // It is perfectly possible for this to remain null, which will happen the face being
                    // triangulated is the only face in the connected component.
                    halfedge_descriptor_t seed_halfedge = hmesh_t::null_halfedge();
                    std::unordered_set<face_descriptor_t> registered_neighbours; // those we have already saved in the wo-enforcer
                    const std::vector<halfedge_descriptor_t>& halfedges_around_face = cc_uptr->kernel_hmesh_data.mesh.get_halfedges_around_face(*f);

                    // for each halfedge of face
                    for (std::vector<halfedge_descriptor_t>::const_iterator hiter = halfedges_around_face.begin(); hiter != halfedges_around_face.end(); ++hiter) {

                        halfedge_descriptor_t h = *hiter;
                        halfedge_descriptor_t opph = cc_uptr->kernel_hmesh_data.mesh.opposite(h);
                        face_descriptor_t neigh = cc_uptr->kernel_hmesh_data.mesh.face(opph);
                        const bool neighbour_exists = (neigh != hmesh_t::null_face());

                        // neighbour exists and we have not already registered it into the enforcer
                        if (neighbour_exists && registered_neighbours.count(neigh) == 0) {

                            if (seed_halfedge == hmesh_t::null_halfedge()) {
                                seed_halfedge = h; // set once
                            } 

                            //
                            // insert the neighbour into "wo_enforcer", the stored winding order information.
                            // will prevent us from inserting triangles with the incorrect orientation.
                            //

                            const std::vector<vertex_descriptor_t>& vertices_around_neighbour = cc_uptr->kernel_hmesh_data.mesh.get_vertices_around_face(neigh);

                            std::vector<vertex_descriptor_t> remapped_descrs; // from CC to enforcer

                            // for each vertex around neighbour
                            for (std::vector<vertex_descriptor_t>::const_iterator neigh_viter = vertices_around_neighbour.cbegin(); neigh_viter != vertices_around_neighbour.cend(); ++neigh_viter) {

                                std::map<vertex_descriptor_t, vertex_descriptor_t>::const_iterator cc_to_enforcer_vmap_iter = cc_to_enforcer_vmap.find(*neigh_viter);

                                if (cc_to_enforcer_vmap_iter == cc_to_enforcer_vmap.cend()) {

                                    const vec3& neigh_vertex_coords = cc_uptr->kernel_hmesh_data.mesh.vertex(*neigh_viter);

                                    const vertex_descriptor_t woe_vdescr = wo_enforcer.add_vertex(neigh_vertex_coords);

                                    enforcer_to_cc_vmap[woe_vdescr] = (*neigh_viter);
                                    cc_to_enforcer_vmap_iter = cc_to_enforcer_vmap.insert(std::make_pair(*neigh_viter, woe_vdescr)).first;
                                }

                                MCUT_ASSERT(cc_to_enforcer_vmap_iter != cc_to_enforcer_vmap.cend());

                                remapped_descrs.push_back(cc_to_enforcer_vmap_iter->second);
                            }

                            face_descriptor_t nfd = wo_enforcer.add_face(remapped_descrs);

                            MCUT_ASSERT(nfd != hmesh_t::null_face());
                        }

                        registered_neighbours.insert(neigh);
                    }

                    // copy/get face vertices and save mapping
                    // =======================================

                    std::map<uint32_t, vertex_descriptor_t> cdt_to_enforcer_vmap;
                    std::map<vertex_descriptor_t, uint32_t> enforcer_to_cdt_vmap;

                    for (uint32_t v = 0; v < face_vertex_count; ++v) {

                        const vertex_descriptor_t cc_face_vertex_descr = vertices_around_face[v];

                        //face_vertex_coords_3d[v] = cc_uptr->kernel_hmesh_data.mesh.vertex(cc_face_vertex_descr);

                        std::map<vertex_descriptor_t, vertex_descriptor_t>::const_iterator fiter = cc_to_enforcer_vmap.find(cc_face_vertex_descr);

                        if (fiter == cc_to_enforcer_vmap.cend()) {
                            vertex_descriptor_t vd = wo_enforcer.add_vertex(face_vertex_coords_3d[v]);
                            fiter = cc_to_enforcer_vmap.insert(std::make_pair(cc_face_vertex_descr, vd)).first;
                        }

                        enforcer_to_cc_vmap[fiter->second] = fiter->first;
                        cdt_to_enforcer_vmap[v] = fiter->second;
                        enforcer_to_cdt_vmap[fiter->second] = v;
                    }

                    //
                    // Handle duplicate vertices in face, which [will] occur if we had a
                    // partial cut between the input source-mesh and cut-mesh.
                    //

                    // Duplicates that exist before we do triangulation
                    const cdt::duplicates_info_t duplicates_info_pre = cdt::find_duplicates<double>(
                        face_polygon_vertices.begin(),
                        face_polygon_vertices.end(),
                        cdt::get_x_coord_vec2d<double>,
                        cdt::get_y_coord_vec2d<double>);

                    const bool have_duplicates = duplicates_info_pre.duplicates.empty() == false;

                    if (have_duplicates) {
                        /*
                            For each duplicate vertex (they come in pairs 'a' and 'b') -> cur
                                1. get prev and next vertex from 'a' and 'b'
                                    - (we actually need to pick the point whose adjacent vectors form the largest angle 'b') -> x
                                2. compute vector from prev to x -> "px"
                                3. compute vector from 'x' to next -> "xn"
                                4. compute perpendicular vector in counter-clockwise dir for "px" and "xn"
                                5. compute average vector of "px" and "xn", and normalize it -> v
                                6. compute maximum perpendicular vector length -> s
                                    - Use mean edge-length scaled by perturbation constant.
                                7. for i in {a, b}
                                    1. construct segment from 'i' to 'i'+'v'*'s'
                                    2. if segment intersects any edge in face
                                        a. compute intersection point as paramater t
                                        b. update tval as tval = min(t, 1);
                                8. shift 'a' by a = a+(v*(s*(tval/2)))
                                9. shift 'b' by b = b+(-v*(s*(tval/2)))

                                # A note regarding steps 8 and 9:
                                # the 2 is a hardcoded constant telling us to
                                # shift 'a' (or 'b') halfway to the position of
                                # the intersected edge of the face to be triangulated
                                # (if we do indeed intersect an edge with the
                                # scaled perp vector). Otherwise, the 'a' is
                                # shifted by the prescribed amount of "v*s"
                        */

                        for (std::vector<std::size_t>::const_iterator dupl_iter = duplicates_info_pre.duplicates.cbegin();
                             dupl_iter != duplicates_info_pre.duplicates.cend(); ++dupl_iter) {

                            const std::uint32_t dupl_vertex_idx = (std::uint32_t)(*dupl_iter); // ... in the cc-face to be triangulated
                            const std::uint32_t dupl_vertex_other_idx = (std::uint32_t)SAFE_ACCESS(duplicates_info_pre.mapping, dupl_vertex_idx);

                            // lets rename the vars
                            const std::uint32_t dupl_vtx_pair_arr[] = { dupl_vertex_idx, dupl_vertex_other_idx };

                            //
                            // compute average vector of the two (directed) edges
                            // eminating from each duplicate vertex e.g. "prev - a" and "next - a"
                            //
                            for (std::uint32_t dv_iter = 0; dv_iter < 2; ++dv_iter) {

                                const std::int32_t cur_dupl_vertex_id = dupl_vtx_pair_arr[dv_iter];
                                const std::uint32_t prev_vtx_id = wrap_integer(cur_dupl_vertex_id - 1, 0, face_polygon_vertices.size() - 1);
                                const std::uint32_t next_vtx_id = wrap_integer(cur_dupl_vertex_id + 1, 0, face_polygon_vertices.size() - 1);

                                const std::int32_t cur_dupl_partner_id = dupl_vtx_pair_arr[(dv_iter + 1) % 2];

                                vec2& cur_dupl_vertex_coords = face_polygon_vertices[dupl_vertex_idx]; // will be modified by shifting
                                const vec2& prev_vtx_coords = face_polygon_vertices[prev_vtx_id];
                                const vec2& next_vtx_coords = face_polygon_vertices[next_vtx_id];

                                const vec2 cur_to_prev = prev_vtx_coords - cur_dupl_vertex_coords;
                                const vec2 cur_to_next = next_vtx_coords - cur_dupl_vertex_coords;

                                // positive-value if in CCW order (sign_t::ON_NEGATIVE_SIDE)
                                // negative-value if in CW order (sign_t::ON_POSITIVE_SIDE)
                                // zero if collinear (sign_t::ON_ORIENTED_BOUNDARY)
                                const double orient2d_res = orient2d(cur_dupl_vertex_coords, next_vtx_coords, prev_vtx_coords);
                                const sign_t orient2d_sgn = sign(orient2d_res);

                                const double cur_to_prev_sqr_len = squared_length(cur_to_prev);
                                const double cur_to_next_sqr_len = squared_length(cur_to_next);

                                vec2 orth_perp_dir;

                                // the error bounds for the orient2d predicate
                                const double ccwerrboundA = 3.3306690738754716e-16;
                                const double errbound = ccwerrboundA * 1e6;

                                // We use "errbound" (instead of "orient2d_res") to determine if the incident edges
                                // are parallel because that leaves us with sufficient precision in the coordinate
                                // to compute the orthogonal perturbation vector as the mean of the two incident edges
                                // eminating from the current vertex. "orient2d()" is exact and may depend on
                                // calculation with numbers that are lower than "errbound", which would complicate
                                // our ability to compute the orthogonal perturbation vector
                                //
                                const bool incident_edges_are_parallel = std::fabs(orient2d_res) <= std::fabs(errbound);

                                if (incident_edges_are_parallel) {
                                    //
                                    // pick the longest of the two incident edges and compute the
                                    // orthogonal perturbation vector as the counter-clockwise rotation
                                    //

                                    // flip sign so that the edge is in the CCW dir to give "prev->cur"
                                    vec2 edge_vec(-cur_to_prev.x(), -cur_to_prev.y());

                                    if (cur_to_prev_sqr_len > cur_to_next_sqr_len) {
                                        edge_vec = cur_to_next; // pick shortest (NOTE: "cur_to_next" is already in CCW dir)
                                    }

                                    const vec2 edge_vec_rotated90(-edge_vec.y(), edge_vec.x());

                                    orth_perp_dir = edge_vec_rotated90;
                                } else {
                                    //
                                    // compute the orthogonal perturbation vector as the average of the
                                    // two incident edges emminating from the current vertex
                                    //
                                    const vec2 avg_vector = (cur_to_prev + cur_to_next) / 2.0;
                                    orth_perp_dir = avg_vector;
                                }

                                //
                                // We will now construct a segment from "cur" to "cur + orth_perp_dir*errbound".
                                // This segment will be tested against every edge in the face using
                                // the (potentially modified) vertices in "face_polygon_vertices".
                                //
                                const vec2& segment_start = cur_dupl_vertex_coords;
                                const vec2 shift = (normalize(orth_perp_dir) * (0.1 * length(orth_perp_dir)));
                                const vec2 segment_end = segment_start + shift;
                                double tval = 1.0; // start of segment, which is "cur_dupl_vertex_coords"

                                // for each edge of face to be triangulated (number of vertices == number of edges)
                                for (std::uint32_t viter = 0; viter < face_vertex_count; ++viter) {
                                    const std::uint32_t edge_start_idx = viter;
                                    const std::uint32_t edge_end_idx = (viter + 1) % face_vertex_count;

                                    if (edge_start_idx == (uint32_t)cur_dupl_vertex_id || edge_end_idx == (uint32_t)cur_dupl_vertex_id || edge_start_idx == (uint32_t)cur_dupl_partner_id || edge_end_idx == (uint32_t)cur_dupl_partner_id) {
                                        continue; // impossible to intersect incident edges
                                    }

                                    const vec2& edge_start_coords = face_polygon_vertices[edge_start_idx];
                                    const vec2& edge_end_coords = face_polygon_vertices[edge_end_idx];

                                    double segment_tval; // parameter along segment
                                    double edge_tval; // parameter along edge
                                    vec2 intersection_point; // if it exists (not used)

                                    const char result = compute_segment_intersection(
                                        segment_start, segment_end, edge_start_coords, edge_end_coords,
                                        intersection_point, segment_tval, edge_tval);

                                    MCUT_ASSERT(result != 'e'); // imagine when this occurs

                                    if (result == '1') {
                                        tval = std::min(tval, segment_tval);
                                    }
                                }

                                if (tval != 1.0) {
                                    tval /= 2.0; // halfway from intersection
                                }

                                vec2 shift_final;

                                // final shifted position of vertex
                                if (orient2d_sgn == sign_t::ON_POSITIVE_SIDE ||
                                    // if collinear then we will be using the rotated
                                    // incident incident edge, whose directiion will
                                    // point torward he correct side of the polygon.
                                    // So shift as normal
                                    (orient2d_sgn == sign_t::ON_ORIENTED_BOUNDARY)) {
                                    shift_final = (shift * tval);
                                } else // if(orient2d_sgn == sign_t::ON_NEGATIVE_SIDE)
                                {
                                    shift_final = (shift * tval * -1.0); // opposite dir
                                }

                                cur_dupl_vertex_coords = cur_dupl_vertex_coords + shift_final;

                            } // for (std::uint32_t dv_iter = 0; dv_iter < 2; ++dv_iter) {
                        } // for (std::vector<std::size_t>::const_iterator dupl_iter = duplicates_info_pre.duplicates.cbegin(); ...
                    } // if (have_duplicates) {

                    //
                    // create the constraint edges for CDT, which is just our face's edges
                    //
                    for (uint32_t i = 0; i < face_vertex_count; ++i) {
                        face_polygon_edges.emplace_back(cdt::edge_t(i, (i + 1) % face_vertex_count));
                    }

                    // check for duplicate vertices again
                    const cdt::duplicates_info_t duplicates_info_post = cdt::find_duplicates<double>(
                        face_polygon_vertices.begin(),
                        face_polygon_vertices.end(),
                        cdt::get_x_coord_vec2d<double>,
                        cdt::get_y_coord_vec2d<double>);

                    if (!duplicates_info_post.duplicates.empty()) {
                        // probably a good idea to email the author
                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_ERROR, 0,
                            MC_DEBUG_SEVERITY_HIGH, "face " + std::to_string(*f) + " has duplicate vertices that could not be resolved (bug)");
                        continue;
                    }

                    constrained_cdt.insert_vertices(face_polygon_vertices);
                    constrained_cdt.insert_edges(face_polygon_edges);

                    // triangulation done here!
                    // NOTE: it seems that the triangulation can be either CW or CCW.
                    constrained_cdt.erase_outer_triangles();

                    const std::unordered_map<cdt::edge_t, std::vector<cdt::edge_t>> tmp = cdt::edge_to_pieces_mapping(constrained_cdt.pieceToOriginals);
                    const std::unordered_map<cdt::edge_t, std::vector<std::uint32_t>> edgeToSplitVerts = cdt::get_edge_to_split_vertices_map(tmp, constrained_cdt.vertices);

                    if (!cdt::check_topology(constrained_cdt)) {

                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "triangulation on face " + std::to_string(*f) + "has wrong topology");
                    }

                    if (constrained_cdt.triangles.empty()) {
                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "triangulation on face " + std::to_string(*f) + " produced zero faces");
                    }

                    // save the triangulation
                    // ======================

                    //const uint32_t face_resulting_triangle_count = (uint32_t)constrained_cdt.triangles.size();

                    //
                    // It is preferrable to without the assumption that the resulting CDT triangles
                    // have the proper winding order, which matches our faces.
                    // Thus, we have to use a BFS flood-fill strategy to "walk" the
                    // triangles of the CDT, thereby inserting then one-by-one into our
                    // triangulated CC mesh without violating the "enforcer"
                    //

                    // map vertices to CDT triangles
                    std::vector<std::vector<uint32_t>> vertex_to_triangle_map(face_vertex_count, std::vector<uint32_t>());

                    // for each CDT triangle
                    for (uint32_t t = 0; t < (uint32_t)constrained_cdt.triangles.size(); ++t) {
                        const cdt::triangle_t& triangle = SAFE_ACCESS(constrained_cdt.triangles, t);

                        for (uint32_t v = 0; v < 3; v++) {
                            const uint32_t vertex_id = SAFE_ACCESS(triangle.vertices, v);
                            std::vector<uint32_t>& incident_triangles = SAFE_ACCESS(vertex_to_triangle_map, vertex_id);
                            incident_triangles.push_back(t);
                        }
                    }

                    // start with any boundary edge (AKA constraint/fixed edge)

                    std::unordered_set<cdt::edge_t>::const_iterator fixed_edge_iter = constrained_cdt.fixedEdges.cbegin();

                    // NOTE: in the case that we DO NOT have a seed halfedge, then the current face
                    // that we are triangulating is the only face in the connected component and therefore
                    // it has no neighbours. In this instance, the winding order of the produced triangles
                    // is dependent on the CDT triangulator. The MCUT frontend will at best be able to
                    // ensure that all CDT triangles have consistent winding order (even if the triangulator
                    // produced mixed winding orders between the resulting triangles) but we cannot guarrantee
                    // the "front-facing" side of triangulated face will match that of the non-triangulated
                    // face from the connected component. We leave that to the user to fix upon inspection.
                    //
                    const bool have_seed_halfedge = seed_halfedge != hmesh_t::null_halfedge();

                    if (have_seed_halfedge) {
                        // if the seend halfedge exists then the triangulated face must have
                        // atleast one neighbour
                        MCUT_ASSERT(wo_enforcer.number_of_faces() != 0);

                        // source and target descriptor in the connected component
                        const vertex_descriptor_t seed_halfedge_src = cc_uptr->kernel_hmesh_data.mesh.source(seed_halfedge);
                        const vertex_descriptor_t seed_halfedge_tgt = cc_uptr->kernel_hmesh_data.mesh.target(seed_halfedge);

                        // source and target descriptor in the face
                        const vertex_descriptor_t woe_src = SAFE_ACCESS(cc_to_enforcer_vmap, seed_halfedge_src);
                        const uint32_t cdt_src = SAFE_ACCESS(enforcer_to_cdt_vmap, woe_src);
                        const vertex_descriptor_t woe_tgt = SAFE_ACCESS(cc_to_enforcer_vmap, seed_halfedge_tgt);
                        const uint32_t cdt_tgt = SAFE_ACCESS(enforcer_to_cdt_vmap, woe_tgt);

                        // find the fixed edge in the CDT matching the vertices of the seed halfedge

                        fixed_edge_iter = std::find_if(
                            constrained_cdt.fixedEdges.cbegin(),
                            constrained_cdt.fixedEdges.cend(),
                            [&](const cdt::edge_t& e) {
                                return (e.v1() == cdt_src && e.v2() == cdt_tgt) || (e.v2() == cdt_src && e.v1() == cdt_tgt);
                            });

                        MCUT_ASSERT(fixed_edge_iter != constrained_cdt.fixedEdges.cend());
                    }

                    // must always exist since cdt edge ultimately came from CC and
                    // due to the fact that we have inserted edges into the CDT
                    MCUT_ASSERT(fixed_edge_iter != constrained_cdt.fixedEdges.cend());

                    // get the two vertices of the "seed" fixed edge (indices into CDT)
                    const std::uint32_t fixed_edge_vtx0_id = fixed_edge_iter->v1();
                    const std::uint32_t fixed_edge_vtx1_id = fixed_edge_iter->v2();

                    // since these vertices share an edge, they will share a triangle in the CDT
                    // So lets get that shared triangle, which will be the seed triangle for the
                    // traversal process, which we will use to walk and insert triangles into
                    // the output user array
                    const std::vector<std::uint32_t>& fixed_edge_vtx0_tris = SAFE_ACCESS(vertex_to_triangle_map, fixed_edge_vtx0_id);
                    MCUT_ASSERT(fixed_edge_vtx0_tris.empty() == false);
                    const std::vector<std::uint32_t>& fixed_edge_vtx1_tris = SAFE_ACCESS(vertex_to_triangle_map, fixed_edge_vtx1_id);
                    MCUT_ASSERT(fixed_edge_vtx1_tris.empty() == false);

                    std::uint32_t fix_edge_seed_triangle = cdt::null_neighbour;

                    for (std::vector<std::uint32_t>::const_iterator it = fixed_edge_vtx0_tris.begin(); it != fixed_edge_vtx0_tris.end(); ++it) {

                        if (*it == cdt::null_neighbour) {
                            continue;
                        }

                        if (std::find(fixed_edge_vtx1_tris.begin(), fixed_edge_vtx1_tris.end(), *it) != fixed_edge_vtx1_tris.end()) {
                            fix_edge_seed_triangle = *it;
                        }
                    }

                    MCUT_ASSERT(fix_edge_seed_triangle != cdt::null_neighbour);

                    std::stack<std::uint32_t> seeds(std::deque<std::uint32_t>(1, fix_edge_seed_triangle));

                    std::unordered_set<std::uint32_t> traversed;

                    while (!seeds.empty()) {

                        const std::uint32_t curr_triangle_id = seeds.top();
                        seeds.pop();

                        traversed.insert(curr_triangle_id); // those we have walked

                        const cdt::triangle_t& triangle = constrained_cdt.triangles[curr_triangle_id];

                        //
                        // insert current triangle into our triangulated CC mesh
                        //

                        for (int i = 0; i < 3; i++)
                            vertex_is_used[triangle.vertices[i]] = true; // marked

                        // convert to local descriptors
                        std::vector<vd_t> triangle_descriptors = {
                            vd_t(cdt_to_enforcer_vmap.at(triangle.vertices[0])),
                            vd_t(cdt_to_enforcer_vmap.at(triangle.vertices[1])),
                            vd_t(cdt_to_enforcer_vmap.at(triangle.vertices[2]))
                        };

                        face_triangulation_indices.emplace_back(triangle.vertices[0]);
                        face_triangulation_indices.emplace_back(triangle.vertices[1]);
                        face_triangulation_indices.emplace_back(triangle.vertices[2]);

                        // check that the winding order matches the triangulated face's order
                        const bool is_insertible = wo_enforcer.is_insertable(triangle_descriptors);

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
                        fd_t fd = wo_enforcer.add_face(triangle_descriptors); // keep track of added triangles from CDT

                        // if this fails then CDT gave us a strange triangulation e.g.
                        // duplicate triangles with opposite winding order
                        if (fd == hmesh_t::null_face()) {
                            // simply remove/ignore the offending triangle
                            face_triangulation_indices.pop_back();
                            face_triangulation_indices.pop_back();
                            face_triangulation_indices.pop_back();

                            const std::string msg = "triangulation on face " + std::to_string(*f) + " produced invalid triangles tht could not be inserted";

                            context_uptr->log(
                                MC_DEBUG_SOURCE_KERNEL,
                                MC_DEBUG_TYPE_OTHER, 0,
                                MC_DEBUG_SEVERITY_HIGH, msg);
                        }

                        //
                        // add neighbouring triangles into queue/stack
                        //

                        for (std::uint32_t i(0); i < std::uint32_t(3); ++i) {
                            const cdt::edge_t opEdge(triangle.vertices[cdt::ccw(i)], triangle.vertices[cdt::cw(i)]);
                            if (constrained_cdt.fixedEdges.count(opEdge)) {
                                continue; // current edge is fixed edge so do not add neigher
                            }

                            const std::uint32_t iN = triangle.neighbors[cdt::get_opposite_neighbour_from_vertex(i)];

                            if (iN != cdt::null_neighbour && traversed.count(iN) == 0) {
                                seeds.push(iN);
                            }
                        }
                    } // while (!seeds.empty()) {

                    // every triangle in the finalized CDT must be walked!
                    MCUT_ASSERT(traversed.size() == constrained_cdt.triangles.size());

#if 0
                    {
                        const std::string fname("triangulation-" + std::to_string(*f) + ".obj");
                        printf("dump %s\n", fname.c_str());

                        std::ofstream file(fname);

                        for (int i = 0; i < constrained_cdt.vertices.size(); ++i) {
                            auto vert = constrained_cdt.vertices[i];
                            file << "v " << vert.x() << " " << vert.y() << " " << 0.0 << "\n";
                        }

                        for (uint32_t i = 0; i < constrained_cdt.triangles.size(); ++i) {
                            // a triangle computed from CDT
                            const cdt::triangle_t& triangle = constrained_cdt.triangles[i];
                            file << "f " << triangle.vertices[0] + 1 << " " << triangle.vertices[1] + 1 << " " << triangle.vertices[2] + 1 << "\n";
                        }

                        file.close();

                        std::ofstream g("face-" + std::to_string(*f) + ".off");
                        g << "OFF\n";
                        g << face_vertex_count << " 1 0\n";
                        for (int i = 0; i < face_vertex_count; ++i) {
                            auto vert = face_polygon_vertices[i];
                            g << vert[0] << " " << vert[1] << " " << 0 << "\n";
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

                            h << i << " " << (i + 1) % face_vertex_count << "\n";
                        }

                        h.close();
                    }
#endif
                    // swap local triangle indices to global index values (in CC) and save
                    // ===================================================================

                    for (std::uint32_t i = 0; i < (std::uint32_t)vertex_is_used.size(); ++i) {
                        if (vertex_is_used[i] != true) {
                            context_uptr->log(
                                MC_DEBUG_SOURCE_KERNEL,
                                MC_DEBUG_TYPE_OTHER, 0,
                                MC_DEBUG_SEVERITY_HIGH, "triangulation on face " + std::to_string(*f) + " did not use vertex " + std::to_string(i));
                        }
                    }

                    const uint32_t face_triangulation_indices_count = (uint32_t)face_triangulation_indices.size();

                    for (uint32_t i = 0; i < face_triangulation_indices_count; ++i) {
                        const uint32_t local_idx = face_triangulation_indices[i]; // id local within the current face that we are triangulating
                        const uint32_t global_idx = (uint32_t)vertices_around_face[local_idx]; // cc_uptr->indexArrayMesh.pFaceIndices[(std::size_t)face_indices_offset + local_idx];

                        face_triangulation_indices[(std::size_t)i] = global_idx; // id in the connected component (mesh)
                    }

                    cc_uptr->constrained_delaunay_triangulation_indices.insert(
                        cc_uptr->constrained_delaunay_triangulation_indices.end(),
                        face_triangulation_indices.begin(),
                        face_triangulation_indices.end());
                } //  if (face_vertex_count == 3)

                face_indices_offset += face_vertex_count;
            }

            MCUT_ASSERT(cc_uptr->constrained_delaunay_triangulation_indices.size() >= 3);

        } // if(cc_uptr->indexArrayMesh.numTriangleIndices == 0)

        const uint32_t num_triangulation_indices = (uint32_t)cc_uptr->constrained_delaunay_triangulation_indices.size();

        if (pMem == nullptr) // client pointer is null (asking for size)
        {
            MCUT_ASSERT(num_triangulation_indices >= 3);
            *pNumBytes = num_triangulation_indices * sizeof(uint32_t); // each each vertex has a map value (intersection point == uint_max)
        } else {
            MCUT_ASSERT(num_triangulation_indices >= 3);

            if (bytes > num_triangulation_indices * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % (sizeof(uint32_t)) != 0 || (bytes / sizeof(uint32_t)) % 3 != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            memcpy(pMem, reinterpret_cast<void*>(cc_uptr->constrained_delaunay_triangulation_indices.data()), bytes);
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