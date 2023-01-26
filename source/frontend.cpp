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

template <class InputIt, class OutputIt>
OutputIt partial_sum(InputIt first, InputIt last, OutputIt d_first)
{
    if (first == last)
        return d_first;

    typename std::iterator_traits<InputIt>::value_type sum = *first;
    *d_first = sum;

    while (++first != last) {
        sum = sum + *first;
        *++d_first = sum;
    }

    return ++d_first;
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

    std::unique_ptr<context_t>& context_uptr = context_entry_iter->second;

    std::map<McConnectedComponent, std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>>::iterator cc_entry_iter = context_uptr->connected_components.find(connCompId);

    if (cc_entry_iter == context_uptr->connected_components.cend()) {
        throw std::invalid_argument("invalid connected component");
    }

    std::unique_ptr<connected_component_t, void (*)(connected_component_t*)>& cc_uptr = cc_entry_iter->second;

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

            float* casted_ptr = reinterpret_cast<float*>(pMem);

            TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT");

#if defined(MCUT_MULTI_THREADED)
            {
                typedef std::tuple<uint64_t> OutputStorageTypesTuple;
                typedef vertex_array_iterator_t InputStorageIteratorType;

                auto fn_copy_vertex_coords = [&casted_ptr, &cc_uptr, &num_vertices_to_copy](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) -> OutputStorageTypesTuple {
                    OutputStorageTypesTuple local_output;
                    uint64_t& stub_LOCAL = std::get<0>(local_output);

                    // thread starting offset (in vertex count) in the "array of vertices"
                    const uint64_t base_offset = std ::distance(cc_uptr->kernel_hmesh_data.mesh.vertices_begin(), block_start_);

                    uint64_t elem_offset = base_offset * 3;

                    for (InputStorageIteratorType vertex_iter = block_start_; vertex_iter != block_end_; ++vertex_iter) {

                        if ((elem_offset / 3) == num_vertices_to_copy) {
                            break; // reach what the user asked for
                        }

                        const vertex_descriptor_t descr = *vertex_iter;
                        const vec3& coords = cc_uptr->kernel_hmesh_data.mesh.vertex(descr);

                        // for each component of coordinate
                        for (int i = 0; i < 3; ++i) {
                            const float val = static_cast<float>(coords[i]);
                            *(casted_ptr + elem_offset) = val;
                            elem_offset += 1;
                        }
                    }

                    stub_LOCAL = elem_offset;

                    return local_output;
                };

                std::vector<std::future<OutputStorageTypesTuple>> futures;
                OutputStorageTypesTuple partial_res;

                parallel_fork_and_join(
                    context_uptr->scheduler,
                    cc_uptr->kernel_hmesh_data.mesh.vertices_begin(),
                    cc_uptr->kernel_hmesh_data.mesh.vertices_end(),
                    cc_uptr->kernel_hmesh_data.mesh.number_of_vertices() / context_uptr->scheduler.get_num_threads(),
                    fn_copy_vertex_coords,
                    partial_res, // output computed by master thread
                    futures);

                // const uint64_t& final_offset_MASTER_THREAD_LOCAL = std::get<0>(partial_res);

                // wait for all worker-threads to finish copies
                for (uint32_t i = 0; i < (uint32_t)futures.size(); ++i) {
                    futures[i].wait();
                }

                MCUT_ASSERT((std::get<0>(partial_res) * sizeof(float)) <= allocated_bytes);
            }
#else // #if defined(MCUT_MULTI_THREADED)
            uint64_t elem_offset = 0;
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
#endif // #if defined(MCUT_MULTI_THREADED)
            TIMESTACK_POP(); // TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT");
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

            double* casted_ptr = reinterpret_cast<double*>(pMem);
            TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE");
#if defined(MCUT_MULTI_THREADED)
            {
                typedef std::tuple<uint64_t> OutputStorageTypesTuple;
                typedef vertex_array_iterator_t InputStorageIteratorType;

                auto fn_copy_vertex_coords = [&casted_ptr, &cc_uptr, &num_vertices_to_copy](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) {
                    // thread starting offset (in vertex count) in the "array of vertices"
                    const uint64_t base_offset = std ::distance(cc_uptr->kernel_hmesh_data.mesh.vertices_begin(), block_start_);

                    uint64_t elem_offset = base_offset * 3;

                    for (InputStorageIteratorType vertex_iter = block_start_; vertex_iter != block_end_; ++vertex_iter) {

                        if ((elem_offset / 3) == num_vertices_to_copy) {
                            break; // reach what the user asked for
                        }

                        const vertex_descriptor_t descr = *vertex_iter;
                        const vec3& coords = cc_uptr->kernel_hmesh_data.mesh.vertex(descr);

                        // for each component of coordinate
                        for (int i = 0; i < 3; ++i) {
                            const double val = static_cast<double>(coords[i]);
                            *(casted_ptr + elem_offset) = val;
                            elem_offset += 1;
                        }
                    }
                };

                std::vector<std::future<OutputStorageTypesTuple>> futures;

                parallel_for(
                    context_uptr->scheduler,
                    cc_uptr->kernel_hmesh_data.mesh.vertices_begin(),
                    cc_uptr->kernel_hmesh_data.mesh.vertices_end(),
                    fn_copy_vertex_coords);
            }
#else // #if defined(MCUT_MULTI_THREADED)
            uint64_t elem_offset = 0;
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
#endif // #if defined(MCUT_MULTI_THREADED)

            TIMESTACK_POP(); // TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE");
        }
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE: {
        TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_FACE");
        if (pMem == nullptr) { // querying for number of bytes
            uint32_t num_indices = 0;

#if defined(MCUT_MULTI_THREADED)
            {
                // each worker-thread will count the number of indices according
                // to the number of faces in its range/block. The master thread
                // will then sum the total from all threads

                // typedef std::tuple<uint32_t> OutputStorageTypesTuple; // store number of indices computed by worker
                typedef face_array_iterator_t InputStorageIteratorType;

                auto fn_count_indices = [&cc_uptr](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) {
                    uint32_t num_indices_LOCAL = 0;

                    // thread starting offset (in vertex count) in the "array of vertices"
                    // const uint64_t face_base_offset = std::distance(cc_uptr->kernel_hmesh_data.mesh.faces_begin(), block_start_);

                    for (InputStorageIteratorType fiter = block_start_; fiter != block_end_; ++fiter) {

                        const uint32_t num_vertices_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_vertices_around_face(*fiter);

                        MCUT_ASSERT(num_vertices_around_face >= 3);

                        num_indices_LOCAL += num_vertices_around_face;
                    }

                    return num_indices_LOCAL;
                };

                std::vector<std::future<uint32_t>> futures;
                uint32_t partial_res;

                parallel_fork_and_join(
                    context_uptr->scheduler,
                    cc_uptr->kernel_hmesh_data.mesh.faces_begin(),
                    cc_uptr->kernel_hmesh_data.mesh.faces_end(),
                    0,
                    fn_count_indices,
                    partial_res, // output computed by master thread
                    futures);

                const uint32_t& num_indices_MASTER_THREAD_LOCAL = partial_res;
                num_indices += num_indices_MASTER_THREAD_LOCAL;

                // wait for all worker-threads to finish copies
                for (uint32_t i = 0; i < (uint32_t)futures.size(); ++i) {
                    const uint32_t num_indices_THREAD_LOCAL = futures[i].get();
                    num_indices += num_indices_THREAD_LOCAL;
                }
            }
#else // #if defined(MCUT_MULTI_THREADED)
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_vertices_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_vertices_around_face(*fiter);

                MCUT_ASSERT(num_vertices_around_face >= 3);

                num_indices += num_vertices_around_face;
            }

#endif // #if defined(MCUT_MULTI_THREADED)
            MCUT_ASSERT(num_indices >= 3); // min is a triangle

            *pNumBytes = num_indices * sizeof(uint32_t);
        } else { // querying for data to copy back to user pointer
            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            const uint32_t num_indices_to_copy = bytes / sizeof(uint32_t);
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

#if defined(MCUT_MULTI_THREADED)
            {
                // step 1: compute array storing face sizes (recursive API call)
                // - for computing exclusive sum
                // step 2: compute exclusive sum array ( in spirit of std::exclusive_scan)
                // - for determining per-thread (work-block) output-array offsets
                // step 3: copy face indices into output array using offsets from previous steps
                // - final result that is stored in user-output array

                const uint32_t nfaces = cc_uptr->kernel_hmesh_data.mesh.number_of_faces();

                //
                // step 1
                //

                // If client already called mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, faceSizes.data(), NULL)
                // then we should have already cached the array of face sizes

                if (!cc_uptr->face_sizes_cache_initialized) { // fill the cache by calling the API, within the API!

                    // this is like resizing output array in the client application, after knowing the number of faces in CC
                    cc_uptr->face_sizes_cache.resize(nfaces);

                    const std::size_t num_bytes = nfaces * sizeof(uint32_t);

                    // recursive Internal API call: populate cache here, which also sets "cc_uptr->face_sizes_cache_initialized" to true
                    get_connected_component_data_impl(context, connCompId, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, num_bytes, cc_uptr->face_sizes_cache.data(), NULL);
                } else { // cache already initialized
                    MCUT_ASSERT(cc_uptr->face_sizes_cache.empty() == false);
                    MCUT_ASSERT(cc_uptr->face_sizes_cache_initialized == true);
                }

                //
                // step 2
                //
                std::vector<uint32_t> partial_sum_vec = cc_uptr->face_sizes_cache; // copy

                parallel_partial_sum(context_uptr->scheduler, partial_sum_vec.begin(), partial_sum_vec.end());

                //
                // step 3
                //

                auto fn_face_indices_copy = [&cc_uptr, &partial_sum_vec, &casted_ptr, &num_indices_to_copy](face_array_iterator_t block_start_, face_array_iterator_t block_end_) {
                    const uint32_t base_face_offset = std::distance(cc_uptr->kernel_hmesh_data.mesh.faces_begin(), block_start_);

                    MCUT_ASSERT(base_face_offset < (uint32_t)cc_uptr->face_sizes_cache.size());

                    // the first face in the range between block start and end
                    const uint32_t base_face_vertex_count = SAFE_ACCESS(cc_uptr->face_sizes_cache, base_face_offset);

                    const uint32_t partial_sum_vec_val = SAFE_ACCESS(partial_sum_vec, base_face_offset);
                    const uint32_t index_arr_base_offset = partial_sum_vec_val - base_face_vertex_count;
                    uint32_t index_arr_offset = index_arr_base_offset;

                    std::vector<vd_t> vertices_around_face; // tmp to prevent reallocations
                    vertices_around_face.reserve(3);

                    for (face_array_iterator_t f_iter = block_start_; f_iter != block_end_; ++f_iter) {

                        vertices_around_face.clear();
                        cc_uptr->kernel_hmesh_data.mesh.get_vertices_around_face(vertices_around_face, *f_iter);
                        const uint32_t num_vertices_around_face = (uint32_t)vertices_around_face.size();

                        MCUT_ASSERT(num_vertices_around_face >= 3u);

                        // for each vertex in face
                        for (uint32_t i = 0; i < num_vertices_around_face; ++i) {
                            const uint32_t vertex_idx = (uint32_t)SAFE_ACCESS(vertices_around_face, i);
                            *(casted_ptr + index_arr_offset) = vertex_idx;
                            ++index_arr_offset;

                            if (index_arr_offset == num_indices_to_copy) {
                                break;
                            }
                        }
                    }
                };

                parallel_for(
                    context_uptr->scheduler,
                    cc_uptr->kernel_hmesh_data.mesh.faces_begin(),
                    cc_uptr->kernel_hmesh_data.mesh.faces_end(),
                    fn_face_indices_copy,
                    (1 << 14)); // blocks until all work is done
            }
#else // #if defined(MCUT_MULTI_THREADED)

            std::vector<vd_t> vertices_around_face;
            uint32_t elem_offset = 0;

            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {

                vertices_around_face.clear();
                cc_uptr->kernel_hmesh_data.mesh.get_vertices_around_face(vertices_around_face, *fiter);
                const uint32_t num_vertices_around_face = (uint32_t)vertices_around_face.size();

                MCUT_ASSERT(num_vertices_around_face >= 3u);

                for (uint32_t i = 0; i < num_vertices_around_face; ++i) {
                    const uint32_t vertex_idx = (uint32_t)vertices_around_face[i];
                    *(casted_ptr + elem_offset) = vertex_idx;
                    ++elem_offset;

                    if (elem_offset == num_indices_to_copy) {
                        break;
                    }
                }
            }

            MCUT_ASSERT(elem_offset == num_indices_to_copy);
#endif // #if defined(MCUT_MULTI_THREADED)

        } // if (pMem == nullptr) { // querying for number of bytes

        TIMESTACK_POP(); // TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_FACE");

    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_SIZE: { // non-triangulated only (don't want to store redundant information)
        TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_FACE_SIZE");
        if (pMem == nullptr) {
            *pNumBytes = cc_uptr->kernel_hmesh_data.mesh.number_of_faces() * sizeof(uint32_t); // each face has a size (num verts)
        } else {
            if (bytes > cc_uptr->kernel_hmesh_data.mesh.number_of_faces() * sizeof(uint32_t)) {
                throw std::invalid_argument("out of bounds memory access");
            }

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

#if defined(MCUT_MULTI_THREADED)
            {
                const uint32_t face_count = cc_uptr->kernel_hmesh_data.mesh.number_of_faces();

                if (!cc_uptr->face_sizes_cache_initialized) { // init cache by storing data into it
                    // the code execution can also reach here because we asked for the cache
                    // to be populated in order to compute/copy the face indices in parallel.
                    // see: MC_CONNECTED_COMPONENT_DATA_FACE case
                    const bool cache_allocated_prior = !cc_uptr->face_sizes_cache.empty();

                    if (!cache_allocated_prior) {
                        cc_uptr->face_sizes_cache.resize(face_count);
                    }

                    auto fn_face_size = [&cc_uptr](std::vector<uint32_t>::iterator block_start_, std::vector<uint32_t>::iterator block_end_) {
                        const uint32_t face_base_offset = (uint32_t)std::distance(cc_uptr->face_sizes_cache.begin(), block_start_);
                        uint32_t face_offset = face_base_offset;

                        for (std::vector<uint32_t>::iterator fs_iter = block_start_; fs_iter != block_end_; ++fs_iter) {

                            const face_descriptor_t descr(face_offset);

                            const uint32_t num_vertices_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_vertices_around_face(descr);

                            MCUT_ASSERT(num_vertices_around_face >= 3);

                            *fs_iter = num_vertices_around_face;

                            face_offset++;
                        }
                    };

                    parallel_for(
                        context_uptr->scheduler,
                        cc_uptr->face_sizes_cache.begin(),
                        cc_uptr->face_sizes_cache.end(),
                        fn_face_size); // blocks until all work is done

                    cc_uptr->face_sizes_cache_initialized = true;
                }

                // the pointers are different if "cc_uptr->face_sizes_cache" is not
                // being populated in the current call
                const void* src_ptr = reinterpret_cast<void*>(&(cc_uptr->face_sizes_cache[0]));
                const void* dst_ptr = pMem;
                const bool writing_to_client_pointer = (src_ptr != dst_ptr);

                if (writing_to_client_pointer) // copy only if "casted_ptr" is client pointer
                {
                    // copy to user pointer
                    memcpy(casted_ptr, &(cc_uptr->face_sizes_cache[0]), sizeof(uint32_t) * face_count);
                }
            }
#else
            //
            uint64_t elem_offset = 0;

            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_vertices_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_vertices_around_face(*fiter);

                MCUT_ASSERT(num_vertices_around_face >= 3);

                *(casted_ptr + elem_offset) = num_vertices_around_face;
                ++elem_offset;
            }
#endif
        }

        TIMESTACK_POP(); // TIMESTACK_PUSH("MC_CONNECTED_COMPONENT_DATA_FACE_SIZE");
    } break;
    case MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE: {
        if (pMem == nullptr) {

            MCUT_ASSERT(pNumBytes != nullptr);

            uint32_t num_face_adjacent_face_indices = 0;

#if defined(MCUT_MULTI_THREADED)
            {
                // each worker-thread will count the number of indices according
                // to the number of faces in its range/block. The master thread
                // will then sum the total from all threads

                auto fn_count_faces_around_face = [&cc_uptr](face_array_iterator_t block_start_, face_array_iterator_t block_end_) {
                    uint32_t num_face_adjacent_face_indices_LOCAL = 0;

                    for (face_array_iterator_t fiter = block_start_; fiter != block_end_; ++fiter) {

                        const uint32_t num_faces_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_faces_around_face(*fiter, nullptr);
                        num_face_adjacent_face_indices_LOCAL += num_faces_around_face;
                    }

                    return num_face_adjacent_face_indices_LOCAL;
                };

                std::vector<std::future<uint32_t>> futures;
                uint32_t partial_res;

                parallel_fork_and_join(
                    context_uptr->scheduler,
                    cc_uptr->kernel_hmesh_data.mesh.faces_begin(),
                    cc_uptr->kernel_hmesh_data.mesh.faces_end(),
                    0,
                    fn_count_faces_around_face,
                    partial_res, // output computed by master thread
                    futures);

                const uint32_t& num_face_adjacent_face_indices_MASTER_THREAD_LOCAL = partial_res;
                num_face_adjacent_face_indices += num_face_adjacent_face_indices_MASTER_THREAD_LOCAL;

                // wait for all worker-threads to finish copies
                for (uint32_t i = 0; i < (uint32_t)futures.size(); ++i) {
                    const uint32_t num_face_adjacent_face_indices_THREAD_LOCAL = futures[i].get();
                    num_face_adjacent_face_indices += num_face_adjacent_face_indices_THREAD_LOCAL;
                }
            }
#else // #if defined(MCUT_MULTI_THREADED)
            for (face_array_iterator_t fiter = cc_uptr->kernel_hmesh_data.mesh.faces_begin(); fiter != cc_uptr->kernel_hmesh_data.mesh.faces_end(); ++fiter) {
                const uint32_t num_faces_around_face = cc_uptr->kernel_hmesh_data.mesh.get_num_faces_around_face(*fiter, nullptr);
                num_face_adjacent_face_indices += num_faces_around_face;
            }
#endif

            *pNumBytes = num_face_adjacent_face_indices * sizeof(uint32_t);
        } else {

            if (bytes % sizeof(uint32_t) != 0) {
                throw std::invalid_argument("invalid number of bytes");
            }

            uint64_t elem_offset = 0;
            uint32_t* casted_ptr = reinterpret_cast<uint32_t*>(pMem);

#if defined(MCUT_MULTI_THREADED)
            {
                // step 1: compute array storing face sizes (recursive API call)
                // - for computing exclusive sum
                // step 2: compute exclusive sum array ( in spirit of std::exclusive_scan)
                // - for determining per-thread (work-block) output-array offsets
                // step 3: copy adjacent face indices into output array using offsets from previous steps
                // - final result that is stored in user-output array

                const uint32_t nfaces = cc_uptr->kernel_hmesh_data.mesh.number_of_faces();

                //
                // step 1
                //

                // If client already called mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE, numBytes, faceAdjFaceSizes.data(), NULL)
                // then we should have already cached the array of face sizes

                if (!cc_uptr->face_adjacent_faces_size_cache_initialized) { // fill the cache by calling the API, within the API!

                    // this is like resizing output array in the client application, after knowing the number of faces in CC
                    cc_uptr->face_adjacent_faces_size_cache.resize(nfaces);

                    const std::size_t num_bytes = nfaces * sizeof(uint32_t);

                    // recursive Internal API call: populate cache here, which also sets "cc_uptr->face_sizes_cache_initialized" to true
                    // TODO: needs to be implemented!!!
                    get_connected_component_data_impl(
                        context,
                        connCompId,
                        MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE_SIZE,
                        num_bytes,
                        cc_uptr->face_adjacent_faces_size_cache.data(),
                        NULL);
                }

                MCUT_ASSERT(cc_uptr->face_adjacent_faces_size_cache.empty() == false);
                MCUT_ASSERT(cc_uptr->face_adjacent_faces_size_cache_initialized == true);

                //
                // step 2
                //
                std::vector<uint32_t> partial_sum_vec = cc_uptr->face_adjacent_faces_size_cache; // copy

                parallel_partial_sum(context_uptr->scheduler, partial_sum_vec.begin(), partial_sum_vec.end());

                //
                // step 3
                //

                auto fn_face_indices_copy = [&cc_uptr, &partial_sum_vec, &casted_ptr](face_array_iterator_t block_start_, face_array_iterator_t block_end_) {
                    const uint32_t base_face_offset = std::distance(cc_uptr->kernel_hmesh_data.mesh.faces_begin(), block_start_);

                    MCUT_ASSERT(base_face_offset < (uint32_t)cc_uptr->face_adjacent_faces_size_cache.size());

                    // the first face in the range between block start and end
                    const uint32_t base_face_vertex_count = SAFE_ACCESS(cc_uptr->face_adjacent_faces_size_cache, base_face_offset);

                    const uint32_t partial_sum_vec_val = SAFE_ACCESS(partial_sum_vec, base_face_offset);
                    const uint32_t index_arr_base_offset = partial_sum_vec_val - base_face_vertex_count;
                    uint32_t index_arr_offset = index_arr_base_offset;

                    std::vector<fd_t> faces_around_face; // tmp to prevent reallocations
                    faces_around_face.reserve(3);

                    for (face_array_iterator_t f_iter = block_start_; f_iter != block_end_; ++f_iter) {

                        faces_around_face.clear();
                        cc_uptr->kernel_hmesh_data.mesh.get_faces_around_face(faces_around_face, *f_iter);
                        const uint32_t num_faces_around_face = (uint32_t)faces_around_face.size();

                        MCUT_ASSERT(num_faces_around_face >= 0u);

                        // for each vertex in face
                        for (uint32_t i = 0; i < num_faces_around_face; ++i) {
                            const uint32_t face_idx = (uint32_t)SAFE_ACCESS(faces_around_face, i);
                            *(casted_ptr + index_arr_offset) = face_idx;
                            ++index_arr_offset;
                        }
                    }
                };

                parallel_for(
                    context_uptr->scheduler,
                    cc_uptr->kernel_hmesh_data.mesh.faces_begin(),
                    cc_uptr->kernel_hmesh_data.mesh.faces_end(),
                    fn_face_indices_copy); // blocks until all work is done
            }
#else // #if defined(MCUT_MULTI_THREADED)
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
#endif // #if defined(MCUT_MULTI_THREADED)
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

            // cc_uptr->face_adjacent_faces_size_cache_initialized

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
            // temp halfedge data structure whose in-built functionality helps us ensure that
            // the winding order that is computed by the constrained delaunay triangulator
            // is consistent with that of the connected component face we are triangulating.
            hmesh_t winding_order_enforcer;

            // add the face which contains the opposite winding order of the
            // connected component face face we are triangulating (we will use this to prevent inserting
            // reversed triangles). This is guarranteed to work because all triangles
            // are adjacent to the border for a constrained delaunay triangulation.
            // This is not true for a conforming delaunay triangulation.
            std::vector<vd_t> face_reversed;

            std::vector<vec2_<double>> face_polygon_vertices;
            std::vector<cdt::edge_t> face_polygon_edges;
            // list of indices which define all triangles that result from the CDT
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
                    winding_order_enforcer.reset();
                    face_reversed.resize(face_vertex_count);
                    face_polygon_edges.clear(); //.resize(face_vertex_count); // |edges| == |vertices|
                    face_polygon_vertices.resize(face_vertex_count);
                    face_triangulation_indices.clear();

                    // copy/get face vertices and save mapping
                    // =======================================

                    for (uint32_t v = 0; v < face_vertex_count; ++v) {

                        const vertex_descriptor_t face_vertex_idx((uint32_t)vertices_around_face[v]);
                        face_vertex_coords_3d[v] = cc_uptr->kernel_hmesh_data.mesh.vertex(face_vertex_idx);
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

                    cdt::triangulator_t<double> tri(cdt::vertex_insertion_order_t::AS_GIVEN); // memory is alway reallocated for this

                    // convert face vertex format & compute revered face
                    // =================================================

                    // the vertices are in counter-clockwise order (user provided order)
                    for (uint32_t i = 0; i < face_vertex_count; ++i) {

                        const vec2& coords = face_vertex_coords_2d[i];

                        face_polygon_vertices[i] = coords; // vec2_<double>::make(coords[0], coords[1]);

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
                        face_polygon_edges.emplace_back(cdt::edge_t(i, (i + 1) % face_vertex_count));
                    }

                    // prepare and do constrained delaunay triangulation
                    // =================================================

                    const cdt::duplicates_info_t duplInfo = cdt::remove_duplicates(face_polygon_vertices);

                    if (!duplInfo.duplicates.empty()) {
                        fprintf(stderr, "triangulation has duplicates\n");
                        // TODO: do stuff with "duplInfo"
                    }

                    tri.insert_vertices(face_polygon_vertices);
                    tri.insert_edges(face_polygon_edges);

                    // triangulation done here!
                    // NOTE: it seems that the triangulation can be either CW or CCW.
                    tri.erase_outer_triangles_and_holes();

                    const std::unordered_map<cdt::edge_t, std::vector<cdt::edge_t>> tmp = cdt::edge_to_pieces_mapping(tri.pieceToOriginals);
                    const std::unordered_map<cdt::edge_t, std::vector<std::uint32_t>> edgeToSplitVerts = cdt::get_edge_to_split_vertices_map(tmp, tri.vertices);

                    if (!cdt::check_topology(tri)) {

                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "triangulator_t on face " + std::to_string(*f) + "has wrong topology");
                    }

                    if (tri.triangles.empty()) {
                        context_uptr->log(
                            MC_DEBUG_SOURCE_KERNEL,
                            MC_DEBUG_TYPE_OTHER, 0,
                            MC_DEBUG_SEVERITY_NOTIFICATION, "triangulator_t on face " + std::to_string(*f) + "produced zero faces");
                    }

                    // save the triangulation
                    // ======================

                    const uint32_t face_resulting_triangle_count = (uint32_t)tri.triangles.size();

                    // TODO: rather than looping through triangles, maybe attempt walking them according to neighbour information
                    for (uint32_t i = 0; i < face_resulting_triangle_count; ++i) {

                        // a triangle computed from CDT
                        const cdt::triangle_t& triangle = tri.triangles[i];

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

                            for (int i = 0; i < tri.vertices.size(); ++i) {
                                auto vert = tri.vertices[i];
                                f << "v " << vert.x << " " << vert.y << " " << 0.0 << "\n";
                            }

                            for (uint32_t i = 0; i < face_resulting_triangle_count; ++i) {
                                // a triangle computed from CDT
                                const cdt::Triangle& triangle = tri.triangles[i];
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