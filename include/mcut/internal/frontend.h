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

/**
 * @file mcut.h
 * @author Floyd M. Chitalu
 * @date 11 July 2022
 *
 * @brief API-function implementations.
 *
 * NOTE: This header file defines the pre- and post-cutting processing of mesh
 * data, which includes any intermediate correctons/modifications to the user's
 * input meshes like 'polygon partitioning'.
 *
 */

#ifndef _FRONTEND_H_
#define _FRONTEND_H_

#include "mcut/mcut.h"

#include <map>
#include <memory>
#include <string>

#if defined(MCUT_MULTI_THREADED)
#include "mcut/internal/tpool.h"
#endif
#include "mcut/internal/kernel.h"

#if 0
// internal frontend data structure which we use to store connected component
// data that is computed by the kernel and requested by a client via the 
// "mcGetConnectedComponentData" function. So the "mcGetConnectedComponentData"
// function will read from this data structure (because halfedge meshes are only 
// used by the backend kernel for resolving the intersections). 
struct array_mesh_t {
    array_mesh_t() { }
    ~array_mesh_t()
    {
    }

    std::unique_ptr<double[]> pVertices;
    std::unique_ptr<uint32_t[]> pSeamVertexIndices;
    std::unique_ptr<uint32_t[]> pVertexMapIndices; // descriptor/index in original mesh (source/cut-mesh), each vertex has an entry
    std::unique_ptr<uint32_t[]> pFaceIndices;
    std::unique_ptr<uint32_t[]> pFaceMapIndices; // descriptor/index in original mesh (source/cut-mesh), each face has an entry
    std::unique_ptr<uint32_t[]> pFaceSizes;
    std::unique_ptr<uint32_t[]> pEdges;
    std::unique_ptr<uint32_t[]> pFaceAdjFaces;
    std::unique_ptr<uint32_t[]> pFaceAdjFacesSizes;
    std::unique_ptr<uint32_t[]> pTriangleIndices; // same as "pFaceIndices" but guaranteed to be only triangles

    uint32_t numVertices = 0;
    uint32_t numSeamVertexIndices = 0;
    uint32_t numFaces = 0;
    uint32_t numFaceIndices = 0;
    uint32_t numEdgeIndices = 0;
    uint32_t numFaceAdjFaceIndices = 0;
    uint32_t numTriangleIndices = 0;
};
#endif

// base struct from which other structs represent connected components inherit
struct connected_component_t {
    virtual ~connected_component_t() {};
    McConnectedComponentType type = (McConnectedComponentType)0;
    // array_mesh_t indexArrayMesh;
    // hmesh_t mesh;
    std::shared_ptr<output_mesh_info_t> kernel_hmesh_data;

    //
    std::shared_ptr< //
        std::unordered_map< //
            fd_t /*child face*/,
            fd_t /*parent face in the [user-provided] source mesh*/
            > //
        >
        source_hmesh_child_to_usermesh_birth_face; // fpPartitionChildFaceToCorrespondingInputSrcMeshFace
    std::shared_ptr< //
        std::unordered_map< //
            fd_t /*child face*/,
            fd_t /*parent face in the [user-provided] cut mesh*/
            >>
        cut_hmesh_child_to_usermesh_birth_face; // fpPartitionChildFaceToCorrespondingInputCutMeshFace
    // descriptors and coordinates of new vertices that are added into an input mesh (source mesh or cut mesh)
    // in order to carry out partitioning
    std::shared_ptr<std::unordered_map<vd_t, vec3>> source_hmesh_new_poly_partition_vertices; // addedFpPartitioningVerticesOnCorrespondingInputSrcMesh
    std::shared_ptr<std::unordered_map<vd_t, vec3>> cut_hmesh_new_poly_partition_vertices; // addedFpPartitioningVerticesOnCorrespondingInputCutMesh
    uint32_t internal_sourcemesh_vertex_count; // init from source_hmesh.number_of_vertices()
    uint32_t client_sourcemesh_vertex_count; // init from numSrcMeshVertices
    uint32_t internal_sourcemesh_face_count; // init from source_hmesh.number_of_faces()
    uint32_t client_sourcemesh_face_count; // init from source_hmesh_face_count OR numSrcMeshFaces
    // Stores the contiguous array of unsigned integers that define
    // a triangulation of all [non-triangle faces] of the connected component.
    // This vector is only populated if client invokes mcGetConnectedComponnentData
    // with flag MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION and has the effect of
    // triangulating every non-triangle face in the connected component.
    std::vector<uint32_t> cdt_index_cache;
    bool cdt_index_cache_initialized = false;
    // stores the mapping between a CDT triangle in the connected component and
    // the original "birth-face" in an input mesh (source mesh or cut mesh)
    std::vector<uint32_t> cdt_face_map_cache;
    bool cdt_face_map_cache_initialized = false;
#if defined(MCUT_MULTI_THREADED)
    // Stores the number of vertices per face of CC. This is an optimization
    // because there is a possibility that face-sizes may (at-minimum) be queried
    // twice by user. The first case is during the populating (i.e. second) call to the API
    // mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, ...);
    // The second case is during the populating (i.e. second) call to the API
    // mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_FACE, ...);.
    // The key detail here is that the second case requires knowledge of the
    // number of vertices in each face in order to know how to schedule parallel
    // work with prefix-sums etc.. Thus, the optimization is useful only if
    // building MCUT with multi-threading
    std::vector<uint32_t> face_sizes_cache;
    bool face_sizes_cache_initialized = false;
    // see documentation of face_sizes_cache above
    // Similar concepts but applied to MC_CONNECTED_COMPONENT_DATA_FACE_ADJACENT_FACE
    std::vector<uint32_t> face_adjacent_faces_size_cache;
    bool face_adjacent_faces_size_cache_initialized = false;
#endif // #if defined(MCUT_MULTI_THREADED)
};

// struct representing a fragment
struct fragment_cc_t : public connected_component_t {
    McFragmentLocation fragmentLocation = (McFragmentLocation)0;
    McFragmentSealType srcMeshSealType = (McFragmentSealType)0;
    McPatchLocation patchLocation = (McPatchLocation)0;
};

// struct representing a patch
struct patch_cc_t : public connected_component_t {
    McPatchLocation patchLocation = (McPatchLocation)0;
};

// struct representing a seam
struct seam_cc_t : public connected_component_t {
    McSeamOrigin origin = (McSeamOrigin)0;
};

// struct representing an input (user provided mesh)
struct input_cc_t : public connected_component_t {
    McInputOrigin origin = (McInputOrigin)0;
};

struct event_t {
    std::future<void> m_future; // used to wait on event
    // used to synchronise access to variables associated with the callback 
    // function.
    // This also also allows us to overcome the edgecase that mcSetEventCallback 
    // is called after the task associated with an event has been completed, 
    // in which case the new callback will be invoked immediately.
    // see "set_callback_data()" below
    std::mutex m_callback_mutex;
    struct {
        // optional user callback, which is invoked when associated task is finished
        pfn_McEvent_CALLBACK m_fn_ptr;
        // pointer passed to user provided callback function
        void* m_data_ptr;
        // atomic boolean flag indicating whether the task associated with event
        // object has completed running
        std::atomic<bool> m_finished;
        // atomic boolean flag indicating whether the callback associated with event
        // object has been called
        std::atomic<bool> m_invoked;
    } m_callback_info;
    McEvent m_user_handle; // handle used by client app to reference this event object
    event_t() : m_user_handle(MC_NULL_HANDLE)
    {
        m_callback_info.m_fn_ptr = nullptr;
        m_callback_info.m_data_ptr = nullptr;
        m_callback_info.m_finished.store(false);
        m_callback_info.m_invoked.store(true); // so that we do not call a null pointer/needless invoke the callback in the destructor
    }

    ~event_t()
    {
        if (m_callback_info.m_invoked.load() == false && m_callback_info.m_fn_ptr != nullptr)
        {
            MCUT_ASSERT(m_user_handle != MC_NULL_HANDLE);
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
        }
    }

    // thread-safe function to set the callback function for an event object
    void set_callback_data(McEvent handle, pfn_McEvent_CALLBACK fn_ptr, void* data_ptr)
    {
        std::lock_guard<std::mutex> lock(m_callback_mutex); // exclusive access

        m_user_handle = handle;
        m_callback_info.m_fn_ptr = fn_ptr;
        m_callback_info.m_data_ptr = data_ptr;
        m_callback_info.m_invoked.store(false);

        if (m_callback_info.m_finished.load() == true) { // see mutex documentation
            // immediately invoke the callback
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
            m_callback_info.m_invoked.store(true);
        }
    }

    // update the status of the event object to "finished"
    void notify_task_complete()
    {
        std::lock_guard<std::mutex> lock(m_callback_mutex);

        m_callback_info.m_finished = true;

        if (m_callback_info.m_invoked.load() == false && m_callback_info.m_fn_ptr != nullptr) {
            MCUT_ASSERT(m_user_handle != MC_NULL_HANDLE);
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
            m_callback_info.m_invoked.store(true);
        }
    }
};

// init in frontened.cpp
extern threadsafe_lookup_table<McEvent, std::shared_ptr<event_t>> g_events = {};
extern std::atomic<std::uintptr_t> g_objects_counter; // a counter that is used to assign a unique value to a McObject handle that will be returned to the user
extern std::once_flag g_objcts_counter_init_flag;

class device_t {
private:
    std::unique_ptr<thread_pool> m_threadpool;

public:
    device_t(uint32_t nthreads)
        : m_threadpool(std::unique_ptr<thread_pool>(new thread_pool(nthreads)))
    {
    }

    template <typename FunctionType>
    std::future<typename std::result_of<FunctionType()>::type> enqueue(FunctionType api_fun)
    {
        return m_threadpool->submit(api_fun);
    }
};

// our custome deleter function for std::unique_ptr variable of an array type
template <typename Derived>
void fn_delete_cc(connected_component_t* p)
{
    delete static_cast<Derived*>(p);
}

// struct defining the state of a context object
struct context_t {
private:
    thread_safe_queue<function_wrapper> m_queue;
    std::atomic<bool> done;

    void device_main()
    {
        do {
            function_wrapper api_fn;

            if (!(m_queue.try_pop(api_fn))) {
                m_queue.wait_and_pop(api_fn);
            }

            if (done) {
                break;
            }

            api_fn();

        } while (true);
    }

public:
    context_t(uint32_t num_workers)
        : done(false)
    {
        m_threadpool = std::unique_ptr<thread_pool>(new thread_pool(num_workers));

        auto device_future = std::async(std::launch::async, context_t::device_main);
    }

    ~context_t()
    {
    }

    template <typename FunctionType>
    std::future<typename std::result_of<FunctionType()>::type> enqueue(FunctionType api_fn)
    {
        typedef typename std::result_of<FunctionType()>::type result_type;

        std::packaged_task<result_type()> task(std::move(f));
        std::future<result_type> res(task.get_future());

        m_device_queue.push(std::move(task));

        return res;
    }

    std::unique_ptr<thread_pool> m_threadpool;

    // the current set of connected components associated with context
    threadsafe_lookup_table<McConnectedComponent, std::shared_ptr<connected_component_t>> connected_components;

    // The state and flag variable current used to configure the next dispatch call
    McFlags flags = (McFlags)0;
    // McFlags dispatchFlags = (McFlags)0;

    // client/user debugging variable
    // ------------------------------

    // function pointer to user-define callback function for status/erro reporting
    pfn_mcDebugOutput_CALLBACK debugCallback = nullptr;
    // user provided data for callback
    const void* debugCallbackUserParam = nullptr;

    // TODO: make use of the following three filter inside the log function

    // controller for permmited messages based on the source of message
    McFlags debugSource = 0;
    // controller for permmited messages based on the type of message
    McFlags debugType = 0;
    // controller for permmited messages based on the severity of message
    McFlags debugSeverity = 0;

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
};

// list of contexts created by client/user
extern "C" threadsafe_lookup_table<McContext, std::shared_ptr<context_t>> g_contexts;

extern "C" void create_context_impl(
    McContext* pContext, McFlags flags) noexcept(false);

extern "C" void debug_message_callback_impl(
    McContext context,
    pfn_mcDebugOutput_CALLBACK cb,
    const void* userParam) noexcept(false);

extern "C" void debug_message_control_impl(
    McContext context,
    McDebugSource source,
    McDebugType type,
    McDebugSeverity severity,
    bool enabled) noexcept(false);

extern "C" void get_info_impl(
    const McContext context,
    McFlags info,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes) noexcept(false);

extern "C" void set_event_callback_impl(
    McEvent eventHandle,
    pfn_McEvent_CALLBACK eventCallback,
    void* data);

extern "C" void wait_for_events_impl(
    uint32_t numEventsInWaitlist,
    const McEvent* pEventWaitList) noexcept(false);

extern "C" void dispatch_impl(
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
    uint32_t numCutMeshFaces,
    uint32_t numEventsInWaitlist,
    const McEvent* pEventWaitList,
    McEvent* pEvent) noexcept(false);

extern "C" void get_connected_components_impl(
    const McContext context,
    const McConnectedComponentType connectedComponentType,
    const uint32_t numEntries,
    McConnectedComponent* pConnComps,
    uint32_t* numConnComps,
    uint32_t numEventsInWaitlist,
    const McEvent* pEventWaitList,
    McEvent* pEvent) noexcept(false);

extern "C" void get_connected_component_data_impl(
    const McContext context,
    const McConnectedComponent connCompId,
    McFlags flags,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes) noexcept(false);

extern "C" void release_connected_components_impl(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps) noexcept(false);

extern "C" void release_context_impl(
    McContext context) noexcept(false);

extern "C" void release_events_impl(uint32_t numEvents, const McEvent* pEvents);

#endif // #ifndef _FRONTEND_H_