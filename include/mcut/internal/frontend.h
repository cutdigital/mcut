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

/*
std::invalid_argument: related to the input parameters
std::runtime_error: system runtime error e.g. out of memory
std::logic_error: a bug caught through an assertion failure
std::exception: unknown error source e.g. probably another bug
*/
#define CATCH_POSSIBLE_EXCEPTIONS(logstr)              \
    catch (std::invalid_argument & e0)                 \
    {                                                  \
        logstr = e0.what();                            \
        return_value = McResult::MC_INVALID_VALUE;     \
    }                                                  \
    catch (std::runtime_error & e1)                    \
    {                                                  \
        logstr = e1.what();                            \
        return_value = McResult::MC_INVALID_OPERATION; \
    }                                                  \
    catch (std::logic_error & e2)                      \
    {                                                  \
        logstr = e2.what();                            \
        return_value = McResult::MC_RESULT_MAX_ENUM;   \
    }                                                  \
    catch (std::exception & e3)                        \
    {                                                  \
        logstr = e3.what();                            \
        return_value = McResult::MC_RESULT_MAX_ENUM;   \
    }

extern thread_local std::string per_thread_api_log_str; // frontend.cpp

extern "C" void create_context_impl(
    McContext* pContext, McFlags flags, uint32_t num_helper_threads) noexcept(false);

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

extern "C" void get_event_info_impl(
    const McEvent event,
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
    uint64_t* pNumBytes,
    uint32_t numEventsInWaitlist,
    const McEvent* pEventWaitList,
    McEvent* pEvent) noexcept(false);

extern "C" void release_connected_components_impl(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps) noexcept(false);

extern "C" void release_context_impl(
    McContext context) noexcept(false);

extern "C" void release_events_impl(uint32_t numEvents, const McEvent* pEvents);

// base struct from which other structs represent connected components inherit
struct connected_component_t {
    virtual ~connected_component_t() {};
    McConnectedComponentType type = (McConnectedComponentType)0;
    McConnectedComponent m_user_handle = MC_NULL_HANDLE;
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
        // atomic boolean flag indicating whether the callback associated with event
        // object has been called
        std::atomic<bool> m_invoked;
    } m_callback_info;
    // atomic boolean flag indicating whether the task associated with event
    // object has completed running
    std::atomic<bool> m_finished;
    McEvent m_user_handle; // handle used by client app to reference this event object
    // the Manager thread which was assigned the task of managing the task associated with this event object.
    uint32_t m_responsible_thread_id;
    McResult m_execution_status; // API return code associated with respective task (for user to query)
    event_t()
        : m_user_handle(MC_NULL_HANDLE)
        , m_responsible_thread_id(UINT32_MAX),m_execution_status(MC_NO_ERROR)
    {
        std::cout << "[MCUT] Create event " << this << std::endl;

        m_callback_info.m_fn_ptr = nullptr;
        m_callback_info.m_data_ptr = nullptr;
        m_finished.store(false);
        m_callback_info.m_invoked.store(true); // so that we do not call a null pointer/needless invoke the callback in the destructor
    }

    ~event_t()
    {

        if (m_callback_info.m_invoked.load() == false && m_callback_info.m_fn_ptr != nullptr) {
            MCUT_ASSERT(m_user_handle != MC_NULL_HANDLE);
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
        }
        std::cout << "[MCUT] Destroy event " << this << "(" << m_user_handle << ")" << std::endl;
    }

    // thread-safe function to set the callback function for an event object
    void set_callback_data(McEvent handle, pfn_McEvent_CALLBACK fn_ptr, void* data_ptr)
    {
        std::lock_guard<std::mutex> lock(m_callback_mutex); // exclusive access

        m_user_handle = handle;
        m_callback_info.m_fn_ptr = fn_ptr;
        m_callback_info.m_data_ptr = data_ptr;
        m_callback_info.m_invoked.store(false);

        if (m_finished.load() == true) { // see mutex documentation
            // immediately invoke the callback
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
            m_callback_info.m_invoked.store(true);
        }
    }

    // update the status of the event object to "finished"
    void notify_task_complete(McResult exec_status)
    {
        m_finished = true;
        m_execution_status = exec_status;

        std::lock_guard<std::mutex> lock(m_callback_mutex);
        if (m_callback_info.m_invoked.load() == false && m_callback_info.m_fn_ptr != nullptr) {
            MCUT_ASSERT(m_user_handle != MC_NULL_HANDLE);
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
            m_callback_info.m_invoked.store(true);
        }
    }
};

// init in frontened.cpp
extern threadsafe_list<std::shared_ptr<event_t>> g_events;
extern std::atomic<std::uintptr_t> g_objects_counter; // a counter that is used to assign a unique value to a McObject handle that will be returned to the user
extern std::once_flag g_objects_counter_init_flag;

// our custome deleter function for std::unique_ptr variable of an array type
template <typename Derived>
void fn_delete_cc(connected_component_t* p)
{

    delete static_cast<Derived*>(p);
    std::cout << "[MCUT] Destroy connected component " << p->m_user_handle << std::endl;
}

// struct defining the state of a context object
struct context_t {
private:
    std::atomic<bool> m_done;
    std::vector<thread_safe_queue<function_wrapper>> m_queues;
    // Master/Manager thread(s) which are responsible for running the API calls
    // When a user of MCUT calls one of the APIs (e.g. mcEnqueueDispatch) the task
    // of actually executing everything related to that task will be handled by
    // one Manager thread. This manager thread itself will be involved in
    // computing some/all part of the respective task (think of it as the "main"
    // thread insofar as the API task is concerned). Some API tasks contain code
    // sections that run in parallel, which is where the Manager thread will also
    // submit tasks to the shared compute threadpool ("m_compute_threadpool").
    // NOTE: must be declared after "thread_pool_terminate" and "work_queues"
    std::vector<std::thread> m_api_threads;
    join_threads m_joiner;

    // A pool of threads that is shared by manager threads to execute e.g. parallel
    // section of MCUT API tasks (see e.g. frontend.cpp and kernel.cpp)
    std::unique_ptr<thread_pool> m_compute_threadpool;

    // The state and flag variable current used to configure the next dispatch call
    McFlags m_flags = (McFlags)0;

    void api_thread_main(uint32_t thread_id)
    {
        std::cout << "[MCUT] Launch API thread " << std::this_thread::get_id() << " (" << thread_id << ")" << std::endl;

        do {
            function_wrapper task;

            m_queues[thread_id].wait_and_pop(task);

            if (m_done) {
                break;
            }

            task();

        } while (true);

        std::cout << "[MCUT] Shutdown API thread " << std::this_thread::get_id() << " (" << thread_id << ")" << std::endl;
    }

public:
    context_t(McContext handle, McFlags flags, uint32_t num_compute_threads)
        : m_done(false)
        , m_joiner(m_api_threads)
        , m_flags(flags)
        , m_user_handle(handle)
    {
        std::cout << "[MCUT] Create context " << m_user_handle << std::endl;

        try {
            const uint32_t manager_thread_count = (flags & MC_OUT_OF_ORDER_EXEC_MODE_ENABLE) ? 2 : 1;

            m_queues = std::vector<thread_safe_queue<function_wrapper>>(manager_thread_count);

            for (uint32_t i = 0; i < manager_thread_count; ++i) {
                m_queues[i].set_done_ptr(&m_done);
                m_api_threads.push_back(std::thread(&context_t::api_thread_main, this, i));
            }

            // create the pool of compute threads. These are the worker threads that
            // can be tasked with work from any manager-thread. Thus, manager threads
            // share the available/user-specified compute threads.
            m_compute_threadpool = std::unique_ptr<thread_pool>(new thread_pool(num_compute_threads));

        } catch (...) {
            m_done = true;
            throw;
        }
    }

    ~context_t()
    {

        m_done = true;
        for (uint32_t i = 0; i < (uint32_t)m_api_threads.size(); ++i) {
            m_queues[i].disrupt_wait_for_data();
        }
        std::cout << "[MCUT] Destory context " << m_user_handle << std::endl;
    }

    McContext m_user_handle;

    // returns the flags which determine the runtime configuration of this context
    const McFlags& get_flags() const
    {
        return this->m_flags;
    }

    thread_pool& get_shared_compute_threadpool()
    {
        return m_compute_threadpool.get()[0];
    }

    template <typename FunctionType>
    McEvent enqueue(uint32_t numEventsInWaitlist, const McEvent* pEventWaitList, FunctionType api_fn)
    {
        // List of events the enqueued task depends on
        //
        // local copy that will be captured by-value (user permitted to re-use pEventWaitList)
        const std::vector<McEvent> event_waitlist(pEventWaitList, pEventWaitList + numEventsInWaitlist);

        //
        // Determine which manager thread to assign the task
        //

        // the id of manager thread that will be assigned the current task
        uint32_t responsible_thread_id = UINT32_MAX;

        for (std::vector<McEvent>::const_iterator waitlist_iter = event_waitlist.cbegin(); waitlist_iter != event_waitlist.cend(); ++waitlist_iter) {
            const McEvent& parent_task_event_handle = *waitlist_iter;

            const std::shared_ptr<event_t> parent_task_event_ptr = g_events.find_first_if([=](std::shared_ptr<event_t> e) { return e->m_user_handle == parent_task_event_handle; });

            MCUT_ASSERT(parent_task_event_ptr != nullptr);

            const bool parent_task_is_not_finished = parent_task_event_ptr->m_finished == false;

            if (parent_task_is_not_finished) {
                // id of manager thread, which was assigned the parent task
                const uint32_t responsible_thread_id = parent_task_event_ptr->m_responsible_thread_id;

                MCUT_ASSERT(responsible_thread_id != UINT32_MAX);
                MCUT_ASSERT(responsible_thread_id < (uint32_t)m_api_threads.size());

                break;
            }
        }

        const bool have_responsible_thread = responsible_thread_id != UINT32_MAX;

        if (!have_responsible_thread) {
            uint32_t thread_with_empty_queue = UINT32_MAX;

            for (uint32_t i = 0; i < (uint32_t)m_api_threads.size(); ++i) {
                if (m_queues[i].empty() == true) {
                    thread_with_empty_queue = i;
                    break;
                }
            }

            if (thread_with_empty_queue != UINT32_MAX) {
                responsible_thread_id = thread_with_empty_queue;
            } else { // all threads have work to do
                responsible_thread_id = 0; // just pick thread 0
            }
        }

        //
        // create the event object associated with the enqueued task
        //

        std::shared_ptr<event_t> event_ptr = std::shared_ptr<event_t>(new event_t);
        MCUT_ASSERT(event_ptr != nullptr);

        g_events.push_front(event_ptr);

        event_ptr->m_user_handle = reinterpret_cast<McEvent>(g_objects_counter++);

        //
        // Package-up the task as a synchronised operation that will wait for
        // other tasks in the event_waitlist, compute the operation, and finally update
        // the respective event state with the completion status.
        //

        std::weak_ptr<event_t> event_weak_ptr(event_ptr);

        std::packaged_task<void()> task(
            [=]() {
                if (!event_waitlist.empty()) {
                    wait_for_events_impl((uint32_t)event_waitlist.size(), &event_waitlist[0]); // block until events are done
                }

                 MCUT_ASSERT(event_weak_ptr.expired() == false);

                std::shared_ptr<event_t> event = event_weak_ptr.lock();

                McResult return_value = McResult::MC_NO_ERROR;
                per_thread_api_log_str.clear();

                try {
                    api_fn(); // execute the API function. 
                }
                CATCH_POSSIBLE_EXCEPTIONS(per_thread_api_log_str); // exceptions may be thrown due to runtime errors, which must be reported back to user

                if (!per_thread_api_log_str.empty()) {

                    std::fprintf(stderr, "%s(...) -> %s (EventID=%p)\n", __FUNCTION__, per_thread_api_log_str.c_str(), event == nullptr ? (McEvent)0 : event->m_user_handle);

                    if (return_value == McResult::MC_NO_ERROR) // i.e. problem with basic local parameter checks
                    {
                        return_value = McResult::MC_INVALID_VALUE;
                    }
                }

                if (event) // not null
                {
                    event->notify_task_complete(return_value); // updated event state to indicate task completion (lock-based)
                }
            });

        event_ptr->m_future = task.get_future(); // the future we can later wait on via mcWaitForEVents
        event_ptr->m_responsible_thread_id = responsible_thread_id;

        m_queues[responsible_thread_id].push(std::move(task)); // enqueue task to be executed when responsible thread is free

        return event_ptr->m_user_handle;
    }

    // the current set of connected components associated with context
    threadsafe_list<std::shared_ptr<connected_component_t>> connected_components;

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
extern "C" threadsafe_list<std::shared_ptr<context_t>> g_contexts;

#endif // #ifndef _FRONTEND_H_