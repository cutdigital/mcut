/***************************************************************************
 *  This file is part of the MCUT project, which is comprised of a library 
 *  for surface mesh cutting, example programs and test programs.
 * 
 *  Copyright (C) 2024 CutDigital Enterprise Ltd
 *  
 *  MCUT is dual-licensed software that is available under an Open Source 
 *  license as well as a commercial license. The Open Source license is the 
 *  GNU Lesser General Public License v3+ (LGPL). The commercial license 
 *  option is for users that wish to use MCUT in their products for commercial 
 *  purposes but do not wish to release their software under the LGPL. 
 *  Email <contact@cut-digital.com> for further information.
 *
 *  You may not use this file except in compliance with the License. A copy of 
 *  the Open Source license can be obtained from
 *
 *      https://www.gnu.org/licenses/lgpl-3.0.en.html.
 *
 *  For your convenience, a copy of this License has been included in this
 *  repository.
 *
 *  MCUT is distributed in the hope that it will be useful, but THE SOFTWARE IS 
 *  PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR 
 *  A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF 
 *  OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

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

#include <future>
#include <map>
#include <memory>
#include <string>

#include "mcut/internal/tpool.h"

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
    const McVoid* userParam) noexcept(false);

extern "C" void get_debug_message_log_impl(McContext context,
    McUint32 count, McSize bufSize,
    McDebugSource* sources, McDebugType* types, McDebugSeverity* severities,
    McSize* lengths, McChar* messageLog, McUint32& numFetched)noexcept(false);

extern "C" void debug_message_control_impl(
    McContext context,
    McDebugSource source,
    McDebugType type,
    McDebugSeverity severity,
    bool enabled) noexcept(false);

extern "C" void get_info_impl(
    const McContext context,
    McFlags info,
    McSize bytes,
    McVoid* pMem,
    McSize* pNumBytes) noexcept(false);

extern "C" void bind_state_impl(
    const McContext context,
    McFlags stateInfo,
    McSize bytes,
    const McVoid* pMem)noexcept(false);

extern "C" void create_user_event_impl(McEvent* event, McContext context)noexcept(false);

extern "C" void set_user_event_status_impl(McEvent event, McInt32 execution_status)noexcept(false);

extern "C" void get_event_info_impl(
    const McEvent event,
    McFlags info,
    McSize bytes,
    McVoid* pMem,
    McSize* pNumBytes) noexcept(false);

extern "C" void set_event_callback_impl(
    McEvent eventHandle,
    pfn_McEvent_CALLBACK eventCallback,
    McVoid* data) noexcept(false);

extern "C" void wait_for_events_impl(
    uint32_t numEventsInWaitlist,
    const McEvent* pEventWaitList,
    McResult& runtimeStatusFromAllPrecedingEvents) noexcept(false);

extern "C" void dispatch_impl(
    McContext context,
    McFlags flags,
    const McVoid* pSrcMeshVertices,
    const uint32_t* pSrcMeshFaceIndices,
    const uint32_t* pSrcMeshFaceSizes,
    uint32_t numSrcMeshVertices,
    uint32_t numSrcMeshFaces,
    const McVoid* pCutMeshVertices,
    const uint32_t* pCutMeshFaceIndices,
    const uint32_t* pCutMeshFaceSizes,
    uint32_t numCutMeshVertices,
    uint32_t numCutMeshFaces,
    uint32_t numEventsInWaitlist,
    const McEvent* pEventWaitList,
    McEvent* pEvent) noexcept(false);

extern "C" void dispatch_planar_section_impl(
    McContext context,
    McFlags flags,
    const McVoid* pSrcMeshVertices,
    const uint32_t* pSrcMeshFaceIndices,
    const uint32_t* pSrcMeshFaceSizes,
    uint32_t numSrcMeshVertices,
    uint32_t numSrcMeshFaces,
    const McDouble* pNormalVector,
    const McDouble sectionOffset,
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
    McSize bytes,
    McVoid* pMem,
    McSize* pNumBytes,
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
#if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
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
#endif // #if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
    // non-zero if origin source and cut-mesh where perturbed
    vec3 perturbation_vector = vec3(0.0);
    //
    // An array storing the lists of vertices that define seams/intersection 
    // contours along the cut path of a CC. We need this cache because it will be 
    // typically needed twice by users (to query number of elements for mem 
    // allocation; and to actually copy the data to some user output array).
    //
    // Format: [
    //  Total number of sequences,
    //  number of vertices in [first] sequence,
    //  boolean (uint) flag stating whether [first] sequence forms a loop,
    //  N consecutive unsigned integers which identify the vertices of the [first] sequence,
    //  number of vertices in [second] sequence,
    //  boolean (uint) flag stating whether [second] sequence forms a loop,
    //  N consecutive unsigned integers which identify the vertices of the [second] sequence,
    //  ... and so on.
    // ]
    //
    std::vector<uint32_t> seam_vertex_sequence_array_cache;
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
        McVoid* m_data_ptr;
        // atomic boolean flag indicating whether the callback associated with event
        // object has been called
        std::atomic<bool> m_invoked;
    } m_callback_info;
    // atomic boolean flag indicating whether the task associated with event
    // object has completed running
    std::atomic<bool> m_finished;
    McEvent m_user_handle; // handle used by client app to reference this event object
    // the Manager thread which was assigned the task of managing the task associated with this event object.
    std::atomic<uint32_t> m_responsible_thread_id;
    std::atomic<int32_t> m_runtime_exec_status; // API return code associated with respective task (for user to query)
    std::atomic<size_t> m_timestamp_submit;
    std::atomic<size_t> m_timestamp_start;
    std::atomic<size_t> m_timestamp_end;
    std::atomic<uint32_t> m_command_exec_status;
    bool m_profiling_enabled;
    McCommandType m_command_type;
    // A callable object that also holds an std::future. Its purpose is to emulate
    // the internal representation of an API task, where this time the task is actually
    // a user command since this pointer is define ONLY for user events.
    // The std::future object of the packaged task is used to initialize m_future
    // when this event is a user event. This our internal mechanism allowing for
    // API command to be able to effectively wait on user events.
    std::unique_ptr<std::packaged_task<void()>> m_user_API_command_task_emulator;
    McContext m_context;

    const char* get_cmd_type_str()
    {
        switch (m_command_type) {
        case McCommandType::MC_COMMAND_DISPATCH: {
            return "MC_COMMAND_DISPATCH";
        } break;
        case McCommandType::MC_COMMAND_GET_CONNECTED_COMPONENT_DATA: {
            return "MC_COMMAND_GET_CONNECTED_COMPONENT_DATA";
        } break;
        case McCommandType::MC_COMMAND_GET_CONNECTED_COMPONENTS: {
            return "MC_COMMAND_GET_CONNECTED_COMPONENTS";
        } break;
        case McCommandType::MC_COMMAND_USER: {
            return "MC_COMMAND_USER";
        } break;
        case McCommandType::MC_COMMAND_UKNOWN: {
            return "MC_COMMAND_UKNOWN";
        } break;
        default:
            fprintf(stderr, "unknown command type value (%d)\n", (int)m_command_type);
            return "UNKNOWN VALUE";
        }
    }
    explicit event_t(McEvent user_handle, McCommandType command_type)
        : m_user_handle(user_handle)
        , m_responsible_thread_id(UINT32_MAX)
        , m_runtime_exec_status(MC_NO_ERROR)
        , m_timestamp_submit(0)
        , m_timestamp_start(0)
        , m_timestamp_end(0)
        , m_command_exec_status((McUint32)((int)(MC_RESULT_MAX_ENUM)))
        , m_profiling_enabled(true)
        , m_command_type(command_type)
        , m_user_API_command_task_emulator(nullptr)
        , m_context(nullptr)
    {
        log_msg("[MCUT] Create event (type=" << get_cmd_type_str() <<", handle=" << m_user_handle << ")");

        m_callback_info.m_fn_ptr = nullptr;
        m_callback_info.m_data_ptr = nullptr;
        m_finished.store(false);
        m_callback_info.m_invoked.store(true); // so that we do not call a null pointer/needless invoke the callback in the destructor
    }

    ~event_t()
    {

        if (m_callback_info.m_invoked.load() == false && m_callback_info.m_fn_ptr != nullptr && m_runtime_exec_status.load() == MC_NO_ERROR) {
            MCUT_ASSERT(m_user_handle != MC_NULL_HANDLE);
            (*(m_callback_info.m_fn_ptr))(m_user_handle, m_callback_info.m_data_ptr);
        }

        log_msg("[MCUT] Destroy event (type=" << get_cmd_type_str() << ", handle=" << m_user_handle << ")");
    }

    inline std::size_t get_time_since_epoch()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    inline void log_submit_time()
    {
        if (m_profiling_enabled) {
            this->m_timestamp_submit.store(get_time_since_epoch());
        }
        // TODO: use specific acquire-release semantics
        // see e.g.: https://stackoverflow.com/questions/13632344/understanding-c11-memory-fences
        m_command_exec_status.store(McEventCommandExecStatus::MC_SUBMITTED);
    }

    inline void log_start_time()
    {
        if (m_profiling_enabled) {
            this->m_timestamp_start.store(get_time_since_epoch());
        }
        m_command_exec_status = McEventCommandExecStatus::MC_RUNNING;
    }

    // logs time and sets m_command_exec_status to MC_COMPLETE
    inline void log_end_time()
    {
        if (m_profiling_enabled) {
            this->m_timestamp_end.store(get_time_since_epoch());
        }
        m_command_exec_status.store(McEventCommandExecStatus::MC_COMPLETE);
    }

    // thread-safe function to set the callback function for an event object
    void set_callback_data(McEvent handle, pfn_McEvent_CALLBACK fn_ptr, McVoid* data_ptr)
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
        m_runtime_exec_status = exec_status;

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
    log_msg("[MCUT] Destroy connected component " << p->m_user_handle);
    delete dynamic_cast<Derived*>(p);
}

// struct defining the state of a context object
struct context_t {
private:
    std::atomic<bool> m_done; // are we finished with the context (i.e. indicate to shutdown threadpool and freeup respective resourses)
    // the work-queues associated with each API (device) thread
    std::vector<thread_safe_queue<function_wrapper>> m_queues;
    // Master/Manager thread(s) which are responsible for running the API calls.
    // Also called "device" threads.
    // When a user of MCUT calls one of the APIs (e.g. mcEnqueueDispatch) the task
    // of actually executing everything related to that task will be handled by
    // one Manager thread. This manager thread itself will be involved in
    // computing some/all part of the respective task (think of it as the "main"
    // thread insofar as the respective API task is concerned). Some API tasks contain code
    // sections that run in parallel, which is where the Manager thread will also
    // submit tasks to the shared compute threadpool ("m_compute_threadpool").
    // NOTE: must be declared after "thread_pool_terminate" and "work_queues" (to avoid MT bugs when freeing up context memory)
    std::vector<std::thread> m_api_threads;
    // join_threads m_joiner;

#if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
    // A pool of threads that is shared by manager threads to execute e.g. parallel
    // section of MCUT API tasks (see e.g. frontend.cpp and kernel.cpp)
    std::unique_ptr<thread_pool> m_compute_threadpool;
#endif

    // The state and flag variable currently used to configure the next dispatch call
    McFlags m_flags = (McFlags)0;

    // as it says on the tin: "tiny" number used to enforce general position
    std::atomic<McDouble> m_general_position_enforcement_constant;
    // The maximum number of iterations/attempts over which to try and resolve the input meshes into general position
    std::atomic<McUint32> m_max_num_perturbation_attempts;
    // The state and flag variable currently used to configure the next mcGetConnectedComponentData call 
    // (with either MC_CONNECTED_COMPONENT_DATA_FACE or MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION)
    // This flag is primailly useful for flipping the winding order of vertices queried for a connected component
    // of type FRAGMENT-SEALED-EXTERIOR-BELOW, which will have reversed normal orientation due to MCUT internal 
    // handling of winding order
    std::atomic<McConnectedComponentFaceWindingOrder> m_connected_component_winding_order;
    // The type of intersection (between source-mesh and cut-mesh) that was detected during the last/most-recent
    // dispatch call.
    std::atomic<McDispatchIntersectionType> m_most_recent_dispatch_intersection_type;

    // This is the "main" function of each device/API/manger thread. When a context is created and 
    // the internal scheduling threadpool is initialised, each (device) thread is launched with this
    // function. The device thread loops indefinitely (sleeping most of the time and waking up to do 
    // work) until the context is destroyed. 
    void api_thread_main(uint32_t thread_id)
    {
        log_msg("[MCUT] Launch API thread " << std::this_thread::get_id() << " (" << thread_id << ")");

        do {
            function_wrapper task;

            // We try_pop() first in case the task "producer" (API/device/manager) thread
            // already invoked cond_var.notify_one() of "m_queues[thread_id]""
            // BEFORE current thread first-entered this function.
            if (!m_queues[thread_id].try_pop(task)) {
                // there is no work, so thread will wait until user thread gives it some work to do.
                m_queues[thread_id].wait_and_pop(task);
            }

            if (m_done) { // are we finished?
                break; // quit to free up resources.
            }

            task();

        } while (true);

        log_msg("[MCUT] Shutdown API thread " << std::this_thread::get_id() << " (" << thread_id << ")");
    }

public:
    context_t(McContext handle, McFlags flags
#if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
        ,
        uint32_t num_compute_threads
#endif
        )
        : m_done(false)
        // , m_joiner(m_api_threads)
        , m_flags(flags)
        , m_general_position_enforcement_constant(1e-4) // 0.0001
        , m_max_num_perturbation_attempts(1 << 2), // 4
        // default winding order (as determing from the normals of the input mesh faces)
        m_connected_component_winding_order(McConnectedComponentFaceWindingOrder::MC_CONNECTED_COMPONENT_FACE_WINDING_ORDER_AS_GIVEN)
        , m_most_recent_dispatch_intersection_type(McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM)
        , m_user_handle(handle), // i.e. the McContext handle that the client application uses to reference an instance of the context
        // debug callback flags (all zero/unset by default). User must specify what they want via debug control function.
         dbgCallbackBitfieldSource(0)
        , dbgCallbackBitfieldType(0)
        , dbgCallbackBitfieldSeverity(0)
    {
        log_msg("\n[MCUT] Create context " << m_user_handle);

        try {
            const uint32_t manager_thread_count = (flags & MC_OUT_OF_ORDER_EXEC_MODE_ENABLE) ? 2 : 1;

            m_queues = std::vector<thread_safe_queue<function_wrapper>>(manager_thread_count);

            for (uint32_t i = 0; i < manager_thread_count; ++i) {
                m_queues[i].set_done_ptr(&m_done);
                m_api_threads.push_back(std::thread(&context_t::api_thread_main, this, i));
            }
#if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
            // create the pool of compute threads. These are the worker threads that
            // can be tasked with work from any manager-thread. Thus, manager threads
            // share the available/user-specified compute threads.
            m_compute_threadpool = std::unique_ptr<thread_pool>(new thread_pool(num_compute_threads, manager_thread_count));
#endif
        } catch (...) {
            shutdown(); // free up memory shutdown device threads etc.

            log_msg("[MCUT] Destroy context due to exception" << m_user_handle);
            throw;
        }
    }

    ~context_t()
    {
        shutdown();
        log_msg("[MCUT] Destroy context " << m_user_handle);
    }

    void shutdown()
    {
        m_done = true; // notify device threads that they need to finished up
        
        std::atomic_thread_fence(std::memory_order_acq_rel); // just for safety, put a memory barrier to make shutdown notification is recieved (probably not needed)

#if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
        m_compute_threadpool.reset(); // wake-up and destroy the helper/worker threads
#endif
        for (int i = (int)m_api_threads.size() - 1; i >= 0; --i) {
            m_queues[i].disrupt_wait_for_data(); // wake up device thread from sleeping
            if (m_api_threads[i].joinable()) { // if not already joined/shutdown
                m_api_threads[i].join(); // wait for it to finish 
            }
        }
    }
       
    // reference handle used by client application to access and modify context state
    McContext m_user_handle;

    // returns the flags which determine the runtime configuration of this context
    const McFlags& get_flags() const
    {
        return this->m_flags;
    }

    // returns (user controllable) epsilon representing the maximum by which the cut-mesh
    // can be perturbed on any axis
    McDouble get_general_position_enforcement_constant() const
    {
        return this->m_general_position_enforcement_constant.load(std::memory_order_acquire);
    }

    void set_general_position_enforcement_constant(McDouble new_value)
    {
        this->m_general_position_enforcement_constant.store(new_value, std::memory_order_release);
    }

    // returns (user controllable) maximum number of times by which the cut-mesh
    // can be perturbed before giving (input likely need to be preprocessed)
    McUint32 get_general_position_enforcement_attempts() const
    {
        return this->m_max_num_perturbation_attempts.load(std::memory_order_acquire);
    }

    void set_general_position_enforcement_attempts(McUint32 new_value)
    {
        this->m_max_num_perturbation_attempts.store(new_value, std::memory_order_release);
    }

 McConnectedComponentFaceWindingOrder get_connected_component_winding_order() const
    {
        return this->m_connected_component_winding_order.load(std::memory_order_acquire);
    }

    void set_connected_component_winding_order(McConnectedComponentFaceWindingOrder new_value)
    {
        this->m_connected_component_winding_order.store(new_value, std::memory_order_release);
    }

    McDispatchIntersectionType get_most_recent_dispatch_intersection_type()const
    {
        return this->m_most_recent_dispatch_intersection_type.load(std::memory_order_acquire);
    }

    void set_most_recent_dispatch_intersection_type(McDispatchIntersectionType new_value)
    {
        this->m_most_recent_dispatch_intersection_type.store(new_value, std::memory_order_release);
    }
    

#if defined(MCUT_WITH_COMPUTE_HELPER_THREADPOOL)
    thread_pool& get_shared_compute_threadpool()
    {
        return m_compute_threadpool.get()[0];
    }
#endif

    /*
    This function serves the purpose of scheduling an API task (i.e. an async API function call from the user) by submitting this
    task into an internal work-queue that will be checked and popped-from by a device thread.
    */
    template <typename FunctionType>
    McEvent prepare_and_submit_API_task(
        McCommandType cmdType, // the type of command that the user has submitted
        uint32_t numEventsInWaitlist, // the number of (previously submitted or user) events to wait for
        const McEvent* pEventWaitList, // the list/array of events to wait for _before_ executing the submitted task
        FunctionType api_fn // a function (lambda/functor) encapsulating the API task to be executed asynchronously
    )
    {
        //
        // create the event object associated with the enqueued task
        //

        std::shared_ptr<event_t> event_ptr = std::shared_ptr<event_t>(new event_t(
            reinterpret_cast<McEvent>(g_objects_counter.fetch_add(1, std::memory_order_relaxed)), // the handle is just a lightweight integer counter
            cmdType));

        MCUT_ASSERT(event_ptr != nullptr);

        g_events.push_front(event_ptr);

        //event_ptr->m_user_handle = reinterpret_cast<McEvent>(g_objects_counter.fetch_add(1, std::memory_order_relaxed));
        event_ptr->m_profiling_enabled = (this->m_flags & MC_PROFILING_ENABLE) != 0;
        // event_ptr->m_command_type = cmdType;

        event_ptr->log_submit_time();

        // List of events the enqueued task depends on
        //
        // local copy that will be captured by-value (client application is permitted to re-use the memory pointed to by "pEventWaitList")
        const std::vector<McEvent> event_waitlist(pEventWaitList, pEventWaitList + numEventsInWaitlist);

        //
        // Determine which manager thread to assign the task to
        //

        // the id of manager thread that will be assigned the current task
        uint32_t responsible_thread_id = UINT32_MAX;

        // for each event to wait for (before executing the current task)
        for (std::vector<McEvent>::const_iterator waitlist_iter = event_waitlist.cbegin(); waitlist_iter != event_waitlist.cend(); ++waitlist_iter) {
            const McEvent& parent_task_event_handle = *waitlist_iter;
            // get actual event object
            const std::shared_ptr<event_t> parent_task_event_ptr = g_events.find_first_if([=](std::shared_ptr<event_t> e) { return e->m_user_handle == parent_task_event_handle; });

            if (parent_task_event_ptr == nullptr) { // not found
                throw std::invalid_argument("invalid event in waitlist"); // client gave us something we don't recognise
            }

            const bool parent_task_is_not_finished = parent_task_event_ptr->m_finished.load() == false;

            // task associated with event is still running and the event is a user-event
            if (parent_task_is_not_finished && parent_task_event_ptr->m_command_type != McCommandType::MC_COMMAND_USER) {
                // id of manager thread, which was assigned the parent task
                responsible_thread_id = parent_task_event_ptr->m_responsible_thread_id.load();

                MCUT_ASSERT(responsible_thread_id != UINT32_MAX);
                MCUT_ASSERT(responsible_thread_id < (uint32_t)m_api_threads.size());

                break;
            }
        }

        const bool have_responsible_thread = responsible_thread_id != UINT32_MAX;

        if (!have_responsible_thread) {
            uint32_t assignedThread = UINT32_MAX;
            
            for (uint32_t i = 0; i < (uint32_t)m_api_threads.size(); ++i) {  // assign to any thread with an empty queue
                const uint32_t index = (i + 1) % (uint32_t)m_api_threads.size();
                if (m_queues[index].empty()) {
                    assignedThread = index;
                    break;
                }
            }
        

            if (assignedThread != UINT32_MAX) {
                responsible_thread_id = assignedThread;
            } else { // all threads have work to do
                responsible_thread_id = 0; // just pick thread 0
            }
        }

        //
        // Package-up the task as a synchronised operation that will wait for
        // other tasks in the event_waitlist, compute the operation, and finally update
        // the respective event state with the completion status.
        //

        std::weak_ptr<event_t> event_weak_ptr(event_ptr);

        std::packaged_task<void()> task(
            [=]() {
                McResult runtime_status_from_all_preceding_events = McResult::MC_NO_ERROR;

                if (!event_waitlist.empty()) {
                    wait_for_events_impl((uint32_t)event_waitlist.size(), &event_waitlist[0], runtime_status_from_all_preceding_events); // block until events are done
                }

                // if any previous event failed then we cannot proceed with this task.
                // i.e. no-Op
                if (runtime_status_from_all_preceding_events != McResult::MC_NO_ERROR) {
                    return;
                }

                MCUT_ASSERT(!event_weak_ptr.expired());

                {
                    std::shared_ptr<event_t> event = event_weak_ptr.lock();

                    MCUT_ASSERT(event != nullptr);

                    {
                        McResult return_value = McResult::MC_NO_ERROR;
                        per_thread_api_log_str.clear();

                        event->log_start_time();

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

                        event->notify_task_complete(return_value); // updated event state to indicate task completion (lock-based)
                        event->log_end_time();
                    }
                }
            });

        event_ptr->m_future = task.get_future(); // the future we can later wait on via mcWaitForEVents
        event_ptr->m_responsible_thread_id = responsible_thread_id;

        m_queues[responsible_thread_id].push(std::move(task)); // enqueue task to be executed when responsible API thread is free

        return event_ptr->m_user_handle;
    }

    // the current set of connected components associated with context
    threadsafe_list<std::shared_ptr<connected_component_t>> connected_components;

    // McFlags dispatchFlags = (McFlags)0;

    // client/user debugging variables
    // ------------------------------

    // function pointer to user-define callback function for status/erro reporting
    pfn_mcDebugOutput_CALLBACK debugCallback = nullptr;
    // user provided data for callback
    const McVoid* debugCallbackUserParam = nullptr;

    std::mutex debugCallbackMutex;
    // controller for permmited messages based on the source of message
    std::atomic<McFlags> dbgCallbackBitfieldSource;
    // controller for permmited messages based on the type of message
    std::atomic<McFlags> dbgCallbackBitfieldType;
    // controller for permmited messages based on the severity of message
    std::atomic<McFlags> dbgCallbackBitfieldSeverity;
    bool dbgCallbackAllowAsyncCalls = true;

    void set_debug_callback_data(pfn_mcDebugOutput_CALLBACK cb, const McVoid* data_ptr)
    {
        std::lock_guard<std::mutex> lguard(debugCallbackMutex);
        debugCallback = cb;
        debugCallbackUserParam = data_ptr;
    }

    struct debug_log_msg_t {
        McDebugSource source;
        McDebugType type;
        McDebugSeverity severity;
        std::string str;
    };

    std::vector<debug_log_msg_t> m_debug_logs;

    // function to invoke the user-provided debug call back
    void dbg_cb(McDebugSource source,
        McDebugType type,
        unsigned int id, // unused
        McDebugSeverity severity,
        const std::string& message)
    {
        if (this->m_flags & McContextCreationFlags::MC_DEBUG) // information logged only during debug mode
        {
            std::unique_lock<std::mutex> ulock(debugCallbackMutex, std::defer_lock);

            if (!dbgCallbackAllowAsyncCalls) {
                ulock.lock();
            } // otherwise the callback will be invoked asynchronously

            // can we log this type of message? (based on user preferences via mcDebugMessageControl)
            const bool canLog = ((uint32_t)source & dbgCallbackBitfieldSource.load(std::memory_order_acquire)) && //
                ((uint32_t)type & dbgCallbackBitfieldType.load(std::memory_order_acquire)) && //
                ((uint32_t)severity & dbgCallbackBitfieldSeverity.load(std::memory_order_acquire));

            if (canLog) {

                if (debugCallback != nullptr) { // user gave us a callback function pointer

                    (*debugCallback)(source, type, id, severity, message.length(), message.c_str(), debugCallbackUserParam);

                } else // write to the internal log
                {
                    m_debug_logs.emplace_back(debug_log_msg_t());

                    debug_log_msg_t& dbg_log = m_debug_logs.back();

                    dbg_log.source = source;
                    dbg_log.type = type;
                    dbg_log.severity = severity;
                    dbg_log.str = message;
                }
            }
        }
    }
};

extern "C" void triangulate_face(
    //  list of indices which define all triangles that result from the CDT
    std::vector<uint32_t>&cc_face_triangulation,
    const std::shared_ptr<context_t>&context_uptr,
    const uint32_t cc_face_vcount,
    const std::vector<vertex_descriptor_t>&cc_face_vertices,
    const hmesh_t & cc,
    const fd_t cc_face_iter);

// list of contexts created by client/user
extern "C" threadsafe_list<std::shared_ptr<context_t>> g_contexts;

#endif // #ifndef _FRONTEND_H_