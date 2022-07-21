#include "mcut/internal/frontend.h"

#include "mcut/internal/geom.h"
#include "mcut/internal/kernel.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"

#include <map>

namespace frontend {

// If the inputs are found to not be in general position, then we perturb the
// cut-mesh by this constant (scaled by bbox diag times a random variable [0.1-1.0]).
const mcut::math::real_number_t GENERAL_POSITION_ENFORCMENT_CONSTANT = 1e-4;
const int MAX_PERTUBATION_ATTEMPTS = 1 << 3;

// internal frontend data structure which we use to store connected component
// data. Information requested by a client/user via "mcGetConnectedComponentData"
// is read from this data structure (halfedge meshes are used by the backend
// kernel)
struct IndexArrayMesh {
    IndexArrayMesh() { }
    ~IndexArrayMesh()
    {
    }

    std::unique_ptr<mcut::math::real_number_t[]> pVertices;
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

// base struct from which other structs represent connected components inherit
struct McConnCompBase {
    virtual ~McConnCompBase() {};
    McConnectedComponentType type = (McConnectedComponentType)0;
    IndexArrayMesh indexArrayMesh;
};

// struct representing a fragment
struct McFragmentConnComp : public McConnCompBase {
    McFragmentLocation fragmentLocation = (McFragmentLocation)0;
    McFragmentSealType srcMeshSealType = (McFragmentSealType)0;
    McPatchLocation patchLocation = (McPatchLocation)0;
};

// struct representing a patch
struct McPatchConnComp : public McConnCompBase {
    McPatchLocation patchLocation = (McPatchLocation)0;
};

// struct representing a seam
struct McSeamConnComp : public McConnCompBase {
    McSeamOrigin origin = (McSeamOrigin)0;
};

// struct representing an input (user provided mesh)
struct McInputConnComp : public McConnCompBase {
    McInputOrigin origin = (McInputOrigin)0;
};

// our custome deleter function for std::unique_ptr variable of an array type
template <typename Derived>
void ccDeletorFunc(McConnCompBase* p)
{
    delete static_cast<Derived*>(p);
}

// struct defining the state of a context object
struct McDispatchContextInternal {
#if defined(MCUT_MULTI_THREADED)
    // work scheduling state
    mcut::thread_pool scheduler;
#endif

    // the current set of connected components associated with context
    std::map<McConnectedComponent, std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)>> connComps = {};

    // The state and flag variable current used to configure the next dispatch call
    McFlags flags = (McFlags)0;
    McFlags dispatchFlags = (McFlags)0;

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
std::map<McContext, std::unique_ptr<McDispatchContextInternal>> gDispatchContexts;

void create_context_impl(McContext* pOutContext, McFlags flags)
{
    MCUT_ASSERT(pOutContext != nullptr);
    MCUT_ASSERT(flags != 0);

    // allocate internal context object (including associated threadpool etc.)
    std::unique_ptr<McDispatchContextInternal> context_uptr = std::unique_ptr<McDispatchContextInternal>(new McDispatchContextInternal());

    // copy context configuration flags
    context_uptr->flags = flags;

    // create handle (ptr) which will be returned and used by client to access rest of API
    const McContext handle = reinterpret_cast<McContext>(context_uptr.get());

    const std::pair<std::map<McContext, std::unique_ptr<McDispatchContextInternal>>::iterator, bool> insertion_result = gDispatchContexts.emplace(handle, std::move(context_uptr));

    const bool context_inserted_ok = insertion_result.second;

    if (!context_inserted_ok) {
        throw std::runtime_error("failed to create context");
    }

    const std::map<McContext, std::unique_ptr<McDispatchContextInternal>>::iterator context_entry_iter = insertion_result.first;

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

    std::map<McContext, std::unique_ptr<McDispatchContextInternal>>::iterator context_entry_iter = gDispatchContexts.find(contextHandle);

    if (context_entry_iter == gDispatchContexts.end()) {
        // "contextHandle" may not be NULL but that does not mean it maps to
        // a valid object in "gDispatchContexts"
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<McDispatchContextInternal>& context_uptr = context_entry_iter->second;

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
    std::map<McContext, std::unique_ptr<McDispatchContextInternal>>::iterator context_entry_iter = gDispatchContexts.find(contextHandle);

    if (context_entry_iter == gDispatchContexts.end()) {
        throw std::invalid_argument("invalid context");
    }

    const std::unique_ptr<McDispatchContextInternal>& context_uptr = context_entry_iter->second;

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

McResult dispatch_impl(
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
    uint32_t numCutMeshFaces) { }

McResult get_info_impl(
    const McContext context,
    McFlags info,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes) { }

McResult get_connected_components_impl(
    const McContext context,
    const McConnectedComponentType connectedComponentType,
    const uint32_t numEntries,
    McConnectedComponent* pConnComps,
    uint32_t* numConnComps) { }

McResult get_connected_component_data_impl(
    const McContext context,
    const McConnectedComponent connCompId,
    McFlags flags,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes) { }

McResult release_connected_components_impl(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps) { }

McResult release_context_impl(
    McContext context) { }

} // namespace frontend{