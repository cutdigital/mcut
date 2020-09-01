/**
 * @file mcut.h
 * @brief The MCUT header file.
 *
 */ 

#ifndef MCUT_API_H_
#define MCUT_API_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define MC_VERSION_1_0 1
#include "platform.h"
#define MC_MAKE_VERSION(major, minor, patch) \
    (((major) << 22) | ((minor) << 12) | (patch))

// mcut 1.0 version number
#define MC_API_VERSION_1_0 MC_MAKE_VERSION(1, 0, 0) // Patch version should always be set to 0

#define MC_VERSION_MAJOR(version) ((uint32_t)(version) >> 22)
#define MC_VERSION_MINOR(version) (((uint32_t)(version) >> 12) & 0x3ff)
#define MC_VERSION_PATCH(version) ((uint32_t)(version)&0xfff)
// Version of this file
#define MC_HEADER_VERSION 100

// Complete version of this file
#define MC_HEADER_VERSION_COMPLETE MC_MAKE_VERSION(1, 0, MC_HEADER_VERSION)

#define MC_NULL_HANDLE 0

#define MC_DEFINE_HANDLE(object) typedef struct object##_T* object;

/**
 * @brief Connected component handle.
 *
 * Opaque type referencing a connected component which the client must use to access mesh data after a dispatch call.
 */
MC_DEFINE_HANDLE(McConnectedComponent)

/**
 * @brief Context handle.
 *
 * Opaque type referencing a working state (e.g. independent thread) which the client must use to initialise, dispatch, and access data.
 */
MC_DEFINE_HANDLE(McContext)

/**
 * @brief Event handle.
 *
 * Opaque type referencing an instance of a task. Event objects are unique and can be used to identify a particular task execution instance later on.
 */
MC_DEFINE_HANDLE(McEvent)

typedef uint32_t McFlags;
typedef uint16_t McFaceSize;
typedef uint32_t McBool;

#define MC_TRUE (1)
#define MC_FALSE (0)

/**
 * @brief API return codes
 *
 * This enum enumerates the possible return values of API functions (integer). The values identify whether a function executed successfully or returned with an error.
 */
typedef enum McResult {
    MC_NO_ERROR = 0, /**< The function was successfully executed/enqueued. */
    MC_INVALID_SRC_MESH = -(1 << 0), /**< The input source-mesh does not the meet requirements of a valid mesh. */
    MC_INVALID_CUT_MESH = -(1 << 1), /**< The input cut-mesh does not the meet requirements of a valid mesh. */
    MC_INVALID_OPERATION = -(1 << 2), /**< An internal operation could not be executed successively. */
    MC_INVALID_VALUE = -(1 << 3), /**< An invalid value has been passed to the API. */
    MC_RESULT_MAX_ENUM = 0xFFFFFFFF /**< Wildcard (match all) . */
} McResult;

/**
 * @brief The possible types of connected components.
 *
 * This enum enumerates the possible types of connected components which can be queried from the API after a dispatch call. 
 */
typedef enum McConnectedComponentType {
    MC_CONNECTED_COMPONENT_TYPE_FRAGMENT = (1 << 0), /**< A connected component which is originates from the source-mesh. */
    MC_CONNECTED_COMPONENT_TYPE_PATCH = (1 << 2), /**< A connected component which is originates from the cut-mesh. */
    MC_CONNECTED_COMPONENT_TYPE_SEAMED = (1 << 3), /**< A connected component which is the same as either the source-mesh or the cut-mesh, but with additional edges defining the intersection contour (seam). */
    MC_CONNECTED_COMPONENT_TYPE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McConnectedComponentType;

/**
 * @brief The possible geometrical locations of a fragment connected component with-respect-to the cut-mesh.
 *
 * This enum enumerates the possible locations at which a fragment connected component can be relative to the cut-mesh. Note that the labels of 'above' or 'below' here are defined with-respect-to the winding-order (and hence, normal orientation) of the cut-mesh.
 */
typedef enum McFragmentLocation {
    MC_FRAGMENT_LOCATION_ABOVE = 1 << 0, /**< Fragment is located above the cut-mesh. */
    MC_FRAGMENT_LOCATION_BELOW = 1 << 1, /**< Fragment is located below the cut-mesh. */
    MC_FRAGMENT_LOCATION_UNDEFINED = 1 << 2, /**< Fragment is located neither above nor below the cut-mesh. */
    MC_FRAGMENT_LOCATION_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McFragmentLocation;

/**
 * @brief Topological configurations of a fragment connected component with-respect-to hole-filling.
 *
 * This enum enumerates the possible configurations that a fragment connected component can be in regarding the hole-filling process. Here, hole-filling refers to the stage/phase when holes produced by a cut are filled with a subset of polygons of the cut-mesh.
 */
typedef enum McFragmentSealType {
    MC_FRAGMENT_SEAL_TYPE_COMPLETE = 1 << 0, /**< Holes are completely sealed (watertight). */
    MC_FRAGMENT_SEAL_TYPE_PARTIAL = 1 << 1, /**< Holes are partially sealed (non watertight). Fragments with this property may have 1 to N-1 hole-filling polygons added, where N is the total number of cut-mesh polygons to create a watertight seal.*/
    MC_FRAGMENT_SEAL_TYPE_NONE = 1 << 2, /**< Holes are not sealed (gaping hole). */
    MC_FRAGMENT_SEAL_TYPE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McFragmentSealType;

/**
 * @brief Geometrical location of a patch connected component with-respect-to the source-mesh.
 *
 * This enum enumerates the possible locations at which a patch connected component can be relative to the source-mesh. Note that the labels of 'inside' or 'outside' here are defined with-respect-to the winding-order (and hence, normal orientation) of the source-mesh.
 */
typedef enum McPatchLocation {
    MC_PATCH_LOCATION_INSIDE = 1 << 0, /**< Patch is located on the interior of the source-mesh (used to seal holes). */
    MC_PATCH_LOCATION_OUTSIDE = 1, /**< Patch is located on the exterior of the source-mesh. Rather than hole-filling these patches seal from the outside so-as to extrude the cut.*/
    MC_PATCH_LOCATION_UNDEFINED = 2, /**< Patch is located neither on the interior nor exterior of the source-mesh. */
    MC_PATCH_LOCATION_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McPatchLocation;

/**
 * @brief Input mesh from which a seamed connected component is derived.
 *
 * This enum enumerates the possible origins of a seamed connected component which can be either the source-mesh or the cut-mesh. 
 */
typedef enum McSeamedConnectedComponentOrigin {
    MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_SRC_MESH = 1 << 0, /**< Seamed connected component from the input source mesh. */
    MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_CUT_MESH = 1 << 1, /**< Seamed connected component from the input cut mesh. */
    MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McSeamedConnectedComponentOrigin;

/**
 * @brief Data that can be queried about a connected component.
 *
 * This enum enumerates the different types of data for a connected component which can be queried from the API after a dispatch call.
 */
typedef enum McConnectedComponentData {
    MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT = (1 << 0), /**< Number of vertices. */
    MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT = (1 << 1), /**< List of vertices as an array of 32 bit floating-point numbers. */
    MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE = (1 << 2), /**< List of vertices as an array of 64 bit floating-point numbers. */
    MC_CONNECTED_COMPONENT_DATA_VERTEX_EXACT = (1 << 3), /**< List of vertices as a character string representing arbitrary-precision numbers. */
    MC_CONNECTED_COMPONENT_DATA_FACE = (1 << 4), /**< List faces as an array of indices. */
    MC_CONNECTED_COMPONENT_DATA_FACE_SIZE = (1 << 5), /**< List face sizes (vertices per face) as an array. */
    MC_CONNECTED_COMPONENT_DATA_EDGE = (1 << 6), /**< List edges as an array of indices. */
    MC_CONNECTED_COMPONENT_DATA_TYPE = (1 << 7), /**< The type of a connected component. */
    MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION = (1 << 8), /**< The location of a fragment connected component. */
    MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION = (1 << 9), /**< The location of a patch as part of a connected component. If teh connected component is a patch then this identifies its location with-respect-to the source-mesh.*/
    MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE = (1 << 10), /**< The Hole-filling configuration of a fragment connected component. */
    MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX = (1 << 11), /**< List of seam-vertices as an array of indices. */
    MC_CONNECTED_COMPONENT_DATA_ORIGIN = (1 << 12) /**< The input mesh (source- or cut-mesh) from which a seamed connected component is derived. */
} McConnectedComponentData;

/**
 * @brief Source of a debug log message.
 *
 * This enum enumerates the different types of sources from which a message is a debug log may originate.
 */
typedef enum McDebugSource {
    MC_DEBUG_SOURCE_API = 1 << 0, /**< messages generated by usage of the MCUT API. */
    MC_DEBUG_SOURCE_KERNEL = 1 << 1, /**< messages generated by the cutting kernel. */
    MC_DEBUG_SOURCE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McDebugSource;

/**
 * @brief Type of debug messages.
 *
 * This enum enumerates the different types messages describing MCUT errors. 
 */
typedef enum McDebugType {
    MC_DEBUG_TYPE_ERROR = 1 << 0, /**< Explicit error message.*/
    MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR = 1 << 1, /**< Attempted use of deprecated features.*/
    MC_DEBUG_TYPE_OTHER = 1 << 2, /**< Other types of messages,.*/
    MC_DEBUG_TYPE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McDebugType;

/**
 * @brief Severity levels of messages.
 *
 * This enum enumerates the different severities: low, medium or high severity messages.
 */
typedef enum McDebugSeverity {
    MC_DEBUG_SEVERITY_HIGH = 1 << 0,
    MC_DEBUG_SEVERITY_MIDIUM = 1 << 1,
    MC_DEBUG_SEVERITY_LOW = 1 << 2,
    MC_DEBUG_SEVERITY_NOTIFICATION = 1 << 3,
    MC_DEBUG_SEVERITY_ALL = 0xFFFFFFFF
} McDebugSeverity;

/**
 * @brief Context creation flags.
 *
 * This enum enumerates the flags with which a context can be created.
 */
typedef enum McContextCreationFlags {
    MC_DEBUG = (1 << 0), /**< Enable debug mode (message logging etc.).*/
    MC_PROFILING_ENABLE = (1 << 1) /**< Enable profiling mode.*/
} McContextCreationFlags;

/**
 * @brief Numerical rounding mode.
 *
 * This enum enumerates the flags indicating the supported rounding modes which applied during a dispatch call.
 * The MC_ROUNDING_MODE_TO_NEAREST mode works as in the IEEE 754 standard: in case the number to be rounded lies exactly in the middle of two representable numbers, it is rounded to the one with the least significant bit set to zero
 */
typedef enum McRoundingModeFlags {
    MC_ROUNDING_MODE_TO_NEAREST = (1 << 2), /**< round to nearest (roundTiesToEven in IEEE 754-2008).*/
    MC_ROUNDING_MODE_TOWARD_ZERO = (1 << 3), /**< round toward zero(roundTowardZero in IEEE 754 - 2008).*/
    MC_ROUNDING_MODE_TOWARD_POS_INF = (1 << 4), /**< round toward plus infinity (roundTowardPositive in IEEE 754-2008).*/
    MC_ROUNDING_MODE_TOWARD_NEG_INF = (1 << 5) /**< round toward minus infinity (roundTowardNegative in IEEE 754-2008).*/
} McRoundingModeFlags;

/**
 * @brief Dispatch configuration flags.
 *
 * This enum enumerates the flags specifying how to interprete input data, and execute the cutting pipeline.
 */
typedef enum McDispatchFlags {
    MC_DISPATCH_VERTEX_ARRAY_FLOAT = (1 << 0), /**< Interpret the input mesh vertices as arrays of 32-bit floating-point numbers.*/
    MC_DISPATCH_VERTEX_ARRAY_DOUBLE = (1 << 1), /**< Interpret the input mesh vertices as arrays of 64-bit floating-point numbers.*/
    MC_DISPATCH_VERTEX_ARRAY_EXACT = (1 << 2), /**< Interpret the input mesh vertices as character strings representing arbitrary-precision numbers.*/
    MC_DISPATCH_REQUIRE_SEVERING_SEAMS = (1 << 3) /**< Require that all intersections partition/disvide the source mesh into two subsets of polygons. Otherwise no-op.*/
} McDispatchFlags;

/**
 * @brief Flags for querying fixed API state.
 *
 * This enum enumerates the flags which are used to query for specific information about the constant state of the API and context. 
 */
typedef enum McQueryFlags {
    MC_CONTEXT_FLAGS = 1 << 0,
    MC_DONT_CARE = 1 << 1,
    MC_DEFAULT_PRECISION = 1 << 2,
    MC_DEFAULT_ROUNDING_MODE = 1 << 3,
    MC_PRECISION_MAX = 1 << 4,
    MC_PRECISION_MIN = 1 << 5,
    MC_DEBUG_LAST_LOGGED_MESSAGE = 1 << 6
} McQueryFlags;

/**
 * @brief Debug function callback signature type.
 *
 * Opaque type referencing an instance of a task. Event objects are unique and can be used to identify a particular task execution instance later on.
 */
typedef void (*pfn_mcDebugOutput_CALLBACK)(McDebugSource source, McDebugType type, unsigned int id, McDebugSeverity severity, size_t length, const char* message, const void* userParam);

/** @brief Create an MCUT context.
*
* This method creates a context object, which is a handle used by a client application to control the API state and access data.
* 
* @param [out] pContext a pointer to the allocated context handle
* @param [in] flags bitfield containing the context creation flags
*
 * An example of usage:
 * @code
 * McContext ctxt = MC_NULL_HANDLE;
 * McResult err = mcCreateContext(&ctxt, MC_DEBUG);
 * @endcode
*
* @return Error code.
*/
MCAPI_ATTR McResult MCAPI_CALL mcCreateContext(
    McContext* pContext, McFlags flags);

/** @brief Set the numerical rounding mode.
*
* This function updates context state to use given rounding mode during numerical operations performed in a dispatch call. @see McRoundingModeFlags
*
* @param [in] context a pointer to the allocated context handle
* @param [in] rmode The rounding mode
*
* @return Error code.
*
* @note This function has no effect if MCUT was built with MCUT_USE_NATIVE_FLOATING_POINT_NUMBERS.
*/
MCAPI_ATTR McResult MCAPI_CALL mcSetRoundingMode(
    McContext context,
    McFlags rmode);

/** @brief Get the numerical rounding mode.
*
* This function retrieves the rounding mode currently used the context. @see McRoundingModeFlags
*
* @param [in] context The context handle
* @param [out] rmode The current rounding mode
*
* @return Error code.
*/
MCAPI_ATTR McResult MCAPI_CALL mcGetRoundingMode(
    McContext context,
    McFlags* rmode);

/** @brief Set the precision bits.
*
* This function sets the default precision to be exactly prec bits, where prec can be any integer between MC_PRECISION_MAX and MC_PRECISION_MIN. The precision of a variable means the number of bits used to store its significand. The default precision is set to 53 bits initially.
*
* @param [in] context a pointer to the allocated context handle
* @param [in] rmode The number precision bits
*
* @return Error code.
*
* @note This function has no effect if MCUT was built with MCUT_USE_NATIVE_FLOATING_POINT_NUMBERS.
*/
MCAPI_ATTR McResult MCAPI_CALL mcSetPrecision(
    McContext context,
    uint32_t prec);

/** @brief Get the number of precision bits.
*
* This function retrieves the number of precision bits currently used by the context.
*
* @param [in] context The context handle
* @param [out] rmode The number of precision bits
*
* @return Error code.
*
* @note This function will always return sizeof(long double) * 8, if MCUT was built with MCUT_USE_NATIVE_FLOATING_POINT_NUMBERS.
*/
MCAPI_ATTR McResult MCAPI_CALL mcGetPrecision(
    McContext context,
    uint32_t* prec);

/** @brief Specify a callback to receive debugging messages from the MCUT library.
*
* mcDebugMessageCallback sets the current debug output callback function to the function whose address is
* given in callback.
*
* This function is defined to have the same calling convention as the MCUT API functions. In most cases
* this is defined as MCAPI_ATTR, although it will vary depending on platform, language and compiler.
*
* Each time a debug message is generated the debug callback function will be invoked with source, type,
* and severity associated with the message, and length set to the length of debug message whose
* character string is in the array pointed to by message userParam will be set to the value passed in
* the userParam parameter to the most recent call to mcDebugMessageCallback.
*
* @param[in] context The context handle that was created by a previous call to mcCreateContext.
* @param[in] cb The address of a callback function that will be called when a debug message is generated. 
* @param[in] userParam A user supplied pointer that will be passed on each invocation of callback.
*
* @return Error code.
*
*/
MCAPI_ATTR McResult MCAPI_CALL mcDebugMessageCallback(
    McContext context,
    pfn_mcDebugOutput_CALLBACK cb,
    const void* userParam);

/**
* Control the reporting of debug messages in a debug context.
*
* @param[in] context The context handle that was created by a previous call to @see mcCreateContext.
* @param[in] source The source of debug messages to enable or disable.
* @param[in] type The type of debug messages to enable or disable.
* @param[in] severity The severity of debug messages to enable or disable.
* @params[in] enabled A Boolean flag determining whether the selected messages should be enabled or disabled.
*
* mcDebugMessageControl controls the reporting of debug messages generated by a debug context. The parameters 
* source, type and severity form a filter to select messages from the pool of potential messages generated by 
* the MCUT library.
*
* [source] may be MC_DEBUG_SOURCE_API, MC_DEBUG_SOURCE_KERNEL to select messages 
* generated by usage of the MCUT API, the MCUT kernel or by the input, respectively. It may also take the 
* value MC_DEBUG_SOURCE_ALL. If source is not MC_DEBUG_SOURCE_ALL then only messages whose source matches 
* source will be referenced.
*
* [type] may be one of MC_DEBUG_TYPE_ERROR, GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR, or GL_DEBUG_TYPE_OTHER to indicate 
* the type of messages describing MCUT errors, attempted use of deprecated features, and other types of messages, 
* respectively. It may also take the value GL_DONT_CARE. If type is not MC_DEBUG_TYPE_ALL then only messages whose 
* type matches type will be referenced.
*
* [severity] may be one of MC_DEBUG_SEVERITY_LOW, MC_DEBUG_SEVERITY_MEDIUM, or MC_DEBUG_SEVERITY_HIGH to 
* select messages of low, medium or high severity messages or to MC_DEBUG_SEVERITY_NOTIFICATION for notifications. 
* It may also take the value MC_DEBUG_SEVERITY_ALL. If severity is not MC_DEBUG_SEVERITY_ALL then only 
* messages whose severity matches severity will be referenced.
*
* If [enabled] is true then messages that match the filter formed by source, type and severity are enabled. 
* Otherwise, those messages are disabled.
*/
MCAPI_ATTR McResult MCAPI_CALL mcDebugMessageControl(
    McContext context,
    McDebugSource source,
    McDebugType type,
    McDebugSeverity severity,
    bool enabled);

/** @brief Waits on the client thread for commands identified by event objects to complete.
*
* This function waits on the client thread for commands identified by event objects in eventList to complete. A command is considered complete if its execution status is MC_COMPLETE or a negative value. The events specified in eventList act as synchronization points.
*
*
* @param [in] context The context handle
* @param [in] rmode The number of events in eventList.
* @param [out] rmode A pointer to a list of event object handles.
*
* @return Error code.
*
* @note This function is not yet implemented.
*/
MCAPI_ATTR McResult MCAPI_CALL mcWaitForEvents(
    const McContext context,
    uint32_t numEvents,
    const McEvent* eventList);

/** @brief Blocks until all previously queued MCUT commands associated with the context have completed.
*
* All previously submitted MCUT commands in context are executed, and the function blocks until all previously sumbitted commands have completed. The function will not return until all previously submitted tasks in the context have been processed and completed. The function is also a synchronization point.
*
* @param [in] context The context handle
*
* @return Error code.
*
* @note This function is not yet implemented.
*/
MCAPI_ATTR McResult MCAPI_CALL mcFinish(
    const McContext context);

/**
* Execute a cutting operation with two meshes - the source mesh, and the cut mesh.
*
* @param[in] context The context handle that was created by a previous call to @see mcCreateContext.
* @param[in] pSrcMeshVertices The vertices (x,y,z) of the source mesh.
* @param[in] pSrcMeshFaceIndices The indices of the faces (polygons) in the source mesh.
* @param[in] pSrcMeshFaceSizes The sizes (in terms of vertex indices) of the faces in the source mesh.
* @params[in] numSrcMeshVertices The number of vertices in the source mesh.
* @params[in] numSrcMeshFaces The number of faces in the source mesh.
* @param[in] pCutMeshVertices The vertices (x,y,z) of the cut mesh.
* @param[in] pCutMeshFaceIndices The indices of the faces (polygons) in the cut mesh.
* @param[in] pCutMeshFaceSizes The sizes (in terms of vertex indices) of the faces in the cut mesh.
* @params[in] numCutMeshVertices The number of vertices in the cut mesh.
* @params[in] numCutMeshFaces The number of faces in the cut mesh.
* @params[in] numEventInWaitList Specify the number of events that need to complete before this particular command can be executed
* @params[in] waitlist Specify events that need to complete before this particular command can be executed
* @params[out] event Returns an event object that identifies this particular task execution instance.
*
* This function specifies the two mesh objects to operate on. The 'source mesh' is the mesh to be cut 
* (i.e. partitioned) along intersection paths prescribed by the 'cut mesh'. Instead of performing 
* intermediate conversions like volumetric/tetrahedral decompositions or level-sets, this function 
* operates directly on the halfedge representations of the two input meshes. 
* 
* Numerical operations are required but only to evaluate polygon intersection points. The rest of 
* the operation resolves the combinatorial structure of the underlying meshes using halfedge 
* connectivity. These numerical operations are represented exact arithemetic which makes the routine
* also robust to floating-point error.
* 
* @return Error code.
*/
MCAPI_ATTR McResult MCAPI_CALL mcDispatch(
    McContext context,
    McFlags flags,
    const void* pSrcMeshVertices,
    const uint32_t* pSrcMeshFaceIndices,
    const McFaceSize* pSrcMeshFaceSizes,
    uint32_t numSrcMeshVertices,
    uint32_t numSrcMeshFaces,
    const void* pCutMeshVertices,
    const uint32_t* pCutMeshFaceIndices,
    const McFaceSize* pCutMeshFaceSizes,
    uint32_t numCutMeshVertices,
    uint32_t numCutMeshFaces,
    uint32_t numEventInWaitList,
    const McEvent* waitlist,
    McEvent* event);

/**
* Return the value of a selected parameter.
*
* @param[in] context The context handle that was created by a previous call to @see mcCreateContext. The symbolic constants in @see McQuery accepted.
* @param[in] info Specifies the information object being queried. @see McQueryFlags
* @param[in] bytes Specifies the size in bytes of memory pointed to by pMem. This size must be great than or equal to the return type size of data type queried
* @param[out] pMem A pointer to memory where the appropriate result being queried is returned. If pMem is NULL, it is ignored.
* @param[out] pNumBytes returns the actual size in bytes of data being queried by info. If pNumBytes is NULL, it is ignored.
*
* @return Error code.
*
* @note Event synchronisation is not yet implemented.
*/
MCAPI_ATTR McResult MCAPI_CALL mcGetInfo(
    const McContext context,
    McFlags info,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes);

/*
 given the type info, this function will return an array of all conn comps matching the given description
 if flags is 0 then it will return all conn comps
*/

/**
* Query the connected components available in a context.
* 
* This function will return an array of connected components matching the given description of flags.
* 
* If blocking is MC_TRUE i.e. the command is blocking, mcGetConnectedComponents does not return until the data has been read and copied into memory pointed to.
* 
* If blocking_read is MC_FALSE i.e. the command is non-blocking, mcGetConnectedComponents submits a non-blocking read command and returns. The contents of the destination ptr cannot be used until the read command has completed. The event argument returns an event object which can be used to query the execution status of the read command. When the read command has completed, the contents of the memory that the destination pointer points to can be used by the application.
*
* @param[in] context The context handle
* @param[in] blocking Indicate if the read and write operations are blocking or non-blocking
* @param[in] connectedComponentType The type(s) of connected component sought. @see McConnectedComponentType
* @param[in] numEntries The number of McConnectedComponent entries that can be added to pConnComps. If pConnComps is not NULL, the numEntries 
* must be the number of elements in pConnComps.
* @param[out] pConnComps Returns a list of MCUT connected components found. The McConnectedComponentType values returned in pConnComps can be used 
* to identify a specific MCUT connected component. If pConnComps is NULL, this argument is ignored. The number of MCUT connected components returned 
* is the minimum of the value specified by numEntries or the number of MCUT connected components whose type matches connectedComponentType.
* @param[out] numConnComps Returns the number of MCUT connected components available that match connectedComponentType. If numConnComps is NULL, 
* this argument is ignored.
* @params[in] numEventInWaitList Specify the number of events that need to complete before this particular command can be executed
* @params[in] waitlist Specify events that need to complete before this particular command can be executed
* @params[out] event Returns an event object that identifies this particular task execution instance.
*
* @return Error code.
*
* @note Event synchronisation is not yet implemented.
*/
MCAPI_ATTR McResult MCAPI_CALL mcGetConnectedComponents(
    const McContext context,
    const McBool blocking,
    const McConnectedComponentType connectedComponentType,
    const uint32_t numEntries,
    McConnectedComponent* pConnComps,
    uint32_t* numConnComps,
    uint32_t numEventInWaitList,
    const McEvent* waitlist,
    McEvent* event);

/**
* Query specific information about a connected component.
*
* @param[in] context The context handle that was created by a previous call to @see mcCreateContext. The symbolic constants in @see McQuery accepted.
* @param[in] blocking Indicate if the read and write operations are blocking or non-blocking
* @param[in] connCompId A connected component returned by 'mcGetConnectedComponents' whose data is to be read.
* @param[in] flags An enumeration constant that identifies the connected component information being queried.
* @param[in] bytes Specifies the size in bytes of memory pointed to by 'flags'.
* @param[out] pMem A pointer to memory location where appropriate values for a given 'flags' will be returned. If 'pMem' is NULL, it is ignored.
* @params[out] pNumBytes Returns the actual size in bytes of data being queried by 'flags'. If 'pNumBytes' is NULL, it is ignored.
* @params[in] numEventInWaitList Specify the number of events that need to complete before this particular command can be executed
* @params[in] waitlist Specify events that need to complete before this particular command can be executed
* @params[out] event Returns an event object that identifies this particular task execution instance.
*
* The connected component queries described in the @see 'McConnectedComponentInfo' should return the same information for a connected component returned by 'mcGetConnectedComponents'.
*
* @return Error code.
*
* @note Event synchronisation is not yet implemented.
*/
MCAPI_ATTR McResult MCAPI_CALL mcGetConnectedComponentData(
    const McContext context,
    const McBool blocking,
    const McConnectedComponent connCompId,
    McFlags flags,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes,
    uint32_t numEventInWaitList,
    const McEvent* waitlist,
    McEvent* event);

/**
* To release the memory of a connected component, call this function.
*
* If numConnComps is zero and pConnComps is NULL, the memory of all connected components associated with the context is freed.
*
* @param[in] context The context handle that was created by a previous call to @see mcCreateContext. The symbolic constants in @see McQuery accepted.
* @param[in] numConnComps Number of connected components in 'pConnComps' whose memory to release.
* @param[in] pConnComps The connected components whose memory will be released.
*
* @return Error code.
* 
*/
MCAPI_ATTR McResult MCAPI_CALL mcReleaseConnectedComponents(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps);

/**
* To release the memory of a context, call this function.
*
* This function ensures that all the state attached to context (such as connected components) are released, and the memory is deleted.
*
* @param[in] context The context handle that was created by a previous call to @see mcCreateContext. 
*
* @return Error code.
*
*/
MCAPI_ATTR McResult MCAPI_CALL mcReleaseContext(
    McContext context);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // #ifndef MCUT_API_H_
