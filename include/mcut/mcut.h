
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
 * 
 * (B)  Commercial license.
 *      - email: contact@cut-digital.com
 * 
 * The commercial license option is for users that wish to use MCUT in 
 * their products for comercial purposes but do not wish to release their 
 * software products under the GPL license. 
 * 
 */

/**
 * @file mcut.h
 * @author Floyd M. Chitalu
 * @date 1 Jan 2021
 * 
 * @brief File containing the MCUT applications programming interface (API).
 * 
 * NOTE: This header file defines all the functionality and accessible features of MCUT.
 * The interface is a standard C API.  
 * 
 */

#ifndef MCUT_API_H_
#define MCUT_API_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "platform.h"

// Macro to encode MCUT version
#define MC_MAKE_VERSION(major, minor, patch) \
    (((major) << 22) | ((minor) << 12) | (patch))

// MCUT 1.0 version number
#define MC_API_VERSION_1_0 MC_MAKE_VERSION(1, 0, 0) // Patch version should always be set to 0

// Macro to decode MCUT version (MAJOR) from MC_HEADER_VERSION_COMPLETE
#define MC_VERSION_MAJOR(version) ((uint32_t)(version) >> 22)

// Macro to decode MCUT version (MINOR) from MC_HEADER_VERSION_COMPLETE
#define MC_VERSION_MINOR(version) (((uint32_t)(version) >> 12) & 0x3ff)

// Macro to decode MCUT version (PATCH) from MC_HEADER_VERSION_COMPLETE
#define MC_VERSION_PATCH(version) ((uint32_t)(version)&0xfff)

// Version of this file
#define MC_HEADER_VERSION 100

// Complete version of this file
#define MC_HEADER_VERSION_COMPLETE MC_MAKE_VERSION(1, 0, MC_HEADER_VERSION)

// Constant value assigned to null variables and parameters
#define MC_NULL_HANDLE 0

// Helper-macro to define opaque handles
#define MC_DEFINE_HANDLE(object) typedef struct object##_T* object;

/**
 * \struct McConnectedComponent
 * @brief Connected component handle.
 *
 * Opaque type referencing a connected component which the client must use to access mesh data after a dispatch call.
 */
typedef struct McConnectedComponent_T* McConnectedComponent;

/**
 * \struct McContext
 * @brief Context handle.
 *
 * Opaque type referencing a working state (e.g. independent thread) which the client must use to initialise, dispatch, and access data.
 */
typedef struct McContext_T* McContext;

/**
 * \struct McEvent
 * @brief Event handle.
 *
 * Opaque type referencing an instance of a task. Event objects are unique and can be used to identify a particular task execution instance later on.
 */
typedef struct McEvent_T* McEvent;

/**
 * @brief Bitfield type.
 *
 * Integral type representing a 32-bit bitfield for storing parameter values.
 */
typedef uint32_t McFlags;

/**
 * @brief Boolean type.
 *
 * Integral type representing a boolean value (MC_TRUE or MC_FALSE).
 */
typedef uint32_t McBool;

/**
 * @brief Boolean constant for "true".
 *
 * Integral constant representing a boolean value evaluating to true.
 */
#define MC_TRUE (1)

/**
 * @brief Boolean constant for "false".
 *
 * Integral constant representing a boolean value evaluating to false.
 */
#define MC_FALSE (0)

/**
 * @brief API return codes
 *
 * This enum structure defines the possible return values of API functions (integer). The values identify whether a function executed successfully or returned with an error.
 */
typedef enum McResult {
    MC_NO_ERROR = 0, /**< The function was successfully executed/enqueued. */
    MC_INVALID_SRC_MESH = -(1 << 0), /**< The input source-mesh does not the meet requirements of a valid mesh. */
    MC_INVALID_CUT_MESH = -(1 << 1), /**< The input cut-mesh does not the meet requirements of a valid mesh. */
    MC_INVALID_OPERATION = -(1 << 2), /**< An internal operation could not be executed successively. */
    MC_INVALID_VALUE = -(1 << 3), /**< An invalid value has been passed to the API. */
    MC_OUT_OF_MEMORY = -(1 << 4), /** Memory allocation operation cannot allocate memory. */
    MC_RESULT_MAX_ENUM = 0xFFFFFFFF /**< Wildcard (match all) . */
} McResult;

/**
 * \enum McConnectedComponentType
 * @brief The possible types of connected components.
 *
 * This enum structure defines the possible types of connected components which can be queried from the API after a dispatch call. 
 */
typedef enum McConnectedComponentType {
    MC_CONNECTED_COMPONENT_TYPE_FRAGMENT = (1 << 0), /**< A connected component which is originates from the source-mesh. */
    MC_CONNECTED_COMPONENT_TYPE_PATCH = (1 << 2), /**< A connected component which is originates from the cut-mesh. */
    MC_CONNECTED_COMPONENT_TYPE_SEAMED = (1 << 3), /**< A connected component which is the same as either the source-mesh or the cut-mesh, but with additional edges defining the intersection contour (seam). */
    MC_CONNECTED_COMPONENT_TYPE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McConnectedComponentType;

/**
 * \enum McFragmentLocation
 * @brief The possible geometrical locations of a fragment connected component with-respect-to the cut-mesh.
 *
 * This enum structure defines the possible locations where a fragment connected component can be relative to the cut-mesh. Note that the labels of 'above' or 'below' here are defined with-respect-to the winding-order (and hence, normal orientation) of the cut-mesh.
 */
typedef enum McFragmentLocation {
    MC_FRAGMENT_LOCATION_ABOVE = 1 << 0, /**< Fragment is located above the cut-mesh. */
    MC_FRAGMENT_LOCATION_BELOW = 1 << 1, /**< Fragment is located below the cut-mesh. */
    MC_FRAGMENT_LOCATION_UNDEFINED = 1 << 2, /**< Fragment is located neither above nor below the cut-mesh. */
    MC_FRAGMENT_LOCATION_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McFragmentLocation;

/**
 * \enum McFragmentSealType
 * @brief Topological configurations of a fragment connected component with-respect-to hole-filling.
 *
 * This enum structure defines the possible configurations that a fragment connected component can be in regarding the hole-filling process. Here, hole-filling refers to the stage/phase when holes produced by a cut are filled with a subset of polygons of the cut-mesh.
 */
typedef enum McFragmentSealType {
    MC_FRAGMENT_SEAL_TYPE_COMPLETE = 1 << 0, /**< Holes are completely sealed (watertight). */
    MC_FRAGMENT_SEAL_TYPE_PARTIAL = 1 << 1, /**< Holes are partially sealed (non watertight). Fragments with this property may have 1 to N-1 hole-filling polygons added, where N is the total number of cut-mesh polygons to create a watertight seal.*/
    MC_FRAGMENT_SEAL_TYPE_NONE = 1 << 2, /**< Holes are not sealed (gaping hole). */
    MC_FRAGMENT_SEAL_TYPE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McFragmentSealType;

/**
 * \enum McPatchLocation
 * @brief Geometrical location of a patch connected component with-respect-to the source-mesh.
 *
 * This enum structure defines the possible locations where a patch connected component can be relative to the source-mesh. Note that the labels of 'inside' or 'outside' here are defined with-respect-to the winding-order (and hence, normal orientation) of the source-mesh.
 */
typedef enum McPatchLocation {
    MC_PATCH_LOCATION_INSIDE = 1 << 0, /**< Patch is located on the interior of the source-mesh (used to seal holes). */
    MC_PATCH_LOCATION_OUTSIDE = 1 << 1, /**< Patch is located on the exterior of the source-mesh. Rather than hole-filling these patches seal from the outside so-as to extrude the cut.*/
    MC_PATCH_LOCATION_UNDEFINED = 1 << 2, /**< Patch is located neither on the interior nor exterior of the source-mesh. */
    MC_PATCH_LOCATION_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McPatchLocation;

/**
 * \enum McSeamedConnectedComponentOrigin
 * @brief Input mesh from which a seamed connected component is derived.
 *
 * This enum structure defines the possible origins of a seamed connected component, which can be either the source-mesh or the cut-mesh. 
 */
typedef enum McSeamedConnectedComponentOrigin {
    MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_SRC_MESH = 1 << 0, /**< Seamed connected component from the input source mesh. */
    MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_CUT_MESH = 1 << 1, /**< Seamed connected component from the input cut mesh. */
    MC_SEAMED_CONNECTED_COMPONENT_ORIGIN_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McSeamedConnectedComponentOrigin;

/**
 * \enum McConnectedComponentData
 * @brief Data that can be queried about a connected component.
 *
 * This enum structure defines the different types of data that are associated with a connected component and can be queried from the API after a dispatch call.
 */
typedef enum McConnectedComponentData {
    MC_CONNECTED_COMPONENT_DATA_VERTEX_COUNT = (1 << 0), /**< Number of vertices. */
    MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT = (1 << 1), /**< List of vertex coordinates as an array of 32 bit floating-point numbers. */
    MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE = (1 << 2), /**< List of vertex coordinates as an array of 64 bit floating-point numbers. */
    MC_CONNECTED_COMPONENT_DATA_VERTEX_EXACT = (1 << 3), /**< List of vertex coordinates as a character string representing arbitrary-precision numbers. Values are exact only if ARBITRARY_PRECISION_NUMBERS is defined. Otherwise, the conversion operation is equivalent to using std::to_string. */
    MC_CONNECTED_COMPONENT_DATA_FACE = (1 << 4), /**< List of faces as an array of indices. */
    MC_CONNECTED_COMPONENT_DATA_FACE_SIZE = (1 << 5), /**< List of face sizes (vertices per face) as an array. */
    MC_CONNECTED_COMPONENT_DATA_EDGE = (1 << 6), /**< List of edges as an array of indices. */
    MC_CONNECTED_COMPONENT_DATA_TYPE = (1 << 7), /**< The type of a connected component (See also: ::McConnectedComponentType.). */
    MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION = (1 << 8), /**< The location of a fragment connected component with respect to the cut mesh (See also: ::McFragmentLocation). */
    MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION = (1 << 9), /**< The location of a patch with respect to the source mesh (See also: ::McPatchLocation).*/
    MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE = (1 << 10), /**< The Hole-filling configuration of a fragment connected component (See also: ::McFragmentSealType). */
    MC_CONNECTED_COMPONENT_DATA_SEAM_VERTEX = (1 << 11), /**< List of seam-vertices as an array of indices. */
    MC_CONNECTED_COMPONENT_DATA_ORIGIN = (1 << 12) /**< The input mesh (source- or cut-mesh) from which a seamed connected component is derived (See also: ::McSeamedConnectedComponentOrigin). */
} McConnectedComponentData;

/**
 * \enum McDebugSource
 * @brief Source of a debug log message.
 *
 * This enum structure defines the sources from which a message in a debug log may originate.
 */
typedef enum McDebugSource {
    MC_DEBUG_SOURCE_API = 1 << 0, /**< messages generated by usage of the MCUT API. */
    MC_DEBUG_SOURCE_KERNEL = 1 << 1, /**< messages generated by the cutting kernel. */
    MC_DEBUG_SOURCE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McDebugSource;

/**
 * \enum McDebugType
 * @brief Type of debug messages.
 *
 * This enum structure defines the types of debug a message relating to an error. 
 */
typedef enum McDebugType {
    MC_DEBUG_TYPE_ERROR = 1 << 0, /**< Explicit error message.*/
    MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR = 1 << 1, /**< Attempted use of deprecated features.*/
    MC_DEBUG_TYPE_OTHER = 1 << 2, /**< Other types of messages,.*/
    MC_DEBUG_TYPE_ALL = 0xFFFFFFFF /**< Wildcard (match all) . */
} McDebugType;

/**
 * \enum McDebugSeverity
 * @brief Severity levels of messages.
 *
 * This enum structure defines the different severities of error: low, medium or high severity messages.
 */
typedef enum McDebugSeverity {
    MC_DEBUG_SEVERITY_HIGH = 1 << 0, /**< All MCUT Errors, mesh conversion/parsing errors, or undefined behavior.*/
    MC_DEBUG_SEVERITY_MEDIUM = 1 << 1, /**< Major performance warnings, debugging warnings, or the use of deprecated functionality.*/
    MC_DEBUG_SEVERITY_LOW = 1 << 2, /**< Redundant state change, or unimportant undefined behavior.*/
    MC_DEBUG_SEVERITY_NOTIFICATION = 1 << 3, /**< Anything that isn't an error or performance issue.*/
    MC_DEBUG_SEVERITY_ALL = 0xFFFFFFFF /**< Match all (wildcard).*/
} McDebugSeverity;

/**
 * \enum McContextCreationFlags
 * @brief Context creation flags.
 *
 * This enum structure defines the flags with which a context can be created.
 */
typedef enum McContextCreationFlags {
    MC_DEBUG = (1 << 0), /**< Enable debug mode (message logging etc.).*/
    MC_PROFILING_ENABLE = (1 << 1) /**< Enable profiling mode.*/
} McContextCreationFlags;

/**
 * \enum McRoundingModeFlags
 * @brief Numerical rounding mode.
 *
 * This enum structure defines the supported rounding modes which are applied when computing intersections during a dispatch call.
 * The MC_ROUNDING_MODE_TO_NEAREST mode works as in the IEEE 754 standard: in case the number to be rounded lies exactly in the middle of two representable numbers, it is rounded to the one with the least significant bit set to zero
 */
typedef enum McRoundingModeFlags {
    MC_ROUNDING_MODE_TO_NEAREST = (1 << 2), /**< round to nearest (roundTiesToEven in IEEE 754-2008).*/
    MC_ROUNDING_MODE_TOWARD_ZERO = (1 << 3), /**< round toward zero(roundTowardZero in IEEE 754 - 2008).*/
    MC_ROUNDING_MODE_TOWARD_POS_INF = (1 << 4), /**< round toward plus infinity (roundTowardPositive in IEEE 754-2008).*/
    MC_ROUNDING_MODE_TOWARD_NEG_INF = (1 << 5) /**< round toward minus infinity (roundTowardNegative in IEEE 754-2008).*/
} McRoundingModeFlags;

/**
 * \enum McDispatchFlags
 * @brief Dispatch configuration flags.
 *
 * This enum structure defines the flags indicating MCUT is to interprete input data, and execute the cutting pipeline.
 */
typedef enum McDispatchFlags {
    MC_DISPATCH_VERTEX_ARRAY_FLOAT = (1 << 0), /**< Interpret the input mesh vertices as arrays of 32-bit floating-point numbers.*/
    MC_DISPATCH_VERTEX_ARRAY_DOUBLE = (1 << 1), /**< Interpret the input mesh vertices as arrays of 64-bit floating-point numbers.*/
    MC_DISPATCH_VERTEX_ARRAY_EXACT = (1 << 2), /**< Interpret the input mesh vertices as character strings representing arbitrary-precision numbers. Values are parsed exact only if ARBITRARY_PRECISION_NUMBERS is defined. Otherwise, the conversion operation is equivalent to std::to_string.*/
    MC_DISPATCH_REQUIRE_SEVERING_SEAMS = (1 << 3), /**< Require that all intersection paths partition/divide the source mesh into two subsets of polygons. Otherwise, ::mcDispatch is a no-op. This flag enforces the requirement that only through-cuts are valid cuts.*/
    MC_DISPATCH_KEEP_PARTIALLY_SEALED_FRAGMENTS = (1 << 3) /** Include partially sealed fragments in the returned output. */
} McDispatchFlags;

/**
 * \enum McQueryFlags
 * @brief Flags for querying fixed API state.
 *
 * This enum structure defines the flags which are used to query for specific information about the state of the API and/or a given context. 
 */
typedef enum McQueryFlags {
    MC_CONTEXT_FLAGS = 1 << 0, /**< Flags used to create a context.*/
    MC_DONT_CARE = 1 << 1, /**< wildcard.*/
    MC_DEFAULT_PRECISION = 1 << 2, /**< Default number of bits used to represent the significand of a floating-point number.*/
    MC_DEFAULT_ROUNDING_MODE = 1 << 3, /**< Default way to round the result of a floating-point operation.*/
    MC_PRECISION_MAX = 1 << 4, /**< Maximum value for precision bits.*/
    MC_PRECISION_MIN = 1 << 5, /**< Minimum value for precision bits.*/
    MC_DEBUG_KERNEL_TRACE = 1 << 6 /**< Verbose log of the kernel execution trace.*/
} McQueryFlags;

/**
 *  
 * @brief Debug callback function signature type.
 *
 * The callback function should have this prototype (in C), or be otherwise compatible with such a prototype.
 */
typedef void (*pfn_mcDebugOutput_CALLBACK)(
    McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

/** @brief Create an MCUT context.
*
* This method creates a context object, which is a handle used by a client application to control the API state and access data.
* 
* @param [out] pContext a pointer to the allocated context handle
* @param [in] flags bitfield containing the context creation flags
*
 * An example of usage:
 * @code
 * McContext myContext = MC_NULL_HANDLE;
 * McResult err = mcCreateContext(&myContext, MC_NULL_HANDLE);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
* 
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL
*   -# Failure to allocate resources
*   -# \p flags defines an invalid bitfield.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcCreateContext(
    McContext* pContext, McFlags flags);

/** @brief Set the numerical rounding mode.
*
* This function updates context state to use given rounding mode during dispatch calls. See ::McRoundingModeFlags.
*
* @param [in] context a pointer to a previous allocated context.
* @param [in] rmode The rounding mode.
*
 * An example of usage:
 * @code
 * McResult err = mcSetRoundingMode(&myContext, MC_ROUNDING_MODE_TO_NEAREST);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL
*   -# \p rmode defines an invalid bitfield (e.g. more than one rounding mode).
*
* @note This function is a no-op if ARBITRARY_PRECISION_NUMBERS is not defined.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcSetRoundingMode(
    McContext context,
    McFlags rmode);

/** @brief Get the numerical rounding mode.
*
* This function retrieves the rounding mode currently used the context. @see McRoundingModeFlags
*
* @param [in] context The context handle
* @param [out] pRmode The returned value for the current rounding mode
*
 * An example of usage:
 * @code
 * McFlags myRoundingMode = 0;
 * McResult err = mcGetRoundingMode(&myContext, &myRoundingMode);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p pRmode is NULL.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcGetRoundingMode(
    McContext context,
    McFlags* pRmode);

/** @brief Set the precision bits.
*
* This function sets the default precision to be exactly prec bits, where prec can be any integer between MC_PRECISION_MAX and MC_PRECISION_MIN. The precision of a variable means the number of bits used to store its significand. The default precision is set to 53 bits initially if ARBITRARY_PRECISION_NUMBERS is defined. Otherwise, the default precision is set to "sizeof(long double) * 8" bits.
*
* @param [in] context a pointer to the allocated context handle
* @param [in] prec The number precision bits
*
*
 * An example of usage:
 * @code
 * uint64_t prec = 64;
 * McResult err = mcSetPrecision(&myContext, prec);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p prec is not an between #MC_PRECISION_MAX and #MC_PRECISION_MIN (See ::McQueryFlags).
*
* @note This function is a no-op if ARBITRARY_PRECISION_NUMBERS is not defined. Do not attempt to set the precision to any value near #MC_PRECISION_MAX, otherwise mcut will abort due to an assertion failure. Moreover, you may reach some memory limit on your platform, in which case the program may abort, crash or have undefined behavior (depending on your C implementation).

*/
extern MCAPI_ATTR McResult MCAPI_CALL mcSetPrecision(
    McContext context,
    uint64_t prec);

/** @brief Get the number of precision bits.
*
* This function retrieves the number of precision bits currently used by the context.
*
* @param [in] context The context handle
* @param [out] pPrec The number of precision bits
*
 * An example of usage:
 * @code
 * uint64_t myPrecisionBits = 0;
 * McResult err = mcGetPrecision(&myContext, &myPrecisionBits);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p pPrec is NULL.
*
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcGetPrecision(
    McContext context,
    uint64_t* pPrec);

/** @brief Specify a callback to receive debugging messages from the MCUT library.
*
* ::mcDebugMessageCallback sets the current debug output callback function to the function whose address is
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
 * An example of usage:
 * @code
 * // define my callback (with type pfn_mcDebugOutput_CALLBACK)
 * MCAPI_ATTR void MCAPI_CALL mcDebugOutput(McDebugSource source,   McDebugType type, unsigned int id, McDebugSeverity severity,size_t length, const char* message,const void* userParam)
 * {
 *  // do stuff
 * }
 * 
 * // ...
 * 
 * void* someData = NULL;
 * McResult err = mcDebugMessageCallback(myContext, mcDebugOutput, someData);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
 * 
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p cb is NULL.
*
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcDebugMessageCallback(
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
* @param[in] enabled A Boolean flag determining whether the selected messages should be enabled or disabled.
*
* ::mcDebugMessageControl controls the reporting of debug messages generated by a debug context. The parameters 
* source, type and severity form a filter to select messages from the pool of potential messages generated by 
* the MCUT library.
*
* \p source may be #MC_DEBUG_SOURCE_API, #MC_DEBUG_SOURCE_KERNEL to select messages 
* generated by usage of the MCUT API, the MCUT kernel or by the input, respectively. It may also take the 
* value #MC_DEBUG_SOURCE_ALL. If source is not #MC_DEBUG_SOURCE_ALL then only messages whose source matches 
* source will be referenced.
*
* \p type may be one of #MC_DEBUG_TYPE_ERROR, #MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR, or #MC_DEBUG_TYPE_OTHER to indicate 
* the type of messages describing MCUT errors, attempted use of deprecated features, and other types of messages, 
* respectively. It may also take the value #MC_DONT_CARE. If type is not #MC_DEBUG_TYPE_ALL then only messages whose 
* type matches type will be referenced.
*
* \p severity may be one of #MC_DEBUG_SEVERITY_LOW, #MC_DEBUG_SEVERITY_MEDIUM, or #MC_DEBUG_SEVERITY_HIGH to 
* select messages of low, medium or high severity messages or to #MC_DEBUG_SEVERITY_NOTIFICATION for notifications. 
* It may also take the value #MC_DEBUG_SEVERITY_ALL. If severity is not #MC_DEBUG_SEVERITY_ALL then only 
* messages whose severity matches severity will be referenced.
*
* If \p enabled is true then messages that match the filter formed by source, type and severity are enabled. 
* Otherwise, those messages are disabled.
*
*
 * An example of usage:
 * @code
 * // ... typically after setting debug callback with ::mcDebugMessageCallback
 * McResult err = mcDebugMessageControl(myContext, MC_DEBUG_SOURCE_ALL, MC_DEBUG_TYPE_ALL, MC_DEBUG_SEVERITY_ALL, true);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p source is not a value define in ::McDebugSource.
*   -# \p type is not a value define in ::McDebugType.
*   -# \p severity is not a value define in ::McDebugSeverity.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcDebugMessageControl(
    McContext context,
    McDebugSource source,
    McDebugType type,
    McDebugSeverity severity,
    bool enabled);

/** @brief Waits on the client thread for commands identified by event objects to complete.
*
* This function waits on the client thread for commands identified by event objects in eventList to complete. A command is considered complete if its execution status is MC_COMPLETE or a negative value. The events specified in eventList act as synchronization points.
*
* @param [in] context The context handle
* @param [in] numEvents The number of events in eventList.
* @param [in] eventList A pointer to a list of event object handles.
*
 * An example of usage:
 * @code
 * // coming soon!
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# if \p numEvents is zero or \p pEventList is NULL .
*   -# event objects specified in \p pEventList are not valid event objects.
*
* @note This function is not yet implemented.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcWaitForEvents(
    const McContext context,
    uint32_t numEvents,
    const McEvent* pEventList);

/** @brief Blocks until all previously queued MCUT commands associated with the context have completed.
*
* All previously submitted MCUT commands in context are executed, and the function blocks until all previously sumbitted commands have completed. The function will not return until all previously submitted tasks in the context have been processed and completed. The function is also a synchronization point.
*
* @param [in] context The context handle.
*
 * An example of usage:
 * @code
 * // coming soon!
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*
* @note This function is not yet implemented.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcFinish(
    const McContext context);

/**
* @brief Execute a cutting operation with two meshes - the source mesh, and the cut mesh.
*
* @param[in] context The context handle that was created by a previous call to ::mcCreateContext.
* @param[in] flags The flags indicating how to interprete input data and configure the execution.
* @param[in] pSrcMeshVertices The vertices (x,y,z) of the source mesh.
* @param[in] pSrcMeshFaceIndices The indices of the faces (polygons) in the source mesh.
* @param[in] pSrcMeshFaceSizes The sizes (in terms of vertex indices) of the faces in the source mesh.
* @param[in] numSrcMeshVertices The number of vertices in the source mesh.
* @param[in] numSrcMeshFaces The number of faces in the source mesh.
* @param[in] pCutMeshVertices The vertices (x,y,z) of the cut mesh.
* @param[in] pCutMeshFaceIndices The indices of the faces (polygons) in the cut mesh.
* @param[in] pCutMeshFaceSizes The sizes (in terms of vertex indices) of the faces in the cut mesh.
* @param[in] numCutMeshVertices The number of vertices in the cut mesh.
* @param[in] numCutMeshFaces The number of faces in the cut mesh.
* @param[in] numEventInWaitList Specify the number of events that need to complete before this particular command can be executed
* @param[in] waitlist Specify events that need to complete before this particular command can be executed
* @param[out] event Returns an event object that identifies this particular task execution instance.
*
* This function specifies the two mesh objects to operate on. The 'source mesh' is the mesh to be cut 
* (i.e. partitioned) along intersection paths prescribed by the 'cut mesh'. Instead of performing 
* intermediate conversions like volumetric/tetrahedral decompositions or level-sets, this function 
* operates directly on the halfedge representations of the two input meshes. 
* 
* Numerical operations are performed but only to evaluate polygon intersection points. The rest of 
* the function pipeline resolves the combinatorial structure of the underlying meshes using halfedge 
* connectivity. These numerical operations are represented exact arithemetic (if enabled) which makes the routine
* also robust to floating-point error.
*
 * An example of usage:
 * @code
 *  McResult err = mcDispatch(
 *       myContext,
*        // parse vertex arrays as 32 bit vertex coordinates (float*)
*        MC_DISPATCH_VERTEX_ARRAY_FLOAT,
*        // source mesh data
*        pSrcMeshVertices,
*        pSrcMeshFaceIndices,
*        pSrcMeshFaceSizes,
*        numSrcMeshVertices,
 *       numSrcMeshFaces,
*        // cut mesh data
*        pCutMeshVertices,
*        pCutMeshFaceIndices,
*        pCutMeshFaceSizes,
*        numCutMeshVertices,
*        numCutMeshFaces,
*        // synchronisation
*        0, NULL, NULL);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
* 
* @return Error code.
*
* <b>Error codes</b> 
* - ::MC_NO_ERROR  
*   -# proper exit 
* - ::MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p flags contains an invalid value.
*   -# A vertex index in \p pSrcMeshFaceIndices or \p pCutMeshFaceIndices is out of bounds.
*   -# Invalid face/polygon definition (vertex list) implying non-manifold mesh \p pSrcMeshFaceIndices or \p pCutMeshFaceIndices is out of bounds.
*   -# invalid character-string format in \p pSrcMeshVertices or \p pCutMeshVertices.
*   -# The MC_DISPATCH_VERTEX_ARRAY_... value has not been specified in \p flags
*   -# \p numEvents is zero or \p pEventList is NULL .
*   -# event objects specified in \p pEventList are not valid event objects.
* - ::MC_INVALID_SRC_MESH
*   -# mesh is not a single connected component
*   -# \p pSrcMeshVertices is NULL.
*   -# \p pSrcMeshFaceIndices is NULL.
*   -# \p pSrcMeshFaceSizes is NULL.
*   -# \p numSrcMeshVertices is less than three.
*   -# \p numSrcMeshFaces is less than one.
* - ::MC_INVALID_CUT_MESH
*   -# mesh is not a single connected component
*   -# \p pCutMeshVertices is NULL.
*   -# \p pCutMeshFaceIndices is NULL.
*   -# \p pCutMeshFaceSizes is NULL.
*   -# \p numCutMeshVertices is less than three.
*   -# \p numCutMeshFaces is less than one.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcDispatch(
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
    uint32_t numEventInWaitList,
    const McEvent* waitlist,
    McEvent* event);

/**
* @brief Return the value of a selected parameter.
*
* @param[in] context The context handle that was created by a previous call to ::mcCreateContext. 
* @param[in] info Information object being queried. ::McQueryFlags
* @param[in] bytes Size in bytes of memory pointed to by \p pMem. This size must be great than or equal to the return type size of data type queried.
* @param[out] pMem Pointer to memory where the appropriate result being queried is returned. If \p pMem is NULL, it is ignored.
* @param[out] pNumBytes returns the actual size in bytes of data being queried by info. If \p pNumBytes is NULL, it is ignored.
*
*
 * An example of usage:
 * @code
 * uint64_t numBytes = 0;
 * McFlags contextFlags;
 * McResult err =  mcGetInfo(context, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
*
 *   err = mcGetInfo(context, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p bytes is greater than the returned size of data type queried
*
* @note Event synchronisation is not implemented.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcGetInfo(
    const McContext context,
    McFlags info,
    uint64_t bytes,
    void* pMem,
    uint64_t* pNumBytes);

/**
* @brief Query the connected components available in a context.
* 
* This function will return an array of connected components matching the given description of flags.
* 
* If \p blocking is set to #MC_TRUE i.e. the command is blocking, ::mcGetConnectedComponents does not return until the data has been read and copied into memory pointed to.
* 
* If \p blocking is #MC_FALSE i.e. the command is non-blocking, ::mcGetConnectedComponents submits a non-blocking read command and returns. The contents of the destination ptr cannot be used until the read command has completed. The event argument returns an event object which can be used to query the execution status of the read command. When the read command has completed, the contents of the memory that the destination pointer points to can be used by the application.
*
* @param[in] context The context handle
* @param[in] blocking Indicate if the read and write operations are blocking or non-blocking.
* @param[in] connectedComponentType The type(s) of connected component sought. See also ::McConnectedComponentType.
* @param[in] numEntries The number of ::McConnectedComponent entries that can be added to \p pConnComps. If \p pConnComps is not NULL, \p numEntries must be the number of elements in \p pConnComps.
* @param[out] pConnComps Returns a list of connected components found. The ::McConnectedComponentType values returned in \p pConnComps can be used 
* to identify a specific connected component. If \p pConnComps is NULL, this argument is ignored. The number of connected components returned 
* is the minimum of the value specified by \p numEntries or the number of connected components whose type matches \p connectedComponentType.
* @param[out] numConnComps Returns the number of connected components available that match \p connectedComponentType. If \p numConnComps is NULL, 
* this argument is ignored.
* @param[in] numEventInWaitList Specify the number of events that need to complete before this particular command can be executed
* @param[in] waitlist Specify events that need to complete before this particular command can be executed
* @param[out] event Returns an event object that identifies this particular task execution instance.
*
 * An example of usage:
 * @code
 * uint32_t numConnComps = 0;
 * McConnectedComponent* pConnComps;
 * McResult err =  err = mcGetConnectedComponents(myContext, MC_TRUE, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numConnComps, 0, NULL, NULL);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 *
 * if (numConnComps == 0) {
 *    // ...
 * }
 *
 * pConnComps = (McConnectedComponent*)malloc(sizeof(McConnectedComponent) * numConnComps);
 *
 * err = mcGetConnectedComponents(myContext, MC_TRUE, MC_CONNECTED_COMPONENT_TYPE_ALL, numConnComps, pConnComps, NULL, 0, NULL, NULL);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p connectedComponentType is not a value in ::McConnectedComponentType.
*   -# \p numConnComps and \p pConnComps are both NULL.
*   -# \p numConnComps is zero and \p pConnComps is not NULL.
*
* @note Event synchronisation is not implemented.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcGetConnectedComponents(
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
* @brief Query specific information about a connected component.
*
* @param[in] context The context handle that was created by a previous call to ::mcCreateContext. 
* @param[in] blocking Indicate if the read and write operations are blocking or non-blocking
* @param[in] connCompId A connected component returned by ::mcGetConnectedComponents whose data is to be read.
* @param[in] flags An enumeration constant that identifies the connected component information being queried.
* @param[in] bytes Specifies the size in bytes of memory pointed to by \p flags.
* @param[out] pMem A pointer to memory location where appropriate values for a given \p flags will be returned. If \p pMem is NULL, it is ignored.
* @param[out] pNumBytes Returns the actual size in bytes of data being queried by \p flags. If \p pNumBytes is NULL, it is ignored.
* @param[in] numEventInWaitList Specify the number of events that need to complete before this particular command can be executed
* @param[in] waitlist Specify events that need to complete before this particular command can be executed.
* @param[out] event Returns an event object that identifies this particular task execution instance.
*
* The connected component queries described in the ::McConnectedComponentInfo should return the same information for a connected component returned by ::mcGetConnectedComponents.
*
 * An example of usage:
 * @code
 * uint64_t numBytes = 0;
 * McResult err = mcGetConnectedComponentData(myContext, MC_TRUE, connCompId, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, 0, NULL, &numBytes, 0, NULL, NULL);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * 
 * double* pVertices = (double*)malloc(numBytes);
 *
 * err = mcGetConnectedComponentData(context, MC_TRUE, connCompId, MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, numBytes, (void*)pVertices, NULL, 0, NULL, NULL);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
 * 
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p connectedComponentType is not a value in ::McConnectedComponentType.
*   -# \p pMem and \p pNumBytes are both NULL (or not NULL).
*   -# \p bytes is zero and \p pMem is not NULL.
*
* @note Event synchronisation is not yet implemented.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcGetConnectedComponentData(
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
* @brief To release the memory of a connected component, call this function.
*
* If \p numConnComps is zero and \p pConnComps is NULL, the memory of all connected components associated with the context is freed.
*
* @param[in] context The context handle that was created by a previous call to ::mcCreateContext.
* @param[in] numConnComps Number of connected components in \p pConnComps whose memory to release.
* @param[in] pConnComps The connected components whose memory will be released.
*
 * An example of usage:
 * @code
 * McResult err = mcReleaseConnectedComponents(myContext, pConnComps, numConnComps);
 * // OR (delete all connected components in context) 
 * //McResult err = mcReleaseConnectedComponents(myContext, NULL, 0);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*   -# \p numConnComps is zero and \p pConnComps is not NULL (and vice versa).
* 
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcReleaseConnectedComponents(
    const McContext context,
    uint32_t numConnComps,
    const McConnectedComponent* pConnComps);

/**
* @brief To release the memory of a context, call this function.
*
* This function ensures that all the state attached to context (such as unreleased connected components) are released, and the memory is deleted.
*
* @param[in] context The context handle that was created by a previous call to ::mcCreateContext. 
*
*
 * An example of usage:
 * @code
 * McResult err = mcReleaseContext(myContext);
 * if(err != MC_NO_ERROR)
 * {
 *  // deal with error
 * }
 * @endcode
*
* @return Error code.
*
* <b>Error codes</b> 
* - MC_NO_ERROR  
*   -# proper exit 
* - MC_INVALID_VALUE 
*   -# \p pContext is NULL or \p pContext is not an existing context.
*/
extern MCAPI_ATTR McResult MCAPI_CALL mcReleaseContext(
    McContext context);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // #ifndef MCUT_API_H_
