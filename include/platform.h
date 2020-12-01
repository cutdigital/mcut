/*******************************************************************************
 * Copyright (c) 2020- CutDigital Ltd.
 *
 * File is licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

/**
 * @file platform.h
 * @author Floyd M. Chitalu
 * @date 4 Sep 2020
 * @brief File containing platform-specific types and definitions.
 *
 * This header file defines platform specific directives and integral type 
 * declarations.
 * Platforms should define these so that MCUT users call MCUT commands
 * with the same calling conventions that the MCUT implementation expects.
 *
 * MCAPI_ATTR - Placed before the return type in function declarations.
 *              Useful for C++11 and GCC/Clang-style function attribute syntax.
 * MCAPI_CALL - Placed after the return type in function declarations.
 *              Useful for MSVC-style calling convention syntax.
 * MCAPI_PTR  - Placed between the '(' and '*' in function pointer types.
 *
 * Function declaration:  MCAPI_ATTR void MCAPI_CALL mcFunction(void);
 * Function pointer type: typedef void (MCAPI_PTR *PFN_mcFunction)(void);
 * 
 */

#ifndef MC_PLATFORM_H_
#define MC_PLATFORM_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#if defined(_WIN32)
    // On Windows, mcut commands use the stdcall convention
    #define MCAPI_ATTR
    #define MCAPI_CALL __stdcall
    #define MCAPI_PTR  MCAPI_CALL
#else
    // On other platforms, use the default calling convention
    #define MCAPI_ATTR
    #define MCAPI_CALL
    #define MCAPI_PTR
#endif

#include <stddef.h> // standard type definitions

#if !defined(MC_NO_STDINT_H)
    #if defined(_MSC_VER) && (_MSC_VER < 1600)
        typedef signed   __int8  int8_t;
        typedef unsigned __int8  uint8_t;
        typedef signed   __int16 int16_t;
        typedef unsigned __int16 uint16_t;
        typedef signed   __int32 int32_t;
        typedef unsigned __int32 uint32_t;
        typedef signed   __int64 int64_t;
        typedef unsigned __int64 uint64_t;
    #else
        #include <stdint.h> //  integer types
    #endif
#endif // !defined(MC_NO_STDINT_H)

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif