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

#ifndef OFF_FILE_H_
#define OFF_FILE_H_

#include <mcut/mcut.h>
#include <vector>
#include <string>

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS 1
#endif

extern "C" void readOFF(
    const char* fpath,
    float** pVertices,
    unsigned int** pFaceIndices,
    unsigned int** pFaceSizes,
    unsigned int* numVertices,
    unsigned int* numFaces);

extern "C" void writeOFF(
    const char* fpath,
    float* pVertices,
    unsigned int* pFaceIndices,
    unsigned int* pFaceSizes,
    unsigned int* pEdgeIndices,
    unsigned int numVertices,
    unsigned int numFaces,
    unsigned int numEdges);

void writeOBJ(const std::string& fname, const std::vector<float>& vertices, const std::vector<uint32_t>& triangles);

extern "C" void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam);

#endif
