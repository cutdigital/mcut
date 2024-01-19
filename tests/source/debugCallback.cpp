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

#include "utest.h"
#include <mcut/mcut.h>
#include <vector>
#include <string>

void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    McUint32 id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{

    // printf("Debug message ( %d ), length=%zu\n%s\n--\n", id, length, message);
    // printf("userParam=%p\n", userParam);

    std::string debug_src;
    switch (source) {
    case MC_DEBUG_SOURCE_API:
        debug_src = "API";
        break;
    case MC_DEBUG_SOURCE_KERNEL:
        debug_src = "KERNEL";
        break;
    case MC_DEBUG_SOURCE_ALL:
        break;
    }
    std::string debug_type;
    switch (type) {
    case MC_DEBUG_TYPE_ERROR:
        debug_type = "ERROR";
        break;
    case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        debug_type = "DEPRECATION";
        break;
    case MC_DEBUG_TYPE_OTHER:
        // printf("Type: Other");
        debug_type = "OTHER";
        break;
    case MC_DEBUG_TYPE_ALL:
        break;
    }

    std::string severity_str;

    switch (severity) {
    case MC_DEBUG_SEVERITY_HIGH:
        severity_str = "HIGH";
        break;
    case MC_DEBUG_SEVERITY_MEDIUM:
        severity_str = "MEDIUM";
        break;
    case MC_DEBUG_SEVERITY_LOW:
        severity_str = "LOW";
        break;
    case MC_DEBUG_SEVERITY_NOTIFICATION:
        severity_str = "NOTIFICATION";
        break;
    case MC_DEBUG_SEVERITY_ALL:
        break;
    }

    printf("callback: [%d:%p,%s:%s:%s:%zu] %s\n", id, userParam, debug_src.c_str(), debug_type.c_str(), severity_str.c_str(), length, message);
}

struct DebugCallback {
    McContext context_ = MC_NULL_HANDLE;
    // 1. Create meshes.
    // -----------------
    // Shape to Cut:
	std::vector<McDouble> cubeVertices = {};
	std::vector<McUint32> cubeFaces = {};
    McInt32 numCubeVertices=0;
    McInt32 numCubeFaces=0;

    std::vector<McUint32> cubeFaceSizes = {};

    // Cutting Shape:

    std::vector<McDouble> cutMeshVertices = {};

    std::vector<McUint32> cutMeshFaces = {};
    McUint32 numCutMeshVertices=0;
    McUint32 numCutMeshFaces=0;
};

UTEST_F_SETUP(DebugCallback)
{
    McResult err = mcCreateContext(&utest_fixture->context_, MC_DEBUG);
    EXPECT_TRUE(utest_fixture->context_ != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    // 1. Create meshes.
    // -----------------
    // Shape to Cut:
    utest_fixture->cubeVertices = {
        -1, -1, 1, // 0
        1, -1, 1, // 1
        -1, 1, 1, // 2
        1, 1, 1, // 3
        -1, -1, -1, // 4
        1, -1, -1, // 5
        -1, 1, -1, // 6
        1, 1, -1 // 7
    };
    utest_fixture->cubeFaces = {
        0, 3, 2, // 0
        0, 1, 3, // 1
        1, 7, 3, // 2
        1, 5, 7, // 3
        5, 6, 7, // 4
        5, 4, 6, // 5
        4, 2, 6, // 6
        4, 0, 2, // 7
        2, 7, 6, // 8
        2, 3, 7, // 9
        4, 1, 0, // 10
        4, 5, 1, // 11

    };
    utest_fixture->numCubeVertices = 8;
    utest_fixture->numCubeFaces = 12;

    utest_fixture->cubeFaceSizes = {
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
    };

    // Cutting Shape:

    utest_fixture->cutMeshVertices = {
        -1.2, 1.6, 0.994070,
        1.4, -1.3, 0.994070,
        -1.2, 1.6, -1.005929,
        1.4, -1.3, -1.005929
    };

    utest_fixture->cutMeshFaces = {
        1, 2, 0,
        1, 3, 2
    };

    utest_fixture->numCutMeshVertices = 4;
    utest_fixture->numCutMeshFaces = 2;
}

UTEST_F_TEARDOWN(DebugCallback)
{
    McResult err = mcReleaseContext(utest_fixture->context_);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F(DebugCallback, MessageControl_EnableAll)
{
    // config debug output
    // -----------------------
    McSize numBytes = 0;
	McFlags contextFlags = 0;
    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes));

    ASSERT_EQ(sizeof(McFlags), numBytes);

    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr));

    if (contextFlags & MC_DEBUG) {
        mcDebugMessageCallback(utest_fixture->context_, mcDebugOutput, nullptr);
        // enable all
        mcDebugMessageControl(utest_fixture->context_, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true);
    }

    ASSERT_EQ(MC_NO_ERROR,
        mcDispatch(utest_fixture->context_,
            MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
            utest_fixture->cubeVertices.data(), utest_fixture->cubeFaces.data(), utest_fixture->cubeFaceSizes.data(), utest_fixture->numCubeVertices, utest_fixture->numCubeFaces,
            utest_fixture->cutMeshVertices.data(), utest_fixture->cutMeshFaces.data(),
            nullptr, // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
            utest_fixture->numCutMeshVertices, utest_fixture->numCutMeshFaces));
}

UTEST_F(DebugCallback, MessageControl_EnableOnlySevereErrors)
{
    // config debug output
    // -----------------------
    McSize numBytes = 0;
    McFlags contextFlags =0;
    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes));

    ASSERT_EQ(sizeof(McFlags), numBytes);

    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr));

    if (contextFlags & MC_DEBUG) {
        mcDebugMessageCallback(utest_fixture->context_, mcDebugOutput, nullptr);
        // enable all
        mcDebugMessageControl(utest_fixture->context_, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_HIGH, true);
    }

    ASSERT_EQ(MC_NO_ERROR,
        mcDispatch(utest_fixture->context_,
            MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
            utest_fixture->cubeVertices.data(), utest_fixture->cubeFaces.data(), utest_fixture->cubeFaceSizes.data(), utest_fixture->numCubeVertices, utest_fixture->numCubeFaces,
            utest_fixture->cutMeshVertices.data(), utest_fixture->cutMeshFaces.data(),
            nullptr, // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
            utest_fixture->numCutMeshVertices, utest_fixture->numCutMeshFaces));
}
