/**
 * Copyright (c) 2023 Floyd M. Chitalu.
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

#include "utest.h"
#include <mcut/mcut.h>
#include <vector>
#include <string>

#if 0
static void MCAPI_PTR mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
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
#endif

struct DebugLog {
    McContext context_;
    // 1. Create meshes.
    // -----------------
    // Shape to Cut:
    std::vector<double> cubeVertices;
    std::vector<uint32_t> cubeFaces;
    int numCubeVertices;
    int numCubeFaces;

    std::vector<uint32_t> cubeFaceSizes;

    // Cutting Shape:

    std::vector<double> cutMeshVertices;

    std::vector<uint32_t> cutMeshFaces;
    uint32_t numCutMeshVertices;
    uint32_t numCutMeshFaces;
};

UTEST_F_SETUP(DebugLog)
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

UTEST_F_TEARDOWN(DebugLog)
{
    McResult err = mcReleaseContext(utest_fixture->context_);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F(DebugLog, MessageControl_EnableAll)
{
    // config debug output
    // -----------------------
    McSize numBytes = 0;
    McFlags contextFlags;
    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes));

    ASSERT_EQ(sizeof(McFlags), numBytes);

    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr));

    if (contextFlags & MC_DEBUG) {
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

    McUint32 numMsgs = 10; // retrieve up to 10

    McSize maxMsgLen = 0;
    ASSERT_EQ(MC_NO_ERROR, mcGetInfo(utest_fixture->context_, MC_CONTEXT_MAX_DEBUG_MESSAGE_LENGTH, sizeof(McSize), &maxMsgLen, NULL));

    std::vector<McChar> msgData(numMsgs * maxMsgLen);
    std::vector<McDebugSource> sources(numMsgs);
    std::vector<McDebugType> types(numMsgs);
    std::vector<McDebugSeverity> severities(numMsgs);
    std::vector<McSize> lengths(numMsgs);
    McUint32 numFound;

    ASSERT_EQ(MC_NO_ERROR, mcGetDebugMessageLog(utest_fixture->context_, numMsgs, msgData.size(), &sources[0], &types[0], &severities[0], &lengths[0], &msgData[0], &numFound));

    ASSERT_GT(numFound, 0);

    sources.resize(numFound);
    types.resize(numFound);
    severities.resize(numFound);
    lengths.resize(numFound);
    std::vector<std::string> messages;
    messages.reserve(numFound);

    std::vector<McChar>::iterator currPos = msgData.begin();

    for (size_t msg = 0; msg < lengths.size(); ++msg) {
        ASSERT_GT(lengths[msg], 0);
        messages.push_back(std::string(currPos, currPos + lengths[msg]));
        currPos = currPos + lengths[msg];

        printf("message from log string: %s\n", messages.back().c_str());
    }


}
