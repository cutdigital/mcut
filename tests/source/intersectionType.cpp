/**
 * Copyright (c) 2021-2023 Floyd M. Chitalu.
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

 /*
 This test creates multiple contexts that run in parallel.

 There are as many contexts as half the number of hardware threads.

 Each context is run in its own thread.
 */

#include "utest.h"

#include <mcut/mcut.h>
#include <vector>

struct Mesh {
    std::vector<McDouble> vertices;
    std::vector<McUint32>  faceIndices;
    std::vector<McUint32> faceSizes;
    McUint32 numVertices;
    McUint32 numFaces;
};

struct IntersectionType {
    McContext context;

    Mesh srcMesh;
    Mesh cutMesh;    
};

UTEST_F_SETUP(IntersectionType)
{
    utest_fixture->context  = MC_NULL_HANDLE;
    ASSERT_EQ(mcCreateContextWithHelpers(&utest_fixture->context , MC_OUT_OF_ORDER_EXEC_MODE_ENABLE, 0), MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->context  != nullptr);
}

UTEST_F_TEARDOWN(IntersectionType)
{
    EXPECT_EQ(mcReleaseContext(utest_fixture->context), MC_NO_ERROR);
}

UTEST_F(IntersectionType, defaultValue)
{
    McSize bytes = 0;

    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, sizeof(McDispatchIntersectionType));

    McDispatchIntersectionType value;
    ASSERT_EQ(mcGetInfo(utest_fixture->context, MC_CONTEXT_DISPATCH_INTERSECTION_TYPE, bytes, &value, 0), MC_NO_ERROR);

    // we have not yet called the dispatch function to the value should be undefined i.e. MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM
    ASSERT_EQ(value, McDispatchIntersectionType::MC_DISPATCH_INTERSECTION_TYPE_MAX_ENUM); 
}