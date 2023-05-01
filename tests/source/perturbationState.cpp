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

#include "utest.h"
#include <mcut/mcut.h>

#include <vector>
#include <cmath>

struct PerturbationState {
    McContext myContext = MC_NULL_HANDLE;
    std::vector<McConnectedComponent> pConnComps_;
    std::vector<float> srcMeshVertices;
    std::vector<uint32_t> meshFaceIndices;
    std::vector<uint32_t> meshFaceSizes;
};

UTEST_F_SETUP(PerturbationState)
{
    // create with no flags (default)
    EXPECT_EQ(mcCreateContext(&utest_fixture->myContext, 0), MC_NO_ERROR);
    EXPECT_TRUE(utest_fixture->myContext != nullptr);

    utest_fixture->srcMeshVertices = {
        -1.f, -1.f, 1.f, // 0
        1.f, -1.f, 1.f, // 1
        1.f, -1.f, -1.f, // 2
        -1.f, -1.f, -1.f, // 3
        -1.f, 1.f, 1.f, // 4
        1.f, 1.f, 1.f, // 5
        1.f, 1.f, -1.f, // 6
        -1.f, 1.f, -1.f // 7
    };

    utest_fixture->meshFaceIndices = {
        3, 2, 1, 0, // bottom
        4, 5, 6, 7, // top
        0, 1, 5, 4, // front
        1, 2, 6, 5, // right
        2, 3, 7, 6, // back
        3, 0, 4, 7 // left
    };

    utest_fixture->meshFaceSizes = { 4, 4, 4, 4, 4, 4 };
}

UTEST_F_TEARDOWN(PerturbationState)
{
    EXPECT_EQ(mcReleaseConnectedComponents(utest_fixture->myContext, (uint32_t)utest_fixture->pConnComps_.size(), utest_fixture->pConnComps_.data()), MC_NO_ERROR);
    EXPECT_EQ(mcReleaseContext(utest_fixture->myContext), MC_NO_ERROR);
}

// Performing a Boolean "union" operation with the same object, while allowing general
// position enforcement (with MC_DISPATCH_ENFORCE_GENERAL_POSITION).
UTEST_F(PerturbationState, getCCPerturbationVector)
{
    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags | MC_DISPATCH_ENFORCE_GENERAL_POSITION,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);

    McUint32 numCCs = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numCCs), MC_NO_ERROR);

    std::vector<McConnectedComponent> cc_vec(numCCs);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)cc_vec.size(), &cc_vec[0], NULL), MC_NO_ERROR);

    for(McUint32 i =0; i < (McUint32)cc_vec.size(); ++i)
    {
        McConnectedComponent cc = cc_vec[i];

        McSize bytes;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_DISPATCH_PERTURBATION_VECTOR, 0, nullptr, &bytes),MC_NO_ERROR);
        ASSERT_GE(bytes, sizeof(McDouble) * 3);

        std::vector<McDouble> perturbation_vector(3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_DISPATCH_PERTURBATION_VECTOR, bytes, &perturbation_vector[0], NULL),MC_NO_ERROR);

        McDouble sqrd_magnitude =0.0;

        for(McUint32 i =0; i < (McUint32)3; ++i)
        {
            McDouble component = perturbation_vector[i];
            sqrd_magnitude += component*component;
        }

        ASSERT_TRUE(sqrd_magnitude > 0);
    }
}

// using MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE
UTEST_F(PerturbationState, setPerturbationConstant)
{
    McDouble eps = 1.5e-5;
    ASSERT_EQ(mcBindState(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_CONSTANT, sizeof(McDouble), (void*)&eps), MC_NO_ERROR);

    const std::vector<float>& srcMeshVertices = utest_fixture->srcMeshVertices;
    const std::vector<uint32_t>& meshFaceIndices = utest_fixture->meshFaceIndices;
    const std::vector<uint32_t>& meshFaceSizes = utest_fixture->meshFaceSizes;

    const McFlags booleanUnionFlags = MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE;

    ASSERT_EQ(mcDispatch(utest_fixture->myContext, //
                  MC_DISPATCH_VERTEX_ARRAY_FLOAT | booleanUnionFlags | MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE,
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size(), //
                  &srcMeshVertices[0], &meshFaceIndices[0], &meshFaceSizes[0], (uint32_t)(srcMeshVertices.size() / 3), (uint32_t)meshFaceSizes.size()),
        MC_NO_ERROR);

    McUint32 numCCs = 0;
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &numCCs), MC_NO_ERROR);

    std::vector<McConnectedComponent> cc_vec(numCCs);
    ASSERT_EQ(mcGetConnectedComponents(utest_fixture->myContext, McConnectedComponentType::MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)cc_vec.size(), &cc_vec[0], NULL), MC_NO_ERROR);

    for(McUint32 i =0; i < (McUint32)cc_vec.size(); ++i)
    {
        McConnectedComponent cc = cc_vec[i];

        McSize bytes;
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_DISPATCH_PERTURBATION_VECTOR, 0, nullptr, &bytes),MC_NO_ERROR);
        ASSERT_GE(bytes, sizeof(McDouble) * 3);

        std::vector<McDouble> perturbation_vector(3);
        ASSERT_EQ(mcGetConnectedComponentData(utest_fixture->myContext, cc, MC_CONNECTED_COMPONENT_DATA_DISPATCH_PERTURBATION_VECTOR, bytes, &perturbation_vector[0], NULL),MC_NO_ERROR);

        for(McUint32 i =0; i < (McUint32)3; ++i)
        {
            McDouble component = perturbation_vector[i];
            ASSERT_LE(std::fabs(component), eps); // this works because we are using "MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE"
        }
    }

    McSize bytes = 0;
    ASSERT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_CONSTANT, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, (McSize)sizeof(McDouble));

    McDouble queried_eps = 0; 
    ASSERT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_CONSTANT, sizeof(McDouble), &queried_eps, NULL), MC_NO_ERROR);
    
    ASSERT_EQ(queried_eps, eps); // must be the same as the value we set above
}

// using MC_DISPATCH_ENFORCE_GENERAL_POSITION_ABSOLUTE
UTEST_F(PerturbationState, setPerturbationAttempts)
{
    McUint32 attempts = 1<<3;
    ASSERT_EQ(mcBindState(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_ATTEMPTS, sizeof(McUint32), (void*)&attempts), MC_NO_ERROR);

    McSize bytes = 0;
    ASSERT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_ATTEMPTS, 0, NULL, &bytes), MC_NO_ERROR);

    ASSERT_EQ(bytes, (McSize)sizeof(McUint32));

    McUint32 queried_attempts = 0; 
    ASSERT_EQ(mcGetInfo(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_ATTEMPTS, sizeof(McUint32), &queried_attempts, NULL), MC_NO_ERROR);
    
    ASSERT_EQ(queried_attempts, attempts); // must be the same as the value we set above

    // cannot be zero!
    attempts = 0;
    ASSERT_EQ(mcBindState(utest_fixture->myContext, MC_CONTEXT_GENERAL_POSITION_ENFORCEMENT_ATTEMPTS, sizeof(McUint32), (void*)&attempts), MC_INVALID_VALUE);

}