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

struct GetContextInfo {
    McContext context_;
    uint64_t bytes;
};

UTEST_F_SETUP(GetContextInfo)
{
    McResult err = mcCreateContext(&utest_fixture->context_, MC_DEBUG);
    ASSERT_TRUE(utest_fixture->context_ != nullptr);
    ASSERT_EQ(err, MC_NO_ERROR);
    utest_fixture->bytes = 0;
}

UTEST_F_TEARDOWN(GetContextInfo)
{
    McResult err = mcReleaseContext(utest_fixture->context_);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F(GetContextInfo, defaultPrecision)
{
    uint64_t defaultPrec = 0;
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_DEFAULT_PRECISION, 0, nullptr, &utest_fixture->bytes), MC_NO_ERROR);
    EXPECT_EQ(utest_fixture->bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_DEFAULT_PRECISION, utest_fixture->bytes, &defaultPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(defaultPrec, 0u);
}

UTEST_F(GetContextInfo, minimumPrecision)
{
    uint64_t minPrec = 0;
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MIN, 0, nullptr, &utest_fixture->bytes), MC_NO_ERROR);
    EXPECT_EQ(utest_fixture->bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MIN, utest_fixture->bytes, &minPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(minPrec, 0u);
}

UTEST_F(GetContextInfo, maximumPrecision)
{
    uint64_t maxPrec = 0;
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MAX, 0, nullptr, &utest_fixture->bytes), MC_NO_ERROR);
    EXPECT_EQ(utest_fixture->bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MAX, utest_fixture->bytes, &maxPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(maxPrec, 0u);
}

UTEST_F(GetContextInfo, roundingMode)
{
    McFlags defaultRoundingMode = 0;
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_DEFAULT_ROUNDING_MODE, 0, nullptr, &utest_fixture->bytes), MC_NO_ERROR);
    EXPECT_EQ(utest_fixture->bytes, sizeof(McFlags));
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_DEFAULT_ROUNDING_MODE, utest_fixture->bytes, &defaultRoundingMode, nullptr), MC_NO_ERROR);
    ASSERT_TRUE(
        defaultRoundingMode == MC_ROUNDING_MODE_TO_NEAREST || //
        defaultRoundingMode == MC_ROUNDING_MODE_TOWARD_ZERO || //
        defaultRoundingMode == MC_ROUNDING_MODE_TOWARD_POS_INF || //
        defaultRoundingMode == MC_ROUNDING_MODE_TOWARD_NEG_INF);
}

