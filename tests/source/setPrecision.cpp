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
#include <algorithm>

struct SetPrecision {
    McContext context_;
    uint64_t bytes;
    uint64_t minPrec;
    uint64_t maxPrec;
};

UTEST_F_SETUP(SetPrecision)
{
    McResult err = mcCreateContext(&utest_fixture->context_, MC_DEBUG);
    ASSERT_TRUE(utest_fixture->context_ != nullptr);
    ASSERT_EQ(err, MC_NO_ERROR);
    utest_fixture->bytes = 0;
    utest_fixture->minPrec = 0;
    utest_fixture->maxPrec = 0;

    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MIN, 0, nullptr, &utest_fixture->bytes ), MC_NO_ERROR);
    EXPECT_EQ(utest_fixture->bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MIN, utest_fixture->bytes , &utest_fixture->minPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(utest_fixture->minPrec, 0u);

    utest_fixture->bytes = 0;
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MAX, 0, nullptr, &utest_fixture->bytes ), MC_NO_ERROR);
    EXPECT_EQ(utest_fixture->bytes, sizeof(uint64_t));
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_PRECISION_MAX, utest_fixture->bytes , &utest_fixture->maxPrec, nullptr), MC_NO_ERROR);
    ASSERT_GT(utest_fixture->maxPrec, 0u);
}

UTEST_F_TEARDOWN(SetPrecision)
{
    McResult err = mcReleaseContext(utest_fixture->context_);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F(SetPrecision, minValue)
{
    uint64_t precValSet =  ((uint64_t)64ul >= utest_fixture->minPrec) ? (uint64_t)64ul : utest_fixture->minPrec; // bits
    EXPECT_EQ(mcSetPrecision(utest_fixture->context_, precValSet), MC_NO_ERROR);

    uint64_t prec = 0;
    EXPECT_EQ(mcGetPrecision(utest_fixture->context_, &prec), MC_NO_ERROR);

    ASSERT_EQ(prec, precValSet);
}

UTEST_F(SetPrecision, maxValue)
{
    uint64_t precValSet = utest_fixture->maxPrec; // bits
    EXPECT_EQ(mcSetPrecision(utest_fixture->context_, precValSet), MC_NO_ERROR);

    uint64_t prec = 0;
    EXPECT_EQ(mcGetPrecision(utest_fixture->context_, &prec), MC_NO_ERROR);

    ASSERT_EQ(prec, precValSet);
}

UTEST_F(SetPrecision, midValue)
{
    uint64_t precValSet = (utest_fixture->minPrec + utest_fixture->maxPrec)/2; // bits
    EXPECT_EQ(mcSetPrecision(utest_fixture->context_, precValSet), MC_NO_ERROR);

    uint64_t prec = 0;
    EXPECT_EQ(mcGetPrecision(utest_fixture->context_, &prec), MC_NO_ERROR);

    ASSERT_EQ(prec, precValSet);
}
