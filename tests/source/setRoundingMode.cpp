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

#define NUM_ROUNDING_MODES 4

McRoundingModeFlags modes[] = {
    MC_ROUNDING_MODE_TO_NEAREST,
    MC_ROUNDING_MODE_TOWARD_ZERO,
    MC_ROUNDING_MODE_TOWARD_POS_INF,
    MC_ROUNDING_MODE_TOWARD_NEG_INF
};

struct SetRoundingMode {
    McContext context_;
    McRoundingModeFlags mode;
};

UTEST_I_SETUP(SetRoundingMode)
{
    if (utest_index < NUM_ROUNDING_MODES) {
        McResult err = mcCreateContext(&utest_fixture->context_, MC_DEBUG);
        ASSERT_TRUE(utest_fixture->context_ != nullptr);
        ASSERT_EQ(err, MC_NO_ERROR);
        utest_fixture->mode = modes[utest_index];
    }
}

UTEST_I_TEARDOWN(SetRoundingMode)
{
    if (utest_index < NUM_ROUNDING_MODES) {
        McResult err = mcReleaseContext(utest_fixture->context_);
        EXPECT_EQ(err, MC_NO_ERROR);
    }
}

UTEST_I(SetRoundingMode, setRoundingMode, NUM_ROUNDING_MODES)
{
    uint32_t roundingModeValSet = MC_ROUNDING_MODE_TO_NEAREST;
    EXPECT_EQ(mcSetRoundingMode(utest_fixture->context_, roundingModeValSet), MC_NO_ERROR);

    uint32_t roundingMode = 0;
    EXPECT_EQ(mcGetRoundingMode(utest_fixture->context_, &roundingMode), MC_NO_ERROR);

    ASSERT_EQ(roundingModeValSet, roundingMode);
}
