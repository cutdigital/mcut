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
#include "off.h"

UTEST(CreateContext, noFlags)
{
    McContext context;
    McResult err = mcCreateContext(&context, 0);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(CreateContext, debugFlag)
{
    McContext context;
    McResult err = mcCreateContext(&context, MC_DEBUG);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(CreateContext, outOfOrderExec)
{
    McContext context;
    McResult err = mcCreateContext(&context, MC_OUT_OF_ORDER_EXEC_MODE_ENABLE);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(CreateContext, outOfOrderExec_debug)
{
    McContext context;
    McResult err = mcCreateContext(&context, MC_OUT_OF_ORDER_EXEC_MODE_ENABLE | MC_DEBUG);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
struct DebugContextConfig {
    McContext context_;
};

UTEST_F_SETUP(DebugContextConfig)
{
    McResult err = mcCreateContext(&utest_fixture->context_, MC_DEBUG);
    EXPECT_TRUE(utest_fixture->context_ != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F_TEARDOWN(DebugContextConfig)
{
    McResult err = mcReleaseContext(utest_fixture->context_);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST_F(DebugContextConfig, queryContextDebugFlag)
{
    uint64_t bytes;
    McFlags flags;
    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, 0, nullptr, &bytes), MC_NO_ERROR);
    EXPECT_EQ(bytes, sizeof(McFlags));

    EXPECT_EQ(mcGetInfo(utest_fixture->context_, MC_CONTEXT_FLAGS, sizeof(McFlags), &flags, nullptr), MC_NO_ERROR);

    EXPECT_EQ(flags & MC_DEBUG, MC_DEBUG);
}

UTEST_F(DebugContextConfig, setDebugCallback)
{
    EXPECT_EQ(mcDebugMessageCallback(utest_fixture->context_, mcDebugOutput, nullptr), MC_NO_ERROR);
}

UTEST_F(DebugContextConfig, setDebugCallbackMessageControl)
{
    EXPECT_EQ(mcDebugMessageControl(utest_fixture->context_, McDebugSource::MC_DEBUG_SOURCE_ALL, McDebugType::MC_DEBUG_TYPE_ALL, McDebugSeverity::MC_DEBUG_SEVERITY_ALL, true), MC_NO_ERROR);
}
