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
#include <string>

static void MCAPI_PTR mcDebugOutput(McDebugSource source,
									McDebugType type,
									McUint32 id,
									McDebugSeverity severity,
									size_t length,
									const char* message,
									const void* userParam)
{
	std::string debug_src;
	switch(source)
	{
	case MC_DEBUG_SOURCE_API:
		debug_src = "API";
		break;
	case MC_DEBUG_SOURCE_KERNEL:
		debug_src = "KERNEL";
		break;
	case MC_DEBUG_SOURCE_FRONTEND:
        debug_src = "FRONTEND";
    case MC_DEBUG_SOURCE_ALL:case MC_DEBUG_SOURCE_IGNORE:
        break;
	}
	std::string debug_type;
	switch(type)
	{
	case MC_DEBUG_TYPE_ERROR:
		debug_type = "ERROR";
		break;
	case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
		debug_type = "DEPRECATION";
		break;
	case MC_DEBUG_TYPE_OTHER:
		debug_type = "OTHER";
		break;
	case MC_DEBUG_TYPE_ALL:case MC_DEBUG_TYPE_IGNORE:
		break;
	}

	std::string severity_str;

	switch(severity)
	{
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

	printf("callback: [%d:%p,%s:%s:%s:%zu] %s\n",
		   id,
		   userParam,
		   debug_src.c_str(),
		   debug_type.c_str(),
		   severity_str.c_str(),
		   length,
		   message);
}

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

UTEST(CreateContext, withOneHelper)
{
    McContext context;
    McResult err = mcCreateContextWithHelpers(&context, MC_OUT_OF_ORDER_EXEC_MODE_ENABLE, 1);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(CreateContext, withTwoHelpers)
{
    McContext context;
    McResult err = mcCreateContextWithHelpers(&context, MC_OUT_OF_ORDER_EXEC_MODE_ENABLE, 2);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(CreateContext, withFourHelpers)
{
    McContext context;
    McResult err = mcCreateContextWithHelpers(&context, MC_NULL_HANDLE, 4);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);

    err = mcReleaseContext(context);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(CreateContext, withOneHundredHelpers)
{
    McContext context;
    McResult err = mcCreateContextWithHelpers(&context, MC_OUT_OF_ORDER_EXEC_MODE_ENABLE, 100); // capped to system thread count
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
    McSize bytes;
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
