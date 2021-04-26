#include "utest.h"
#include <mcut/mcut.h>

UTEST(CreateContext, noFlags)
{
    McContext context;
    McResult err = mcCreateContext(&context, 0);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

UTEST(ContextCreationTest, debugFlag)
{
    McContext context;
    McResult err = mcCreateContext(&context, MC_DEBUG);
    EXPECT_TRUE(context != nullptr);
    EXPECT_EQ(err, MC_NO_ERROR);
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
struct DebugContextConfig {
    McContext context_;
};

MCAPI_ATTR void mcDebugOutput(McDebugSource source,
    McDebugType type,
    unsigned int id,
    McDebugSeverity severity,
    size_t length,
    const char* message,
    const void* userParam)
{
    // blah...
    source;
    type;
    id;
    severity;
    length;
    message;
    userParam;
}

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
