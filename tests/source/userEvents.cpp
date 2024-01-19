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

/*
This test creates multiple contexts that run in parallel.

There are as many contexts as half the number of hardware threads.

Each context is run in its own thread.
*/

#include "utest.h"
#include <chrono>
#include <future>
#include <mcut/mcut.h>
#include <thread>
#include <vector>
struct UserEvents {
    McContext context = MC_NULL_HANDLE;
	McEvent userEvent = MC_NULL_HANDLE;
    //
    // Same inputs as Hello World tutorial
    //

    std::vector<McDouble> cubeVertices = {};
	std::vector<McUint32> cubeFaces = {};
	McInt32 numCubeVertices = 0;
	McInt32 numCubeFaces = 0;

    std::vector<McUint32> cubeFaceSizes = {};

    // Cutting Shape:

    std::vector<McDouble> cutMeshVertices = {};

    std::vector<McUint32> cutMeshFaces = {};

    McUint32 numCutMeshVertices=0;
	McUint32 numCutMeshFaces = 0;
};

UTEST_F_SETUP(UserEvents)
{
    ASSERT_EQ(mcCreateContext(&utest_fixture->context, 0), MC_NO_ERROR);

    utest_fixture->userEvent = MC_NULL_HANDLE;

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

UTEST_F_TEARDOWN(UserEvents)
{
    if (utest_fixture->userEvent != MC_NULL_HANDLE) {
        ASSERT_EQ(mcReleaseEvents(1, &utest_fixture->userEvent), McResult::MC_NO_ERROR);
    }
    ASSERT_EQ(mcReleaseContext(utest_fixture->context), McResult::MC_NO_ERROR);
}

UTEST_F(UserEvents, createAndDestroy)
{
    ASSERT_EQ(mcCreateUserEvent(&utest_fixture->userEvent, utest_fixture->context), McResult::MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->userEvent != MC_NULL_HANDLE);
}

UTEST_F(UserEvents, getExecStatus)
{
    ASSERT_EQ(mcCreateUserEvent(&utest_fixture->userEvent, utest_fixture->context), McResult::MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->userEvent != MC_NULL_HANDLE);

    McSize bytes = 0;
    ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, 0, NULL, &bytes), McResult::MC_NO_ERROR);

    McEventCommandExecStatus status = (McEventCommandExecStatus)0;
    ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &status, NULL), McResult::MC_NO_ERROR);

    ASSERT_TRUE(status& McEventCommandExecStatus::MC_SUBMITTED); // default value
}

UTEST_F(UserEvents, getContext)
{
    ASSERT_EQ(mcCreateUserEvent(&utest_fixture->userEvent, utest_fixture->context), McResult::MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->userEvent != MC_NULL_HANDLE);

    McSize bytes = 0;
    ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_CONTEXT, 0, NULL, &bytes), McResult::MC_NO_ERROR);

    McContext c = MC_NULL_HANDLE;
    ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_CONTEXT, bytes, &c, NULL), McResult::MC_NO_ERROR);

    ASSERT_EQ(c, utest_fixture->context);
}

void myCallback(McEvent event, void* data)
{
    (void)event;
    McInt32* var = (McInt32*)data;
    *var = 42;
}

UTEST_F(UserEvents, setCallbackAndInvokeOnRelease)
{
    ASSERT_EQ(mcCreateUserEvent(&utest_fixture->userEvent, utest_fixture->context), McResult::MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->userEvent != MC_NULL_HANDLE);

    McInt32 myInt(0);
    ASSERT_EQ(mcSetEventCallback(utest_fixture->userEvent, myCallback, (void*)&myInt), McResult::MC_NO_ERROR);
    ;

    //
    // if a set callback is not involved due to task completion, then it will be invoked on event-destruction
    // if the event has no associated runtime error
    ASSERT_EQ(mcReleaseEvents(1, &utest_fixture->userEvent), McResult::MC_NO_ERROR);

    utest_fixture->userEvent = MC_NULL_HANDLE;

    ASSERT_EQ(myInt, 42); // value that was set in the callback
}

UTEST_F(UserEvents, dependOn)
{
    ASSERT_EQ(mcCreateUserEvent(&utest_fixture->userEvent, utest_fixture->context), McResult::MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->userEvent != MC_NULL_HANDLE);

    auto someUserEventFunction = [](McEvent userEvent_) {
        // do stuff that uses use some 100ms
        for (McInt32 i = 0; i < 20; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // notify that the event has finished
        const McResult res = mcSetUserEventStatus(userEvent_, MC_COMPLETE);

        if (res != McResult::MC_NO_ERROR) {
            throw std::runtime_error("failed to set user-event status to MC_COMPLETE");
        }
    };

    McEvent dispatchEvent = MC_NULL_HANDLE;
    ASSERT_EQ(mcEnqueueDispatch(
                  utest_fixture->context,
                  MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
                  utest_fixture->cubeVertices.data(),
                  utest_fixture->cubeFaces.data(),
                  utest_fixture->cubeFaceSizes.data(),
                  utest_fixture->numCubeVertices,
                  utest_fixture->numCubeFaces,
                  utest_fixture->cutMeshVertices.data(),
                  utest_fixture->cutMeshFaces.data(),
                  nullptr, // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
                  utest_fixture->numCutMeshVertices,
                  utest_fixture->numCutMeshFaces,
                  1, &utest_fixture->userEvent, &dispatchEvent),
        MC_NO_ERROR);

    ASSERT_TRUE(dispatchEvent != MC_NULL_HANDLE);

    // start our client side task which is associated with user event
    // "someUserEventFunction" with the status of "userEvent" to complete,
    // which then allows mcEnqueueDispatch to start since it waits on "userEvent"
    auto fut = std::async(std::launch::async, someUserEventFunction, utest_fixture->userEvent);

    // NOTE: mcWaitForEvents must be called AFTER lauching/calling "someUserEventFunction".
    // Otherwise, we get deadlock since mcEnqueueDispatch is dependent on "someUserEventFunction"
    // which updates the status of our user event
    ASSERT_EQ(mcWaitForEvents(1, &dispatchEvent), MC_NO_ERROR);

    ASSERT_EQ(mcReleaseEvents(1, &dispatchEvent), McResult::MC_NO_ERROR);

    try {
        fut.get(); // may throw exceptions due to something going wrong in someUserEventFunction

        McSize bytes = 0;
        ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, 0, NULL, &bytes), McResult::MC_NO_ERROR);

        McEventCommandExecStatus status = (McEventCommandExecStatus)0;
        ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_COMMAND_EXECUTION_STATUS, bytes, &status, NULL), McResult::MC_NO_ERROR);

        ASSERT_TRUE(status & McEventCommandExecStatus::MC_COMPLETE);
    } catch (std::exception&) {
        ASSERT_TRUE(false); // should not reach here (it means "mcSetUserEventStatus" failed)
    }
}

UTEST_F(UserEvents, dependOnWithError)
{
    ASSERT_EQ(mcCreateUserEvent(&utest_fixture->userEvent, utest_fixture->context), McResult::MC_NO_ERROR);
    ASSERT_TRUE(utest_fixture->userEvent != MC_NULL_HANDLE);

    auto someUserEventFunction = [](McEvent userEvent_) {
        // do stuff that uses use some 100ms
        for (McInt32 i = 0; i < 10; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // notify that the event has finished BUT with an error
        const McResult res = mcSetUserEventStatus(userEvent_, (McInt32)McResult::MC_INVALID_OPERATION);

        if (res != McResult::MC_NO_ERROR) {
            throw std::runtime_error("failed to set user-event status to McResult::MC_INVALID_OPERATION");
        }
    };

    //
    // mcEnqueueDispatch will not run because an event which it depends on "utest_fixture->userEvent"
    // has a non-successful runtime status.
    //
    McEvent dispatchEvent = MC_NULL_HANDLE;
    ASSERT_EQ(mcEnqueueDispatch(
                  utest_fixture->context,
                  MC_DISPATCH_VERTEX_ARRAY_DOUBLE,
                  utest_fixture->cubeVertices.data(),
                  utest_fixture->cubeFaces.data(),
                  utest_fixture->cubeFaceSizes.data(),
                  utest_fixture->numCubeVertices,
                  utest_fixture->numCubeFaces,
                  utest_fixture->cutMeshVertices.data(),
                  utest_fixture->cutMeshFaces.data(),
                  nullptr, // cutMeshFaceSizes, // no need to give 'faceSizes' parameter since cut-mesh is a triangle mesh
                  utest_fixture->numCutMeshVertices,
                  utest_fixture->numCutMeshFaces,
                  1, &utest_fixture->userEvent, &dispatchEvent),
        MC_NO_ERROR);

    ASSERT_TRUE(dispatchEvent != MC_NULL_HANDLE);

    auto fut = std::async(std::launch::async, someUserEventFunction, utest_fixture->userEvent);

    // NOTE: mcWaitForEvents must be called AFTER lauching/calling "someUserEventFunction".
    // Otherwise, we get deadlock
    ASSERT_EQ(mcWaitForEvents(1, &dispatchEvent), MC_NO_ERROR);

    ASSERT_EQ(mcReleaseEvents(1, &dispatchEvent), McResult::MC_NO_ERROR);

    try {
        fut.get(); // may throw exceptions if something unexpected happens see "mcSetUserEventStatus"

        McSize bytes = 0;
        ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS, 0, NULL, &bytes), McResult::MC_NO_ERROR);

        McResult status = McResult::MC_NO_ERROR;
        ASSERT_EQ(mcGetEventInfo(utest_fixture->userEvent, MC_EVENT_RUNTIME_EXECUTION_STATUS, bytes, &status, NULL), McResult::MC_NO_ERROR);

        ASSERT_EQ(status, McResult::MC_INVALID_OPERATION); // from "mcSetUserEventStatus"
    } catch (std::exception&) {
        ASSERT_TRUE(false); // should not reach here (it means "mcSetUserEventStatus" failed)
    }
}