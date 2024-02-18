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

#ifndef MCUT_TIMER_H_
#define MCUT_TIMER_H_

#include <chrono>
#include <map>
#include <memory>
#include <stack>
#include <sstream>

#include "mcut/internal/utils.h"
#include "mcut/internal/tpool.h"

//#define PROFILING_BUILD

class mini_timer {
    std::chrono::time_point<std::chrono::steady_clock> m_start;
    const std::string m_name;
    bool m_valid = true;

public:
    mini_timer(const std::string& name)
        : m_start(std::chrono::steady_clock::now())
        , m_name(name)
    {
    }

    ~mini_timer()
    {
        if (m_valid) {
#ifdef MCUT_WITH_API_EVENT_LOGGING
            const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            const std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start);
            unsigned long long elapsed_ = elapsed.count();
            log_msg("[MCUT][PROF:" << std::this_thread::get_id() << "]: \"" << m_name << "\" ("<< elapsed_ << "ms)");
#endif // #ifdef MCUT_WITH_API_EVENT_LOGGING
        }
    }
    void set_invalid()
    {
        m_valid = false;
    }
};

extern thread_local std::stack<std::unique_ptr<mini_timer>> g_thrd_loc_timerstack;

#if defined(PROFILING_BUILD)

#define TIMESTACK_PUSH(name) \
    g_thrd_loc_timerstack.push(std::unique_ptr<mini_timer>(new mini_timer(name)))
#define TIMESTACK_POP() \
    g_thrd_loc_timerstack.pop()
#define TIMESTACK_RESET()                                              \
    while (!g_thrd_loc_timerstack.empty()) {        \
        g_thrd_loc_timerstack.top()->set_invalid(); \
        g_thrd_loc_timerstack.pop();                \
    }
#define SCOPED_TIMER(name) \
    mini_timer _1mt(name)
#else
#define SCOPED_TIMER(name)
#define TIMESTACK_PUSH(name)
#define TIMESTACK_POP()
#define TIMESTACK_RESET()
#endif



#endif