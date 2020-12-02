/**
 * Copyright (c) 2020-2021 CutDigital Ltd.
 * All rights reserved.
 * 
 * NOTE: This file is licensed under GPL-3.0-or-later (default). 
 * A commercial license can be purchased from CutDigital Ltd. 
 *  
 * License details:
 * 
 * (A)  GNU General Public License ("GPL"); a copy of which you should have 
 *      recieved with this file.
 * 	    - see also: <http://www.gnu.org/licenses/>
 * (B)  Commercial license.
 *      - email: contact@cut-digital.com
 * 
 * The commercial license options is for users that wish to use MCUT in 
 * their products for comercial purposes but do not wish to release their 
 * software products under the GPL license. 
 * 
 * Author(s)     : Floyd M. Chitalu
 */


#ifndef MCUT_UTILS_H_
#define MCUT_UTILS_H_

#define stringize(s) #s
#define XSTR(s) stringize(s)

#ifndef NDEBUG
#define MCUT_ASSERT(a)                   \
    do {                                 \
        if (0 == (a)) {                  \
            std::fprintf(stderr,         \
                "Assertion failed: %s, " \
                "%d at \'%s\'\n",        \
                __FILE__,                \
                __LINE__,                \
                XSTR(a));                \
            std::abort();                \
        }                                \
    } while (0)
#else
#define MCUT_ASSERT(a) // do nothing
#endif // !NDEBUG

#include <fstream>
#include <iostream>
#include <sstream>

namespace mcut {

class logger_t {

    std::stringstream m_buffer;
    bool m_verbose;
    std::string m_prepend;
    std::string m_reason_for_failure;

public:
    typedef std::ostream& (*ManipFn)(std::ostream&);
    typedef std::ios_base& (*FlagsFn)(std::ios_base&);

    logger_t()
        : m_verbose(false)
    {
    }

    ~logger_t()
    {
    }

    std::string get_log_string()
    {
        return m_buffer.str();
    }

    void set_reason_for_failure(const std::string& msg)
    {
        m_reason_for_failure = msg;
    }

    const std::string& get_reason_for_failure() const
    {
        return m_reason_for_failure;
    }

    inline bool verbose()
    {
        return m_verbose;
    }

    inline void set_verbose(bool b)
    {
        m_verbose = b;
    }

    inline void reset()
    {
        m_prepend.clear();
    }

    inline void indent()
    {
        if (!verbose()) {
            return;
        }
        m_prepend.append("  ");
    }

    inline void unindent()
    {
        if (!verbose()) {
            return;
        }
        m_prepend.pop_back();
        m_prepend.pop_back();
    }

    template <class T> // int, double, strings, etc
    inline logger_t& operator<<(const T& output)
    {
        if (verbose()) {
            m_buffer << output;
        }
        return *this;
    }

    inline logger_t& operator<<(ManipFn manip) /// endl, flush, setw, setfill, etc.
    {
        if (verbose()) {
            manip(m_buffer);

            if (manip == static_cast<ManipFn>(std::flush) || manip == static_cast<ManipFn>(std::endl)) {
                this->flush();
            }
        }
        return *this;
    }

    inline logger_t& operator<<(FlagsFn manip) /// setiosflags, resetiosflags
    {
        if (verbose()) {
            manip(m_buffer);
        }
        return *this;
    }

    inline void flush()
    {
        if (!(verbose())) {
            return;
        }

#if 0 // dump log to terminal [immediately]
            std::cout << m_prepend << "::" << m_buffer.str();
            m_buffer.str(std::string());
            m_buffer.clear();
#endif
    }
};

} // namespace mcut

#endif // MCUT_UTILS_H_
