#ifndef MCUT_MATH_H_
#define MCUT_MATH_H_

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
#include <mpfr.h>
#else
#include <cmath>
#endif

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "mcut/internal/utils.h"

namespace mcut {
namespace math {

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    
    class high_precision_float_t {
    public:
        inline static mp_rnd_t get_default_rounding_mode()
        {
            return (mp_rnd_t)(mpfr_get_default_rounding_mode());
        }

        inline static mp_prec_t get_default_precision()
        {
            return (mpfr_get_default_prec)(); // 53 bits (from the spec)
        }

        inline static void set_default_precision(mp_prec_t prec)
        {
            mpfr_set_default_prec(prec);
        }

        inline static void set_default_rounding_mode(mp_rnd_t rnd_mode)
        {
            mpfr_set_default_rounding_mode(rnd_mode);
        }

        high_precision_float_t()
        {
            mpfr_init2(get_mpfr_handle(), high_precision_float_t::get_default_precision());
            mpfr_set_ld(get_mpfr_handle(), 0.0, high_precision_float_t::get_default_rounding_mode());
        }

        high_precision_float_t(const long double& value)
        {
            mpfr_init2(get_mpfr_handle(), high_precision_float_t::get_default_precision());
            mpfr_set_ld(get_mpfr_handle(), value, high_precision_float_t::get_default_rounding_mode());
        }

        high_precision_float_t(const char* value)
        {
            mpfr_init2(get_mpfr_handle(), high_precision_float_t::get_default_precision());
            int ret = mpfr_set_str(get_mpfr_handle(), value, 10, high_precision_float_t::get_default_rounding_mode());
            if(ret!=0)
            {
                std::fprintf(stderr, "mpfr_set_str failed\n");
                std::abort();
            }
        }

        // Construct high_precision_float_t from mpfr_t structure.
        // shared = true allows to avoid deep copy, so that high_precision_float_t and 'u' share the same data & pointers.
        inline high_precision_float_t(const high_precision_float_t& u, bool shared = false)
        {
            if (shared) {
                std::memcpy(this->get_mpfr_handle(), u.get_mpfr_handle(), sizeof(mpfr_t));
            } else {
                mpfr_init2(this->get_mpfr_handle(), high_precision_float_t::get_default_precision());
                mpfr_set(this->get_mpfr_handle(), u.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
            }
        }

        ~high_precision_float_t()
        {
            clear();
        }

        void clear()
        {
            if ((nullptr != (get_mpfr_handle())->_mpfr_d)) {
                mpfr_clear(get_mpfr_handle());
            }
        }

        inline high_precision_float_t(high_precision_float_t&& other)
        {
            // make sure "other" holds null-pointer (in uninitialized state)
            ((get_mpfr_handle())->_mpfr_d = 0);
            mpfr_swap(get_mpfr_handle(), other.get_mpfr_handle());
        }

        inline high_precision_float_t& operator=(high_precision_float_t&& other)
        {
            if (this != &other) {
                mpfr_swap(get_mpfr_handle(), other.get_mpfr_handle()); // destructor for "other" will be called just afterwards
            }
            return *this;
        }

        inline const mpfr_t& get_mpfr_handle() const
        {
            return m_mpfr_val;
        }

        inline mpfr_t& get_mpfr_handle()
        {
            return m_mpfr_val;
        }

        operator long double() const { return static_cast<long double>(to_double()); }

        operator double() const { return static_cast<double>(to_double()); }

        operator float() const { return static_cast<float>(to_double()); }

        inline high_precision_float_t& operator=(const high_precision_float_t& v)
        {
            if (this != &v) {

                mp_prec_t tp = mpfr_get_prec(get_mpfr_handle());
                mp_prec_t vp = mpfr_get_prec(v.get_mpfr_handle());

                if (tp != vp) {
                    clear();
                    mpfr_init2(get_mpfr_handle(), vp);
                }

                mpfr_set(get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            }

            return *this;
        }

        inline high_precision_float_t& operator=(const long double v)
        {
            mpfr_set_ld(get_mpfr_handle(), v, get_default_rounding_mode());
            return *this;
        }

        inline high_precision_float_t& operator+=(const high_precision_float_t& v)
        {
            mpfr_add(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        inline high_precision_float_t& operator+=(const long double u)
        {
            *this += high_precision_float_t(u);
            return *this;
        }

        inline high_precision_float_t& operator++()
        {
            return *this += 1;
        }

        inline const high_precision_float_t operator++(int)
        {
            high_precision_float_t x(*this);
            *this += 1;
            return x;
        }

        inline high_precision_float_t& operator--()
        {
            return *this -= 1;
        }

        inline const high_precision_float_t operator--(int)
        {
            high_precision_float_t x(*this);
            *this -= 1;
            return x;
        }

        inline high_precision_float_t& operator-=(const high_precision_float_t& v)
        {
            mpfr_sub(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        inline high_precision_float_t& operator-=(const long double v)
        {
            *this -= high_precision_float_t(v);
            return *this;
        }

        inline high_precision_float_t& operator-=(const int v)
        {
            mpfr_sub_si(get_mpfr_handle(), get_mpfr_handle(), v, get_default_rounding_mode());
            return *this;
        }

        inline const high_precision_float_t operator-() const
        {
            high_precision_float_t u(*this); // copy
            mpfr_neg(u.get_mpfr_handle(), u.get_mpfr_handle(), get_default_rounding_mode());
            return u;
        }

        inline high_precision_float_t& operator*=(const high_precision_float_t& v)
        {
            mpfr_mul(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        inline high_precision_float_t& operator*=(const long double v)
        {
            *this *= high_precision_float_t(v);
            return *this;
        }

        inline high_precision_float_t& operator/=(const high_precision_float_t& v)
        {
            mpfr_div(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        inline high_precision_float_t& operator/=(const long double v)
        {
            *this /= high_precision_float_t(v);
            return *this;
        }

        inline long double to_double() const
        {
            return mpfr_get_ld(get_mpfr_handle(), get_default_rounding_mode());
        }

        inline std::string to_string(const std::string& format = "%e") const
        {
            char *s = NULL;
            std::string out;

            if( !format.empty() )
            {
                if(!(mpfr_asprintf(&s, format.c_str(), mpfr_srcptr()) < 0))
                {
                    out = std::string(s);

                    mpfr_free_str(s);
                }
            }

            return out;
        }

        static inline bool is_nan(const high_precision_float_t& op)
        {
            return (mpfr_nan_p(op.get_mpfr_handle()) != 0);
        }

        static inline bool is_zero(const high_precision_float_t& op)
        {
            return (mpfr_zero_p(op.get_mpfr_handle()) != 0);
        }

    private:
        mpfr_t m_mpfr_val;
    };

    static std::ostream& operator<<(std::ostream& os, high_precision_float_t const& m)
    {
        return os << static_cast<long double>(m);
    }

    inline const high_precision_float_t operator*(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        high_precision_float_t c(0.0);
        mpfr_mul(c.get_mpfr_handle(), a.get_mpfr_handle(), b.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return c;
    }

    inline const high_precision_float_t operator+(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        high_precision_float_t c(0.0);
        mpfr_add(c.get_mpfr_handle(), a.get_mpfr_handle(), b.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return c;
    }

    inline const high_precision_float_t operator-(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        high_precision_float_t c(0.0);
        mpfr_sub(c.get_mpfr_handle(), a.get_mpfr_handle(), b.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return c;
    }

    inline const high_precision_float_t operator/(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        high_precision_float_t c(0.0);
        mpfr_div(c.get_mpfr_handle(), a.get_mpfr_handle(), b.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return c;
    }

    inline const high_precision_float_t operator/(const long double b, const high_precision_float_t& a)
    {
        high_precision_float_t x(0.0);
        // NOTE: mpfr_ld_div not availale
        mpfr_d_div(x.get_mpfr_handle(), b, a.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return x;
    }

    //////////////////////////////////////////////////////////////////////////
    //Relational operators

    // WARNING:
    //
    // Please note that following checks for double-NaN are guaranteed to work only in IEEE math mode:
    //
    // is_nan(b) =  (b != b)
    // is_nan(b) = !(b == b)  (we use in code below)
    //
    // Be cautions if you use compiler options which break strict IEEE compliance (e.g. -ffast-math in GCC).
    // Use std::is_nan instead (C++11).

    inline bool operator>(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        return (mpfr_greater_p(a.get_mpfr_handle(), b.get_mpfr_handle()) != 0);
    }

    inline bool operator>(const high_precision_float_t& a, const long double b)
    {
        return !high_precision_float_t::is_nan(a) && (b == b) && (mpfr_cmp_ld(a.get_mpfr_handle(), b) > 0);
    }

    inline bool operator>=(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        return (mpfr_greaterequal_p(a.get_mpfr_handle(), b.get_mpfr_handle()) != 0);
    }

    inline bool operator>=(const high_precision_float_t& a, const long double b)
    {
        return !high_precision_float_t::is_nan(a) && (b == b) && (mpfr_cmp_ld(a.get_mpfr_handle(), b) >= 0);
    }

    inline bool operator<(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        return (mpfr_less_p(a.get_mpfr_handle(), b.get_mpfr_handle()) != 0);
    }

    inline bool operator<(const high_precision_float_t& a, const long double b)
    {
        return !high_precision_float_t::is_nan(a) && (b == b) && (mpfr_cmp_ld(a.get_mpfr_handle(), b) < 0);
    }

    inline bool operator<=(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        return (mpfr_lessequal_p(a.get_mpfr_handle(), b.get_mpfr_handle()) != 0);
    }

    inline bool operator<=(const high_precision_float_t& a, const long double b)
    {
        return !high_precision_float_t::is_nan(a) && (b == b) && (mpfr_cmp_ld(a.get_mpfr_handle(), b) <= 0);
    }

    inline bool operator==(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        return (mpfr_equal_p(a.get_mpfr_handle(), b.get_mpfr_handle()) != 0);
    }

    inline bool operator==(const high_precision_float_t& a, const long double b)
    {
        return !high_precision_float_t::is_nan(a) && (b == b) && (mpfr_cmp_ld(a.get_mpfr_handle(), b) == 0);
    }

    inline bool operator!=(const high_precision_float_t& a, const high_precision_float_t& b)
    {
        return !(a == b);
    }

    inline bool operator!=(const high_precision_float_t& a, const long double b)
    {
        return !(a == b);
    }

    using real_t = high_precision_float_t;
#else 
using real_t = long double;
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    static inline real_t square_root(const real_t& number)
    {
        return std::sqrt(number);
    }
#else
    static inline high_precision_float_t square_root(const high_precision_float_t& number)
    {
        high_precision_float_t out(number);
        mpfr_sqrt(out.get_mpfr_handle(), number.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return out;
    }
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    static inline real_t absolute_value(const real_t& number)
    {
        return std::fabs(number);
    }
#else
    static inline high_precision_float_t absolute_value(const real_t& number)
    {
        real_t out(number);
        mpfr_abs(out.get_mpfr_handle(), number.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return out;
    }
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

    enum sign_t {
        ON_NEGATIVE_SIDE = -1, // left
        ON_ORIENTED_BOUNDARY = 0, // on boundary
        ON_POSITIVE_SIDE = 1, // right
        //
        NEGATIVE = ON_NEGATIVE_SIDE,
        ZERO = ON_ORIENTED_BOUNDARY,
        POSITIVE = ON_POSITIVE_SIDE,
    };

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    static inline sign_t sign(const real_t& number)
    {
        int s = (real_t(0) < number) - (number < real_t(0));
        sign_t result = sign_t::ZERO;
        if (s > 0) {
            result = sign_t::POSITIVE;
        } else if (s < 0) {
            result = sign_t::NEGATIVE;
        }
        return result;
    }
#else
    static inline sign_t sign(const real_t& number)
    {
        real_t out(number);
        int s = mpfr_sgn(number.get_mpfr_handle());
        sign_t result = sign_t::ZERO;
        if (s > 0) {
            result = sign_t::POSITIVE;
        } else if (s < 0) {
            result = sign_t::NEGATIVE;
        }
        return result;
    }
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

    class vec2 {
    public:
        vec2() { }

        vec2(const real_t& value)
            : m_x(value)
            , m_y(value)
        {
        }

        vec2(const real_t& x, const real_t& y)
            : m_x(x)
            , m_y(y)
        {
        }

        virtual ~vec2() { }

        static int cardinality()
        {
            return 2;
        }

        const real_t& operator[](int index) const
        {
            MCUT_ASSERT(index >= 0 && index <= 1);

            const real_t* val = nullptr;
            if (index == 0) {
                val = &m_x;
            } else {
                val = &m_y;
            }

            return *val;
        }

        real_t& operator[](int index)
        {
            MCUT_ASSERT(index >= 0 && index <= 1);

            real_t* val = nullptr;
            if (index == 0) {
                val = &m_x;
            } else {
                val = &m_y;
            }

            return *val;
        }

        const vec2 operator-(const vec2& other) const
        {
            return vec2(m_x - other.m_x, m_y - other.m_y);
        }

        const vec2 operator/(const real_t& number) const
        {
            return vec2(m_x / number, m_y / number);
        }

        inline const real_t& x() const
        {
            return m_x;
        }

        inline const real_t& y() const
        {
            return m_y;
        }

    protected:
        real_t m_x, m_y;
    };

    class vec3 : public vec2 {

    public:
        vec3()
        {
        }

        vec3(const real_t& value)
            : vec2(value, value)
            , m_z(value)
        {
        }

        vec3(const real_t& x, const real_t& y, const real_t& z)
            : vec2(x, y)
            , m_z(z)
        {
        }
        ~vec3()
        {
        }

        static int cardinality()
        {
            return 3;
        }

        const real_t& operator[](int index) const
        {
            MCUT_ASSERT(index >= 0 && index <= 2);
            const real_t* val = nullptr;
            if (index <= 1) {
                val = &vec2::operator[](index);
            } else {
                val = &m_z;
            }
            return *val;
        }

        real_t& operator[](int index)
        {
            MCUT_ASSERT(index >= 0 && index <= 2);
            real_t* val = nullptr;
            if (index <= 1) {
                val = &vec2::operator[](index);
            } else {
                val = &m_z;
            }
            return *val;
        }

        vec3 operator-(const vec3& other) const
        {
            return vec3(this->m_x - other.m_x, this->m_y - other.m_y, this->m_z - other.m_z);
        }

        vec3 operator+(const vec3& other) const
        {
            return vec3(this->m_x + other.m_x, this->m_y + other.m_y, this->m_z + other.m_z);
        }

        const vec3 operator/(const real_t& number) const
        {
            return vec3(this->m_x / number, this->m_y / number, this->m_z / number);
        }

        const vec3 operator*(const real_t& number) const
        {
            return vec3(this->m_x * number, this->m_y * number, this->m_z * number);
        }

        inline const real_t& z() const
        {
            return m_z;
        }

    protected:
        real_t m_z;
    };

    static std::ostream& operator<<(std::ostream& os, const vec3& v)
    {
        return os << static_cast<long double>(v.x()) << ", " << static_cast<long double>(v.y()) << ", " << static_cast<long double>(v.z());
    }

    static const real_t& min(const real_t& a, const real_t& b)
    {
        return ((b < a) ? b : a);
    }

    static const real_t& max(const real_t& a, const real_t& b)
    {
        return ((a < b) ? b : a);
    }

    static inline bool operator==(const vec3& a, const vec3& b)
    {
        return (a.x() == b.x()) && (a.y() == b.y()) && (a.z() == b.z());
    }

    static vec2 compwise_min(const vec2& a, const vec2& b)
    {
        return vec2(min(a.x(), b.x()), min(a.y(), b.y()));
    }

    static vec3 compwise_min(const vec3& a, const vec3& b)
    {
        return vec3(min(a.x(), b.x()), min(a.y(), b.y()), min(a.z(), b.z()));
    }

    static vec2 compwise_max(const vec2& a, const vec2& b)
    {
        return vec2(max(a.x(), b.x()), max(a.y(), b.y()));
    }

    static vec3 compwise_max(const vec3& a, const vec3& b)
    {
        return vec3(max(a.x(), b.x()), max(a.y(), b.y()), max(a.z(), b.z()));
    }

    static vec3 cross_product(const vec3& a, const vec3& b)
    {
        return vec3(
            a.y() * b.z() - a.z() * b.y(),
            a.z() * b.x() - a.x() * b.z(),
            a.x() * b.y() - a.y() * b.x());
    }

    static real_t dot_product(const vec3& a, const vec3& b)
    {
        return (a.x() * b.x()) + (a.y() * b.y()) + (a.z() * b.z());
    }

#if 0
		static real_t dot_product(const vec2& a, const vec2& b)
    {
        return (a.x() * b.x()) + (a.y() * b.y());
    }
#endif
    template <typename vector_type>
    typename math::real_t squared_length(const vector_type& v)
    {
        return dot_product(v, v);
    }

    template <typename vector_type>
    typename math::real_t length(const vector_type& v)
    {
        return square_root(squared_length(v));
    }

    template <typename vector_type>
    vector_type normalize(const vector_type& v)
    {
        return v / length(v);
    }

    class matrix_t {
    public:
        matrix_t(int rows, int cols)
            : m_row_count(rows)
            , m_column_count(cols)
            , m_entries(std::vector<int>(rows * cols, 0))
        {
        }

        int& operator()(int i, int j)
        {
            return m_entries[j * m_row_count + i];
        }

        int operator()(int i, int j) const
        {
            return m_entries[j * m_row_count + i];
        }

        matrix_t operator*(const matrix_t& rhs) const
        {
            matrix_t result(m_row_count, m_column_count);

            for (int i = 0; i < m_row_count; ++i) {
                for (int j = 0; j < m_column_count; ++j) {
                    for (int k = 0; k < m_row_count; ++k) {
                        result(i, j) += (*this)(i, k) * rhs(k, j);
                    }
                }
            }

            return result;
        }

        inline const int rows() const
        {
            return m_row_count;
        }

        inline const int cols() const
        {
            return m_column_count;
        }

    private:
        int m_row_count;
        int m_column_count;
        std::vector<int> m_entries;
    };

    static std::ostream& operator<<(std::ostream& os, const matrix_t& m)
    {
        for (int i = 0; i < m.rows(); i++) {
            for (int j = 0; j < m.cols(); j++) {
                os << m(i, j) << ", ";
            }
            os << "\n";
        }
        return os;
    }
} // namespace math
} // namespace mcut {

#endif // MCUT_MATH_H_
