#ifndef NUMBER_H_
#define NUMBER_H_

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
#include <mpfr.h>
#endif

#include <string>
#include <cstring>
#include <cstdint>
#include <cstdio>

namespace mcut {
namespace math {

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    
    class high_precision_float_t {
    public:
        static mp_rnd_t get_default_rounding_mode()
        {
            return (mp_rnd_t)(mpfr_get_default_rounding_mode());
        }

        static mp_prec_t get_default_precision()
        {
            return (mpfr_get_default_prec)(); // 53 bits (from the spec)
        }

        static void set_default_precision(mp_prec_t prec)
        {
            mpfr_set_default_prec(prec);
        }

        static void set_default_rounding_mode(mp_rnd_t rnd_mode)
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
        high_precision_float_t(const high_precision_float_t& u, bool shared = false)
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

        high_precision_float_t(high_precision_float_t&& other)
        {
            // make sure "other" holds null-pointer (in uninitialized state)
            ((get_mpfr_handle())->_mpfr_d = 0);
            mpfr_swap(get_mpfr_handle(), other.get_mpfr_handle());
        }

        high_precision_float_t& operator=(high_precision_float_t&& other)
        {
            if (this != &other) {
                mpfr_swap(get_mpfr_handle(), other.get_mpfr_handle()); // destructor for "other" will be called just afterwards
            }
            return *this;
        }

        const mpfr_t& get_mpfr_handle() const
        {
            return m_mpfr_val;
        }

        mpfr_t& get_mpfr_handle()
        {
            return m_mpfr_val;
        }

        operator long double() const { return static_cast<long double>(to_double()); }

        operator double() const { return static_cast<double>(to_double()); }

        operator float() const { return static_cast<float>(to_double()); }

        high_precision_float_t& operator=(const high_precision_float_t& v)
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

        high_precision_float_t& operator=(const long double v)
        {
            mpfr_set_ld(get_mpfr_handle(), v, get_default_rounding_mode());
            return *this;
        }

        high_precision_float_t& operator+=(const high_precision_float_t& v)
        {
            mpfr_add(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        high_precision_float_t& operator+=(const long double u)
        {
            *this += high_precision_float_t(u);
            return *this;
        }

        high_precision_float_t& operator++()
        {
            return *this += 1;
        }

        const high_precision_float_t operator++(int)
        {
            high_precision_float_t x(*this);
            *this += 1;
            return x;
        }

        high_precision_float_t& operator--()
        {
            return *this -= 1;
        }

        const high_precision_float_t operator--(int)
        {
            high_precision_float_t x(*this);
            *this -= 1;
            return x;
        }

        high_precision_float_t& operator-=(const high_precision_float_t& v)
        {
            mpfr_sub(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        high_precision_float_t& operator-=(const long double v)
        {
            *this -= high_precision_float_t(v);
            return *this;
        }

        high_precision_float_t& operator-=(const int v)
        {
            mpfr_sub_si(get_mpfr_handle(), get_mpfr_handle(), v, get_default_rounding_mode());
            return *this;
        }

        const high_precision_float_t operator-() const
        {
            high_precision_float_t u(*this); // copy
            mpfr_neg(u.get_mpfr_handle(), u.get_mpfr_handle(), get_default_rounding_mode());
            return u;
        }

        high_precision_float_t& operator*=(const high_precision_float_t& v)
        {
            mpfr_mul(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        high_precision_float_t& operator*=(const long double v)
        {
            *this *= high_precision_float_t(v);
            return *this;
        }

        high_precision_float_t& operator/=(const high_precision_float_t& v)
        {
            mpfr_div(get_mpfr_handle(), get_mpfr_handle(), v.get_mpfr_handle(), get_default_rounding_mode());
            return *this;
        }

        high_precision_float_t& operator/=(const long double v)
        {
            *this /= high_precision_float_t(v);
            return *this;
        }

        long double to_double() const
        {
            return mpfr_get_ld(get_mpfr_handle(), get_default_rounding_mode());
        }

        std::string to_string() const
        {
            // NOTE: number of decimal digits in the significand is dependent on the current precision and rounding mode
            mpfr_exp_t decimalLocation = 0;
            char *s = mpfr_get_str (NULL, &decimalLocation, 10, 0, get_mpfr_handle(), get_default_rounding_mode());
            std::string out = std::string(s);
            if(out[0] == '-') // first char is minus sign
            {
                // The generated string is a fraction, with an implicit radix point immediately to the left of the 
                // first digit. For example, the number −3.1416 would be returned as "−31416" in the string and
                // 1 written at "&decimalLocation".
                decimalLocation += 1; // account for minus sign.            
            }
            out.insert(decimalLocation, ".");
            mpfr_free_str(s);
            return out;
        }

        static bool is_nan(const high_precision_float_t& op)
        {
            return (mpfr_nan_p(op.get_mpfr_handle()) != 0);
        }

        static bool is_zero(const high_precision_float_t& op)
        {
            return (mpfr_zero_p(op.get_mpfr_handle()) != 0);
        }

    private:
        mpfr_t m_mpfr_val;
    };

    extern std::ostream& operator<<(std::ostream& os, high_precision_float_t const& m);

    extern high_precision_float_t operator*(const high_precision_float_t& a, const high_precision_float_t& b);

    extern high_precision_float_t operator+(const high_precision_float_t& a, const high_precision_float_t& b);

    extern high_precision_float_t operator-(const high_precision_float_t& a, const high_precision_float_t& b);

    extern high_precision_float_t operator/(const high_precision_float_t& a, const high_precision_float_t& b);

    extern high_precision_float_t operator/(const long double b, const high_precision_float_t& a);

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

    extern bool operator>(const high_precision_float_t& a, const high_precision_float_t& b);

    extern bool operator>(const high_precision_float_t& a, const long double b);

    extern bool operator>=(const high_precision_float_t& a, const high_precision_float_t& b);

    extern bool operator>=(const high_precision_float_t& a, const long double b);

    extern bool operator<(const high_precision_float_t& a, const high_precision_float_t& b);

    extern bool operator<(const high_precision_float_t& a, const long double b);

    extern bool operator<=(const high_precision_float_t& a, const high_precision_float_t& b);

    extern bool operator<=(const high_precision_float_t& a, const long double b);

    extern bool operator==(const high_precision_float_t& a, const high_precision_float_t& b);

    extern bool operator==(const high_precision_float_t& a, const long double b);

    extern bool operator!=(const high_precision_float_t& a, const high_precision_float_t& b);

    extern bool operator!=(const high_precision_float_t& a, const long double b);

    using real_t = high_precision_float_t;
#else 
using real_t = long double;
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

}
}

#endif // NUMBER_H_