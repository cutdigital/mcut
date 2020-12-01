#include "mcut/internal/math.h"
#include <cstdlib>
namespace mcut {
namespace math{

    real_t square_root(const real_t& number)
    {
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        return std::sqrt(number);
#else 
        high_precision_float_t out(number);
        mpfr_sqrt(out.get_mpfr_handle(), number.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
        return out;
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    }

real_t absolute_value(const real_t& number)
{
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    return std::fabs(number);
#else
    real_t out(number);
    mpfr_abs(out.get_mpfr_handle(), number.get_mpfr_handle(), high_precision_float_t::get_default_rounding_mode());
    return out;
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
}


sign_t sign(const real_t& number)
{
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        int s = (real_t(0) < number) - (number < real_t(0));
        sign_t result = sign_t::ZERO;
        if (s > 0) {
            result = sign_t::POSITIVE;
        } else if (s < 0) {
            result = sign_t::NEGATIVE;
        }
        return result;
#else
        real_t out(number);
        int s = mpfr_sgn(number.get_mpfr_handle());
        sign_t result = sign_t::ZERO;
        if (s > 0) {
            result = sign_t::POSITIVE;
        } else if (s < 0) {
            result = sign_t::NEGATIVE;
        }
        return result;
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
}

    std::ostream& operator<<(std::ostream& os, const vec3& v)
    {
        return os << static_cast<long double>(v.x()) << ", " << static_cast<long double>(v.y()) << ", " << static_cast<long double>(v.z());
    }

     std::ostream& operator<<(std::ostream& os, const matrix_t& m)
    {
        for (int i = 0; i < m.rows(); i++) {
            for (int j = 0; j < m.cols(); j++) {
                os << m(i, j) << ", ";
            }
            os << "\n";
        }
        return os;
    }

    const real_t& min(const real_t& a, const real_t& b)
    {
        return ((b < a) ? b : a);
    }

    const real_t& max(const real_t& a, const real_t& b)
    {
        return ((a < b) ? b : a);
    }

    bool operator==(const vec3& a, const vec3& b)
    {
        return (a.x() == b.x()) && (a.y() == b.y()) && (a.z() == b.z());
    }

    vec2 compwise_min(const vec2& a, const vec2& b)
    {
        return vec2(min(a.x(), b.x()), min(a.y(), b.y()));
    }

    vec3 compwise_min(const vec3& a, const vec3& b)
    {
        return vec3(min(a.x(), b.x()), min(a.y(), b.y()), min(a.z(), b.z()));
    }

    vec2 compwise_max(const vec2& a, const vec2& b)
    {
        return vec2(max(a.x(), b.x()), max(a.y(), b.y()));
    }

    vec3 compwise_max(const vec3& a, const vec3& b)
    {
        return vec3(max(a.x(), b.x()), max(a.y(), b.y()), max(a.z(), b.z()));
    }

    vec3 cross_product(const vec3& a, const vec3& b)
    {
        return vec3(
            a.y() * b.z() - a.z() * b.y(),
            a.z() * b.x() - a.x() * b.z(),
            a.x() * b.y() - a.y() * b.x());
    }

    real_t dot_product(const vec3& a, const vec3& b)
    {
        return (a.x() * b.x()) + (a.y() * b.y()) + (a.z() * b.z());
    }

    } // namespace math
} // namespace mcut {