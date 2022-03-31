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

#ifndef MCUT_MATH_H_
#define MCUT_MATH_H_

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
#else
#include <cmath>
#endif

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "mcut/internal/number.h"
#include "mcut/internal/utils.h"

namespace mcut
{
namespace math
{

enum sign_t
{
    ON_NEGATIVE_SIDE = -1,    // left
    ON_ORIENTED_BOUNDARY = 0, // on boundary
    ON_POSITIVE_SIDE = 1,     // right
    //
    NEGATIVE = ON_NEGATIVE_SIDE,
    ZERO = ON_ORIENTED_BOUNDARY,
    POSITIVE = ON_POSITIVE_SIDE,
};

template <typename T = real_number_t> class vec2_
{
  public:
    typedef T element_type;
    vec2_() : m_x(0.0), m_y(0.0)
    {
    }

    vec2_(const T &value) : m_x(value), m_y(value)
    {
    }

    vec2_(const T &x, const T &y) : m_x(x), m_y(y)
    {
    }

    virtual ~vec2_()
    {
    }

    static int cardinality()
    {
        return 2;
    }

    const T &operator[](int index) const
    {
        MCUT_ASSERT(index >= 0 && index <= 1);

        if (index == 0)
        {
            return m_x;
        }
        else
        {
            return m_y;
        }
    }

    T &operator[](int index)
    {
        MCUT_ASSERT(index >= 0 && index <= 1);

        T *val = nullptr;
        if (index == 0)
        {
            val = &m_x;
        }
        else
        {
            val = &m_y;
        }

        return *val;
    }

    const vec2_ operator-(const vec2_ &other) const
    {
        return vec2_(m_x - other.m_x, m_y - other.m_y);
    }

    const vec2_ operator/(const T &number) const
    {
        return vec2_(m_x / number, m_y / number);
    }

    const T &x() const
    {
        return m_x;
    }

    const T &y() const
    {
        return m_y;
    }

  protected:
    T m_x, m_y;
}; // vec2_

typedef vec2_<> vec2;
typedef vec2_<fixed_precision_number_t> fast_vec2;

template <typename T = real_number_t> class vec3_ : public vec2_<T>
{

  public:
    vec3_() : vec2_<T>(0.0, 0.0), m_z(0.0)
    {
    }

    vec3_(const T &value) : vec2_<T>(value, value), m_z(value)
    {
    }

    vec3_(const T &x, const T &y, const T &z) : vec2_<T>(x, y), m_z(z)
    {
    }
    ~vec3_()
    {
    }

    static int cardinality()
    {
        return 3;
    }

    const T &operator[](int index) const
    {
        MCUT_ASSERT(index >= 0 && index <= 2);
        const T *val = nullptr;
        if (index <= 1)
        {
            val = &vec2_<T>::operator[](index);
        }
        else
        {
            val = &m_z;
        }
        return *val;
    }

    T &operator[](int index)
    {
        MCUT_ASSERT(index >= 0 && index <= 2);
        T *val = nullptr;
        if (index <= 1)
        {
            val = &vec2_<T>::operator[](index);
        }
        else
        {
            val = &m_z;
        }
        return *val;
    }

    // intended for converting from arb-prec to fast vec

    operator vec3_<mcut::math::fixed_precision_number_t>() const
    {
        return vec3_<mcut::math::fixed_precision_number_t>(
            static_cast<mcut::math::fixed_precision_number_t>(this->m_x),
            static_cast<mcut::math::fixed_precision_number_t>(this->m_y),
            static_cast<mcut::math::fixed_precision_number_t>(this->m_z));
    }

    vec3_ operator-(const vec3_ &other) const
    {
        return vec3_(this->m_x - other.m_x, this->m_y - other.m_y, this->m_z - other.m_z);
    }

    vec3_ operator+(const vec3_ &other) const
    {
        return vec3_(this->m_x + other.m_x, this->m_y + other.m_y, this->m_z + other.m_z);
    }

    const vec3_ operator/(const T &number) const
    {
        return vec3_(this->m_x / number, this->m_y / number, this->m_z / number);
    }

    const vec3_ operator*(const T &number) const
    {
        return vec3_(this->m_x * number, this->m_y * number, this->m_z * number);
    }

    const T &z() const
    {
        return m_z;
    }

  protected:
    T m_z;
}; // vec3_

typedef vec3_<> vec3;
typedef vec3_<fixed_precision_number_t> fast_vec3;

template <typename T = int> class matrix_t
{
  public:
    matrix_t() : m_rows(-1), m_cols(-1)
    {
    }

    matrix_t(unsigned int rows, unsigned int cols)
        : m_rows(rows), m_cols(cols), m_entries(std::vector<T>((size_t)rows * cols, 0)) // zeroes
    {
    }

    T &operator()(unsigned int row, unsigned int col)
    {
        return m_entries[(size_t)row * m_cols + col];
    }

    T operator()(unsigned int row, unsigned int col) const
    {
        return m_entries[(size_t)row * m_cols + col];
    }

    // multiply
    matrix_t<T> operator*(const matrix_t<T> &other) const
    {
        matrix_t<T> result(this->rows(), other.cols());

        for (int i = 0; i < this->rows(); ++i)
        {
            for (int j = 0; j < other.cols(); ++j)
            {
                for (int k = 0; k < this->cols(); ++k)
                {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }

        return result;
    }

    matrix_t<T> operator*(const real_number_t &s) const
    {
        matrix_t<T> result(m_rows, m_cols);

        for (int i = 0; i < m_rows; ++i)
        {
            for (int j = 0; j < m_cols; ++j)
            {
                result(i, j) = (*this)(i, j) * s;
            }
        }

        return result;
    }

    matrix_t<T> operator/(const real_number_t &s) const
    {
        matrix_t<T> result(m_rows, m_cols);

        for (int i = 0; i < m_rows; ++i)
        {
            for (int j = 0; j < m_cols; ++j)
            {
                result(i, j) = (*this)(i, j) / s;
            }
        }

        return result;
    }

    matrix_t<T> operator-(const matrix_t<T> &m) const
    {
        MCUT_ASSERT(m.rows() == this->rows());
        MCUT_ASSERT(m.cols() == this->cols());

        matrix_t<T> result(m_rows, m_cols);

        for (int i = 0; i < m_rows; ++i)
        {
            for (int j = 0; j < m_cols; ++j)
            {
                result(i, j) = (*this)(i, j) - m(i, j);
            }
        }

        return result;
    }

    // matrix vector multiply
    template <typename Vec> Vec operator*(const Vec &v) const
    {
        MCUT_ASSERT(this->cols() == Vec::cardinality());

        // columns
        const Vec c0((*this)(0, 0), (*this)(1, 0), (*this)(2, 0));
        const Vec c1((*this)(0, 1), (*this)(1, 1), (*this)(2, 1));
        const Vec c2((*this)(0, 2), (*this)(1, 2), (*this)(2, 2));
        const Vec result = c0 * v[0] + c1 * v[1] + c2 * v[2];

        return result;
    }

    inline int rows() const
    {
        return m_rows;
    }

    inline int cols() const
    {
        return m_cols;
    }

  private:
    int m_rows;
    int m_cols;
    std::vector<T> m_entries;
};

extern real_number_t square_root(const real_number_t &number);
extern real_number_t absolute_value(const real_number_t &number);
extern sign_t sign(const real_number_t &number);
extern std::ostream &operator<<(std::ostream &os, const vec3 &v);

template <typename U> std::ostream &operator<<(std::ostream &os, const matrix_t<U> &m)
{
    for (int i = 0; i < m.rows(); i++)
    {
        for (int j = 0; j < m.cols(); j++)
        {
            os << m(i, j) << ", ";
        }
        os << "\n";
    }
    return os;
}

template <typename T> const T &min(const T &a, const T &b)
{
    return ((b < a) ? b : a);
}
template <typename T> const T &max(const T &a, const T &b)
{
    return ((a < b) ? b : a);
}

extern bool operator==(const vec3 &a, const vec3 &b);

template <typename T> vec2_<T> compwise_min(const vec2_<T> &a, const vec2_<T> &b)
{
    return vec2_<T>(min(a.x(), b.x()), min(a.y(), b.y()));
}

template <typename T> vec2_<T> compwise_max(const vec2_<T> &a, const vec2_<T> &b)
{
    return vec2_<T>(max(a.x(), b.x()), max(a.y(), b.y()));
}

template <typename T> vec3_<T> compwise_min(const vec3_<T> &a, const vec3_<T> &b)
{
    return vec3_<T>(min(a.x(), b.x()), min(a.y(), b.y()), min(a.z(), b.z()));
}

template <typename T> vec3_<T> compwise_max(const vec3_<T> &a, const vec3_<T> &b)
{
    return vec3_<T>(max(a.x(), b.x()), max(a.y(), b.y()), max(a.z(), b.z()));
}

extern vec3 cross_product(const vec3 &a, const vec3 &b);

template <typename vector_type> math::real_number_t dot_product(const vector_type &a, const vector_type &b)
{
    math::real_number_t out(0.0);
    for (int i = 0; i < vector_type::cardinality(); ++i)
    {
        out += (a[i] * b[i]);
    }
    return out;
}

// compute a*b^T, which is a matrix
template <typename vector_type>
math::matrix_t<typename vector_type::element_type> outer_product(const vector_type &a, const vector_type &b)
{
    math::matrix_t<typename vector_type::element_type> out(vector_type::cardinality(), vector_type::cardinality());
    const vector_type c0 = a * b[0]; // colmuns
    const vector_type c1 = a * b[1];

    if (vector_type::cardinality() == 3)
    {
        const vector_type c2 = a * b[2];

        out(0, 0) = c0[0];
        out(1, 0) = c0[1];
        out(2, 0) = c0[2];

        out(0, 1) = c1[0];
        out(1, 1) = c1[1];
        out(2, 1) = c1[2];

        out(0, 2) = c2[0];
        out(1, 2) = c2[1];
        out(2, 2) = c2[2];
    }
    else
    {
        out(0, 0) = c0[0];
        out(1, 0) = c0[1];

        out(0, 1) = c1[0];
        out(1, 1) = c1[1];
    }

    return out;
}

template <typename vector_type> typename math::real_number_t squared_length(const vector_type &v)
{
    return dot_product(v, v);
}

template <typename vector_type> typename math::real_number_t length(const vector_type &v)
{
    return square_root(squared_length(v));
}

template <typename vector_type> vector_type normalize(const vector_type &v)
{
    return v / length(v);
}
} // namespace math
} // namespace mcut

#endif // MCUT_MATH_H_
