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



#include "mcut/internal/utils.h"
#include "mcut/internal/number.h"

namespace mcut {
namespace math {

 enum sign_t {
        ON_NEGATIVE_SIDE = -1, // left
        ON_ORIENTED_BOUNDARY = 0, // on boundary
        ON_POSITIVE_SIDE = 1, // right
        //
        NEGATIVE = ON_NEGATIVE_SIDE,
        ZERO = ON_ORIENTED_BOUNDARY,
        POSITIVE = ON_POSITIVE_SIDE,
    };

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

        const real_t& x() const
        {
            return m_x;
        }

        const real_t& y() const
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

        const real_t& z() const
        {
            return m_z;
        }

    protected:
        real_t m_z;
    };

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

        int rows() const
        {
            return m_row_count;
        }

        int cols() const
        {
            return m_column_count;
        }

    private:
        int m_row_count;
        int m_column_count;
        std::vector<int> m_entries;
    };

    real_t square_root(const real_t& number);
    real_t absolute_value(const real_t& number);
    sign_t sign(const real_t& number);
    extern std::ostream& operator<<(std::ostream& os, const vec3& v);
    extern std::ostream& operator<<(std::ostream& os, const matrix_t& m);
    const real_t& min(const real_t& a, const real_t& b);
    const real_t& max(const real_t& a, const real_t& b);
    bool operator==(const vec3& a, const vec3& b);
    vec2 compwise_min(const vec2& a, const vec2& b);
    vec3 compwise_min(const vec3& a, const vec3& b);
    vec2 compwise_max(const vec2& a, const vec2& b);
    vec3 compwise_max(const vec3& a, const vec3& b);
    vec3 cross_product(const vec3& a, const vec3& b);
    real_t dot_product(const vec3& a, const vec3& b);

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
} // namespace math
} // namespace mcut {

#endif // MCUT_MATH_H_
