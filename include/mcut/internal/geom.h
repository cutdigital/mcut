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

#ifndef MCUT_GEOM_H_
#define MCUT_GEOM_H_

#include "mcut/internal/math.h"

namespace mcut {
namespace geom {

     enum sign_t {
        ON_NEGATIVE_SIDE = -1, // left
        ON_ORIENTED_BOUNDARY = 0, // on boundary
        ON_POSITIVE_SIDE = 1 // right
    };

    typedef sign_t orientation_t;

    template <typename vector_type>
    struct bounding_box_t {

        vector_type m_minimum;
        vector_type m_maximum;

        bounding_box_t(const vector_type& minimum, const vector_type& maximum)
        {
            m_minimum = minimum;
            m_maximum = maximum;
        }

        bounding_box_t()
        {
            m_minimum = vector_type(std::numeric_limits<double>::max());
            m_maximum = vector_type(std::numeric_limits<double>::min());
        }

        const vector_type& minimum() const
        {
            return m_minimum;
        }

        const vector_type& maximum() const
        {
            return m_maximum;
        }

        void expand(const vector_type& point)
        {
            m_maximum = compwise_max(m_maximum, point);
            m_minimum = compwise_min(m_minimum, point);
        }

        void expand(const bounding_box_t<vector_type>& bbox)
        {
            m_maximum = compwise_max(m_maximum, bbox.maximum());
            m_minimum = compwise_min(m_minimum, bbox.minimum());
        }
    };

    bool intersect_bounding_boxes(const bounding_box_t<math::vec3>& a, const bounding_box_t<math::vec3>& b);

    bool point_in_bounding_box(const math::vec2& point, const bounding_box_t<math::vec2>& bbox);

    bool point_in_bounding_box(const math::vec3& point, const bounding_box_t<math::vec3>& bbox);

    // http://cs.haifa.ac.il/~gordon/plane.pdf
    void polygon_normal(math::vec3& normal, const math::vec3* vertices, const int num_vertices);

    void polygon_plane_d_param(math::real_t& d, const math::vec3& plane_normal, const math::vec3* polygon_vertices, const int num_polygon_vertices);

    void project_point(math::vec2& projected_point, const math::vec3& point, const math::vec3& u, const math::vec3& v);

    void polygon_span(math::vec3& span, math::real_t& span_length, const math::vec3* vertices, const int num_vertices);

    inline long direction(const math::vec2& a, const math::vec2& b, const math::vec2& c);

    // on which side does "c" lie on the left of, or right right of "ab"

    inline orientation_t orientation(const math::vec2& a, const math::vec2& b, const math::vec2& c);

    bool point_on_segment(const math::vec2& pi, const math::vec2& pj, const math::vec2& pk);

    // segment 0: ab
    // segment 1: cd
    bool segments_intersect(const math::vec2& a, const math::vec2& b, const math::vec2& c, const math::vec2& d);

    template <typename vector_type>
    void make_bbox(bounding_box_t<vector_type>& bbox, const vector_type* vertices, const int num_vertices)
    {
        MCUT_ASSERT(vertices != nullptr);
        MCUT_ASSERT(num_vertices >= 3);

        for (int i = 0; i < num_vertices; ++i) {
            const vector_type& vertex = vertices[i];
            bbox.expand(vertex);
        }
    }

    // returns true if point lies [inside] the given polygon.
    // returns false if point lies on the [border] or [outside] the polygon

    bool point_in_polygon(const math::vec3& point, const math::vec3& polygon_normal, const math::vec3* vertices, const int num_vertices);

    bool intersect_plane_with_segment(
        math::vec3& point,
        math::real_t& t,
        // plane
        const math::vec3& normal,
        math::real_t& distance,
        // segment
        const math::vec3& source,
        const math::vec3& target);

} // namespace geom {
} // namespace mcut {

#endif // MCUT_GEOM_H_
