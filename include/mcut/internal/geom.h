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

#if defined(MCUT_USE_SHEWCHUK_EXACT_PREDICATES)

// Exact versions of Shewchuk's predicates.
// NOTE:  if 'MCUT_USE_EXACT_PREDICATES' is defined and MCUT_WITH_ARBITRARY_PRECISION_NUMBERS' is defined, then
//        these functions are faster than directly using arbitrary precision floating-point numbers. This is
//        because the these functions are adaptive in the sense that they do only as much work as necessary
//        to guarantee a correct result.
extern "C" double orient2d(const double* pa, const double* pb, const double* pc);
extern "C" double orient3d(const double* pa, const double* pb, const double* pc, const double* pd);

#else

// Generic (i.e. optionally inexact) versions of Shewchuk's predicates.
// NOTE:  if 'MCUT_WITH_ARBITRARY_PRECISION_NUMBERS' is defined, then these functions are parameterized an
//        exact-number type. Otherwise, they are parameterised by 'long double' i.e. the function definitions
//        are equivalent to Shewchuk's fast-variant predicates.
extern "C" mcut::math::real_number_t orient2d(const mcut::math::real_number_t* pa, const mcut::math::real_number_t* pb, const mcut::math::real_number_t* pc);
extern "C" mcut::math::real_number_t orient3d(const mcut::math::real_number_t* pa, const mcut::math::real_number_t* pb, const mcut::math::real_number_t* pc, const double* pd);

#endif

namespace mcut {
namespace geom {

    bool Collinear(const math::vec2& a, const math::vec2& b, const math::vec2& c)
    {
        return orient2d(reinterpret_cast<const math::real_number_t*>(&a),
                   reinterpret_cast<const math::real_number_t*>(&b),
                   reinterpret_cast<const math::real_number_t*>(&c))
            == math::real_number_t(0.0);
    }

    bool Between(const math::vec2& a, const math::vec2& b, const math::vec2& c)
    {
        math::vec2 ba;
        math::vec2 ca;

        /* If ab not vertical check betweenness on x; else on y. */
        if (a.x() != b.x())
            return ((a.x() <= c.x()) && (c.x() <= b.x())) || ((a.x() >= c.x()) && (c.x() >= b.x()));
        else
            return ((a.y() <= c.y()) && (c.y() <= b.y())) || ((a.y() >= c.y()) && (c.y() >= b.y()));
    }

    // Check if segment ab is parralel to and overlaps with segment ab. The segments overlap
    // iff an endpoint of one lies  between the endpoints of the other.
    //
    // Note that an endpoint is returned as the point of intersection p. It is possible to have
    // the midpoint of overlap returned but we dont do that (not necessary).
    //
    // Note that the case where two collinear segments share just one point, an endpoint of
    // each, is classified as 'e' in this scheme, although 'v' might be more appropriate in some contexts
    char ParallelInt(const math::vec2& a, const math::vec2& b, const math::vec2& c, const math::vec2& d, math::vec2& p)
    {
        // If c is not collinear with ab, then the parallel segments ab and cd do not intersect
        if (!Collinear(a, b, c)) {
            return '0';
        }

        if (Between(a, b, c)) {
            p = c;
            return 'e'; // The segments collinearly overlap, sharing a point; 'e' stands for 'edge.'
        }

        if (Between(a, b, d)) {
            p = d;
            return 'e';
        }

        if (Between(c, d, a)) {
            p = a;
            return 'e';
        }

        if (Between(c, d, b)) {
            p = b;
            return 'e';
        }

        return '0';
    }

    char SegSegInt(const math::vec2& a, const math::vec2& b, const math::vec2& c, const math::vec2& d, math::vec2& p)
    {
        char code = ' ? '; // Return char characterizing intersection.

        // Denominator of equations.
        math::real_number_t denom = a.x() * (d.y() - c.y()) + b.x() * (c.y() - d.y()) + d.x() * (b.y() - a.y()) + c.x() * (a.y() - b.y());

        // If denom is zero, then segments are parallel: handle separately.
        if (denom == math::real_number_t(0.0)) {
            return ParallelInt(a, b, c, d, p);
        }

        //
        // NOTE: Checking the 'v' case is done with "num" rather than with "s" and "t" after division;
        // this skirts possible floating-point inaccuracy in the division.
        //

        // Numerator of equations.
        math::real_number_t num = a.x() * (d.y() - c.y()) + c.x() * (a.y() - d.y()) + d.x() * (c.y() - a.y());

        if ((num == math::real_number_t(0.0)) || (num == denom)) {
            code = 'v'; // An endpoint of one segment is on the other segment, but 'e' doesn't hold; 'v' stands for 'vertex.'
        }

        // The parameter of the parametric-equation for edge ab.
        math::real_number_t s = num / denom;
        num = -(a.x() * (c.y() - b.y()) + b.x() * (a.y() - c.y()) + c.x() * (b.y() - a.y()));

        if ((num == math::real_number_t(0.0)) || (num == denom)) {
            code = 'v';
        }

        // The parameter of the parametric-equation for edge cd.
        math::real_number_t t = num / denom;

        // The check for proper intersection is 0 < s < 1 and 0 < t < 1; the reverse inequalities yield no intersection

        if ((math::real_number_t(0.0) < s) && (s < math::real_number_t(1.0)) && //
            (math::real_number_t(0.0) < t) && (t < math::real_number_t(1.0))) {
            code = '1'; // The segments intersect properly (i.e., they share a point and neither 'v' nor 'e' holds); '1' stands for TRUE
        } else if ((math::real_number_t(0.0) > s) || (s > math::real_number_t(1.0)) || //
            (math::real_number_t(0.0) > t) || (t > math::real_number_t(1.0))) {
            code = '0'; // The segments do not intersect (i.e., they share no points); '0' stands for FALSE
        }

        //p.x() = a.x() + s * (b.x() - a.x());
        //p.y() = a.y() + s * (b.y() - a.y());
        p = math::vec2(a.x() + s * (b.x() - a.x()), a.y() + s * (b.y() - a.y()));
        return code;
    }

    // http://cs.haifa.ac.il/~gordon/plane.pdf
    void polygon_normal(math::vec3& normal, const math::vec3* vertices, const int num_vertices);

    // Compute a polygon's plane coefficients (i.e. normal and d parameters).
    // The computed normal is not normalized. This function returns the largest component of the normal.
    int PlaneCoeff(math::vec3& polygonNormal, math::real_number_t& d, const math::vec3* polygonVertices, const int polygonVertexCount)
    {
        // compute normal
        polygon_normal(polygonNormal, polygonVertices, polygonVertexCount);

        d = math::dot_product(polygonVertices[0], polygonNormal);

        math::real_number_t largestComponent(0.0);
        math::real_number_t tmp(0.0);
        int largestComponentIdx = 0;

        for (int i = 0; i < 3; ++i) {
            tmp = math::absolute_value(polygonNormal[i]);
            if (tmp > largestComponent) {
                largestComponent = tmp;
                largestComponentIdx = i;
            }
        }

        return largestComponentIdx;
    }

    char SegPlaneInt(math::vec3& p, int& planeNormalLargestComponent, const math::vec3& q, const math::vec3& r, const math::vec3* polygonVertices, const int polygonVertexCount)
    {
        double num, denom, t;
        int i;

        math::vec3 planeNormal;
        math::real_number_t planeDCoeff;
        planeNormalLargestComponent = PlaneCoeff(planeNormal, planeDCoeff, polygonVertices, polygonVertexCount);

        num = planeDCoeff - math::dot_product(q, planeNormal);
        const math::vec3 rq = (r - q);
        denom = math::dot_product(rq, planeNormal);

        if (denom == 0.0) { /// Segment is parallel to plane.
            if (num == 0.0) { // 'q' is on plane.
                return 'p'; // The segment lies wholly within the plane
            } else {
                return '0';
            }
        } else {
            t = num / denom;
        }

        for (int i = 0; i < 3; ++i) {
            p[i] = q[i] + t * (r[i] - q[i]);
        }

        if ((0.0 < t) && (t < 1.0)) {
            return '1'; // The segment intersects the plane, and none of {p, q, r} hold
        } else if (num == 0) // t==0
        {
            return 'q'; // The (first) q endpoint is on the plane (but not 'p').
        } else if (num == denom) // t==1
        {
            return 'r'; // The (second) r endpoint is on the plane (but not 'p').
        } else {
            return '0'; // The segment lies strictly to one side or the other of the plane
        }
    }

    char InPoly2D(const math::vec2 p, const math::vec2* polygonVertices, const int polygonVertexCount)
    {
    }

    char InPoly3D(const math::vec3 p, const math::vec3* polygonVertices, const int polygonVertexCount, const int planeNormalLargestComponent)
    {
        /* Project out coordinate m in both p and the triangular face */

        int k = 0;
        math::vec2 pp; /*projected p */
        for (int j = 0; j < 3; j++) { // for each component
            if (j != planeNormalLargestComponent) { /* skip largest coordinate */
                pp[k] = p[j];
            }
            k++;
        }

        std::vector<math::vec2> polygonVertices2d(polygonVertexCount, math::vec3());
        k = 0;

        for (int i = 0; i < polygonVertexCount; ++i) { // for each vertex
            math::vec2& Tp = polygonVertices2d[i];
            for (int j = 0; j < 3; j++) { // for each component
                if (j != planeNormalLargestComponent) { /* skip largest coordinate */

                    Tp[k] = polygonVertices[i][j];
                }
                k++;
            }
        }

        return InPoly2D(pp, polygonVertices2d.data(), polygonVertices2d.size());
    }

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

    template <typename T>
    bool intersect_bounding_boxes(const bounding_box_t<math::vec3_<T>>& a, const bounding_box_t<math::vec3_<T>>& b)
    {
        return (a.minimum().x() <= b.maximum().x() && a.maximum().x() >= b.minimum().x()) && //
            (a.minimum().y() <= b.maximum().y() && a.maximum().y() >= b.minimum().y()) && //
            (a.minimum().z() <= b.maximum().z() && a.maximum().z() >= b.minimum().z());
    }

    bool point_in_bounding_box(const math::vec2& point, const bounding_box_t<math::vec2>& bbox);

    bool point_in_bounding_box(const math::vec3& point, const bounding_box_t<math::vec3>& bbox);

    void polygon_plane_d_param(math::real_number_t& d, const math::vec3& plane_normal, const math::vec3* polygon_vertices, const int num_polygon_vertices);

    void project_point(math::vec2& projected_point, const math::vec3& point, const math::vec3& u, const math::vec3& v);

    void polygon_span(math::vec3& span, math::real_number_t& span_length, const math::vec3* vertices, const int num_vertices);

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
        math::real_number_t& t,
        // plane
        const math::vec3& normal,
        math::real_number_t& distance,
        // segment
        const math::vec3& source,
        const math::vec3& target);

} // namespace geom {
} // namespace mcut {

#endif // MCUT_GEOM_H_
