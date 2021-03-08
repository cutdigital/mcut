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

#include "mcut/internal/geom.h"

#if defined(MCUT_USE_SHEWCHUK_EXACT_PREDICATES)
#include "mcut/../source/shewchuk_predicates.c"
#else // #if defined(MCUT_USE_SHEWCHUK_EXACT_PREDICATES)

// basically the Shewchuk's orient2dfast()
mcut::math::real_number_t orient2d(const mcut::math::real_number_t* pa, const mcut::math::real_number_t* pb, const mcut::math::real_number_t* pc)
{
    const mcut::math::real_number_t acx = pa[0] - pc[0];
    const mcut::math::real_number_t bcx = pb[0] - pc[0];
    const mcut::math::real_number_t acy = pa[1] - pc[1];
    const mcut::math::real_number_t bcy = pb[1] - pc[1];

    return (acx * bcy) - (acy * bcx);
}

// basically the Shewchuk's orient3dfast()
mcut::math::real_number_t orient3d(const mcut::math::real_number_t* pa, const mcut::math::real_number_t* pb, const mcut::math::real_number_t* pc, const mcut::math::real_number_t* pd)
{
    const mcut::math::real_number_t adx = pa[0] - pd[0];
    const mcut::math::real_number_t bdx = pb[0] - pd[0];
    const mcut::math::real_number_t cdx = pc[0] - pd[0];
    const mcut::math::real_number_t ady = pa[1] - pd[1];
    const mcut::math::real_number_t bdy = pb[1] - pd[1];
    const mcut::math::real_number_t cdy = pc[1] - pd[1];
    const mcut::math::real_number_t adz = pa[2] - pd[2];
    const mcut::math::real_number_t bdz = pb[2] - pd[2];
    const mcut::math::real_number_t cdz = pc[2] - pd[2];

    return (adx * ((bdy * cdz) - (bdz * cdy))) + (bdx * ((cdy * adz) - (cdz * ady))) + (cdx * ((ady * bdz) - (adz * bdy)));
}

#endif // #if defined(MCUT_USE_SHEWCHUK_EXACT_PREDICATES)

namespace mcut {
namespace geom {

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

    // Intersect a line segment with a plane
    //
    // Return values:
    // 'p': The segment lies wholly within the plane.
    // 'q': The(first) q endpoint is on the plane (but not 'p').
    // 'r' : The(second) r endpoint is on the plane (but not 'p').
    // '0' : The segment lies strictly to one side or the other of the plane.
    // '1': The segment intersects the plane, and none of {p, q, r} hold.
    char SegPlaneInt(math::vec3& p, int& planeNormalLargestComponent, const math::vec3& q, const math::vec3& r, const math::vec3* polygonVertices, const int polygonVertexCount)
    {
        math::real_number_t num, denom, t;
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

    // Count the number ray crossings to determine if a point 'q' lies inside or outside a given polygon.
    //
    // Return values:
    // 'i': q is strictly interior
    // 'o': q is strictly exterior (outside).
    // 'e': q is on an edge, but not an endpoint.
    // 'v': q is a vertex.
    char InPoly2D(const math::vec2 q, math::vec2* polygonVertices, const int polygonVertexCount)
    {
        int i, il; /* point index; il = i—1 mod n */
        int d; /* dimension index */
        double x; /* x intersection ofe with ray */
        int Rcross = 0; /* number of right edge/ray crossings */
        int Lcross = 0; /* number ofleft edge/ray crossings */
        bool Rstrad, Lstrad; /*flags indicating the edge strads the x axis. */

        /* Shift so that q is the origin. Note this destroys the polygon.
    This is done for pedagogical clarity. */
        for (i = 0; i < polygonVertexCount; i++) {
            for (d = 0; d < 2; d++) {
                polygonVertices[i][d] = polygonVertices[i][d] - q[d];
            }
        }

        /* For each edge e = (i—lj), see if crosses ray. */
        for (i = 0; i < polygonVertexCount; i++) {

            /* First check if q = (0, 0) is a vertex. */
            if (polygonVertices[i].x() == 0 && polygonVertices[i].y() == 0) {
                return 'v';
            }

            il = (i + polygonVertexCount - 1) % polygonVertexCount;

            /* Check if e straddles x axis, with bias above/below.*/

            // Rstrad is TRUE iff one endpoint of e is strictly above the x axis and the other is not (i.e., the other is on or below)
            Rstrad = (polygonVertices[i].y() > 0) != (polygonVertices[il].y() > 0);
            Lstrad = (polygonVertices[i].y() < 0) != (polygonVertices[il].y() < 0);

            if (Rstrad || Lstrad) {
                /* Compute intersection ofe with x axis. */

                // the computation of x is needed whenever either of these straddle variables is TRUE, which
                // only excludes edges passing through q = (0, 0) (and incidentally protects against division by 0).
                x = (polygonVertices[i].x() * polygonVertices[il].y() - polygonVertices[il].x() * polygonVertices[i].y())
                    / (polygonVertices[il].y() - polygonVertices[i].y());
                if (Rstrad && x > 0) {
                    Rcross++;
                }
                if (Lstrad && x < 0) {
                    Lcross++;
                }
            } /* end straddle computation*/
        } // end for

        /* q on an edge if L/Rcross counts are not the same parity.*/
        if ((Rcross % 2) != (Lcross % 2)) {
            return 'e';
        }

        /* q inside iff an odd number of crossings. */
        if ((Rcross % 2) == 1) {
            return 'i';
        } else {
            return 'o';
        }
    }

    char InPoly3D(const math::vec3 p, const math::vec3* polygonVertices, const int polygonVertexCount, const int planeNormalLargestComponent)
    {
        /* Project out coordinate m in both p and the triangular face */

        int k = 0;
        math::vec2 pp; /*projected p */
        for (int j = 0; j < 3; j++) { // for each component
            if (j != planeNormalLargestComponent) { /* skip largest coordinate */
                pp[k] = p[j];
                k++;
            }
        }

        std::vector<math::vec2> polygonVertices2d(polygonVertexCount, math::vec3());

        for (int i = 0; i < polygonVertexCount; ++i) { // for each vertex
            math::vec2& Tp = polygonVertices2d[i];
            k = 0;
            for (int j = 0; j < 3; j++) { // for each component
                if (j != planeNormalLargestComponent) { /* skip largest coordinate */

                    Tp[k] = polygonVertices[i][j];
                    k++;
                }
            }
        }

        return InPoly2D(pp, polygonVertices2d.data(), polygonVertices2d.size());
    }

    bool point_in_bounding_box(const math::vec2& point, const bounding_box_t<math::vec2>& bbox)
    {
        if ((point.x() < bbox.m_minimum.x() || point.x() > bbox.m_maximum.x()) || //
            (point.y() < bbox.m_minimum.y() || point.y() > bbox.m_maximum.y())) {
            return false;
        } else {
            return true;
        }
    }

    bool point_in_bounding_box(const math::vec3& point, const bounding_box_t<math::vec3>& bbox)
    {
        if ((point.x() < bbox.m_minimum.x() || point.x() > bbox.m_maximum.x()) || //
            (point.y() < bbox.m_minimum.y() || point.y() > bbox.m_maximum.y()) || //
            (point.z() < bbox.m_minimum.z() || point.z() > bbox.m_maximum.z())) { //
            return false;
        } else {
            return true;
        }
    }

    // http://cs.haifa.ac.il/~gordon/plane.pdf
    void polygon_normal(math::vec3& normal, const math::vec3* vertices, const int num_vertices)
    {
        normal = math::vec3(0.0);
        for (int i = 0; i < num_vertices; ++i) {
            normal = normal + cross_product(vertices[i] - vertices[0], vertices[(i + 1) % num_vertices] - vertices[0]);
        }
        normal = normalize(normal);
    }

    void polygon_plane_d_param(math::real_number_t& d, const math::vec3& plane_normal, const math::vec3* polygon_vertices, const int num_polygon_vertices)
    {
        math::real_number_t frac = 1.0 / num_polygon_vertices;
        math::vec3 p = polygon_vertices[0];
        for (int i = 1; i < num_polygon_vertices; ++i) {
            p = p + polygon_vertices[i];
        }

        p = p * frac;

        d = (dot_product(p, plane_normal));
    }

    void project_point(math::vec2& projected_point, const math::vec3& point, const math::vec3& u, const math::vec3& v)
    {
        MCUT_ASSERT(length(u) > math::real_number_t(0.0));
        MCUT_ASSERT(length(v) > math::real_number_t(0.0));

        projected_point = math::vec2(dot_product(point, u), dot_product(point, v));
    }

    void polygon_span(math::vec3& span, math::real_number_t& span_length, const math::vec3* vertices, const int num_vertices)
    {
        MCUT_ASSERT(num_vertices >= 3);

        span = math::vec3(0.0);
        span_length = math::real_number_t(0.0);

        for (int i = 0; i < num_vertices; ++i) {
            for (int j = 0; j < i; ++j) {
                const math::vec3 diff = vertices[i] - vertices[j];
                const math::real_number_t len = length(diff);
                if (len > span_length) {
                    span_length = len;
                    span = diff;
                }
            }
        }
    }

    inline long direction(const math::vec2& a, const math::vec2& b, const math::vec2& c)
    {
        math::real_number_t acx = a.x() - c.x();
        math::real_number_t bcx = b.x() - c.x();
        math::real_number_t acy = a.y() - c.y();
        math::real_number_t bcy = b.y() - c.y();
        math::real_number_t result = acx * bcy - acy * bcx;
        return (result == math::real_number_t(0.0) ? 0 : (result > math::real_number_t(0.0) ? 1 : -1));
    }

    inline orientation_t orientation(const math::vec2& a, const math::vec2& b, const math::vec2& c)
    {
        const long result = direction(a, b, c);

        sign_t side = ON_ORIENTED_BOUNDARY;

        if (result > 0) {
            side = ON_POSITIVE_SIDE;
        } else if (result < 0) {
            side = ON_NEGATIVE_SIDE;
        }
        return side;
    }

    bool point_on_segment(const math::vec2& pi, const math::vec2& pj, const math::vec2& pk)
    {
        return (math::min(pi.x(), pj.x()) <= pk.x() && //
            pk.x() <= math::max(pi.x(), pj.x()) && //
            math::min(pi.y(), pj.y()) <= pk.y() && //
            pk.y() <= math::max(pi.y(), pj.y()));
    }

    // segment 0: ab
    // segment 1: cd
    bool segments_intersect(const math::vec2& a, const math::vec2& b, const math::vec2& c, const math::vec2& d)
    {
        orientation_t orient_a = orientation(c, d, a);
        orientation_t orient_b = orientation(c, d, b);
        orientation_t orient_c = orientation(a, b, c);
        orientation_t orient_d = orientation(a, b, d);

        bool result = false;

        if (((orient_a == ON_POSITIVE_SIDE && orient_b == ON_NEGATIVE_SIDE) || (orient_a == ON_NEGATIVE_SIDE && orient_b == ON_POSITIVE_SIDE)) && //
            ((orient_c == ON_POSITIVE_SIDE && orient_d == ON_NEGATIVE_SIDE) || (orient_c == ON_NEGATIVE_SIDE && orient_d == ON_POSITIVE_SIDE))) {
            result = true;
        } else if (orient_a == ON_ORIENTED_BOUNDARY && point_on_segment(c, d, a)) {
            result = true;
        } else if (orient_b == ON_ORIENTED_BOUNDARY && point_on_segment(c, d, b)) {
            result = true;
        } else if (orient_c == ON_ORIENTED_BOUNDARY && point_on_segment(a, b, c)) {
            result = true;
        } else if (orient_d == ON_ORIENTED_BOUNDARY && point_on_segment(a, b, d)) {
            result = true;
        }

        return result;
    }

    bool point_in_polygon(const math::vec3& point, const math::vec3& polygon_normal, const math::vec3* vertices, const int num_vertices)
    {
        MCUT_ASSERT(vertices != nullptr);
        MCUT_ASSERT(num_vertices >= 3);

        // bounding_box_t<math::vec3 > bbox3;
        // make_bbox(bbox3, vertices, num_vertices);

        //if (!point_in_bounding_box(point, bbox3)) { // TODO: this check should be done outside of this function
        //    return false;
        //}

        std::unique_ptr<math::vec2[]> projected_vertices = std::unique_ptr<math::vec2[]>(new math::vec2[num_vertices]);

        //math::vec3 normal;
        // polygon_normal(normal, vertices, num_vertices);

        math::vec3 span;
        math::real_number_t span_length;
        polygon_span(span, span_length, vertices, num_vertices);

        const math::vec3 u = normalize(span); // first unit vector on the plane
        const math::vec3 v = normalize(cross_product(u, polygon_normal)); // second unit vector just calculate

        for (int i = 0; i < num_vertices; ++i) {
            project_point(projected_vertices[i], vertices[i], u, v);
        }

        bounding_box_t<math::vec2> bbox2;
        make_bbox(bbox2, projected_vertices.get(), num_vertices);

        math::vec2 projected_point;
        project_point(projected_point, point, u, v);

        if (!point_in_bounding_box(projected_point, bbox2)) {
            return false;
        }

        //create a ray (segment) starting from the given point,
        // and to the point outside of polygon.
        math::vec2 outside(bbox2.m_minimum.x() - math::real_number_t(1.0), bbox2.m_minimum.y());

        int intersections = 0;

        // check intersections between the ray and every side of the polygon.
        for (int i = 0; i < num_vertices - 1; ++i) {
            if (segments_intersect(projected_point, outside, projected_vertices[i], projected_vertices[i + 1])) {
                intersections++;
            }
        }

        // check the last line
        if (segments_intersect(projected_point, outside, projected_vertices[num_vertices - 1], projected_vertices[0])) {
            intersections++;
        }

        return (intersections % 2 != 0);
    }

    bool intersect_plane_with_segment(
        math::vec3& point,
        math::real_number_t& t,
        // plane
        const math::vec3& normal,
        math::real_number_t& distance,
        // segment
        const math::vec3& source,
        const math::vec3& target)
    {
        MCUT_ASSERT(!(target == source));
        MCUT_ASSERT(length(normal) != math::real_number_t(0.0));

        // Compute the t value for the directed line ab intersecting the plane
        const math::vec3 dir = target - source;
        const math::real_number_t nom = (distance - dot_product(normal, source));
        const math::real_number_t denom = dot_product(normal, dir);
        bool result = false;

#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        if (!(0 == denom)) {
#else
        if (!math::real_number_t::is_zero(denom)) {
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
            t = nom / denom;
            // If t in [0..1] compute and return intersection point
            if (t >= math::real_number_t(0.0) && t <= math::real_number_t(1.0)) {
                point = source + dir * t;
                result = true;
            }
        }

        return result;
    }

} // namespace mcut {
} //namespace geom {
