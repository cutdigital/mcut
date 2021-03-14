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
#if 0
    void polygon_normal(math::vec3& normal, const math::vec3* vertices, const int num_vertices)
    {
        normal = math::vec3(0.0);
        for (int i = 0; i < num_vertices; ++i) {
            normal = normal + cross_product(vertices[i] - vertices[0], vertices[(i + 1) % num_vertices] - vertices[0]);
        }
        normal = normalize(normal);
    }
#endif

    int compute_polygon_plane_coefficients(
        math::vec3& normal,
        math::real_number_t& d_coeff,
        const math::vec3* polygon_vertices,
        const int polygon_vertex_count)
    {
        // compute polygon normal (http://cs.haifa.ac.il/~gordon/plane.pdf)
        normal = math::vec3(0.0);
        for (int i = 1; i < polygon_vertex_count - 1; ++i) {
            normal = normal + cross_product(polygon_vertices[i] - polygon_vertices[0], polygon_vertices[(i + 1) % polygon_vertex_count] - polygon_vertices[0]);
        }

        // In our calculations we need the normal be be of unit length so that the d-coeff
        // represents the distance of the plane from the origin
        //normal = math::normalize(normal);

        d_coeff = math::dot_product(polygon_vertices[0], normal);

        math::real_number_t largest_component(0.0);
        math::real_number_t tmp(0.0);
        int largest_component_idx = 0;

        for (int i = 0; i < 3; ++i) {
            tmp = math::absolute_value(normal[i]);
            if (tmp > largest_component) {
                largest_component = tmp;
                largest_component_idx = i;
            }
        }

        return largest_component_idx;
    }

    // Intersect a line segment with a plane
    //
    // Return values:
    // 'p': The segment lies wholly within the plane.
    // 'q': The(first) q endpoint is on the plane (but not 'p').
    // 'r' : The(second) r endpoint is on the plane (but not 'p').
    // '0' : The segment lies strictly to one side or the other of the plane.
    // '1': The segment intersects the plane, and none of {p, q, r} hold.
    char compute_segment_plane_intersection(
        math::vec3& p,
        const math::vec3& normal,
        const math::real_number_t& d_coeff,
        const math::vec3& q,
        const math::vec3& r)
    {

        //math::vec3 planeNormal;
        //math::real_number_t planeDCoeff;
        //planeNormalLargestComponent = compute_polygon_plane_coefficients(planeNormal, planeDCoeff, polygonVertices, polygonVertexCount);

        math::real_number_t num = d_coeff - math::dot_product(q, normal);
        const math::vec3 rq = (r - q);
        math::real_number_t denom = math::dot_product(rq, normal);

        if (denom == 0.0 /* Segment is parallel to plane.*/) {
            if (num == 0.0) { // 'q' is on plane.
                return 'p'; // The segment lies wholly within the plane
            } else {
                return '0';
            }
        }

        math::real_number_t t = num / denom;

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
    char compute_point_in_polygon_test(
        const math::vec2& q,
        const math::vec2* polygon_vertices,
        const int polygon_vertex_count)
    {
        std::vector<math::vec2> vertices(polygon_vertex_count, math::vec2());

        // Shift so that q is the origin. Note this destroys the polygon.
        // This is done for pedagogical clarity.
        for (int i = 0; i < polygon_vertex_count; i++) {
            for (int d = 0; d < 2; d++) {
                vertices[i][d] = polygon_vertices[i][d] - q[d];
            }
        }

        int Rcross = 0; /* number of right edge/ray crossings */
        int Lcross = 0; /* number ofleft edge/ray crossings */

        /* For each edge e = (i—lj), see if crosses ray. */
        for (int i = 0; i < polygon_vertex_count; i++) {

            /* First check if q = (0, 0) is a vertex. */
            if (vertices[i].x() == 0 && vertices[i].y() == 0) {
                return 'v';
            }

            int il = (i + polygon_vertex_count - 1) % polygon_vertex_count;

            // Check if e straddles x axis, with bias above/below.

            // Rstrad is TRUE iff one endpoint of e is strictly above the x axis and the other is not (i.e., the other is on or below)
            bool Rstrad = (vertices[i].y() > 0) != (vertices[il].y() > 0);
            bool Lstrad = (vertices[i].y() < 0) != (vertices[il].y() < 0);

            if (Rstrad || Lstrad) {
                /* Compute intersection of e with x axis. */

                // The computation of x is needed whenever either of these straddle variables is TRUE, which
                // only excludes edges passing through q = (0, 0) (and incidentally protects against division by 0).
                math::real_number_t x = (vertices[i].x() * vertices[il].y() - vertices[il].x() * vertices[i].y())
                    / (vertices[il].y() - vertices[i].y());
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

    char compute_point_in_polygon_test(
        const math::vec3& p,
        const math::vec3* polygon_vertices,
        const int polygon_vertex_count,
        const int polygon_plane_normal_largest_component)
    {
        /* Project out coordinate m in both p and the triangular face */

        int k = 0;
        math::vec2 pp; /*projected p */
        for (int j = 0; j < 3; j++) { // for each component
            if (j != polygon_plane_normal_largest_component) { /* skip largest coordinate */
                pp[k] = p[j];
                k++;
            }
        }

        std::vector<math::vec2> polygon_vertices2d(polygon_vertex_count, math::vec2());

        for (int i = 0; i < polygon_vertex_count; ++i) { // for each vertex
            math::vec2& Tp = polygon_vertices2d[i];
            k = 0;
            for (int j = 0; j < 3; j++) { // for each component
                if (j != polygon_plane_normal_largest_component) { /* skip largest coordinate */

                    Tp[k] = polygon_vertices[i][j];
                    k++;
                }
            }
        }

        return compute_point_in_polygon_test(pp, polygon_vertices2d.data(), (int)polygon_vertices2d.size());
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
} // namespace mcut {
} //namespace geom {
