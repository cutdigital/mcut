/***************************************************************************
 *  This file is part of the MCUT project, which is comprised of a library 
 *  for surface mesh cutting, example programs and test programs.
 * 
 *  Copyright (C) 2024 CutDigital Enterprise Ltd
 *  
 *  MCUT is dual-licensed software that is available under an Open Source 
 *  license as well as a commercial license. The Open Source license is the 
 *  GNU Lesser General Public License v3+ (LGPL). The commercial license 
 *  option is for users that wish to use MCUT in their products for commercial 
 *  purposes but do not wish to release their software under the LGPL. 
 *  Email <contact@cut-digital.com> for further information.
 *
 *  You may not use this file except in compliance with the License. A copy of 
 *  the Open Source license can be obtained from
 *
 *      https://www.gnu.org/licenses/lgpl-3.0.en.html.
 *
 *  For your convenience, a copy of this License has been included in this
 *  repository.
 *
 *  MCUT is distributed in the hope that it will be useful, but THE SOFTWARE IS 
 *  PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR 
 *  A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF 
 *  OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author(s):
 *
 *    Floyd M. Chitalu    CutDigital Enterprise Ltd.
 *
 **************************************************************************/

#include "mcut/internal/math.h"
#include <algorithm> // std::sort
#include <cstdlib>
#include <tuple> // std::make_tuple std::get<>


    scalar_t square_root(const scalar_t& number, double 
        #ifdef MCUT_WITH_ARBITRARY_PRECISION_NUMBERS
        multiplier
    #endif
    )
    {
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        return std::sqrt(number);
#else
		const double dequantized = scalar_t::dequantize(number, multiplier); // to native user coordinates/from rational coordinates
		const double sqrt_val = std::sqrt(dequantized);
		const scalar_t quantized = scalar_t::quantize(sqrt_val, multiplier); // to rational coordinates
		return quantized;
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    }

    scalar_t absolute_value(const scalar_t& number)
    {
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
        return std::fabs(number);
#else
		return scalar_t::abs(number);
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    }

    sign_t sign(const scalar_t& number)
    {
		int s = (scalar_t(0) < number) - (number < scalar_t(0));
        sign_t result = sign_t::ZERO;
        if (s > 0) {
            result = sign_t::POSITIVE;
        } else if (s < 0) {
            result = sign_t::NEGATIVE;
        }
        return result;

    }

    std::ostream& operator<<(std::ostream& os, const vec3& v)
    {
        return os << static_cast<scalar_t>(v.x()) << ", " << static_cast<scalar_t>(v.y()) << ", " << static_cast<scalar_t>(v.z());
    }

    bool operator==(const vec3& a, const vec3& b)
    {
        return (a.x() == b.x()) && (a.y() == b.y()) && (a.z() == b.z());
    }

    /*vec3 cross_product(const vec3& a, const vec3& b)
    {
        return vec3(
            a.y() * b.z() - a.z() * b.y(),
            a.z() * b.x() - a.x() * b.z(),
            a.x() * b.y() - a.y() * b.x());
    }*/


    scalar_t orient2d(const vec2& pa, const vec2& pb, const vec2& pc)
    {
		const scalar_t pa_[2] = {static_cast<scalar_t>(pa.x()), static_cast<scalar_t>(pa.y())};
		const scalar_t pb_[2] = {static_cast<scalar_t>(pb.x()), static_cast<scalar_t>(pb.y())};
		const scalar_t pc_[2] = {static_cast<scalar_t>(pc.x()), static_cast<scalar_t>(pc.y())};

        return ::orient2d(pa_, pb_, pc_); // shewchuk predicate
    }

    scalar_t orient3d(const vec3& pa, const vec3& pb, const vec3& pc,
        const vec3& pd)
    {
        const scalar_t pa_[3] = { static_cast<scalar_t>(pa.x()), static_cast<scalar_t>(pa.y()), static_cast<scalar_t>(pa.z()) };
        const scalar_t pb_[3] = { static_cast<scalar_t>(pb.x()), static_cast<scalar_t>(pb.y()), static_cast<scalar_t>(pb.z()) };
        const scalar_t pc_[3] = { static_cast<scalar_t>(pc.x()), static_cast<scalar_t>(pc.y()), static_cast<scalar_t>(pc.z()) };
        const scalar_t pd_[3] = { static_cast<scalar_t>(pd.x()), static_cast<scalar_t>(pd.y()), static_cast<scalar_t>(pd.z()) };

        return ::orient3d(pa_, pb_, pc_, pd_); // shewchuk predicate
    }

#if 0
    void polygon_normal(vec3& normal, const vec3* vertices, const int num_vertices)
    {
      normal = vec3(0.0);
      for (int i = 0; i < num_vertices; ++i) {
        normal = normal + cross_product(vertices[i] - vertices[0], vertices[(i + 1) % num_vertices] - vertices[0]);
      }
      normal = normalize(normal);
    }
#endif

    int compute_polygon_plane_coefficients(vec3& normal, scalar_t& d_coeff,
										   const vec3* polygon_vertices,
										   const int polygon_vertex_count,
										   const double
#ifdef MCUT_WITH_ARBITRARY_PRECISION_NUMBERS 
        multiplier
    #endif
    )
    {
        // compute polygon normal using Newell's Formula 
            for(size_t i = 0; i < polygon_vertex_count; ++i)
			{
				size_t j = (i + 1) % polygon_vertex_count;
				const auto& current = polygon_vertices[i];
				const auto& next = polygon_vertices[j];

				normal.x() += (current.y() - next.y()) * (current.z() + next.z());
				normal.y() += (current.z() - next.z()) * (current.x() + next.x());
				normal.z() += (current.x() - next.x()) * (current.y() + next.y());
			}
#if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
			if(std::isnan(normal.x()) || std::isnan(normal.y()) || std::isnan(normal.z()) ||
			   squared_length(normal) < 1e-9)
			{
                #if 0
				{
					std::ofstream f("bad-face.obj");
					for(auto i = 0; i < polygon_vertex_count; ++i)
					{
						f << "v " << polygon_vertices[i].x() << " " << polygon_vertices[i].y()
						  << " " << polygon_vertices[i].z() << std::endl;
					}

					f << "f ";
					for(auto i = 0; i < polygon_vertex_count; ++i)
					{
						f << i + 1 << " ";
					}

					f << std::endl;
				}
                #endif
				normal = vec3(0.0);
				return 0;
			}
					 
#else
#if 0	
		for(int i = 1; i < polygon_vertex_count - 1; ++i)
		{
			auto vec0 = polygon_vertices[i] - polygon_vertices[0];

			auto vec1 = polygon_vertices[(i + 1) % polygon_vertex_count] - polygon_vertices[0];
			if(squared_length(vec0) > scalar_t::zero() && squared_length(vec1) > scalar_t::zero())
			{
				vec3_<double> vec0_(scalar_t::dequantize(vec0[0], multiplier),
									scalar_t::dequantize(vec0[1], multiplier),
									scalar_t::dequantize(vec0[2], multiplier));
				vec3_<double> vec1_(scalar_t::dequantize(vec1[0], multiplier),
									scalar_t::dequantize(vec1[1], multiplier),
									scalar_t::dequantize(vec1[2], multiplier));

				auto cross_ = cross_product(vec0_, vec1_);
				if(squared_length(cross_) > 1e-7)
				{
					auto cross_n = normalize(cross_);
					vec3 cross = vec3(scalar_t::quantize(cross_n[0], multiplier),
									  scalar_t::quantize(cross_n[1], multiplier),
									  scalar_t::quantize(cross_n[2], multiplier));
					normal = normal + cross;
					break;
				}
			}
		}
#endif
#endif
        
        if(squared_length(normal) == scalar_t(0))
			{
			return 0 ;
			}
        
        normal = normalize(normal
#ifdef MCUT_WITH_ARBITRARY_PRECISION_NUMBERS
            , multiplier
#endif
        );

        /*normal = normal / (double)(crossprods);*/ // mean

        // In our calculations we need the normal be be of unit length so that the d-coeff
        // represents the distance of the plane from the origin
        // normal = normalize(normal);

        d_coeff = dot_product(polygon_vertices[0], normal);

        scalar_t largest_component(0.0);
        scalar_t tmp(0.0);
        int largest_component_idx = 0;

        for (int i = 0; i < 3; ++i) {
            tmp = absolute_value(normal[i]);
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
    char compute_segment_plane_intersection(vec3& p, const vec3& normal, const scalar_t& d_coeff,
        const vec3& q, const vec3& r)
    {

        // vec3 planeNormal;
        // scalar_t planeDCoeff;
        // planeNormalLargestComponent = compute_polygon_plane_coefficients(planeNormal, planeDCoeff, polygonVertices,
        // polygonVertexCount);

        scalar_t num = d_coeff - dot_product(q, normal);
        const vec3 rq = (r - q);
        scalar_t denom = dot_product(rq, normal);

        if (denom == scalar_t(0.0) /* Segment is parallel to plane.*/) {
            if (num == scalar_t(0.0)) { // 'q' is on plane.
                return 'p'; // The segment lies wholly within the plane
            } else {
                return '0';
            }
        }

        scalar_t t = num / denom;

        for (int i = 0; i < 3; ++i) {
            p[i] = q[i] + t * (r[i] - q[i]);
        }

        if ((scalar_t(0.0) < t) && (t < scalar_t(1.0))) {
            return '1'; // The segment intersects the plane, and none of {p, q, r} hold
        } else if (num == scalar_t(0.0)) // t==0
        {
            return 'q'; // The (first) q endpoint is on the plane (but not 'p').
        } else if (num == denom) // t==1
        {
            return 'r'; // The (second) r endpoint is on the plane (but not 'p').
        } else {
            return '0'; // The segment lies strictly to one side or the other of the plane
        }
    }

    bool determine_three_noncollinear_vertices(int& i, int& j, int& k, const std::vector<vec3>& polygon_vertices,

        const vec3& polygon_normal,
											   const int polygon_normal_largest_component,
											   const double multiplier)
    {
        const int polygon_vertex_count = (int)polygon_vertices.size();
        MCUT_ASSERT(polygon_vertex_count >= 3);

        /*{ 
            std::ofstream f("poly.obj");

            for(int v = 0; v < polygon_vertices.size(); ++v)
			{
				f << "v " << scalar_t::dequantize(polygon_vertices[v][0], multiplier) << " "
				  << scalar_t::dequantize(polygon_vertices[v][1], multiplier) << " "
				  << scalar_t::dequantize(polygon_vertices[v][2], multiplier) << std::endl;
			}

            f << "f ";
			for(int v = 0; v < polygon_vertices.size(); ++v)
			{
				f << v << " ";
			}
			f << std::endl;
			f.close();
        }*/

        std::vector<vec2> x;
        project_to_2d(x, polygon_vertices, polygon_normal, polygon_normal_largest_component, multiplier);
        MCUT_ASSERT(x.size() == (size_t)polygon_vertex_count);

        /*{
			std::ofstream f("poly2.obj");

			for(int v = 0; v < x.size(); ++v)
			{
				f << "v " << scalar_t::dequantize(x[v][0], multiplier) << " "
				  << scalar_t::dequantize(x[v][1], multiplier) << " "
				  << 0 << std::endl;
			}

			f << "f ";
			for(int v = 0; v < x.size(); ++v)
			{
				f << v << " ";
			}
			f << std::endl;
			f.close();
		}*/

        /*
            NOTE: We cannot just use _any_/the first result of "colinear(x[i], x[j], x[k])" which returns true since
            any three points that are _nearly_ colinear (in the limit of floating point precision)
            will be found to be not colinear when using the exact predicate "orient2d".
            This would have implications "further down the line", where e.g. the three nearly colinear points
            are determined to be non-colinear (in the exact sense) but using them to evaluate
            operations like segment-plane intersection would then give a false positive.
            To overcome this, must find the three vertices of the polygon, which maximise non-colinearity.

            NOTE: if the polygon has 3 vertices and they are indeed nearly colinear then answer is determined by the
            exact predicate (i.e. i j k = 0 1 2)
        */

        // get any three vertices that are not collinear
        /*i = 0;
        j = i + 1;
        k = j + 1;*/
        std::vector<std::tuple<int, int, int, scalar_t>> non_colinear_triplets;

        for (int i_=0; i_ < polygon_vertex_count; ++i_) 
        {
            for (int j_=i_+1; j_ < polygon_vertex_count; ++j_) {
				for(int k_ = j_ + 1; k_ < polygon_vertex_count; ++k_)
				{
                    scalar_t predRes;
					//std::cout << "i=" << i_ << " j=" << j_ << " k="<<k_ << std::endl;
                    if (!collinear(x[i_], x[j_], x[k_], predRes)) {
                        non_colinear_triplets.emplace_back(std::make_tuple(i_, j_, k_, predRes));
                    }
                }
            }
        }

        std::sort(non_colinear_triplets.begin(), non_colinear_triplets.end(),
            [](const std::tuple<int, int, int, scalar_t>& a, const std::tuple<int, int, int, scalar_t>& b) {
#if 1 // work with double or rational_number
				return absolute_value(std::get<3>(a)) > absolute_value(std::get<3>(b));
#else
                return std::fabs(std::get<3>(a)) > std::fabs(std::get<3>(b));
#endif
            });

        std::tuple<int, int, int, scalar_t> best_triplet = non_colinear_triplets.front(); // maximising non-colinearity

        i = std::get<0>(best_triplet);
        j = std::get<1>(best_triplet);
        k = std::get<2>(best_triplet);

        return !non_colinear_triplets.empty(); // need at least one non-colinear triplet
    }

    char compute_segment_plane_intersection_type(const vec3& q, const vec3& r,
        const std::vector<vec3>& polygon_vertices,
        const vec3& polygon_normal,
												 const int polygon_normal_largest_component,
												 const double multiplier)
    {
        // TODO: we could also return i,j and k so that "determine_three_noncollinear_vertices" is not called multiple times,
        // which we do to determine the type of intersection and the actual intersection point
        const int polygon_vertex_count = (int)polygon_vertices.size();
        // ... any three vertices that are not collinear
        int i = 0;
        int j = 1;
        int k = 2;
        if (polygon_vertex_count > 3) { // case where we'd have the possibility of noncollinearity
            bool b = determine_three_noncollinear_vertices(i, j, k, polygon_vertices, polygon_normal,
                polygon_normal_largest_component, multiplier);

            if (!b) {
                return '0'; // all polygon points are collinear
            }
        }

        scalar_t qRes = orient3d(polygon_vertices[i], polygon_vertices[j], polygon_vertices[k], q);
        scalar_t rRes = orient3d(polygon_vertices[i], polygon_vertices[j], polygon_vertices[k], r);

        if (qRes == scalar_t(0.0) && rRes == scalar_t(0.0)) {
            return 'p';
        } else if (qRes == scalar_t(0.0)) {
            return 'q';
        } else if (rRes == scalar_t(0.0)) {
            return 'r';
        } else if ((rRes < scalar_t(0.0) && qRes < scalar_t(0.0)) || (rRes > scalar_t(0.0) && qRes > scalar_t(0.0))) {
            return '0';
        } else {
            return '1';
        }
    }

    char compute_segment_line_plane_intersection_type(const vec3& q, const vec3& r,
        const std::vector<vec3>& polygon_vertices,

        const int polygon_normal_max_comp,
        const vec3& polygon_plane_normal,
													  const double multiplier)
    {
        const int polygon_vertex_count = (int)polygon_vertices.size();
        // ... any three vertices that are not collinear
        int i = 0;
        int j = 1;
        int k = 2;
        if (polygon_vertex_count > 3) { // case where we'd have the possibility of noncollinearity
            bool b = determine_three_noncollinear_vertices(i, j, k, polygon_vertices, polygon_plane_normal,
														   polygon_normal_max_comp,
														   multiplier);

            if (!b) {
                return '0'; // all polygon points are collinear
            }
        }

        scalar_t qRes = orient3d(polygon_vertices[i], polygon_vertices[j], polygon_vertices[k], q);
        scalar_t rRes = orient3d(polygon_vertices[i], polygon_vertices[j], polygon_vertices[k], r);

        if (qRes == scalar_t(0.0) && rRes == scalar_t(0.0)) {
            // both points used to define line lie on plane therefore we have an in-plane intersection
            // or the polygon is a degenerate triangle
            return 'p';
        } else {
            if ((rRes < scalar_t(0.0) && qRes < scalar_t(0.0)) || (rRes > scalar_t(0.0) && qRes > scalar_t(0.0))) { // both points used to define line lie on same side of plane
                // check if line is parallel to plane
                // const scalar_t num = polygon_plane_d_coeff - dot_product(q, polygon_plane_normal);
                const vec3 rq = (r - q);
                const scalar_t denom = dot_product(rq, polygon_plane_normal);

                if (denom == scalar_t(0.0) /* Segment is parallel to plane.*/) {
                    // MCUT_ASSERT(num != 0.0); // implies 'q' is on plane (see: "compute_segment_plane_intersection(...)")
                    // but we have already established that q and r are on same side.
                    return '0';
                }
            }
        }

        // q and r are on difference sides of the plane, therefore we have an intersection
        return '1';
    }

    // Compute the intersection point between a line (not a segment) and a plane defined by a polygon.
    //
    // Parameters:
    //  'p' : output intersection point (computed if line does indeed intersect the plane)
    //  'q' : first point defining your line
    //  'r' : second point defining your line
    //  'polygon_vertices' : the vertices of the polygon defineing the plane (assumed to not be degenerate)
    //  'polygon_vertex_count' : number of olygon vertices
    //  'polygon_normal_max_comp' : largest component of polygon normal.
    //  'polygon_plane_normal' : normal of the given polygon
    //  'polygon_plane_d_coeff' : the distance coefficient of the plane equation corresponding to the polygon's plane
    //
    // Return values:
    // '0': line is parallel to plane (or polygon is degenerate ... within available precision)
    // '1': an intersection exists.
    // 'p': q and r lie in the plane (technically they are parallel to the plane too but we need to report this because it
    // violates GP).
    char compute_line_plane_intersection(vec3& p, // intersection point
        const vec3& q, const vec3& r,
        const std::vector<vec3>& polygon_vertices, const int polygon_normal_max_comp,
        const vec3& polygon_plane_normal,
        const scalar_t& polygon_plane_d_coeff, const double multiplier)
    {

        const int polygon_vertex_count = (int)polygon_vertices.size();
        // ... any three vertices that are not collinear
        int i = 0;
        int j = 1;
        int k = 2;
        if (polygon_vertex_count > 3) { // case where we'd have the possibility of noncollinearity
            bool b = determine_three_noncollinear_vertices(i, j, k, polygon_vertices, polygon_plane_normal,
														   polygon_normal_max_comp,
														   multiplier);

            if (!b) {
                return '0'; // all polygon points are collinear
            }
        }

        scalar_t qRes = orient3d(polygon_vertices[i], polygon_vertices[j], polygon_vertices[k], q);
        scalar_t rRes = orient3d(polygon_vertices[i], polygon_vertices[j], polygon_vertices[k], r);

        if (qRes == scalar_t(0.) && rRes == scalar_t(0.)) {
            return 'p'; // both points used to define line lie on plane therefore we have an in-plane intersection
        } else {

            const scalar_t num = polygon_plane_d_coeff - dot_product(q, polygon_plane_normal);
            const vec3 rq = (r - q);
            const scalar_t denom = dot_product(rq, polygon_plane_normal);

            if ((rRes < scalar_t(0.) && qRes < scalar_t(0.)) || (rRes > scalar_t(0.) && qRes > scalar_t(0.))) { // both q an r are on same side of plane
                if (denom == scalar_t(0.) /* line is parallel to plane.*/) {
                    return '0';
                }
            }

            // q and r are on difference sides of the plane, therefore we have an intersection

            // compute the intersection point
            const scalar_t t = num / denom;

            for (int it = 0; it < 3; ++it) {
                p[it] = q[it] + t * (r[it] - q[it]);
            }

            return '1';
        }
    }

    // Count the number ray crossings to determine if a point 'q' lies inside or outside a given polygon.
    //
    // Return values:
    // 'i': q is strictly interior
    // 'o': q is strictly exterior (outside).
    // 'e': q is on an edge, but not an endpoint.
    // 'v': q is a vertex.
    char compute_point_in_polygon_test(const vec2& q, const std::vector<vec2>& polygon_vertices)
    {
        const int polygon_vertex_count = (int)polygon_vertices.size();

#if 0 // Algorithm 1 in :
      // https://pdf.sciencedirectassets.com/271512/1-s2.0-S0925772100X00545/1-s2.0-S0925772101000128/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEAEaCXVzLWVhc3QtMSJHMEUCIBr6Fu%2F%2FtscErn%2Fl4pn2UGNA45sAw9vggQscK7Tnl0ssAiEAzfnyy4B4%2BXkZp8xcEk7utDBrxgmGyH7pyqk0efKOUEoq%2BgMIKhAEGgwwNTkwMDM1NDY4NjUiDHQT4kOfk4%2B6kBB0xCrXAzd8SWlnELJbM5l93HSvv4nUgtO85%2FGyx5%2BOoHmYoUuVwJjCXChmLJmlz%2BxYUQE%2Fr8vQa2hPlUEPfiTVGgoHaK8NkSMP6LRs%2F3WjyZ9OxzHbqSwZixNceW34OAkq0E1QgYLdGVjpPKxNK1haXDfBTMN6bF2IOU9dKi9wTv3uTlep0kHDa1BNNl6M6yZk5QlF2bPF9XmNjjZCpFQLhr%2BPoHo%2Bx4xy39aH8hCkkTqGdy2KrKGN6lv0%2FduIaItyZfqalYS%2BwW6cll2F5G11g0tSu7yKu6mug94LUTzsRmzD0UhzeGl2WV6Ev2qhw26mwFEKgnTMqGst8XAHjFjjAQyMzXNHCQqNBRIevHIzVWuUY4QZMylSRsodo0dfwwCFhzl0%2BJA1ZXb0%2BoyB8c11meQBO8FpMSshngNcbiAYUtIOFcHNCTCUMk0JPOZ%2FxvANsopnivbrPARL71zU4PaTujg5jfC2zoO6ZDUL8E6Vn%2BNtfb%2BuQV7DwtIH51Bv%2F%2F1m6h5mjbpRiGYcH%2F5SDD%2Fk%2BpHfKRQzKu7jJc%2B0XO0bQvoLSrAte0Qk10PwOvDl5jMIMdmxTBDDiDGErRykYxMQhq5EwjyiWPXzM3ll9uK59Vy0bAEj5Qemj5by1jCDht6IBjqlAV4okAPQO5wWdWojCYeKvluKfXCvudrUxrLhsxb7%2BTZNMODTG%2Fn%2Fbw875Yr6fOQ42u40pOsnPB%2FTP3cWwjiB%2BEHzDqN8AhCVQyoedw7QrU3OBWlSl6lB%2BGLAVqrhcizgFUiM2nj3HaVP2m7S%2FXqpv%2FoWlEXt4gR8iI9XsIlh6L6SBE22FqbsU5ewCxXaqip19VXhAGnlvjTihXUg6yZGWhExHj%2BKcA%3D%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20210814T103958Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTY3PPQSW2L%2F20210814%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=94403dce5c12ede9507e9612c000db5772607e59a2134d30d4e5b44212056dc7&hash=cada083b165f9786e6042e21d3d22147ec08b1e2c8fa2572ceeb2445cb5e730a&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S0925772101000128&tid=spdf-f9e7c4b7-808d-4a8e-a474-6430ff687798&sid=8dddff5b32fff2499908bf6501965aec3d0fgxrqb&type=client

            const scalar_t zero(0.0);
            int i = 0;
            int k = 0;

            scalar_t f = zero;
            scalar_t u1 = 0.0;
            scalar_t v1 = 0.0;
            scalar_t u2 = 0.0;
            scalar_t v2 = 0.0;
            const int n = polygon_vertex_count;
            for (i = 0; i < n; ++i)
            {
                v1 = polygon_vertices[i].y() - q.y();
                v2 = polygon_vertices[(i + 1) % n].y() - q.y();

                if ((v1 < zero && v2 < zero) || (v1 > zero && v2 > zero))
                {
                    continue;
                }

                u1 = polygon_vertices[i].x() - q.x();
                u2 = polygon_vertices[(i + 1) % n].x() - q.x();

                if (v2 > zero && v1 <= zero)
                {
                    f = u1 * v2 - u2 * v1;
                    if (f > zero)
                    {
                        k = k + 1;
                    }
                    else if (f == zero)
                    {
                        return 'e'; //-1; // boundary
                    }
                }
                else if (v1 > zero && v2 <= zero)
                {
                    f = u1 * v2 - u2 * v1;
                    if (f < zero)
                    {
                        k = k + 1;
                    }
                    else if (f == zero)
                    {
                        return 'e'; //-1; // boundary
                    }
                }
                else if (v2 == zero && v1 < zero)
                {
                    f = u1 * v2 - u2 * v1;
                    if (f == zero)
                    {
                        return 'e'; //-1; // // boundary
                    }
                }
                else if (v1 == zero && v2 < zero)
                {
                    f = u1 * v2 - u2 * v1;
                    if (f == zero)
                    {
                        return 'e'; //-1; // boundary
                    }
                }
                else if (v1 == zero && v2 == zero)
                {
                    if (u2 <= zero && u1 >= zero)
                    {
                        return 'e'; //-1; // boundary
                    }
                    else if (u1 <= zero && u2 >= zero)
                    {
                        return 'e'; //-1; // boundary
                    }
                }
            }
            if (k % 2 == 0)
            {
                return 'o'; //0; // outside
            }
            else
            {
                return 'i'; //1; // inside
            }
#else // http://www.science.smith.edu/~jorourke/books/compgeom.html

        std::vector<vec2> vertices(polygon_vertex_count, vec2());
        // Shift so that q is the origin. Note this destroys the polygon.
        // This is done for pedagogical clarity.
        for (int i = 0; i < polygon_vertex_count; i++) {
            for (int d = 0; d < 2; d++) {
                const scalar_t& a = polygon_vertices[i][d];
                const scalar_t& b = q[d];
                scalar_t& c = vertices[i][d];
                c = a - b;
            }
        }

        int Rcross = 0; /* number of right edge/ray crossings */
        int Lcross = 0; /* number ofleft edge/ray crossings */

        /* For each edge e = (i�lj), see if crosses ray. */
        for (int i = 0; i < polygon_vertex_count; i++) {

            /* First check if q = (0, 0) is a vertex. */
            if (vertices[i].x() == scalar_t(0.) && vertices[i].y() == scalar_t(0.)) {
                return 'v';
            }

            int il = (i + polygon_vertex_count - 1) % polygon_vertex_count;

            // Check if e straddles x axis, with bias above/below.

            // Rstrad is TRUE iff one endpoint of e is strictly above the x axis and the other is not (i.e., the other is on
            // or below)
            bool Rstrad = (vertices[i].y() > scalar_t(0.)) != (vertices[il].y() > scalar_t(0.));
            bool Lstrad = (vertices[i].y() < scalar_t(0.)) != (vertices[il].y() < scalar_t(0.));

            if (Rstrad || Lstrad) {
                /* Compute intersection of e with x axis. */

                // The computation of x is needed whenever either of these straddle variables is TRUE, which
                // only excludes edges passing through q = (0, 0) (and incidentally protects against division by 0).
                scalar_t x = (vertices[i].x() * vertices[il].y() - vertices[il].x() * vertices[i].y()) / (vertices[il].y() - vertices[i].y());
                if (Rstrad && x > scalar_t(0.)) {
                    Rcross++;
                }
                if (Lstrad && x < scalar_t(0.)) {
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
#endif
    }

    // given a normal vector (not necessarily normalized) and its largest component, calculate
    // a matrix P that will project any 3D vertex to 2D by removing the 3D component that
    // corresponds to the largest component of the normal..
    // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/2672702#2672702
    matrix_t<scalar_t> calculate_projection_matrix(const vec3& polygon_normal,
        const int polygon_normal_largest_component, double multiplier)
    {
        MCUT_ASSERT(squared_length(polygon_normal) > scalar_t(0.0));
        MCUT_ASSERT(polygon_normal_largest_component >= 0);
        MCUT_ASSERT(polygon_normal_largest_component <= 2);

        // unit length normal vector of polygon
		const vec3 a = normalize(polygon_normal, multiplier);
        // unit length basis vector corresponding to the largest component of the normal vector
        const vec3 b = [&]() {
            vec3 x(scalar_t(0.0));
            const sign_t s = sign(polygon_normal[polygon_normal_largest_component]);
            MCUT_ASSERT(s != sign_t::ZERO); // implies that the normal vector has a magnitude of zero
            // The largest component of the normal is the one we will "remove"
            // NOTE: we multiple by the sign here to ensure that "a_plus_b" below is not zero when a == b
            x[polygon_normal_largest_component] = scalar_t((1.0) * static_cast<double>(s));
            return x;
        }();

        matrix_t<scalar_t> I(3, 3); // 3x3 identity with reflection
        I(0, 0) = 1.0;
        I(1, 1) = -1.0;
        I(2, 2) = 1.0;

        matrix_t<scalar_t> R = I;

        // NOTE: While this will map vectors a to b, it can add a lot of unnecessary "twist".
        // For example, if a=b=e_{z} this formula will produce a 180-degree rotation about the z-axis rather
        // than the identity one might expect.
        if ((a[0] != b[0]) || (a[1] != b[1]) || (a[2] != b[2])) // a != b
        {
            const vec3 a_plus_b = a + b;
            const scalar_t a_dot_b = dot_product(a, b);

            // this will never be zero because we set 'b' as the canonical basis vector
            // that has the largest projection onto the normal
            MCUT_ASSERT(a_dot_b != scalar_t(0.0));

            const matrix_t<scalar_t> outer = outer_product(a_plus_b, a_plus_b);

            // compute the 3x3 rotation matrix R to orient the polygon into the canonical axes "i" and "j",
            //  where "i" and "j" != "polygon_normal_largest_component"
            R = ((outer / a_dot_b) * 2.0) - I; // rotation
        }

        // compute the 2x3 selector matrix K to select the polygon-vertex components that correspond
        // to canonical axes "i" and "j".
        matrix_t<scalar_t> K = matrix_t<scalar_t>(2, 3);

        if (polygon_normal_largest_component == 0) // project by removing x-component
        {
            // 1st row
            K(0, 0) = 0.0; // col 0
            K(0, 1) = 1.0; // col 1
            K(0, 2) = 0.0; // col 2
            // 2nd row
            K(1, 0) = 0.0; // col 0
            K(1, 1) = 0.0; // col 1
            K(1, 2) = 1.0; // col 2
        } else if (polygon_normal_largest_component == 1) // project by removing y-component
        {
            // 1st row
            K(0, 0) = 1.0; // col 0
            K(0, 1) = 0.0; // col 1
            K(0, 2) = 0.0; // col 2
            // 2nd row
            K(1, 0) = 0.0; // col 0
            K(1, 1) = 0.0; // col 1
            K(1, 2) = 1.0; // col 2
        } else if (polygon_normal_largest_component == 2) // project by removing z-component
        {
            // 1st row
            K(0, 0) = 1.0; // col 0
            K(0, 1) = 0.0; // col 1
            K(0, 2) = 0.0; // col 2
            // 2nd row
            K(1, 0) = 0.0; // col 0
            K(1, 1) = 1.0; // col 1
            K(1, 2) = 0.0; // col 2
        }

        return K * R;
    }

  //  // https://answers.unity.com/questions/1522620/converting-a-3d-polygon-into-a-2d-polygon.html
  //  void project_to_2d(std::vector<vec2>& out, const std::vector<vec3>& polygon_vertices, const vec3& polygon_normal, const double multiplier)
  //  {
  //      const uint32_t N = (uint32_t)polygon_vertices.size();
  //      out.resize(N);

  //      const vec3 normal = normalize(polygon_normal, multiplier);

  //      // first unit vector on plane (rotation by 90 degrees)
  //      vec3 u(-normal.y(), normal.x(), normal.z());
		//vec3 v = normalize(cross_product(u, normal), multiplier);

  //      for(uint32_t i =0; i < N; ++i)
  //      {
  //          const vec3 point = polygon_vertices[i];
  //          const vec2 projected(
  //              dot_product(point, u),
  //              dot_product(point, v)
  //          );

  //          out[i] = projected;
  //      }
  //  }

    void project_to_2d(std::vector<vec2>& out, const std::vector<vec3>& polygon_vertices,
        const vec3& polygon_normal, const int polygon_normal_largest_component, const double multiplier)
    {
        const int polygon_vertex_count = (int)polygon_vertices.size();
        out.clear();
        out.resize(polygon_vertex_count);

#if 1
        // 3x3 matrix for projecting a point to 2D
		matrix_t<scalar_t> P = calculate_projection_matrix(
			polygon_normal, polygon_normal_largest_component, multiplier);
        for (int i = 0; i < polygon_vertex_count; ++i) { // for each vertex
            const vec3& x = polygon_vertices[i];
            out[i] = P * x; // vertex in xz plane
        }
#else // This code is not reliable because it shadow-projects a polygon
        for (int i = 0; i < polygon_vertex_count; ++i) { // for each vertex
            vec2& Tp = out[i];
            int k = 0;
            for (int j = 0; j < 3; j++) { // for each component
				if(j != polygon_normal_largest_component)
				{ /* skip largest coordinate */

                    Tp[k] = polygon_vertices[i][j];
                    k++;
                }
            }
        }
#endif
    }

    // TODO: update this function to use "project_to_2d" for projection step
    char compute_point_in_polygon_test(const vec3& p, const std::vector<vec3>& polygon_vertices,
									   const vec3& polygon_normal,
									   const int polygon_normal_largest_component,
									   const double multiplier)
    {
        const int polygon_vertex_count = (int)polygon_vertices.size();
        /* Project out coordinate m in both p and the triangular face */
        vec2 pp; /*projected p */
#if 0
    int k = 0;
    
    for (int j = 0; j < 3; j++)
    { // for each component
        if (j != polygon_plane_normal_largest_component)
        { /* skip largest coordinate */
            pp[k] = p[j];
            k++;
        }
    }
#endif

        const matrix_t<scalar_t> P = calculate_projection_matrix(
			polygon_normal, polygon_normal_largest_component, multiplier);

        pp = P * p;

        std::vector<vec2> polygon_vertices2d(polygon_vertex_count, vec2());

        for (int i = 0; i < polygon_vertex_count; ++i) { // for each vertex
            const vec3& x = polygon_vertices[i];
            polygon_vertices2d[i] = P * x; // vertex in xz plane
        }

#if 0
    for (int i = 0; i < polygon_vertex_count; ++i)
    { // for each vertex
        vec2 &Tp = polygon_vertices2d[i];
        k = 0;
        for (int j = 0; j < 3; j++)
        { // for each component
            if (j != polygon_plane_normal_largest_component)
            { /* skip largest coordinate */

                Tp[k] = polygon_vertices[i][j];
                k++;
            }
        }
    }
#endif

        return compute_point_in_polygon_test(pp, polygon_vertices2d);
    }

    inline bool Between(const vec2& a, const vec2& b, const vec2& c)
    {
        vec2 ba, ca;
        /* If ab not vertical check betweenness on x; else on y. */
        if (a[0] != b[0])
            return ((a[0] <= c[0]) && (c[0] <= b[0])) || //
                ((a[0] >= c[0]) && (c[0] >= b[0]));
        else
            return ((a[1] <= c[1]) && (c[1] <= b[1])) || //
                ((a[1] >= c[1]) && (c[1] >= b[1]));
    }

    bool coplaner(const vec3& pa, const vec3& pb, const vec3& pc,
        const vec3& pd)
    {
        const scalar_t val = orient3d(pa, pb, pc, pd);
        // typedef std::numeric_limits<scalar_t> dbl;
#ifdef MCUT_WITH_ARBITRARY_PRECISION_NUMBERS
        // scalar_t d = 3.14159265358979;
        // std::cout.precision(dbl::max_digits10);
        // std::cout << "value=" << (scalar_t)val << std::endl;

        // NOTE: thresholds are chosen based on benchmark meshes that are used for testing.
        // It is extremely difficult to get this right because of intermediate conversions
        // between exact and fixed precision representations during cutting.
        // Thus, general advise is for user to ensure that the input polygons are really
        // co-planer. It might be possible for MCUT to help here (see eps used during poly
        // partitioning).
        return absolute_value(val) <= scalar_t(4e-7);
        #else
		return absolute_value(val);
        #endif
    }

    bool collinear(const vec2& a, const vec2& b, const vec2& c, scalar_t& predResult)
    {
        predResult = orient2d(a, b, c);
		return predResult == scalar_t(0.);
    }

    bool collinear(const vec2& a, const vec2& b, const vec2& c)
    {
		return orient2d(a, b, c) == scalar_t(0.);
    }
    
    

    char Parallellnt(const vec2& a, const vec2& b, const vec2& c, const vec2& d, vec2& p)
    {
        if (!collinear(a, b, c)) {
            return '0';
        }

        if (Between(a, b, c)) {
            p = c;
            return 'e';
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

    char Parallellntd(const vec2_<double>& a,
					 const vec2_<double>& b,
					 const vec2_<double>& c,
					 const vec2_<double>& d,
					 vec2_<double>& p)
	{
		vec2 p_;
		auto result = Parallellnt(  vec2(a.x(), a.y()),
						               vec2(b.x(), b.y()),
						               vec2(c.x(), c.y()),
						               vec2(d.x(), d.y()), p_);
#ifdef MCUT_WITH_ARBITRARY_PRECISION_NUMBERS
		p[0] = p_.x().get_d();
		p[1] = p_.y().get_d();
#else
		p[0] = p_.x(); 
        p[1] = p_.y();
#endif
		return result;
    }

    char compute_segment_intersection(
        const vec2& a, const vec2& b, const vec2& c, 
        const vec2& d, vec2& p, scalar_t& s, scalar_t& t)
    {
        // scalar_t s, t; /* The two parameters of the parametric eqns. */
        scalar_t num, denom; /* Numerator and denominator of equations. */
        char code = '?'; /* Return char characterizing intersection.*/

        denom = a[0] * (d[1] - c[1]) + //
            b[0] * (c[1] - d[1]) + //
            d[0] * (b[1] - a[1]) + //
            c[0] * (a[1] - b[1]);

        /* If denom is zero, then segments are parallel: handle separately. */
        if (denom == scalar_t(0.0)) {
            return Parallellnt(a, b, c, d, p);
        }

        num = a[0] * (d[1] - c[1]) + //
            c[0] * (a[1] - d[1]) + //
            d[0] * (c[1] - a[1]);
		//std::cout << "test\n";
        //std::cout << num.get_str() << std::endl << denom.get_str() << std::endl;
        if ((num == scalar_t(0.0)) || (num == denom)) {
            code = 'v';
        }

        s = num / denom;

        num = -(a[0] * (c[1] - b[1]) + //
            b[0] * (a[1] - c[1]) + //
            c[0] * (b[1] - a[1]));

        if ((num == scalar_t(0.0)) || (num == denom)) {
            code = 'v';
        }

        t = num / denom;

        if ((scalar_t(0.0) < s) && (s < scalar_t(1.0)) && (scalar_t(0.0) < t) && (t < scalar_t(1.0))) {
            code = '1';
        } else if ((scalar_t(0.0) > s) || (s > scalar_t(1.0)) || (scalar_t(0.0) > t) || (t > scalar_t(1.0))) {
            code = '0';
        }

        p[0] = a[0] + s * (b[0] - a[0]);
        p[1] = a[1] + s * (b[1] - a[1]);

        return code;
    }

    inline bool point_in_bounding_box(const vec2& point, const bounding_box_t<vec2>& bbox)
    {
        if ((point.x() < bbox.m_minimum.x() || point.x() > bbox.m_maximum.x()) || //
            (point.y() < bbox.m_minimum.y() || point.y() > bbox.m_maximum.y())) {
            return false;
        } else {
            return true;
        }
    }

    inline bool point_in_bounding_box(const vec3& point, const bounding_box_t<vec3>& bbox)
    {
        if ((point.x() < bbox.m_minimum.x() || point.x() > bbox.m_maximum.x()) || //
            (point.y() < bbox.m_minimum.y() || point.y() > bbox.m_maximum.y()) || //
            (point.z() < bbox.m_minimum.z() || point.z() > bbox.m_maximum.z())) { //
            return false;
        } else {
            return true;
        }
    }


