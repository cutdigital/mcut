#ifndef _CDT_UTILITIES_H_
#define _CDT_UTILITIES_H_

#include "mcut/internal/math.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include <array>
#include <functional>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace cdt {

/// X- coordinate getter for vec2d_t
template <typename T>
const T& get_x_coord_vec2d(const vec2& v)
{
    return v.x();
}

/// Y-coordinate getter for vec2d_t
template <typename T>
const T& get_y_coord_vec2d(const vec2& v)
{
    return v.y();
}

/// If two 2D vectors are exactly equal
bool operator==(const vec2& lhs, const vec2& rhs)
{
    return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

/// Constant representing no valid neighbor for a triangle
const static std::uint32_t null_neighbour(std::numeric_limits<std::uint32_t>::max());
/// Constant representing no valid vertex for a triangle
const static std::uint32_t null_vertex(std::numeric_limits<std::uint32_t>::max());

#if 0
/// 2D bounding box
template <typename T>
struct box2d_t {
    vec2 min; ///< min box corner
    vec2 max; ///< max box corner

    /// Envelop box around a point
    void expand_with_point(const vec2& p)
    {
        expand_with_point(p.x(), p.y());
    }
    /// Envelop box around a point with given coordinates
    void expand_with_point(const T x, const T y)
    {
        min.x() = std::min(x, min.x());
        max.x() = std::max(x, max.x());
        min.y() = std::min(y, min.y());
        max.y() = std::max(y, max.y());
    }
};
#endif
/// Bounding box of a collection of custom 2D points given coordinate getters
template <
    typename TVertexIter //,
    // typename TGetVertexCoordX,
    // typename TGetVertexCoordY
    >
bounding_box_t<vec2> construct_bbox_containing_points(
    TVertexIter first,
    TVertexIter last //,
    // TGetVertexCoordX get_x_coord,
    // TGetVertexCoordY get_y_coord
)
{
    const double max = std::numeric_limits<double>::max();
    bounding_box_t<vec2> box = { { max, max }, { -max, -max } };

    for (; first != last; ++first) {
        box.expand_with_point(first->x(), first->y());
    }
    return box;
}

/// Bounding box of a collection of 2D points
bounding_box_t<vec2> construct_bbox_containing_points(const std::vector<vec2>& points);

/// edge_t connecting two vertices: vertex with smaller index is always first
/// \note: hash edge_t is specialized at the bottom
struct edge_t {

    edge_t(std::uint32_t vertex0_global_id, std::uint32_t vertex1_global_id)
        : m_vertex_pair(
            vertex0_global_id < vertex1_global_id ? std::make_pair(vertex0_global_id, vertex1_global_id) : std::make_pair(vertex1_global_id, vertex0_global_id))
    {
    }

    inline bool operator==(const edge_t& other) const
    {
        return m_vertex_pair == other.m_vertex_pair;
    }

    inline bool operator!=(const edge_t& other) const
    {
        return !(this->operator==(other));
    }

    inline std::uint32_t get_vertex(const std::uint32_t index) const
    {
        std::uint32_t result = m_vertex_pair.first; 
        
        if(index != 0)
        {
            MCUT_ASSERT(index == 1);
            result = m_vertex_pair.second;
        }

        return result;
    }

    inline const std::pair<std::uint32_t, std::uint32_t>& get_vertex_pair() const
    {
        return m_vertex_pair;
    }

private:
    std::pair<std::uint32_t, std::uint32_t> m_vertex_pair;
};

/// Get edge first vertex
inline std::uint32_t edge_get_v1(const edge_t& e)
{
    return e.get_vertex(0);
}

/// Get edge second vertex
inline std::uint32_t edge_get_v2(const edge_t& e)
{
    return e.get_vertex(1);
}

/// Get edge second vertex
inline edge_t edge_make(std::uint32_t iV1, std::uint32_t iV2)
{
    return edge_t(iV1, iV2);
}

/// triangulator_t triangle (CCW winding)
/* Counter-clockwise winding:
       v3
       /\
    n3/  \n2
     /____\
   v1  n1  v2                 */
struct triangle_t {

    std::array<std::uint32_t, 3> vertices;
    std::array<std::uint32_t, 3> neighbors;

    /**
     * Factory method
     * @note needed for c++03 compatibility (no uniform initialization
     * available)
     */
    static triangle_t
    make(const std::array<std::uint32_t, 3>& vertices, const std::array<std::uint32_t, 3>& neighbors)
    {
        triangle_t t = { vertices, neighbors };
        return t;
    }
};

/// Location of point on a triangle
struct point_to_triangle_location_t {
    /// Enum
    enum Enum {
        INSIDE,
        OUTSIDE,
        ON_1ST_EDGE,
        ON_2ND_EDGE,
        ON_3RD_EDGE,
    };
};

/// Relative location of point to a line
struct point_to_line_location_t {
    /// Enum
    enum Enum {
        LEFT_SIDE,
        RIGHT_SIDE,
        COLLINEAR,
    };
};

} // namespace cdt

#ifndef CDT_USE_AS_COMPILED_LIBRARY

#include <stdexcept>

namespace cdt {

// returns a 2D bounding box containing the vertices

bounding_box_t<vec2> construct_bbox_containing_points(const std::vector<vec2>& points)
{
    return construct_bbox_containing_points(
        points.begin(), points.end() /*, get_x_coord_vec2d<T>, get_y_coord_vec2d<T>*/);
}

//*****************************************************************************
// Utility functions
//*****************************************************************************

/// Advance vertex or neighbor index counter-clockwise
inline std::uint32_t ccw(std::uint32_t i)
{
    return std::uint32_t((i + 1) % 3);
}

/// Advance vertex or neighbor index clockwise
inline std::uint32_t cw(std::uint32_t i)
{
    return std::uint32_t((i + 2) % 3);
}

/// Check if location is classified as on any of three edges
inline bool check_on_edge(const point_to_triangle_location_t::Enum location)
{
    return location > point_to_triangle_location_t::OUTSIDE;
}

/// Neighbor index from a on-edge location
/// \note Call only if located on the edge!
inline std::uint32_t edge_neighbour(const point_to_triangle_location_t::Enum location)
{
    MCUT_ASSERT(location >= point_to_triangle_location_t::ON_1ST_EDGE);
    // make index be in range 0,2
    return static_cast<std::uint32_t>(location - point_to_triangle_location_t::ON_1ST_EDGE);
}

#if 0
/// Orient p against line v1-v2 2D: robust geometric predicate
template <typename T>
T orient2D(const vec2& p, const vec2& v1, const vec2& v2)
{
    return orient2d(v1.x(), v1.y(), v2.x(), v2.y(), p.x(), p.y());
}
#endif

// TODO: replace with "sign_t sign()" from math.h
/// Classify value of orient2d predicate
point_to_line_location_t::Enum
classify_orientation(const double orientation, const double orientationTolerance = double(0))
{
    if (orientation < -orientationTolerance)
        return point_to_line_location_t::RIGHT_SIDE;
    if (orientation > orientationTolerance)
        return point_to_line_location_t::LEFT_SIDE;
    return point_to_line_location_t::COLLINEAR;
}

/// Check if point lies to the left of, to the right of, or on a line
point_to_line_location_t::Enum locate_point_wrt_line(
    const vec2& point_coords,
    const vec2& line_vertex0_coords,
    const vec2& line_vertex1_coords,
    const double eps = double(0.0))
{
    const double orient2d_result = orient2d(point_coords, line_vertex0_coords, line_vertex1_coords);
    const point_to_line_location_t::Enum orientation_of_point_wrt_line = classify_orientation(orient2d_result, eps);
    return orientation_of_point_wrt_line;
}

/// Check if point a lies inside of, outside of, or on an edge of a triangle
point_to_triangle_location_t::Enum locate_point_wrt_triangle(
    const vec2& p,
    const vec2& v1,
    const vec2& v2,
    const vec2& v3)
{
    point_to_triangle_location_t::Enum result = point_to_triangle_location_t::INSIDE;
    point_to_line_location_t::Enum edgeCheck = locate_point_wrt_line(p, v1, v2);
    if (edgeCheck == point_to_line_location_t::RIGHT_SIDE)
        return point_to_triangle_location_t::OUTSIDE;
    if (edgeCheck == point_to_line_location_t::COLLINEAR)
        result = point_to_triangle_location_t::ON_1ST_EDGE;
    edgeCheck = locate_point_wrt_line(p, v2, v3);
    if (edgeCheck == point_to_line_location_t::RIGHT_SIDE)
        return point_to_triangle_location_t::OUTSIDE;
    if (edgeCheck == point_to_line_location_t::COLLINEAR)
        result = point_to_triangle_location_t::ON_2ND_EDGE;
    edgeCheck = locate_point_wrt_line(p, v3, v1);
    if (edgeCheck == point_to_line_location_t::RIGHT_SIDE)
        return point_to_triangle_location_t::OUTSIDE;
    if (edgeCheck == point_to_line_location_t::COLLINEAR)
        result = point_to_triangle_location_t::ON_3RD_EDGE;
    return result;
}

// Opposed neighbor index from vertex index
inline std::uint32_t get_local_index_of_neighbour_opposite_vertex(const std::uint32_t vertex_local_index)
{
    MCUT_ASSERT(vertex_local_index >= 0 && vertex_local_index <= 2);

    std::uint32_t opposite_neighbour_local_index = null_neighbour;

    if (vertex_local_index == std::uint32_t(0)) {
        opposite_neighbour_local_index = std::uint32_t(1);
    } else if (vertex_local_index == std::uint32_t(1)) {
        opposite_neighbour_local_index = std::uint32_t(2);
    } else if (vertex_local_index == std::uint32_t(2)) {
        opposite_neighbour_local_index = std::uint32_t(0);
    }

    MCUT_ASSERT(opposite_neighbour_local_index != null_neighbour);

    return opposite_neighbour_local_index;
}

/// Opposed local vertex index from neighbor index
inline std::uint32_t get_opposite_vertex_local_index_from_neighbour_local_index(const std::uint32_t neighbour_local_index)
{
    MCUT_ASSERT(neighbour_local_index >= 0 && neighbour_local_index <= 2);

    std::uint32_t opposite_vertex_local_index = null_vertex;

    if (neighbour_local_index == std::uint32_t(0)) {
        opposite_vertex_local_index = std::uint32_t(2);
    } else if (neighbour_local_index == std::uint32_t(1)) {
        opposite_vertex_local_index = std::uint32_t(0);
    } else if (neighbour_local_index == std::uint32_t(2)) {
        opposite_vertex_local_index = std::uint32_t(1);
    }

    MCUT_ASSERT(opposite_vertex_local_index != null_vertex); // Invalid neighbor index

    return opposite_vertex_local_index;
}

// Local index of triangle's neighbor opposed to a vertex
inline std::uint32_t get_local_index_of_neighbour_opposite_vertex(const triangle_t& triangle, const std::uint32_t vertex_index)
{
    std::uint32_t neighbour_opp_vertex_index_local = null_neighbour;

    // for each vertex of triangle
    for (std::uint32_t i = std::uint32_t(0); i < std::uint32_t(3); ++i) {
        // does the (global) index match the sought vertex
        if (vertex_index == triangle.vertices[i]) {
            
            neighbour_opp_vertex_index_local = get_local_index_of_neighbour_opposite_vertex(i);
            break;
        }
    }

    // the neighbour of a border triangle might be null (non-existent) but
    // the local index is always defined.
    MCUT_ASSERT(neighbour_opp_vertex_index_local != null_neighbour);

    return neighbour_opp_vertex_index_local;
}

/// Index of triangle's neighbor opposed to an edge
inline std::uint32_t get_local_index_of_neighbour_opposite_vertex(
    const triangle_t& tri,
    const std::uint32_t iVedge1,
    const std::uint32_t iVedge2)
{
    for (std::uint32_t vi = std::uint32_t(0); vi < std::uint32_t(3); ++vi) {
        const std::uint32_t iVert = tri.vertices[vi];
        if (iVert != iVedge1 && iVert != iVedge2)
            return get_local_index_of_neighbour_opposite_vertex(vi);
    }
    throw std::runtime_error("Could not find opposed-to-edge triangle index");
}

// Index of triangle's vertex opposed to a triangle
inline std::uint32_t get_vertex_local_id_opposite_neighbour(const triangle_t& triangle, const std::uint32_t neighbour_triangle_index)
{
    std::uint32_t opposite_vertex = null_vertex;

    // for each neighbour
    for (std::uint32_t i = std::uint32_t(0); i < std::uint32_t(3); ++i) {
        // does the neighbour match the one I'm looking for?
        if (neighbour_triangle_index == triangle.neighbors[i]) {
            opposite_vertex = get_opposite_vertex_local_index_from_neighbour_local_index(i);
            break;
        }
    }

    MCUT_ASSERT(opposite_vertex != null_vertex);

    return opposite_vertex;
}

/// If triangle has a given neighbor return neighbor-index, throw otherwise
inline std::uint32_t get_neighbour_index(const triangle_t& tri, std::uint32_t iTnbr)
{
    for (std::uint32_t ni = std::uint32_t(0); ni < std::uint32_t(3); ++ni)
        if (iTnbr == tri.neighbors[ni])
            return ni;
    throw std::runtime_error("Could not find neighbor triangle index");
}

/// If triangle has a given vertex return vertex-index
inline std::uint32_t triangle_get_vertex_local_id(const triangle_t& triangle, const std::uint32_t vertex_global_id)
{
    MCUT_ASSERT(vertex_global_id != null_vertex);

    std::uint32_t result = null_vertex;

    // for each vertex of triangle
    for (std::uint32_t i = std::uint32_t(0); i < std::uint32_t(3); ++i)
    {
        // does the global id of the current triangle vertex match the one
        // we are looking for?
        if (vertex_global_id == triangle.vertices[i])
        {
            result = i; // save local index
            break;
        }
    }
    
    MCUT_ASSERT(result != null_vertex);

    return result;
}

/// Given triangle and a vertex find opposed triangle
inline std::uint32_t get_global_triangle_index_opposite_vertex(const triangle_t& triangle, const std::uint32_t vertex_index)
{   
    const std::uint32_t local_index_of_neighbour_opposite_vertex = get_local_index_of_neighbour_opposite_vertex(triangle, vertex_index);
    const std::uint32_t global_index_of_neighbour_opposite_vertex = triangle.neighbors[local_index_of_neighbour_opposite_vertex];
    return global_index_of_neighbour_opposite_vertex;
}

/// Given two triangles, return vertex of first triangle opposed to the second
inline std::uint32_t
get_opposed_vertex_index(const triangle_t& tri, std::uint32_t iTopo)
{
    return tri.vertices[get_vertex_local_id_opposite_neighbour(tri, iTopo)];
}

/// Test if point lies in a circumscribed circle of a triangle
bool check_is_in_circumcircle(
    const vec2& p,
    const vec2& v1,
    const vec2& v2,
    const vec2& v3)
{
    const double p_[2] = { static_cast<double>(p.x()), static_cast<double>(p.y()) };
    const double v1_[2] = { static_cast<double>(v1.x()), static_cast<double>(v1.y()) };
    const double v2_[2] = { static_cast<double>(v2.x()), static_cast<double>(v2.y()) };
    const double v3_[2] = { static_cast<double>(v3.x()), static_cast<double>(v3.y()) };

    return ::incircle(v1_, v2_, v3_, p_) > double(0);
}

/// Test if two vertices share at least one common triangle
inline bool check_vertices_share_edge(
    const std::vector<std::uint32_t>& vtx0_adjacent_triangles, 
    const std::vector<std::uint32_t>& vtx1_adjacent_triangles)
{
    bool result = false;

    for (std::vector<std::uint32_t>::const_iterator it = vtx0_adjacent_triangles.begin(); it != vtx0_adjacent_triangles.end(); ++it)
    {
        const std::vector<std::uint32_t>::const_iterator fiter = std::find(vtx1_adjacent_triangles.begin(), vtx1_adjacent_triangles.end(), *it);

        if (fiter != vtx1_adjacent_triangles.end())
        {
            result = true;
            break;
        }
    }

    return result;
}

#if 0
double get_square_distance(const double ax, const double ay, const double bx, const double by)
{
    const double dx = bx - ax;
    const double dy = by - ay;
    return dx * dx + dy * dy;
}

double distance(const double ax, const double ay, const double bx, const double by)
{
    return std::sqrt(get_square_distance(ax, ay, bx, by));
}

double distance(const vec2& a, const vec2& b)
{
    return distance(a.x(), a.y(), b.x(), b.y());
}

double get_square_distance(const vec2& a, const vec2& b)
{
    return get_square_distance(a.x(), a.y(), b.x(), b.y());
}
#endif

} // namespace cdt

#endif

//*****************************************************************************
// Specialize hash functions
//*****************************************************************************
namespace std {
/// edge_t hasher
template <>
struct hash<cdt::edge_t> {
    /// Hash operator
    std::size_t operator()(const cdt::edge_t& e) const
    {
        return get_hashed_edge_index(e);
    }

private:
    static void combine_hash_values(std::size_t& seed, const std::uint32_t& key)
    {
        seed ^= std::hash<std::uint32_t>()(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    static std::size_t get_hashed_edge_index(const cdt::edge_t& e)
    {
        const std::pair<std::uint32_t, std::uint32_t>& vv = e.get_vertex_pair();
        std::size_t seed1(0);
        combine_hash_values(seed1, vv.first);
        combine_hash_values(seed1, vv.second);
        std::size_t seed2(0);
        combine_hash_values(seed2, vv.second);
        combine_hash_values(seed2, vv.first);
        return std::min(seed1, seed2);
    }
};
} // namespace std/boost

#endif // header guard
