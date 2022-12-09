#ifndef CDT_vW1vZ0lO8rS4gY4uI4fB
#define CDT_vW1vZ0lO8rS4gY4uI4fB

#include "mcut/internal/cdt/kdtree.h"
#include "mcut/internal/cdt/utils.h"

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <stack>
#include <stdexcept>
#include <utility>
#include <vector>

/// Namespace containing triangulation functionality
namespace cdt {

/// @addtogroup API
/// @{

/**
 * Enum of strategies specifying order in which a range of vertices is inserted
 * @note vertex_insertion_order_t::RANDOM will only randomize order of
 * inserting in triangulation, vertex indices will be preserved as they were
 * specified in the final triangulation
 */
struct vertex_insertion_order_t {
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum {
        RANDOM, ///< vertices will be inserted in random order
        AS_GIVEN, ///< vertices will be inserted in the same order as provided
    };
};

/// Enum of what type of geometry used to embed triangulation into
struct super_geometry_type_t {
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum {
        SUPER_TRIANGLE, ///< conventional super-triangle
        CUSTOM, ///< user-specified custom geometry (e.g., grid)
    };
};

/**
 * Enum of strategies for treating intersecting constraint edges
 */
struct action_on_intersecting_constraint_edges_t {
    /**
     * The Enum itself
     * @note needed to pre c++11 compilers that don't support 'class enum'
     */
    enum Enum {
        IGNORE, ///< constraint edge intersections are not checked
        RESOLVE, ///< constraint edge intersections are resolved
    };
};

/**
 * Type used for storing layer depths for triangles
 * @note layer_depth_t should support 60K+ layers, which could be to much or
 * too little for some use cases. Feel free to re-define this typedef.
 */
typedef unsigned short layer_depth_t;
typedef layer_depth_t boundary_overlap_count_t;

namespace detail {

    /// Needed for c++03 compatibility (no uniform initialization available)
    std::array<std::uint32_t, 3> arr3(const double& v0, const double& v1, const double& v2)
    {
        const std::array<std::uint32_t, 3> out = { v0, v1, v2 };
        return out;
    }

    namespace defaults {

        const std::size_t nTargetVerts = 0;
        const super_geometry_type_t::Enum superGeomType = super_geometry_type_t::SUPER_TRIANGLE;
        const vertex_insertion_order_t::Enum vertexInsertionOrder = vertex_insertion_order_t::RANDOM;
        const action_on_intersecting_constraint_edges_t::Enum intersectingEdgesStrategy = action_on_intersecting_constraint_edges_t::IGNORE;
        const float minDistToConstraintEdge(0);

    } // namespace defaults

    // add element to 'to' if not already in 'to'
    template <typename Allocator1>
    void insert_unique(std::vector<Allocator1>& to, const double& elem)
    {
        if (std::find(to.begin(), to.end(), elem) == to.end()) {
            to.push_back(elem);
        }
    }

    // add elements of 'from' that are not present in 'to' to 'to'
    template <typename Allocator1, typename Allocator2>
    void insert_unique(
        std::vector<Allocator1>& to,
        const std::vector<Allocator2>& from)
    {
        typedef typename std::vector<Allocator2>::const_iterator Cit;
        to.reserve(to.size() + from.size());
        for (Cit cit = from.begin(); cit != from.end(); ++cit) {
            insert_unique(to, *cit);
        }
    }

} // namespace detail

/**
 * @defgroup triangulator_t triangulator_t Class
 * Class performing triangulations.
 */
/// @{

/**
 * Data structure representing a 2D constrained Delaunay triangulation
 *
 * @tparam TNearPointLocator class providing locating near point for efficiently
 * inserting new points. Provides methods: 'add_point(vPos, iV)' and
 * 'find_nearest_point(vPos) -> iV'
 */
class triangulator_t {
public:
    // typedef std::vector<vec2> vec2_vector_t; ///< Vertices vector
    std::vector<vec2> vertices; ///< triangulation's vertices
    std::vector<triangle_t> triangles; ///< triangulation's triangles
    std::unordered_set<edge_t> m_constrained_edges; ///< triangulation's constraints (fixed edges)
    /**
     * triangles adjacent to each vertex
     * @note will be reset to empty when super-triangle is removed and
     * triangulation is finalized. To re-calculate adjacent triangles use
     * cdt::get_vertex_to_triangles_map helper
     */
    std::vector<std::vector<std::uint32_t>> per_vertex_adjacent_triangles;

    /** Stores count of overlapping boundaries for a fixed edge. If no entry is
     * present for an edge: no boundaries overlap.
     * @note map only has entries for fixed for edges that represent overlapping
     * boundaries
     * @note needed for handling depth calculations and hole-removel in case of
     * overlapping boundaries
     */
    std::unordered_map<edge_t, boundary_overlap_count_t> overlapCount;

    /** Stores list of original edges represented by a given fixed edge
     * @note map only has entries for edges where multiple original fixed edges
     * overlap or where a fixed edge is a part of original edge created by
     * conforming Delaunay triangulation vertex insertion
     */
    std::unordered_map<edge_t, std::vector<edge_t>> pieceToOriginals;

    /*____ API _____*/
    /// Default constructor
    triangulator_t();
    /**
     * Constructor
     * @param vertexInsertionOrder strategy used for ordering vertex insertions
     */
    triangulator_t(vertex_insertion_order_t::Enum vertexInsertionOrder);
    /**
     * Constructor
     * @param vertexInsertionOrder strategy used for ordering vertex insertions
     * @param intersectingEdgesStrategy strategy for treating intersecting
     * constraint edges
     * @param minDistToConstraintEdge distance within which point is considered
     * to be lying on a constraint edge. Used when adding constraints to the
     * triangulation.
     */
    triangulator_t(
        vertex_insertion_order_t::Enum vertexInsertionOrder,
        action_on_intersecting_constraint_edges_t::Enum intersectingEdgesStrategy,
        double minDistToConstraintEdge);
    /**
     * Constructor
     * @param vertexInsertionOrder strategy used for ordering vertex insertions
     * @param nearPtLocator class providing locating near point for efficiently
     * inserting new points
     * @param intersectingEdgesStrategy strategy for treating intersecting
     * constraint edges
     * @param minDistToConstraintEdge distance within which point is considered
     * to be lying on a constraint edge. Used when adding constraints to the
     * triangulation.
     */
    triangulator_t(
        vertex_insertion_order_t::Enum vertexInsertionOrder,
        const locator_kdtree_t<>& nearPtLocator,
        action_on_intersecting_constraint_edges_t::Enum intersectingEdgesStrategy,
        double minDistToConstraintEdge);
    /**
     * Insert custom point-types specified by iterator range and X/Y-getters
     * @tparam TVertexIter iterator that dereferences to custom point type
     * @tparam TGetVertexCoordX function object getting x coordinate from
     * vertex. Getter signature: const TVertexIter::value_type& -> T
     * @tparam TGetVertexCoordY function object getting y coordinate from
     * vertex. Getter signature: const TVertexIter::value_type& -> T
     * @param first beginning of the range of vertices to add
     * @param last end of the range of vertices to add
     * @param get_x_coord getter of X-coordinate
     * @param get_y_coord getter of Y-coordinate
     */
    template <
        typename TVertexIter//,
        // typename TGetVertexCoordX,
        // typename TGetVertexCoordY
        >
    void insert_vertices(
        TVertexIter first,
        TVertexIter last //,
        // TGetVertexCoordX get_x_coord,
        // TGetVertexCoordY get_y_coord
    );
    /**
     * Insert vertices into triangulation
     * @param vertices vector of vertices to insert
     */
    void insert_vertices(const std::vector<vec2>& vertices);
    /**
     * Insert constraints (custom-type fixed edges) into triangulation
     * @note Each fixed edge is inserted by deleting the triangles it crosses,
     * followed by the triangulation of the polygons on each side of the edge.
     * <b> No new vertices are inserted.</b>
     * @note If some edge appears more than once in the input this means that
     * multiple boundaries overlap at the edge and impacts how hole detection
     * algorithm of triangulator_t::erase_outer_triangles_and_holes works.
     * <b>Make sure there are no erroneous duplicates.</b>
     * @tparam TEdgeIter iterator that dereferences to custom edge type
     * @tparam TGetEdgeVertexStart function object getting start vertex index
     * from an edge.
     * Getter signature: const TEdgeIter::value_type& -> std::uint32_t
     * @tparam TGetEdgeVertexEnd function object getting end vertex index from
     * an edge. Getter signature: const TEdgeIter::value_type& -> std::uint32_t
     * @param first beginning of the range of edges to add
     * @param last end of the range of edges to add
     * @param getStart getter of edge start vertex index
     * @param getEnd getter of edge end vertex index
     */
    template <
        typename TEdgeIter,
        typename TGetEdgeVertexStart,
        typename TGetEdgeVertexEnd>
    void insert_edges(
        TEdgeIter first,
        TEdgeIter last,
        TGetEdgeVertexStart getStart,
        TGetEdgeVertexEnd getEnd);
    /**
     * Insert constraint edges into triangulation
     * @note Each fixed edge is inserted by deleting the triangles it crosses,
     * followed by the triangulation of the polygons on each side of the edge.
     * <b> No new vertices are inserted.</b>
     * @note If some edge appears more than once in the input this means that
     * multiple boundaries overlap at the edge and impacts how hole detection
     * algorithm of triangulator_t::erase_outer_triangles_and_holes works.
     * <b>Make sure there are no erroneous duplicates.</b>
     * @tparam edges constraint edges
     */
    void insert_edges(const std::vector<edge_t>& edges);

    /*!
     * Returns:
     *  - intersected triangle index
     *  - index of point on the left of the line
     *  - index of point on the right of the line
     * If left point is right on the line: no triangle is intersected:
     *  - triangle index is no-neighbor (invalid)
     *  - index of point on the line
     *  - index of point on the right of the line
     */
    std::tuple<std::uint32_t, std::uint32_t, std::uint32_t>
    get_intersected_triangle(
        const std::uint32_t iA,
        const std::vector<std::uint32_t>& candidates,
        const vec2& a,
        const vec2& b,
        const double orientationTolerance) const
    {
        typedef std::vector<std::uint32_t>::const_iterator TriIndCit;
        for (TriIndCit it = candidates.begin(); it != candidates.end(); ++it) {
            const std::uint32_t iT = *it;
            const triangle_t t = triangles[iT];
            const std::uint32_t i = get_vertex_index(t, iA);
            const std::uint32_t iP2 = t.vertices[ccw(i)];
            const double orientP2 = orient2d(vertices[iP2], a, b);
            const point_to_line_location_t::Enum locP2 = classify_orientation(orientP2);

            if (locP2 == point_to_line_location_t::RIGHT_SIDE) {
                const std::uint32_t iP1 = t.vertices[cw(i)];
                const double orientP1 = orient2d(vertices[iP1], a, b);
                const point_to_line_location_t::Enum locP1 = classify_orientation(orientP1);
                if (locP1 == point_to_line_location_t::COLLINEAR) {
                    return std::make_tuple(null_neighbour, iP1, iP1);
                }
                if (locP1 == point_to_line_location_t::LEFT_SIDE) {
                    if (orientationTolerance) {
                        double closestOrient;
                        std::uint32_t iClosestP;
                        if (std::abs(orientP1) <= std::abs(orientP2)) {
                            closestOrient = orientP1;
                            iClosestP = iP1;
                        } else {
                            closestOrient = orientP2;
                            iClosestP = iP2;
                        }
                        if (classify_orientation(
                                closestOrient, orientationTolerance)
                            == point_to_line_location_t::COLLINEAR) {
                            return std::make_tuple(null_neighbour, iClosestP, iClosestP);
                        }
                    }
                    return std::make_tuple(iT, iP1, iP2);
                }
            }
        }
        throw std::runtime_error("Could not find vertex triangle intersected by "
                                 "edge. Note: can be caused by duplicate points.");
    }

    /// Returns indices of four resulting triangles
    /* Inserting a point on the edge between two triangles
     *    T0 (top)        v0
     *                   /|\
     *              n0 /  |  \ n3
     *               /    |    \
     *             /  T0' | Tnew0\
     *           v1-------v-------v3
     *             \ Tnew1| T1'  /
     *               \    |    /
     *              n1 \  |  / n2
     *                   \|/
     *   T1 (bottom)      v2
     */
    std::stack<std::uint32_t> insert_point_on_edge(
        const std::uint32_t vertex_index,
        const std::uint32_t edge_triangle0_index,
        const std::uint32_t edge_triangle1_index)
    {
        // create space for two new triangles
        const std::uint32_t new_triangle0_index = add_triangle();
        const std::uint32_t new_triangle1_index = add_triangle();

        // get the two triangles incident on edge to be split
        triangle_t& edge_triangle0 = triangles[edge_triangle0_index];
        triangle_t& edge_triangle1 = triangles[edge_triangle1_index];

        // get vertex of "edge_triangle0" that is opposite to triangle "edge_triangle1_index"
        const std::uint32_t triangle0_vertex_opposite_neighbour = get_vertex_local_id_opposite_neighbour(edge_triangle0, edge_triangle1_index);

        const std::uint32_t v0 = edge_triangle0.vertices[triangle0_vertex_opposite_neighbour];
        const std::uint32_t v1 = edge_triangle0.vertices[ccw(triangle0_vertex_opposite_neighbour)];

        const std::uint32_t n0 = edge_triangle0.neighbors[triangle0_vertex_opposite_neighbour];
        const std::uint32_t n3 = edge_triangle0.neighbors[cw(triangle0_vertex_opposite_neighbour)];

        // get vertex of "edge_triangle1" that is opposite to triangle "edge_triangle0_index"
        const std::uint32_t triangle1_opp_vertex_index = get_vertex_local_id_opposite_neighbour(edge_triangle1, edge_triangle0_index);

        const std::uint32_t v2 = edge_triangle1.vertices[triangle1_opp_vertex_index];
        const std::uint32_t v3 = edge_triangle1.vertices[ccw(triangle1_opp_vertex_index)];

        const std::uint32_t n2 = edge_triangle1.neighbors[triangle1_opp_vertex_index];
        const std::uint32_t n1 = edge_triangle1.neighbors[cw(triangle1_opp_vertex_index)];

        //
        // add new triangles and change existing ones
        //

        edge_triangle0 = {
            { v0, v1, vertex_index },
            { n0, new_triangle1_index, new_triangle0_index }
        }; // triangle_t::make(arr3(v0, v1, vertex_index), arr3(n0, new_triangle1_index, new_triangle0_index));

        edge_triangle1 = {
            { v2, v3, vertex_index },
            { n2, new_triangle0_index, new_triangle1_index }
        }; // triangle_t::make(arr3(v2, v3, vertex_index), arr3(n2, new_triangle0_index, new_triangle1_index));

        triangles[new_triangle0_index] = {
            { v0, vertex_index, v3 },
            { edge_triangle0_index, edge_triangle1_index, n3 }
        }; // triangle_t::make(arr3(v0, vertex_index, v3), arr3(edge_triangle0_index, edge_triangle1_index, n3));

        triangles[new_triangle1_index] = {
            { v2, vertex_index, v1 },
            { edge_triangle1_index, edge_triangle0_index, n1 }
        }; // triangle_t::make(arr3(v2, vertex_index, v1), arr3(edge_triangle1_index, edge_triangle0_index, n1));

        // make and add new vertex
        vertex_register_adjacent_triangles(
            vertex_index,
            // incident triangles
            edge_triangle0_index,
            new_triangle1_index,
            edge_triangle1_index,
            new_triangle0_index);

        // adjust neighboring triangles and vertices
        triangle_replace_neighbour(n3, edge_triangle0_index, new_triangle0_index);
        triangle_replace_neighbour(n1, edge_triangle1_index, new_triangle1_index);

        vertex_register_adjacent_triangle(v0, new_triangle0_index);
        vertex_register_adjacent_triangle(v2, new_triangle1_index);

        vertex_deregister_adjacent_triangle(v1, edge_triangle1_index);

        vertex_register_adjacent_triangle(v1, new_triangle1_index);

        vertex_deregister_adjacent_triangle(v3, edge_triangle0_index);

        vertex_register_adjacent_triangle(v3, new_triangle0_index);

        // return newly added triangles
        std::stack<std::uint32_t> new_triangles_stack;
        new_triangles_stack.push(edge_triangle0_index);
        new_triangles_stack.push(new_triangle1_index);
        new_triangles_stack.push(edge_triangle1_index);
        new_triangles_stack.push(new_triangle0_index);

        return new_triangles_stack;
    }

    std::array<std::uint32_t, 2> trianglesAt(const vec2& pos) const
    {
        std::array<std::uint32_t, 2> out = { null_neighbour, null_neighbour };
        for (std::uint32_t i = std::uint32_t(0); i < std::uint32_t(triangles.size()); ++i) {
            const triangle_t& t = triangles[i];
            const vec2& v0 = vertices[t.vertices[0]];
            const vec2& v1 = vertices[t.vertices[1]];
            const vec2& v2 = vertices[t.vertices[2]];
            const point_to_triangle_location_t::Enum loc = locate_point_wrt_triangle(pos, v0, v1, v2);
            if (loc == point_to_triangle_location_t::OUTSIDE)
                continue;
            out[0] = i;
            if (check_on_edge(loc))
                out[1] = t.neighbors[edge_neighbour(loc)];
            return out;
        }
        throw std::runtime_error("No triangle was found at position");
    }

    /**
     * Ensure that triangulation conforms to constraints (fixed edges)
     * @note For each fixed edge that is not present in the triangulation its
     * midpoint is recursively added until the original edge is represented by a
     * sequence of its pieces. <b> New vertices are inserted.</b>
     * @note If some edge appears more than once the input this
     * means that multiple boundaries overlap at the edge and impacts how hole
     * detection algorithm of triangulator_t::erase_outer_triangles_and_holes works.
     * <b>Make sure there are no erroneous duplicates.</b>
     * @tparam TEdgeIter iterator that dereferences to custom edge type
     * @tparam TGetEdgeVertexStart function object getting start vertex index
     * from an edge.
     * Getter signature: const TEdgeIter::value_type& -> std::uint32_t
     * @tparam TGetEdgeVertexEnd function object getting end vertex index from
     * an edge. Getter signature: const TEdgeIter::value_type& -> std::uint32_t
     * @param first beginning of the range of edges to add
     * @param last end of the range of edges to add
     * @param getStart getter of edge start vertex index
     * @param getEnd getter of edge end vertex index
     */
    template <
        typename TEdgeIter,
        typename TGetEdgeVertexStart,
        typename TGetEdgeVertexEnd>
    void conform_to_edges(
        TEdgeIter first,
        TEdgeIter last,
        TGetEdgeVertexStart getStart,
        TGetEdgeVertexEnd getEnd);

    /*!
     * Returns:
     *  - intersected triangle index
     *  - index of point on the left of the line
     *  - index of point on the right of the line
     * If left point is right on the line: no triangle is intersected:
     *  - triangle index is no-neighbor (invalid)
     *  - index of point on the line
     *  - index of point on the right of the line
     */
    std::tuple<std::uint32_t, std::uint32_t, std::uint32_t>
    TintersectedTriangle(
        const std::uint32_t iA,
        const std::vector<std::uint32_t>& candidates,
        const vec2& a,
        const vec2& b,
        const double orientationTolerance = double(0)) const
    {
        typedef std::vector<std::uint32_t>::const_iterator TriIndCit;
        for (TriIndCit it = candidates.begin(); it != candidates.end(); ++it) {
            const std::uint32_t iT = *it;
            const triangle_t t = triangles[iT];
            const std::uint32_t i = get_vertex_index(t, iA);
            const std::uint32_t iP2 = t.vertices[ccw(i)];
            const double orientP2 = orient2D(vertices[iP2], a, b);
            const point_to_line_location_t::Enum locP2 = classify_orientation(orientP2);
            if (locP2 == point_to_line_location_t::RIGHT_SIDE) {
                const std::uint32_t iP1 = t.vertices[cw(i)];
                const double orientP1 = orient2D(vertices[iP1], a, b);
                const point_to_line_location_t::Enum locP1 = classify_orientation(orientP1);
                if (locP1 == point_to_line_location_t::COLLINEAR) {
                    return std::make_tuple(null_neighbour, iP1, iP1);
                }
                if (locP1 == point_to_line_location_t::LEFT_SIDE) {
                    if (orientationTolerance) {
                        double closestOrient;
                        std::uint32_t iClosestP;
                        if (std::abs(orientP1) <= std::abs(orientP2)) {
                            closestOrient = orientP1;
                            iClosestP = iP1;
                        } else {
                            closestOrient = orientP2;
                            iClosestP = iP2;
                        }
                        if (classify_orientation(
                                closestOrient, orientationTolerance)
                            == point_to_line_location_t::COLLINEAR) {
                            return std::make_tuple(null_neighbour, iClosestP, iClosestP);
                        }
                    }
                    return std::make_tuple(iT, iP1, iP2);
                }
            }
        }
        throw std::runtime_error("Could not find vertex triangle intersected by "
                                 "edge. Note: can be caused by duplicate points.");
    }

    /// Returns indices of three resulting triangles
    /* Insert point into triangle: split into 3 triangles:
     *  - create 2 new triangles
     *  - re-use old triangle for the 3rd
     *                      v3
     *                    / | \
     *                   /  |  \ <-- original triangle (t)
     *                  /   |   \
     *              n3 /    |    \ n2
     *                /newT2|newT1\
     *               /      v      \
     *              /    __/ \__    \
     *             /  __/       \__  \
     *            / _/      t'     \_ \
     *          v1 ___________________ v2
     *                     n1
     */
    std::stack<std::uint32_t> insert_point_in_triangle(
        const std::uint32_t point_index,
        const std::uint32_t triangle_index)
    {
        //
        // create new triangle (their vertices and neighbour information)
        //

        // add triangle into the current triangulation (with default status)
        const std::uint32_t new_triangle0_index = add_triangle();
        const std::uint32_t new_triangle1_index = add_triangle();

        // ... to split
        triangle_t& triangle = triangles[triangle_index];

        const std::array<std::uint32_t, 3>& triangle_vertices = triangle.vertices;
        const std::array<std::uint32_t, 3>& triangle_neighbours = triangle.neighbors;

        // const std::uint32_t triangle_vertices[0] = triangle_vertices[0], v2 = triangle_vertices[1], v3 = triangle_vertices[2];
        // const std::uint32_t n1 = triangle_neighbours[0], n2 = triangle_neighbours[1], n3 = triangle_neighbours[2];
        //  make two new triangles and convert current triangle to 3rd new
        //  triangle
        // using detail::arr3;
        triangles[new_triangle0_index] = {
            { triangle_vertices[1], triangle_vertices[2], point_index },
            { triangle_neighbours[1], new_triangle1_index, triangle_index }
        };
        triangles[new_triangle1_index] = {
            { triangle_vertices[2], triangle_vertices[0], point_index },
            { triangle_neighbours[2], triangle_index, new_triangle0_index }
        };
        triangle = {
            { triangle_vertices[0], triangle_vertices[1], point_index },
            { triangle_neighbours[0], new_triangle0_index, new_triangle1_index }
        };

        //
        // Update internal connectivity graph
        //

        // make and add a new vertex
        vertex_register_adjacent_triangles(point_index, triangle_index, new_triangle0_index, new_triangle1_index);
        // adjust lists of adjacent triangles for triangle_vertices[0], triangle_vertices[1], triangle_vertices[2]
        vertex_register_adjacent_triangle(triangle_vertices[0], new_triangle1_index);
        vertex_register_adjacent_triangle(triangle_vertices[1], new_triangle0_index);

        // dissassociate triangle from vertex (triangle has taken on a new role even
        // though index has not changed)
        vertex_deregister_adjacent_triangle(triangle_vertices[2], triangle_index);

        vertex_register_adjacent_triangle(triangle_vertices[2], new_triangle0_index);
        vertex_register_adjacent_triangle(triangle_vertices[2], new_triangle1_index);

        // change triangle neighbours to new triangles
        triangle_replace_neighbour(triangle_neighbours[1], triangle_index, new_triangle0_index);
        triangle_replace_neighbour(triangle_neighbours[2], triangle_index, new_triangle1_index);

        // return newly added triangles
        std::stack<std::uint32_t> new_triangles_stack;

        new_triangles_stack.push(triangle_index);
        new_triangles_stack.push(new_triangle0_index);
        new_triangles_stack.push(new_triangle1_index);

        return new_triangles_stack;
    }

    // find the triangle containing "position"
    std::array<std::uint32_t, 2> find_triangle_containing_point(const vec2& position) const
    {
        std::array<std::uint32_t, 2> out = { null_neighbour, null_neighbour };

        // Query  for a vertex close to "position", to start the search
        const std::uint32_t nearest_point_index = m_nearPtLocator.find_nearest_point(position, vertices);
        const std::uint32_t starting_vertex_index = nearest_point_index;
        // https://raw.githubusercontent.com/klutometis/aima/master/papers/green-computing-dirichlet-tessellations-in-the-plane.pdf
        const std::uint32_t containing_triangle_index = walk_triangles(starting_vertex_index, position);
        // Finished walk, locate point in "containing_triangle"
        const triangle_t& containing_triangle = triangles[containing_triangle_index];
        const vec2& vertex0_coords = vertices[containing_triangle.vertices[0]];
        const vec2& vertex1_coords = vertices[containing_triangle.vertices[1]];
        const vec2& vertex2_coords = vertices[containing_triangle.vertices[2]];

        const point_to_triangle_location_t::Enum loc = locate_point_wrt_triangle(position, vertex0_coords, vertex1_coords, vertex2_coords);

        if (loc == point_to_triangle_location_t::OUTSIDE) {
            // logic error: it means "walk_triangles" was not actually able to find
            // a triangle containing "position"
            throw std::runtime_error("Failed to find triangle containing position");
        }

        out[0] = containing_triangle_index;

        if (check_on_edge(loc)) // does "position" actually lies on an edge of "containing_triangle"?
        {
            // The neighbouring triangle that shares the edge on which "position"
            // lies.
            // NOTE: we need this information to later handle one this nasty
            // point-on-edge cases.
            const uint32_t neighbour_triangle_index = edge_neighbour(loc);
            out[1] = containing_triangle.neighbors[neighbour_triangle_index];
        }

        return out;
    }
    /**
     * Ensure that triangulation conforms to constraints (fixed edges)
     * @note For each fixed edge that is not present in the triangulation its
     * midpoint is recursively added until the original edge is represented by a
     * sequence of its pieces. <b> New vertices are inserted.</b>
     * @note If some edge appears more than once the input this
     * means that multiple boundaries overlap at the edge and impacts how hole
     * detection algorithm of triangulator_t::erase_outer_triangles_and_holes works.
     * <b>Make sure there are no erroneous duplicates.</b>
     * @tparam edges edges to conform to
     */
    void conform_to_edges(const std::vector<edge_t>& edges);
    /**
     * Erase triangles adjacent to super triangle
     *
     * @note does nothing if custom geometry is used
     */
    void eraseSuperTriangle();
    /// Erase triangles outside of constrained boundary using growing
    void erase_outer_triangles();
    /**
     * Erase triangles outside of constrained boundary and auto-detected holes
     *
     * @note detecting holes relies on layer peeling based on layer depth
     * @note supports overlapping or touching boundaries
     */
    void erase_outer_triangles_and_holes();
    /**
     * Call this method after directly setting custom super-geometry via
     * vertices and triangles members
     */
    void initialise_with_custom_supergeometry();

    /**
     * Check if the triangulation was finalized with `erase...` method and
     * super-triangle was removed.
     * @return true if triangulation is finalized, false otherwise
     */
    bool is_finalized() const;

    /**
     * Calculate depth of each triangle in constraint triangulation. Supports
     * overlapping boundaries.
     *
     * Perform depth peeling from super triangle to outermost boundary,
     * then to next boundary and so on until all triangles are traversed.@n
     * For example depth is:
     *  - 0 for triangles outside outermost boundary
     *  - 1 for triangles inside boundary but outside hole
     *  - 2 for triangles in hole
     *  - 3 for triangles in island and so on...
     * @return vector where element at index i stores depth of i-th triangle
     */

    std::vector<layer_depth_t>
    calculate_triangle_depths() const
    {
        std::vector<layer_depth_t> triDepths(triangles.size(), std::numeric_limits<layer_depth_t>::max());
        std::stack<std::uint32_t> seeds(std::deque<std::uint32_t>(1, per_vertex_adjacent_triangles[0].front()));
        layer_depth_t layerDepth = 0;
        layer_depth_t deepestSeedDepth = 0;

        std::unordered_map<layer_depth_t, std::unordered_set<std::uint32_t>> seedsByDepth;

        do {
            const std::unordered_map<std::uint32_t, layer_depth_t>& newSeeds = peel_layer(seeds, layerDepth, triDepths);

            seedsByDepth.erase(layerDepth);
            for (std::unordered_map<std::uint32_t, layer_depth_t>::const_iterator it = newSeeds.begin(); it != newSeeds.end(); ++it) {
                deepestSeedDepth = std::max(deepestSeedDepth, it->second);
                seedsByDepth[it->second].insert(it->first);
            }
            const std::unordered_set<std::uint32_t>& nextLayerSeeds = seedsByDepth[layerDepth + 1];
            seeds = std::stack<std::uint32_t>(
                std::deque<std::uint32_t>(nextLayerSeeds.begin(), nextLayerSeeds.end()));
            ++layerDepth;
        } while (!seeds.empty() || deepestSeedDepth > layerDepth);

        return triDepths;
    }

    /**
     * @defgroup Advanced Advanced triangulator_t Methods
     * Advanced methods for manually modifying the triangulation from
     * outside. Please only use them when you know what you are doing.
     */
    /// @{

    /**
     * Flip an edge between two triangle.
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     * @param iT first triangle
     * @param iTopo second triangle

     */
    void do_edgeflip(std::uint32_t iT, std::uint32_t iTopo);

    /**
     * Remove triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @param removedTriangles indices of triangles to remove
     */
    void remove_triangles(const std::unordered_set<std::uint32_t>& removedTriangles);
    /// @}

private:
    /*____ Detail __*/
    void add_super_triangle(const bounding_box_t<vec2>& box);
    void create_vertex(const vec2& pos, const std::vector<std::uint32_t>& tris);
    void insert_vertex_into_triangulation(std::uint32_t vertex_index);
    void enforce_delaunay_property_using_edge_flips(
        const vec2& v,
        std::uint32_t vertex_index,
        std::stack<std::uint32_t>& stack_of_triangle_indices);
    /// Flip fixed edges and return a list of flipped fixed edges
    std::vector<edge_t> insert_vertex_and_flip_fixed_edges(std::uint32_t vertex_index);
    /**
     * Insert an edge into constraint Delaunay triangulation
     * @param edge edge to insert
     * @param originalEdge original edge inserted edge is part of
     */
    void insert_edge(edge_t edge, edge_t originalEdge);
    /**
     * Conform Delaunay triangulation to a fixed edge by recursively inserting
     * mid point of the edge and then conforming to its halves
     * @param edge fixed edge to conform to
     * @param originalEdges original edges that new edge is piece of
     * @param overlaps count of overlapping boundaries at the edge. Only used
     * when re-introducing edge with overlaps > 0
     * @param orientationTolerance tolerance for orient2d predicate,
     * values [-tolerance,+tolerance] are considered as 0.
     */
    void conform_to_edge(
        edge_t edge,
        std::vector<edge_t> originalEdges,
        boundary_overlap_count_t overlaps);

    std::uint32_t walk_triangles(std::uint32_t startVertex, const vec2& pos) const;
    bool check_if_edgeflip_required(
        const vec2& v,
        std::uint32_t iV,
        std::uint32_t iV1,
        std::uint32_t iV2,
        std::uint32_t iV3) const;
    bool
    check_if_edgeflip_required(const vec2& v, std::uint32_t iT, std::uint32_t iTopo, std::uint32_t iVert) const;
    void triangle_replace_neighbour(std::uint32_t iT, std::uint32_t oldNeighbor, std::uint32_t newNeighbor);
    void triangle_replace_neighbour(
        std::uint32_t iT,
        std::uint32_t iVedge1,
        std::uint32_t iVedge2,
        std::uint32_t newNeighbor);
    void vertex_register_adjacent_triangle(std::uint32_t iVertex, std::uint32_t iTriangle);
    void
    vertex_register_adjacent_triangles(std::uint32_t iVertex, std::uint32_t iT1, std::uint32_t iT2, std::uint32_t iT3);
    void vertex_register_adjacent_triangles(
        std::uint32_t iVertex,
        std::uint32_t iT1,
        std::uint32_t iT2,
        std::uint32_t iT3,
        std::uint32_t iT4);
    void vertex_deregister_adjacent_triangle(std::uint32_t iVertex, std::uint32_t iTriangle);
    std::uint32_t triangulate_pseudo_polygon(
        std::uint32_t ia,
        std::uint32_t ib,
        std::vector<std::uint32_t>::const_iterator pointsFirst,
        std::vector<std::uint32_t>::const_iterator pointsLast);
    std::uint32_t find_delaunay_point(
        std::uint32_t ia,
        std::uint32_t ib,
        std::vector<std::uint32_t>::const_iterator pointsFirst,
        std::vector<std::uint32_t>::const_iterator pointsLast) const;
    std::uint32_t pseudo_polygon_outer_triangle(std::uint32_t ia, std::uint32_t ib) const;
    std::uint32_t add_triangle(const triangle_t& t); // note: invalidates iterators!
    std::uint32_t add_triangle(); // note: invalidates triangle iterators!
    /**
     * Remove super-triangle (if used) and triangles with specified indices.
     * Adjust internal triangulation state accordingly.
     * @removedTriangles indices of triangles to remove
     */
    void finalise_triangulation(const std::unordered_set<std::uint32_t>& removedTriangles);
    std::unordered_set<std::uint32_t> grow_to_boundary(std::stack<std::uint32_t> seeds) const;

    void fixEdge(
        const edge_t& edge,
        const boundary_overlap_count_t overlaps)
    {
        m_constrained_edges.insert(edge);
        overlapCount[edge] = overlaps; // override overlap counter
    }

    void fixEdge(const edge_t& edge)
    {
        if (!m_constrained_edges.insert(edge).second) {
            ++overlapCount[edge]; // if edge is already fixed increment the counter
        }
    }

    void fixEdge(
        const edge_t& edge,
        const edge_t& originalEdge)
    {
        fixEdge(edge);
        if (edge != originalEdge)
            detail::insert_unique(pieceToOriginals[edge], originalEdge);
    }
    /**
     * Flag triangle as dummy
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     * @param iT index of a triangle to flag
     */
    void make_dummies(const std::uint32_t iT)
    {
        const triangle_t& t = triangles[iT];

        typedef std::array<std::uint32_t, 3>::const_iterator VCit;
        for (VCit iV = t.vertices.begin(); iV != t.vertices.end(); ++iV)
            vertex_deregister_adjacent_triangle(*iV, iT);

        typedef std::array<std::uint32_t, 3>::const_iterator NCit;
        for (NCit iTn = t.neighbors.begin(); iTn != t.neighbors.end(); ++iTn)
            triangle_replace_neighbour(*iTn, iT, null_neighbour);

        m_dummyTris.push_back(iT);
    }
    /**
     * Erase all dummy triangles
     * @note Advanced method for manually modifying the triangulation from
     * outside. Please call it when you know what you are doing.
     */
    void erase_dummies()
    {
        if (m_dummyTris.empty())
            return;
        const std::unordered_set<std::uint32_t> dummySet(m_dummyTris.begin(), m_dummyTris.end());
        std::unordered_map<std::uint32_t, std::uint32_t> triIndMap;
        triIndMap[null_neighbour] = null_neighbour;
        for (std::uint32_t iT(0), iTnew(0); iT < std::uint32_t(triangles.size()); ++iT) {
            if (dummySet.count(iT))
                continue;
            triIndMap[iT] = iTnew;
            triangles[iTnew] = triangles[iT];
            iTnew++;
        }
        triangles.erase(triangles.end() - dummySet.size(), triangles.end());

        // remap adjacent triangle indices for vertices
        typedef typename std::vector<std::vector<std::uint32_t>>::iterator VertTrisIt;
        for (VertTrisIt vTris = per_vertex_adjacent_triangles.begin(); vTris != per_vertex_adjacent_triangles.end(); ++vTris) {
            for (std::vector<std::uint32_t>::iterator iT = vTris->begin(); iT != vTris->end(); ++iT)
                *iT = triIndMap[*iT];
        }
        // remap neighbor indices for triangles
        for (std::vector<triangle_t>::iterator t = triangles.begin(); t != triangles.end(); ++t) {
            std::array<std::uint32_t, 3>& nn = t->neighbors;
            for (std::array<std::uint32_t, 3>::iterator iN = nn.begin(); iN != nn.end(); ++iN)
                *iN = triIndMap[*iN];
        }
        // clear dummy triangles
        m_dummyTris = std::vector<std::uint32_t>();
    }

    /**
     * Depth-peel a layer in triangulation, used when calculating triangle
     * depths
     *
     * It takes starting seed triangles, traverses neighboring triangles, and
     * assigns given layer depth to the traversed triangles. Traversal is
     * blocked by constraint edges. Triangles behind constraint edges are
     * recorded as seeds of next layer and returned from the function.
     *
     * @param seeds indices of seed triangles
     * @param layerDepth current layer's depth to mark triangles with
     * @param[in, out] triDepths depths of triangles
     * @return triangles of the deeper layers that are adjacent to the peeled
     * layer. To be used as seeds when peeling deeper layers.
     */
    std::unordered_map<std::uint32_t, layer_depth_t>
    peel_layer(
        std::stack<std::uint32_t> seeds,
        const layer_depth_t layerDepth,
        std::vector<layer_depth_t>& triDepths) const
    {
        std::unordered_map<std::uint32_t, layer_depth_t> behindBoundary;
        while (!seeds.empty()) {
            const std::uint32_t iT = seeds.top();
            seeds.pop();
            triDepths[iT] = layerDepth;
            behindBoundary.erase(iT);
            const triangle_t& t = triangles[iT];
            for (std::uint32_t i(0); i < std::uint32_t(3); ++i) {
                const edge_t opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
                const std::uint32_t iN = t.neighbors[get_local_index_of_neighbour_opposite_vertex(i)];
                if (iN == null_neighbour || triDepths[iN] <= layerDepth)
                    continue;
                if (m_constrained_edges.count(opEdge)) {
                    const std::unordered_map<edge_t, layer_depth_t>::const_iterator cit = overlapCount.find(opEdge);
                    const layer_depth_t triDepth = cit == overlapCount.end()
                        ? layerDepth + 1
                        : layerDepth + cit->second + 1;
                    behindBoundary[iN] = triDepth;
                    continue;
                }
                seeds.push(iN);
            }
        }
        return behindBoundary;
    }

    std::vector<std::uint32_t> m_dummyTris;
    locator_kdtree_t<> m_nearPtLocator;
    std::size_t m_nTargetVerts;
    super_geometry_type_t::Enum m_superGeomType;
    vertex_insertion_order_t::Enum m_vertexInsertionOrder;
    action_on_intersecting_constraint_edges_t::Enum m_intersectingEdgesStrategy;
    double m_minDistToConstraintEdge;
};

/// @}
/// @}

namespace detail {

    static std::mt19937 randGenerator(9001);

    template <class RandomIt>
    void random_shuffle(RandomIt first, RandomIt last)
    {
        typename std::iterator_traits<RandomIt>::difference_type i, n;
        n = last - first;
        for (i = n - 1; i > 0; --i) {
            std::swap(first[i], first[randGenerator() % (i + 1)]);
        }
    }

} // namespace detail

//-----------------------
// triangulator_t methods
//-----------------------
template <typename TVertexIter>
void triangulator_t::insert_vertices(
    const TVertexIter first,
    const TVertexIter last /*,
     TGetVertexCoordX get_x_coord,
     TGetVertexCoordY get_y_coord*/
)
{
    MCUT_ASSERT(is_finalized() == false); // super-triangle was erased already

    //if () {
    //    throw std::runtime_error(
    //        "triangulator_t was finalized with 'erase...' method. Inserting new "
    //        "vertices is not possible");
    //}

    // TODO: remove this.
    //detail::randGenerator.seed(9001); // ensure deterministic behavior

    if (vertices.empty()) {
        const bounding_box_t<vec2> bbox = construct_bbox_containing_points(first, last /*, get_x_coord, get_y_coord*/);
        // first time we are inserting vertices, so let us create the super triangle
        add_super_triangle(bbox);
    }

    // i.e before inserting the new vertices
    const std::uint32_t preexisting_vertex_count = (std::uint32_t)vertices.size();
    const std::uint32_t new_vertex_count = (std::uint32_t)std::distance(first, last);

    // reserve some internal memory for all vertices
    vertices.reserve(preexisting_vertex_count + new_vertex_count);

    // for each new vertex
    for (TVertexIter it = first; it != last; ++it) {
        // create/insert new vertex with its coordinates, and (empty) list of associated triangles that we are yet to determine
        const vec2& coordinates = *it; // get_x_coord(*it), get_y_coord(*it));
        create_vertex(coordinates, std::vector<std::uint32_t>());
    }

    //switch (m_vertexInsertionOrder) {

    //case vertex_insertion_order_t::AS_GIVEN: {

        // for each vertex to be inserted
        for (TVertexIter it = first; it != last; ++it) {
            const std::uint32_t vertex_local_id = (std::uint32_t)std::distance(first, it); // amongst the new vertices
            const std::uint32_t vertex_global_id = preexisting_vertex_count + vertex_local_id; // amongst all vertices

            // do the usual steps:
            // find nearest points in current triangulation
            // get nearest triangle from nearest point
            // walk triangles to find triangle containing vertex
            // split triangle (may be more than one triangle to split if vertex lies on edge) 
            // enforce delauney property with edge flips
            insert_vertex_into_triangulation(vertex_global_id);
        }

     //   break;
   // }
    //case vertex_insertion_order_t::RANDOM: {
    //    std::vector<std::uint32_t> ii(std::distance(first, last));
    //    typedef std::vector<std::uint32_t>::iterator Iter;
    //    std::uint32_t value = static_cast<std::uint32_t>(preexisting_vertex_count);
    //    for (Iter it = ii.begin(); it != ii.end(); ++it, ++value)
    //        *it = value;
    //    detail::random_shuffle(ii.begin(), ii.end());
    //    for (Iter it = ii.begin(); it != ii.end(); ++it)
    //        insert_vertex_into_triangulation(*it);
    //    break;
    //}
    }
}

template <
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd>
void triangulator_t::insert_edges(
    TEdgeIter first,
    const TEdgeIter last,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd)
{
    if (is_finalized()) {
        throw std::runtime_error(
            "triangulator_t was finalized with 'erase...' method. Inserting new "
            "edges is not possible");
    }
    for (; first != last; ++first) {
        // +3 to account for super-triangle vertices
        const edge_t edge(
            std::uint32_t(getStart(*first) + m_nTargetVerts),
            std::uint32_t(getEnd(*first) + m_nTargetVerts));
        insert_edge(edge, edge);
    }
    erase_dummies();
}

template <
    typename TEdgeIter,
    typename TGetEdgeVertexStart,
    typename TGetEdgeVertexEnd>
void triangulator_t::conform_to_edges(
    TEdgeIter first,
    const TEdgeIter last,
    TGetEdgeVertexStart getStart,
    TGetEdgeVertexEnd getEnd)
{
    if (is_finalized()) {
        throw std::runtime_error(
            "triangulator_t was finalized with 'erase...' method. Conforming to "
            "new edges is not possible");
    }
    for (; first != last; ++first) {
        // +3 to account for super-triangle vertices
        const edge_t e(
            std::uint32_t(getStart(*first) + m_nTargetVerts),
            std::uint32_t(getEnd(*first) + m_nTargetVerts));
        conform_to_edge(e, std::vector<edge_t>(1, e), 0);
    }
    erase_dummies();
}

} // namespace cdt

#ifndef CDT_USE_AS_COMPILED_LIBRARY
// #include "triangulator_t.hpp"

#include <algorithm>
#include <cassert>
#include <deque>
#include <stdexcept>

namespace cdt {

triangulator_t::triangulator_t()
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(detail::defaults::vertexInsertionOrder)
    , m_intersectingEdgesStrategy(detail::defaults::intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(detail::defaults::minDistToConstraintEdge)
{
}

triangulator_t::triangulator_t(
    const vertex_insertion_order_t::Enum vertexInsertionOrder)
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(vertexInsertionOrder)
    , m_intersectingEdgesStrategy(detail::defaults::intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(detail::defaults::minDistToConstraintEdge)
{
}

triangulator_t::triangulator_t(
    const vertex_insertion_order_t::Enum vertexInsertionOrder,
    const action_on_intersecting_constraint_edges_t::Enum intersectingEdgesStrategy,
    const double minDistToConstraintEdge)
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(vertexInsertionOrder)
    , m_intersectingEdgesStrategy(intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(minDistToConstraintEdge)
{
}

triangulator_t::triangulator_t(
    const vertex_insertion_order_t::Enum vertexInsertionOrder,
    const locator_kdtree_t<>& nearPtLocator,
    const action_on_intersecting_constraint_edges_t::Enum intersectingEdgesStrategy,
    const double minDistToConstraintEdge)
    : m_nTargetVerts(detail::defaults::nTargetVerts)
    , m_nearPtLocator(nearPtLocator)
    , m_superGeomType(detail::defaults::superGeomType)
    , m_vertexInsertionOrder(vertexInsertionOrder)
    , m_intersectingEdgesStrategy(intersectingEdgesStrategy)
    , m_minDistToConstraintEdge(minDistToConstraintEdge)
{
}

void triangulator_t::triangle_replace_neighbour(
    const std::uint32_t iT,
    const std::uint32_t iVedge1,
    const std::uint32_t iVedge2,
    const std::uint32_t newNeighbor)
{
    triangle_t& t = triangles[iT];
    t.neighbors[get_local_index_of_neighbour_opposite_vertex(t, iVedge1, iVedge2)] = newNeighbor;
}

void triangulator_t::eraseSuperTriangle()
{
    if (m_superGeomType != super_geometry_type_t::SUPER_TRIANGLE)
        return;
    // find triangles adjacent to super-triangle's vertices
    std::unordered_set<std::uint32_t> toErase;
    toErase.reserve(
        per_vertex_adjacent_triangles[0].size() + per_vertex_adjacent_triangles[1].size() + per_vertex_adjacent_triangles[2].size());
    for (std::uint32_t iT(0); iT < std::uint32_t(triangles.size()); ++iT) {
        triangle_t& t = triangles[iT];
        if (t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3)
            toErase.insert(iT);
    }
    finalise_triangulation(toErase);
}

// remove those triangles whch lie in-between the super triangle and the
// (outermost) polygon boundary

void triangulator_t::erase_outer_triangles()
{
    // make dummy triangles adjacent to super-triangle's vertices
    const std::stack<std::uint32_t> seed(std::deque<std::uint32_t>(1, per_vertex_adjacent_triangles[0].front()));
    const std::unordered_set<std::uint32_t> toErase = grow_to_boundary(seed);
    finalise_triangulation(toErase);
}

void triangulator_t::erase_outer_triangles_and_holes()
{
    const std::vector<layer_depth_t> triDepths = calculate_triangle_depths();
    std::unordered_set<std::uint32_t> toErase;
    toErase.reserve(triangles.size());

    for (std::size_t iT = 0; iT != triangles.size(); ++iT) {
        if (triDepths[iT] % 2 == 0) {
            toErase.insert(static_cast<std::uint32_t>(iT));
        }
    }

    finalise_triangulation(toErase);
}

/// Remap removing super-triangle: subtract 3 from vertices
inline edge_t remap_no_supertriangle(const edge_t& e)
{
    return edge_t(e.v1() - 3, e.v2() - 3);
}

void triangulator_t::remove_triangles(
    const std::unordered_set<std::uint32_t>& removedTriangles)
{
    if (removedTriangles.empty())
        return;
    // remove triangles and calculate triangle index mapping
    std::unordered_map<std::uint32_t, std::uint32_t> triIndMap;
    for (std::uint32_t iT(0), iTnew(0); iT < std::uint32_t(triangles.size()); ++iT) {
        if (removedTriangles.count(iT))
            continue;
        triIndMap[iT] = iTnew;
        triangles[iTnew] = triangles[iT];
        iTnew++;
    }
    triangles.erase(triangles.end() - removedTriangles.size(), triangles.end());
    // adjust triangles' neighbors
    per_vertex_adjacent_triangles = std::vector<std::vector<std::uint32_t>>();
    for (std::uint32_t iT = 0; iT < triangles.size(); ++iT) {
        triangle_t& t = triangles[iT];
        // update neighbors to account for removed triangles
        std::array<std::uint32_t, 3>& nn = t.neighbors;
        for (std::array<std::uint32_t, 3>::iterator n = nn.begin(); n != nn.end(); ++n) {
            if (removedTriangles.count(*n)) {
                *n = null_neighbour;
            } else if (*n != null_neighbour) {
                *n = triIndMap[*n];
            }
        }
    }
}

void triangulator_t::finalise_triangulation(
    const std::unordered_set<std::uint32_t>& removedTriangles)
{
    erase_dummies();
    // remove super-triangle
    if (m_superGeomType == super_geometry_type_t::SUPER_TRIANGLE) {
        vertices.erase(vertices.begin(), vertices.begin() + 3);
        if (removedTriangles.empty())
            per_vertex_adjacent_triangles.erase(per_vertex_adjacent_triangles.begin(), per_vertex_adjacent_triangles.begin() + 3);
        // edge_t re-mapping
        { // fixed edges
            std::unordered_set<edge_t> updatedFixedEdges;
            typedef std::unordered_set<edge_t>::const_iterator It;
            for (It e = m_constrained_edges.begin(); e != m_constrained_edges.end(); ++e) {
                updatedFixedEdges.insert(remap_no_supertriangle(*e));
            }
            m_constrained_edges = updatedFixedEdges;
        }
        { // overlap count
            std::unordered_map<edge_t, boundary_overlap_count_t> updatedOverlapCount;
            typedef std::unordered_map<edge_t, boundary_overlap_count_t>::const_iterator
                It;
            for (It it = overlapCount.begin(); it != overlapCount.end(); ++it) {
                updatedOverlapCount.insert(std::make_pair(
                    remap_no_supertriangle(it->first), it->second));
            }
            overlapCount = updatedOverlapCount;
        }
        { // split edges mapping
            std::unordered_map<edge_t, std::vector<edge_t>> updatedPieceToOriginals;
            typedef std::unordered_map<edge_t, std::vector<edge_t>>::const_iterator It;
            for (It it = pieceToOriginals.begin(); it != pieceToOriginals.end();
                 ++it) {
                std::vector<edge_t> ee = it->second;
                for (std::vector<edge_t>::iterator eeIt = ee.begin(); eeIt != ee.end();
                     ++eeIt) {
                    *eeIt = remap_no_supertriangle(*eeIt);
                }
                updatedPieceToOriginals.insert(
                    std::make_pair(remap_no_supertriangle(it->first), ee));
            }
            pieceToOriginals = updatedPieceToOriginals;
        }
    }
    // remove other triangles
    remove_triangles(removedTriangles);
    // adjust triangle vertices: account for removed super-triangle
    if (m_superGeomType == super_geometry_type_t::SUPER_TRIANGLE) {
        for (std::vector<triangle_t>::iterator t = triangles.begin(); t != triangles.end();
             ++t) {
            std::array<std::uint32_t, 3>& vv = t->vertices;
            for (std::array<std::uint32_t, 3>::iterator v = vv.begin(); v != vv.end(); ++v) {
                *v -= 3;
            }
        }
    }
}

void triangulator_t::initialise_with_custom_supergeometry()
{
    m_nearPtLocator.initialize(vertices);
    m_nTargetVerts = vertices.size();
    m_superGeomType = super_geometry_type_t::CUSTOM;
}

std::unordered_set<std::uint32_t> triangulator_t::grow_to_boundary(
    std::stack<std::uint32_t> seeds) const
{
    std::unordered_set<std::uint32_t> traversed;
    while (!seeds.empty()) {
        const std::uint32_t iT = seeds.top();
        seeds.pop();
        traversed.insert(iT);
        const triangle_t& t = triangles[iT];
        for (std::uint32_t i(0); i < std::uint32_t(3); ++i) {
            const edge_t opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
            if (m_constrained_edges.count(opEdge))
                continue;
            const std::uint32_t iN = t.neighbors[get_local_index_of_neighbour_opposite_vertex(i)];
            if (iN != null_neighbour && traversed.count(iN) == 0)
                seeds.push(iN);
        }
    }
    return traversed;
}

std::uint32_t triangulator_t::add_triangle(const triangle_t& t)
{
    if (m_dummyTris.empty()) {
        triangles.push_back(t);
        return std::uint32_t(triangles.size() - 1);
    }
    const std::uint32_t nxtDummy = m_dummyTris.back();
    m_dummyTris.pop_back();
    triangles[nxtDummy] = t;
    return nxtDummy;
}

// create a new triangle
std::uint32_t triangulator_t::add_triangle()
{
    // check cache
    if (m_dummyTris.empty()) {
        // create cache entry
        const triangle_t dummy = {
            { null_vertex, null_vertex, null_vertex },
            { null_neighbour, null_neighbour, null_neighbour }
        };

        // add new triangle (with default status)
        triangles.push_back(dummy);

        // return the index of the newly added triangle
        return std::uint32_t(triangles.size() - 1);
    }
    const std::uint32_t nxtDummy = m_dummyTris.back();
    m_dummyTris.pop_back();
    return nxtDummy;
}

void triangulator_t::insert_edges(
    const std::vector<edge_t>& edges)
{
    insert_edges(edges.begin(), edges.end(), edge_get_v1, edge_get_v2);
}

void triangulator_t::conform_to_edges(
    const std::vector<edge_t>& edges)
{
    conform_to_edges(edges.begin(), edges.end(), edge_get_v1, edge_get_v2);
}

namespace detail {

    double lerp(const double& a, const double& b, const double t)
    {
        return (double(1) - t) * a + t * b;
    }

    // Precondition: ab and cd intersect normally
    vec2 get_intersection_point_coords(
        const vec2& a,
        const vec2& b,
        const vec2& c,
        const vec2& d)
    {
        // interpolate point on the shorter segment
        if (get_square_distance(a, b) < get_square_distance(c, d)) {
            // const double a_cd = orient2d(c.x(), c.y(), d.x(), d.y(), a.x(), a.y());
            // const double b_cd = orient2d(c.x(), c.y(), d.x(), d.y(), b.x(), b.y());
            const double a_cd = orient2d(c, d, a);
            const double b_cd = orient2d(c, d, b);
            const double t = a_cd / (a_cd - b_cd);
            return vec2::make(lerp(a.x(), b.x(), t), lerp(a.y(), b.y(), t));
        } else {
            // const double c_ab = orient2d(a.x(), a.y(), b.x(), b.y(), c.x(), c.y());
            // const double d_ab = orient2d(a.x(), a.y(), b.x(), b.y(), d.x(), d.y());
            const double c_ab = orient2d(a, b, c);
            const double d_ab = orient2d(a, b, d);
            const double t = c_ab / (c_ab - d_ab);
            return vec2::make(lerp(c.x(), d.x(), t), lerp(c.y(), d.y(), t));
        }
    }

} // namespace detail

void triangulator_t::insert_edge(
    const edge_t edge,
    const edge_t originalEdge)
{
    const std::uint32_t iA = edge.v1();
    std::uint32_t iB = edge.v2();
    if (iA == iB) // edge connects a vertex to itself
        return;
    const std::vector<std::uint32_t>& aTris = per_vertex_adjacent_triangles[iA];
    const std::vector<std::uint32_t>& bTris = per_vertex_adjacent_triangles[iB];
    const vec2& a = vertices[iA];
    const vec2& b = vertices[iB];
    if (check_vertices_share_edge(aTris, bTris)) {
        fixEdge(edge, originalEdge);
        return;
    }

    const double distanceTolerance = m_minDistToConstraintEdge == double(0)
        ? double(0)
        : m_minDistToConstraintEdge * distance(a, b);

    std::uint32_t iT;
    std::uint32_t iVleft, iVright;
    std::tie(iT, iVleft, iVright) = get_intersected_triangle(iA, aTris, a, b, distanceTolerance);
    // if one of the triangle vertices is on the edge, move edge start
    if (iT == null_neighbour) {
        const edge_t edgePart(iA, iVleft);
        fixEdge(edgePart, originalEdge);
        return insert_edge(edge_t(iVleft, iB), originalEdge);
    }
    std::vector<std::uint32_t> intersected(1, iT);
    std::vector<std::uint32_t> ptsLeft(1, iVleft);
    std::vector<std::uint32_t> ptsRight(1, iVright);
    std::uint32_t iV = iA;
    triangle_t t = triangles[iT];
    while (std::find(t.vertices.begin(), t.vertices.end(), iB) == t.vertices.end()) {
        const std::uint32_t iTopo = get_global_triangle_index_opposite_vertex(t, iV);
        const triangle_t& tOpo = triangles[iTopo];
        const std::uint32_t iVopo = get_opposed_vertex_index(tOpo, iT);
        const vec2 vOpo = vertices[iVopo];

        // RESOLVE intersection between two constraint edges if needed
        if (m_intersectingEdgesStrategy == action_on_intersecting_constraint_edges_t::RESOLVE && m_constrained_edges.count(edge_t(iVleft, iVright))) {
            const std::uint32_t iNewVert = static_cast<std::uint32_t>(vertices.size());

            // split constraint edge that already exists in triangulation
            const edge_t splitEdge(iVleft, iVright);
            const edge_t half1(iVleft, iNewVert);
            const edge_t half2(iNewVert, iVright);
            const boundary_overlap_count_t overlaps = overlapCount[splitEdge];
            // remove the edge that will be split
            m_constrained_edges.erase(splitEdge);
            overlapCount.erase(splitEdge);
            // add split edge's halves
            fixEdge(half1, overlaps);
            fixEdge(half2, overlaps);
            // maintain piece-to-original mapping
            std::vector<edge_t> newOriginals(1, splitEdge);
            const std::unordered_map<edge_t, std::vector<edge_t>>::const_iterator originalsIt = pieceToOriginals.find(splitEdge);
            if (originalsIt != pieceToOriginals.end()) { // edge being split was split before: pass-through originals
                newOriginals = originalsIt->second;
                pieceToOriginals.erase(originalsIt);
            }
            detail::insert_unique(pieceToOriginals[half1], newOriginals);
            detail::insert_unique(pieceToOriginals[half2], newOriginals);

            // add a new point at the intersection of two constraint edges
            const vec2 newV = detail::get_intersection_point_coords(
                vertices[iA],
                vertices[iB],
                vertices[iVleft],
                vertices[iVright]);
            create_vertex(newV, std::vector<std::uint32_t>());
            std::stack<std::uint32_t> stack_of_triangle_indices = insert_point_on_edge(iNewVert, iT, iTopo);
            enforce_delaunay_property_using_edge_flips(newV, iNewVert, stack_of_triangle_indices);
            // TODO: is it's possible to re-use pseudo-polygons
            //  for inserting [iA, iNewVert] edge half?
            insert_edge(edge_t(iA, iNewVert), originalEdge);
            insert_edge(edge_t(iNewVert, iB), originalEdge);
            return;
        }

        intersected.push_back(iTopo);
        iT = iTopo;
        t = triangles[iT];

        const point_to_line_location_t::Enum loc = locate_point_wrt_line(vOpo, a, b, distanceTolerance);
        if (loc == point_to_line_location_t::LEFT_SIDE) {
            ptsLeft.push_back(iVopo);
            iV = iVleft;
            iVleft = iVopo;
        } else if (loc == point_to_line_location_t::RIGHT_SIDE) {
            ptsRight.push_back(iVopo);
            iV = iVright;
            iVright = iVopo;
        } else // encountered point on the edge
            iB = iVopo;
    }
    // Remove intersected triangles
    typedef std::vector<std::uint32_t>::const_iterator TriIndCit;
    for (TriIndCit it = intersected.begin(); it != intersected.end(); ++it)
        make_dummies(*it);
    // Triangulate pseudo-polygons on both sides
    const std::uint32_t iTleft = triangulate_pseudo_polygon(iA, iB, ptsLeft.begin(), ptsLeft.end());
    std::reverse(ptsRight.begin(), ptsRight.end());
    const std::uint32_t iTright = triangulate_pseudo_polygon(iB, iA, ptsRight.begin(), ptsRight.end());
    triangle_replace_neighbour(iTleft, null_neighbour, iTright);
    triangle_replace_neighbour(iTright, null_neighbour, iTleft);

    if (iB != edge.v2()) // encountered point on the edge
    {
        // fix edge part
        const edge_t edgePart(iA, iB);
        fixEdge(edgePart, originalEdge);
        return insert_edge(edge_t(iB, edge.v2()), originalEdge);
    } else {
        fixEdge(edge, originalEdge);
    }
}

void triangulator_t::conform_to_edge(
    const edge_t edge,
    std::vector<edge_t> originalEdges,
    const boundary_overlap_count_t overlaps)
{
    const std::uint32_t iA = edge.v1();
    std::uint32_t iB = edge.v2();
    if (iA == iB) // edge connects a vertex to itself
        return;
    const std::vector<std::uint32_t>& aTris = per_vertex_adjacent_triangles[iA];
    const std::vector<std::uint32_t>& bTris = per_vertex_adjacent_triangles[iB];
    const vec2& a = vertices[iA];
    const vec2& b = vertices[iB];
    if (check_vertices_share_edge(aTris, bTris)) {
        overlaps > 0 ? fixEdge(edge, overlaps) : fixEdge(edge);
        // avoid marking edge as a part of itself
        if (!originalEdges.empty() && edge != originalEdges.front()) {
            detail::insert_unique(pieceToOriginals[edge], originalEdges);
        }
        return;
    }

    const double distanceTolerance = m_minDistToConstraintEdge == double(0)
        ? double(0)
        : m_minDistToConstraintEdge * distance(a, b);
    std::uint32_t iT;
    std::uint32_t iVleft, iVright;
    std::tie(iT, iVleft, iVright) = get_intersected_triangle(iA, aTris, a, b, distanceTolerance);
    // if one of the triangle vertices is on the edge, move edge start
    if (iT == null_neighbour) {
        const edge_t edgePart(iA, iVleft);
        overlaps > 0 ? fixEdge(edgePart, overlaps) : fixEdge(edgePart);
        detail::insert_unique(pieceToOriginals[edgePart], originalEdges);
        return conform_to_edge(edge_t(iVleft, iB), originalEdges, overlaps);
    }

    std::uint32_t iV = iA;
    triangle_t t = triangles[iT];
    while (std::find(t.vertices.begin(), t.vertices.end(), iB) == t.vertices.end()) {
        const std::uint32_t iTopo = get_global_triangle_index_opposite_vertex(t, iV);
        const triangle_t& tOpo = triangles[iTopo];
        const std::uint32_t iVopo = get_opposed_vertex_index(tOpo, iT);
        const vec2 vOpo = vertices[iVopo];

        // RESOLVE intersection between two constraint edges if needed
        if (m_intersectingEdgesStrategy == action_on_intersecting_constraint_edges_t::RESOLVE && m_constrained_edges.count(edge_t(iVleft, iVright))) {
            const std::uint32_t iNewVert = static_cast<std::uint32_t>(vertices.size());

            // split constraint edge that already exists in triangulation
            const edge_t splitEdge(iVleft, iVright);
            const edge_t half1(iVleft, iNewVert);
            const edge_t half2(iNewVert, iVright);
            const boundary_overlap_count_t overlaps = overlapCount[splitEdge];
            // remove the edge that will be split
            m_constrained_edges.erase(splitEdge);
            overlapCount.erase(splitEdge);
            // add split edge's halves
            fixEdge(half1, overlaps);
            fixEdge(half2, overlaps);
            // maintain piece-to-original mapping
            std::vector<edge_t> newOriginals(1, splitEdge);
            const std::unordered_map<edge_t, std::vector<edge_t>>::const_iterator originalsIt = pieceToOriginals.find(splitEdge);
            if (originalsIt != pieceToOriginals.end()) { // edge being split was split before: pass-through originals
                newOriginals = originalsIt->second;
                pieceToOriginals.erase(originalsIt);
            }
            detail::insert_unique(pieceToOriginals[half1], newOriginals);
            detail::insert_unique(pieceToOriginals[half2], newOriginals);

            // add a new point at the intersection of two constraint edges
            const vec2 newV = detail::get_intersection_point_coords(
                vertices[iA],
                vertices[iB],
                vertices[iVleft],
                vertices[iVright]);
            create_vertex(newV, std::vector<std::uint32_t>());
            std::stack<std::uint32_t> stack_of_triangle_indices = insert_point_on_edge(iNewVert, iT, iTopo);
            enforce_delaunay_property_using_edge_flips(newV, iNewVert, stack_of_triangle_indices);
            conform_to_edge(edge_t(iA, iNewVert), originalEdges, overlaps);
            conform_to_edge(edge_t(iNewVert, iB), originalEdges, overlaps);
            return;
        }

        iT = iTopo;
        t = triangles[iT];

        const point_to_line_location_t::Enum loc = locate_point_wrt_line(vOpo, a, b, distanceTolerance);
        if (loc == point_to_line_location_t::LEFT_SIDE) {
            iV = iVleft;
            iVleft = iVopo;
        } else if (loc == point_to_line_location_t::RIGHT_SIDE) {
            iV = iVright;
            iVright = iVopo;
        } else // encountered point on the edge
            iB = iVopo;
    }
    /**/

    // add mid-point to triangulation
    const std::uint32_t iMid = static_cast<std::uint32_t>(vertices.size());
    const vec2& start = vertices[iA];
    const vec2& end = vertices[iB];
    create_vertex(
        vec2::make((start.x() + end.x()) / 2.0, (start.y() + end.y()) / 2.0),
        std::vector<std::uint32_t>());
    const std::vector<edge_t> flippedFixedEdges = insert_vertex_and_flip_fixed_edges(iMid);

    conform_to_edge(edge_t(iA, iMid), originalEdges, overlaps);
    conform_to_edge(edge_t(iMid, iB), originalEdges, overlaps);
    // re-introduce fixed edges that were flipped
    // and make sure overlap count is preserved
    for (std::vector<edge_t>::const_iterator it = flippedFixedEdges.begin();
         it != flippedFixedEdges.end();
         ++it) {
        m_constrained_edges.erase(*it);

        boundary_overlap_count_t prevOverlaps = 0;
        const std::unordered_map<edge_t, boundary_overlap_count_t>::const_iterator
            overlapsIt
            = overlapCount.find(*it);
        if (overlapsIt != overlapCount.end()) {
            prevOverlaps = overlapsIt->second;
            overlapCount.erase(overlapsIt);
        }
        // override overlapping boundaries count when re-inserting an edge
        std::vector<edge_t> prevOriginals(1, *it);
        const std::unordered_map<edge_t, std::vector<edge_t>>::const_iterator originalsIt = pieceToOriginals.find(*it);
        if (originalsIt != pieceToOriginals.end()) {
            prevOriginals = originalsIt->second;
        }
        conform_to_edge(*it, prevOriginals, prevOverlaps);
    }
    if (iB != edge.v2())
        conform_to_edge(edge_t(iB, edge.v2()), originalEdges, overlaps);
}

void triangulator_t::add_super_triangle(const bounding_box_t<vec2>& box)
{
    m_nTargetVerts = 3;
    m_superGeomType = super_geometry_type_t::SUPER_TRIANGLE; // TODO remove this (we only care about super triangle)

    const vec2 bbox_centre = (box.minimum() + box.maximum()) / 2.0; // {
    //    (box.min.x() + box.max.x()) / 2.0, (box.min.y() + box.max.y()) / 2.0
    //};

    const double width = box.maximum().x() - box.minimum().x();
    const double height = box.maximum().y() - box.minimum().y();
    const double diagonal_length = (width * width) + (height * height); // i.e. squared length of hypotenuse

    // https://en.wikipedia.org/wiki/Incircle_and_excircles_of_a_triangle
    // NOTE: the incircle or inscribed circle of a triangle is the largest circle
    // that can be contained in the triangle;
    double incircle_radius = std::sqrt(diagonal_length) / 2.0; // half of actual length of hypotenuse
    incircle_radius *= 1.1; // scale up by a small contact
    // NOTE: An excircle or escribed circle of the triangle is a circle
    // lying outside the triangle, tangent to one of its sides and tangent to
    // the extensions of the other two. Every triangle has three distinct
    // excircles, each tangent to one of the triangle's sides
    // (we care only about one excircle because super triangle is equilateral)
    const double excircle_radius = 2.0 * incircle_radius; // excircle radius
    const double shift_x = excircle_radius * std::sqrt(3.0) / 2.0; // excircle_radius * cos(30 deg)

    // vertex coordinates of super triangle
    const vec2 vertex0_coords = { bbox_centre.x() - shift_x, bbox_centre.y() - incircle_radius };
    const vec2 vertex1_coords = { bbox_centre.x() + shift_x, bbox_centre.y() - incircle_radius };
    const vec2 vertex2_coords = { bbox_centre.x(), bbox_centre.y() + excircle_radius };

    // the super triangle is the first triangle, and its three vertices will
    // be associated with it.
    create_vertex(vertex0_coords, std::vector<std::uint32_t>(1, 0));
    create_vertex(vertex1_coords, std::vector<std::uint32_t>(1, 0));
    create_vertex(vertex2_coords, std::vector<std::uint32_t>(1, 0));

    // internal triangle representation of the super-triangle
    const triangle_t super_triangle = {
        { std::uint32_t(0), std::uint32_t(1), std::uint32_t(2) },
        // no neighbours (yet)
        { null_neighbour, null_neighbour, null_neighbour }
    };

    // register
    add_triangle(super_triangle);

    // add our new vertices into the KD-tree
    m_nearPtLocator.initialize(vertices);
}

// registers a new vertex with its coordinates and associated list of triangles

void triangulator_t::create_vertex(
    const vec2& pos,
    const std::vector<std::uint32_t>& tris)
{
    vertices.push_back(pos);
    per_vertex_adjacent_triangles.push_back(tris);
}

std::vector<edge_t>
triangulator_t::insert_vertex_and_flip_fixed_edges(
    const std::uint32_t iVert)
{
    std::vector<edge_t> flippedFixedEdges;

    const vec2& v = vertices[iVert];
    std::array<std::uint32_t, 2> triangle_containing_point_info = find_triangle_containing_point(v);
    std::stack<std::uint32_t> stack_of_triangle_indices = triangle_containing_point_info[1] == null_neighbour
        ? insert_point_in_triangle(iVert, triangle_containing_point_info[0])
        : insert_point_on_edge(iVert, triangle_containing_point_info[0], triangle_containing_point_info[1]);
    while (!stack_of_triangle_indices.empty()) {
        const std::uint32_t iT = stack_of_triangle_indices.top();
        stack_of_triangle_indices.pop();

        const triangle_t& t = triangles[iT];
        const std::uint32_t iTopo = get_global_triangle_index_opposite_vertex(t, iVert);
        if (iTopo == null_neighbour)
            continue;

        /*
         *                       v3         original edge: (v1, v3)
         *                      /|\   flip-candidate edge: (v,  v2)
         *                    /  |  \
         *                  /    |    \
         *                /      |      \
         * new vertex--> v       |       v2
         *                \      |      /
         *                  \    |    /
         *                    \  |  /
         *                      \|/
         *                       v1
         */
        const triangle_t& tOpo = triangles[iTopo];
        const std::uint32_t i = get_vertex_local_id_opposite_neighbour(tOpo, iT);
        const std::uint32_t iV2 = tOpo.vertices[i];
        const std::uint32_t iV1 = tOpo.vertices[cw(i)];
        const std::uint32_t iV3 = tOpo.vertices[ccw(i)];

        if (check_if_edgeflip_required(v, iVert, iV1, iV2, iV3)) {
            // if flipped edge is fixed, remember it
            const edge_t flippedEdge(iV1, iV3);
            if (m_constrained_edges.count(flippedEdge))
                flippedFixedEdges.push_back(flippedEdge);

            do_edgeflip(iT, iTopo);
            stack_of_triangle_indices.push(iT);
            stack_of_triangle_indices.push(iTopo);
        }
    }

    m_nearPtLocator.add_point(iVert, vertices);
    return flippedFixedEdges;
}

// formerlly insert a (previously allocated) vertex into the current
// triangulation. This vertex is identified by its index.
void triangulator_t::insert_vertex_into_triangulation(
    // index of vertex inserted into triangle
    const std::uint32_t vertex_index)
{
    // coordinates of vertex inserted into triangle
    const vec2& vertex_coords = vertices[vertex_index];

    // Array of two elements:
    // In the normal case, only the first element is defined (i.e. "triangle_containing_point_info[0]"),
    // which is the index of the triangle containing the point "vertex_coords".
    // Otherwise, the second element (i.e. "triangle_containing_point_info[1]") will also be
    // defined, which occurs when the point "vertex_coords" actually lies on an
    // edge of the containing triangle (i.e. "triangle_containing_point_info[0]").
    // In this instance, the second element (i.e. "triangle_containing_point_info[1]") refers
    // to the neighbour of the containing triangle that share the edge on which the point lies.
    std::array<std::uint32_t, 2> triangle_containing_point_info = find_triangle_containing_point(vertex_coords);

    const std::uint32_t triangle_containing_point_index = triangle_containing_point_info[0];
    const std::uint32_t triangle_containing_point_neighbour_index = triangle_containing_point_info[1];

    // if this is false then the vertex lies on an edge
    const bool vertex_lies_inside_triangle = (triangle_containing_point_neighbour_index == null_neighbour);

    // stack of the new triangles that have been created as a result of inserting
    // the vertex labelled "vertex_index" into the current triangulation
    std::stack<std::uint32_t> stack_of_triangle_indices;

    if (vertex_lies_inside_triangle) {
        // just insert the point into the triangle and sub-divide it as usual
        stack_of_triangle_indices = insert_point_in_triangle(vertex_index, triangle_containing_point_index);
    } else {
        // awkward case of having to splitting an edge and replacing the two adjacent triangles four new one
        stack_of_triangle_indices = insert_point_on_edge(vertex_index, triangle_containing_point_index, triangle_containing_point_neighbour_index);
    }

    // make all triangles in the current triangulation satisfy the delauney property
    enforce_delaunay_property_using_edge_flips(vertex_coords, vertex_index, stack_of_triangle_indices);

    m_nearPtLocator.add_point(vertex_index, vertices);
}

void triangulator_t::enforce_delaunay_property_using_edge_flips(
    // coordinates of a vertex (e.g that has just been inserted into the current triangulation)
    const vec2& vertex_coords,
    // label/index of a vertex (e.g that has just been inserted into the current triangulation)
    const std::uint32_t vertex_index,
    // the stack of the newly created triangules as a result of e.g. inserting a new vertex
    std::stack<std::uint32_t>& stack_of_triangle_indices)
{
    // while we have triangles to check i.e. to check if no other vertex in the
    // triangulation lies inside the circumcircle of a triangle's vertices.
    while (!stack_of_triangle_indices.empty()) {

        // index of the current triangle we are checking for the delauney property
        const std::uint32_t current_triangle_index = stack_of_triangle_indices.top();
        stack_of_triangle_indices.pop(); // remove

        const triangle_t& current_triangle = triangles[current_triangle_index];
        // get the triangle that lies opposite to the vertex labelled "vertex_index" in "current_triangle"
        const std::uint32_t global_index_of_neighbour_opposite_vertex = get_global_triangle_index_opposite_vertex(current_triangle, vertex_index);

        if (global_index_of_neighbour_opposite_vertex == null_neighbour) {
            continue;
        }

        // do we need to flip the edge that is shared by "current_triangle_index" and "global_index_of_neighbour_opposite_vertex"?
        const bool edge_flip_required = check_if_edgeflip_required(
            vertex_coords,
            current_triangle_index,
            global_index_of_neighbour_opposite_vertex,
            vertex_index);

        if (edge_flip_required) {

            //
            // flip the edges and add the new triangles to the stack so that we can
            // check whether they satisfy the delauney property
            //

            do_edgeflip(current_triangle_index, global_index_of_neighbour_opposite_vertex);

            stack_of_triangle_indices.push(current_triangle_index);
            stack_of_triangle_indices.push(global_index_of_neighbour_opposite_vertex);
        }
    } // while (!stack_of_triangle_indices.empty()) {
}

/*!
 * This function check if an edge flip is required by first checking with 
 * super-triangle vertices, which must be treated specially.
 * 
 * Moreover, these super-triangle vertices are not infinitely far away and will 
 * influence the input points.
 * 
 * There are three possible cases that may arise with super-triangle vertices
 * (see also the illustration below):
 * 
 *  1.  If any one of the opposed vertices (i.e. v1, v2 or v3) is a super-triangle vertex: 
 *          --> no flip needed (edge formed by v1 and v3 is a border edge)
 *  2.  If any one of the shared vertices is super-triangle vertex:
 *          --> check if on point is same side of line formed by non-super-tri vertices as the non-super-tri shared vertex
 *  3.  None of the vertices are super-tri: normal circumcircle test
 * 
 *  OR ALTERNATIVELY: These are the possible cases that may arise
 * 
 * * None of vertices belongs to the super-triangle.
 *      -> This is the most common caseand it is solved by the empty circle test
 * * Both vertices v1 and v3 belong to the super-triangle (i.e. common edge is one of the edges of the super-triangle). 
 *      -> This case is NOT possible because the edge of the super-triangle does not belong to more than one triangle
 * * One of vertices "v2" or "v" belongs to the super-triangle.
 *      -> Edge v1-v3 is taken as a correct (i.e. border) edge and there is no need to enforce delauney property.
 *  * One of vertices "v1" or "v3" belongs to the super-triangle
 *      -> This case causes problems if it is not considered adequately.
 */
/*
 *                       v3         original edge: (v1, v3)
 *                      /|\   flip-candidate edge: (v,  v2)
 *                    /  |  \
 *                  /    |    \
 *                /      |      \
 * new vertex--> v       |       v2
 *                \      |      /
 *                  \    |    /
 *                    \  |  /
 *                      \|/
 *                       v1
 * 
 */
bool triangulator_t::check_if_edgeflip_required(
    // coordinates of a vertex (e.g that has just been inserted into the current triangulation)
    const vec2& vertex_coords,
    const std::uint32_t vertex_index,
    const std::uint32_t iV1,
    const std::uint32_t iV2,
    const std::uint32_t iV3) const
{
    const vec2& v1 = vertices[iV1];
    const vec2& v2 = vertices[iV2];
    const vec2& v3 = vertices[iV3];

    bool answer = true;
    // if (m_superGeomType == super_geometry_type_t::SUPER_TRIANGLE) { // TODO: remove this check
    
    //
    //  The following if-condition check whether any one of the vertices in the
    //  stencil of the edge flip belongs to the super-triangle
    //

    // If the flip-candidate edge touches/is part of the super-triangle, then the 
    // "in-circumcircle" test has to be replaced with an "orient2d" test against 
    // the line formed by two non-artificial vertices (that don't belong to
    // super-triangle)
    if (vertex_index < 3 /*super-tri verticess have index < 3*/) // Does the "flip-candidate edge" touch the super-triangle? (i.e. does "v2" belongs to super-triangle)
    {
        // does original edge also touch super-triangle?
        if (iV1 < 3) {
            answer = locate_point_wrt_line(v1, v2, v3) == locate_point_wrt_line(vertex_coords, v2, v3);
        } else if (iV3 < 3) {
            answer = locate_point_wrt_line(v3, v1, v2) == locate_point_wrt_line(vertex_coords, v1, v2);
        } else {
            answer = false; // original edge does not touch super-triangle
        }
    } else if (iV2 < 3) // flip-candidate edge touches super-triangle
    {
        // does original edge also touch super-triangle?
        if (iV1 < 3) {
            answer = locate_point_wrt_line(v1, vertex_coords, v3) == locate_point_wrt_line(v2, vertex_coords, v3);
        } else if (iV3 < 3) {
            answer = locate_point_wrt_line(v3, v1, vertex_coords) == locate_point_wrt_line(v2, v1, vertex_coords);
        } else {
            answer = false; // original edge does not touch super-triangle
        }
    }
    // flip-candidate edge does not touch super-triangle
    else if (iV1 < 3) {
        answer = locate_point_wrt_line(v1, v2, v3) == locate_point_wrt_line(vertex_coords, v2, v3);
    } else if (iV3 < 3) {
        answer = locate_point_wrt_line(v3, v1, v2) == locate_point_wrt_line(vertex_coords, v1, v2);
    } else {
        answer = check_is_in_circumcircle(vertex_coords, v1, v2, v3);
    }
    //}
    return answer;
}

// This function 1) extracts the indices of the (four) vertices in the stencil of an edge flip
// 2) checks if the edge to be flipped is actually constrained and 3) check if the
// edge flip even required (i.e. maybe the current triangle does not even
// violate the delaunay property)
//
bool triangulator_t::check_if_edgeflip_required(
    // coordinates of a vertex (e.g that has just been inserted into the current triangulation)
    const vec2& vertex_coords,
    // index of the current triangle we are checking for the delauney property
    const std::uint32_t current_triangle_index,
    // the triangle that lies opposite to the vertex labelled "vertex_index" in "current_triangle"
    const std::uint32_t current_triangle_neighbour_index,
    // index of a vertex (e.g that has just been inserted into the current triangulation)
    const std::uint32_t vertex_index) const
{
    /*
     *                       v2         original edge: (v0, v2)
     *                      /|\   flip-candidate edge: (v,  v1)
     *                    /  |  \
     *                  /    |    \
     *                /      |      \
     * new vertex--> v       |       v1
     *                \      |      /
     *                  \    |    /
     *                    \  |  /
     *                      \|/
     *                       v0
     */

    // get the neighbour triangle's data
    const triangle_t& neighbour = triangles[current_triangle_neighbour_index];
    // index of the vertex that lies opposite to the current triangle in the neighbour (i.e. v1 in the illustation)
    const std::uint32_t neighbour_opp_vertex_local_index = get_vertex_local_id_opposite_neighbour(neighbour, current_triangle_index);
    const std::uint32_t neighbour_vertex1_index = neighbour.vertices[neighbour_opp_vertex_local_index];
    const std::uint32_t neighbour_vertex0_index = neighbour.vertices[cw(neighbour_opp_vertex_local_index)];
    const std::uint32_t neighbour_vertex2_index = neighbour.vertices[ccw(neighbour_opp_vertex_local_index)];

    // NOTE: we only check vertices 0 and 2 because, they are already forming an
    // edge that is part of the current triangulation and we do not know (without
    // checking) whether the edge they form is specified as  "fixed" by th user
    // (i.e. cannot be flipped).
    edge_t edge(neighbour_vertex0_index, neighbour_vertex2_index);
    const bool edge_is_constrained = m_constrained_edges.count(edge);

    // flip not needed if the original edge is fixed
    if (edge_is_constrained) {
        return false;
    }

    const bool edge_flip_requred = check_if_edgeflip_required(
        vertex_coords,
        vertex_index,
        neighbour_vertex0_index,
        neighbour_vertex1_index,
        neighbour_vertex2_index);

    return edge_flip_requred;
}

// Find the triangle containing "position". So what we do is start from
// the vertex that has the index "starting_vertex_index" (which we determined from KD-tree
// search to speed things up). We use the (first) triangle associated with "starting_vertex_index"
// to walk/traverse the current triangulation in-order-to find the triangle that contains
// the location "position".
std::uint32_t triangulator_t::walk_triangles(const std::uint32_t starting_vertex_index, const vec2& position) const
{
    // begin the walk/traversal to search for the triangle containing "position"
    std::uint32_t current_triangle_index = per_vertex_adjacent_triangles[starting_vertex_index][0];
    // set of visited triangles
    std::unordered_set<std::uint32_t> visited;

    bool found = false;
    while (!found) {

        const triangle_t& current_triangle = triangles[current_triangle_index];
        found = true;
        // stochastic offset to randomize which edge we check first
        const std::uint32_t offset(detail::randGenerator() % 3); // TODO: remove this

        // for each edge of current triangle (num edges = num vertices)
        for (std::uint32_t i_ = 0; i_ < std::uint32_t(3); ++i_) {

            const std::uint32_t i((i_ + offset) % 3);
            const vec2& edge_v0_coords = vertices[current_triangle.vertices[i]];
            const vec2& edge_v1_coords = vertices[current_triangle.vertices[ccw(i)]];
            const point_to_line_location_t::Enum orientation = locate_point_wrt_line(position, edge_v0_coords, edge_v1_coords);

            if ( // point is on RHS of edge (based on winding order) i.e. it is outside triangle
                orientation == point_to_line_location_t::RIGHT_SIDE && //
                // neighbour (of RHS of edge) exists
                current_triangle.neighbors[i] != null_neighbour && //
                // we have NOT already visited neighbour on RHS of edge
                visited.insert(current_triangle.neighbors[i]).second) {
                found = false;
                // update the current triangle to the neighbour, which we will visit
                // next (to check if it might contain "position")
                current_triangle_index = current_triangle.neighbors[i];
                break;
            }
        }
    }
    return current_triangle_index;
}

/* Flip edge between T and Topo:
 *
 *                v4         | - old edge
 *               /|\         ~ - new edge
 *              / | \
 *          n3 /  T' \ n4
 *            /   |   \
 *           /    |    \
 *     T -> v1~~~~~~~~~v3 <- Topo
 *           \    |    /
 *            \   |   /
 *          n1 \Topo'/ n2
 *              \ | /
 *               \|/
 *                v2
 */

void triangulator_t::do_edgeflip(
    const std::uint32_t triangle_global_id,
    const std::uint32_t triangle_neighbour_global_id)
{
    triangle_t& triangle = triangles[triangle_global_id];
    triangle_t& neighbour = triangles[triangle_neighbour_global_id];

    const std::array<std::uint32_t, 3>& triangle_neighbours = triangle.neighbors;
    const std::array<std::uint32_t, 3>& neighbour_neighbours = neighbour.neighbors;
    const std::array<std::uint32_t, 3>& triangle_vertices = triangle.vertices;
    const std::array<std::uint32_t, 3>& neighbour_vertices = neighbour.vertices;

    // find vertices and neighbors
    std::uint32_t vertex_local_id_opposite_neighbour = get_vertex_local_id_opposite_neighbour(triangle, triangle_neighbour_global_id);

    const std::uint32_t v1 = triangle_vertices[vertex_local_id_opposite_neighbour];
    const std::uint32_t v2 = triangle_vertices[ccw(vertex_local_id_opposite_neighbour)];

    const std::uint32_t n1 = triangle_neighbours[vertex_local_id_opposite_neighbour];
    const std::uint32_t n3 = triangle_neighbours[cw(vertex_local_id_opposite_neighbour)];

    std::uint32_t vertex_local_id_opposite_triangle = get_vertex_local_id_opposite_neighbour(neighbour, triangle_neighbour_global_id);

    const std::uint32_t v3 = neighbour_vertices[vertex_local_id_opposite_triangle];
    const std::uint32_t v4 = neighbour_vertices[ccw(vertex_local_id_opposite_triangle)];

    const std::uint32_t n4 = neighbour_neighbours[vertex_local_id_opposite_triangle];
    const std::uint32_t n2 = neighbour_neighbours[cw(vertex_local_id_opposite_triangle)];

    // change vertices and neighbors
    using detail::arr3;

    triangle = {{v4, v1, v3}, {n3, triangle_neighbour_global_id, n4}};// triangle_t::make(arr3(v4, v1, v3), arr3(n3, triangle_neighbour_global_id, n4));
    neighbour = {{v2, v3, v1}, {n2, triangle_global_id, n1}}; //triangle_t::make(arr3(v2, v3, v1), arr3(n2, triangle_global_id, n1));

    // adjust neighboring triangles and vertices
    triangle_replace_neighbour(n1, triangle_global_id, triangle_neighbour_global_id);
    triangle_replace_neighbour(n4, triangle_neighbour_global_id, triangle_global_id);

    // only adjust adjacent triangles if triangulation is not finalized:
    // can happen when called from outside on an already finalized triangulation
    if (!is_finalized()) {

        vertex_register_adjacent_triangle(v1, triangle_neighbour_global_id);
        vertex_register_adjacent_triangle(v3, triangle_global_id);

        vertex_deregister_adjacent_triangle(v2, triangle_global_id);
        vertex_deregister_adjacent_triangle(v4, triangle_neighbour_global_id);
    }
}

void triangulator_t::triangle_replace_neighbour(
    const std::uint32_t triangle_index,
    const std::uint32_t old_neighbour,
    const std::uint32_t new_neighbour)
{
    if (triangle_index == null_neighbour) {
        return;
    }

    triangle_t& triangle = triangles[triangle_index];

    triangle.neighbors[get_neighbour_index(triangle, old_neighbour)] = new_neighbour;
}

void triangulator_t::vertex_register_adjacent_triangle(
    const std::uint32_t vertex_index,
    const std::uint32_t triangle_index)
{
    per_vertex_adjacent_triangles[vertex_index].push_back(triangle_index);
}

// associates the given three triangles to the vertex labelled "vertex_index"
void triangulator_t::vertex_register_adjacent_triangles(
    const std::uint32_t vertex_index,
    const std::uint32_t triangle0_index,
    const std::uint32_t triangle1_index,
    const std::uint32_t triangle2_index)
{
    std::vector<std::uint32_t>& adjacent_triangles = per_vertex_adjacent_triangles[vertex_index];

    adjacent_triangles.reserve(adjacent_triangles.size() + 3ul);

    adjacent_triangles.push_back(triangle0_index);
    adjacent_triangles.push_back(triangle1_index);
    adjacent_triangles.push_back(triangle2_index);
}

void triangulator_t::vertex_register_adjacent_triangles(
    const std::uint32_t vertex_index,
    const std::uint32_t triangle0_index,
    const std::uint32_t triangle1_index,
    const std::uint32_t triangle2_index,
    const std::uint32_t triangle3_index)
{
    std::vector<std::uint32_t>& adjacent_triangles = per_vertex_adjacent_triangles[vertex_index];

    adjacent_triangles.reserve(adjacent_triangles.size() + 4);

    adjacent_triangles.push_back(triangle0_index);
    adjacent_triangles.push_back(triangle1_index);
    adjacent_triangles.push_back(triangle2_index);
    adjacent_triangles.push_back(triangle3_index);
}

// disassociate the triangle from vertex
void triangulator_t::vertex_deregister_adjacent_triangle(
    const std::uint32_t vertex_index,
    const std::uint32_t triangle_index)
{
    std::vector<std::uint32_t>& adjacent_triangles = per_vertex_adjacent_triangles[vertex_index];
    std::vector<std::uint32_t>::iterator fiter = std::find(adjacent_triangles.begin(), adjacent_triangles.end(), triangle_index);
    // The iterator pos must be valid and dereferenceable. Thus the end()
    // iterator (which is valid, but is not dereferenceable) cannot be used as
    // a value for input to "erase".
    MCUT_ASSERT(fiter != adjacent_triangles.end());
    adjacent_triangles.erase(fiter);
}

std::uint32_t triangulator_t::triangulate_pseudo_polygon(
    const std::uint32_t ia,
    const std::uint32_t ib,
    const std::vector<std::uint32_t>::const_iterator pointsFirst,
    const std::vector<std::uint32_t>::const_iterator pointsLast)
{
    if (pointsFirst == pointsLast)
        return pseudo_polygon_outer_triangle(ia, ib);

    // Find delaunay point
    const std::uint32_t ic = find_delaunay_point(ia, ib, pointsFirst, pointsLast);
    // Find pseudopolygons split by the delaunay point
    std::vector<std::uint32_t>::const_iterator newLast = pointsFirst;

    while (*newLast != ic) {
        ++newLast;
    }

    const std::vector<std::uint32_t>::const_iterator newFirst = newLast + 1;
    // triangulate splitted pseudo-polygons
    const std::uint32_t iT2 = triangulate_pseudo_polygon(ic, ib, newFirst, pointsLast);
    const std::uint32_t iT1 = triangulate_pseudo_polygon(ia, ic, pointsFirst, newLast);
    // add new triangle
    const triangle_t t = { { ia, ib, ic }, { null_neighbour, iT2, iT1 } };
    const std::uint32_t iT = add_triangle(t);

    // adjust neighboring triangles and vertices
    if (iT1 != null_neighbour) {
        if (pointsFirst == newLast) {
            triangle_replace_neighbour(iT1, ia, ic, iT);
        } else {
            triangles[iT1].neighbors[0] = iT;
        }
    }
    if (iT2 != null_neighbour) {
        if (newFirst == pointsLast) {
            triangle_replace_neighbour(iT2, ic, ib, iT);
        } else {
            triangles[iT2].neighbors[0] = iT;
        }
    }

    vertex_register_adjacent_triangle(ia, iT);
    vertex_register_adjacent_triangle(ib, iT);
    vertex_register_adjacent_triangle(ic, iT);

    return iT;
}

std::uint32_t triangulator_t::find_delaunay_point(
    const std::uint32_t ia,
    const std::uint32_t ib,
    const std::vector<std::uint32_t>::const_iterator pointsFirst,
    const std::vector<std::uint32_t>::const_iterator pointsLast) const
{
    MCUT_ASSERT(pointsFirst != pointsLast);

    const vec2& a = vertices[ia];
    const vec2& b = vertices[ib];

    std::uint32_t ic = *pointsFirst;
    vec2 c = vertices[ic];

    for (std::vector<std::uint32_t>::const_iterator it = pointsFirst + 1; it != pointsLast; ++it) {

        const vec2 v = vertices[*it];

        if (!check_is_in_circumcircle(v, a, b, c)) {
            continue;
        }

        ic = *it;
        c = vertices[ic];
    }
    return ic;
}

std::uint32_t triangulator_t::pseudo_polygon_outer_triangle(
    const std::uint32_t ia,
    const std::uint32_t ib) const
{

    const std::vector<std::uint32_t>& aTris = per_vertex_adjacent_triangles[ia];
    const std::vector<std::uint32_t>& bTris = per_vertex_adjacent_triangles[ib];

    for (std::vector<std::uint32_t>::const_iterator it = aTris.begin(); it != aTris.end(); ++it) {
        if (std::find(bTris.begin(), bTris.end(), *it) != bTris.end()) {
            return *it;
        }
    }

    return null_neighbour;
}

void triangulator_t::insert_vertices(
    const std::vector<vec2>& new_vertices)
{
    return insert_vertices(
        new_vertices.begin(),
        new_vertices.end() /*,
         get_x_coord_vec2d<T>,
         get_y_coord_vec2d<T>*/
    );
}

bool triangulator_t::is_finalized() const
{
    return per_vertex_adjacent_triangles.empty() && !vertices.empty();
}

} // namespace cdt
#endif

#endif // header-guard
