#ifndef CDT_obwOaxOTdAWcLNTlNnaq
#define CDT_obwOaxOTdAWcLNTlNnaq

#define CDT_CXX11_IS_SUPPORTED
#define CDT_EXPORT
/**
 * Macro for inlining non-template functions when in header-only mode to
 * avoid multiple declaration errors.
 */
#define CDT_INLINE_IF_HEADER_ONLY inline

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

// use fall-backs for c++11 features
#ifdef CDT_CXX11_IS_SUPPORTED

#include <array>
#include <functional>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
namespace CDT {
using std::array;
using std::get;
using std::make_tuple;
using std::mt19937;
using std::tie;
using std::tuple;
using std::unordered_map;
using std::unordered_set;
} // namespace CDT

#else

#endif

namespace CDT {

/// 2D vector
template <typename T>
struct CDT_EXPORT V2d {
    T x; ///< X-coordinate
    T y; ///< Y-coordinate

    /// Create vector from X and Y coordinates
    static V2d make(T x, T y);
};

/// X- coordinate getter for V2d
template <typename T>
const T& getX_V2d(const V2d<T>& v)
{
    return v.x;
}

/// Y-coordinate getter for V2d
template <typename T>
const T& getY_V2d(const V2d<T>& v)
{
    return v.y;
}

/// If two 2D vectors are exactly equal
template <typename T>
bool operator==(const CDT::V2d<T>& lhs, const CDT::V2d<T>& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

#ifdef CDT_USE_64_BIT_INDEX_TYPE
typedef unsigned long long IndexSizeType;
#else
typedef unsigned int IndexSizeType;
#endif

#ifndef CDT_USE_STRONG_TYPING
/// Index in triangle
typedef unsigned char Index;
/// Vertex index
typedef IndexSizeType VertInd;
/// Triangle index
typedef IndexSizeType TriInd;
#else
#endif

/// Constant representing no valid neighbor for a triangle
const static TriInd noNeighbor(std::numeric_limits<TriInd>::max());
/// Constant representing no valid vertex for a triangle
const static VertInd noVertex(std::numeric_limits<VertInd>::max());

typedef std::vector<TriInd> TriIndVec; ///< Vector of triangle indices
typedef array<VertInd, 3> VerticesArr3; ///< array of three vertex indices
typedef array<TriInd, 3> NeighborsArr3; ///< array of three neighbors

/// 2D bounding box
template <typename T>
struct CDT_EXPORT Box2d {
    V2d<T> min; ///< min box corner
    V2d<T> max; ///< max box corner

    /// Envelop box around a point
    void envelopPoint(const V2d<T>& p)
    {
        envelopPoint(p.x, p.y);
    }
    /// Envelop box around a point with given coordinates
    void envelopPoint(const T x, const T y)
    {
        min.x = std::min(x, min.x);
        max.x = std::max(x, max.x);
        min.y = std::min(y, min.y);
        max.y = std::max(y, max.y);
    }
};

/// Bounding box of a collection of custom 2D points given coordinate getters
template <
    typename T,
    typename TVertexIter,
    typename TGetVertexCoordX,
    typename TGetVertexCoordY>
Box2d<T> envelopBox(
    TVertexIter first,
    TVertexIter last,
    TGetVertexCoordX getX,
    TGetVertexCoordY getY)
{
    const T max = std::numeric_limits<T>::max();
    Box2d<T> box = { { max, max }, { -max, -max } };
    for (; first != last; ++first) {
        box.envelopPoint(getX(*first), getY(*first));
    }
    return box;
}

/// Bounding box of a collection of 2D points
template <typename T>
CDT_EXPORT Box2d<T> envelopBox(const std::vector<V2d<T>>& vertices);

/// Edge connecting two vertices: vertex with smaller index is always first
/// \note: hash Edge is specialized at the bottom
struct CDT_EXPORT Edge {
    /// Constructor
    Edge(VertInd iV1, VertInd iV2);
    /// Equals operator
    bool operator==(const Edge& other) const;
    /// Not-equals operator
    bool operator!=(const Edge& other) const;
    /// V1 getter
    VertInd v1() const;
    /// V2 getter
    VertInd v2() const;
    /// Edges' vertices
    const std::pair<VertInd, VertInd>& verts() const;

private:
    std::pair<VertInd, VertInd> m_vertices;
};

/// Get edge first vertex
inline VertInd edge_get_v1(const Edge& e)
{
    return e.v1();
}

/// Get edge second vertex
inline VertInd edge_get_v2(const Edge& e)
{
    return e.v2();
}

/// Get edge second vertex
inline Edge edge_make(VertInd iV1, VertInd iV2)
{
    return Edge(iV1, iV2);
}

typedef std::vector<Edge> EdgeVec; ///< Vector of edges
typedef unordered_set<Edge> EdgeUSet; ///< Hash table of edges
typedef unordered_set<TriInd> TriIndUSet; ///< Hash table of triangles
typedef unordered_map<TriInd, TriInd> TriIndUMap; ///< Triangle hash map

/// Triangulation triangle (CCW winding)
/* Counter-clockwise winding:
       v3
       /\
    n3/  \n2
     /____\
   v1  n1  v2                 */
struct CDT_EXPORT Triangle {
    VerticesArr3 vertices; ///< triangle's three vertices
    NeighborsArr3 neighbors; ///< triangle's three neighbors

    /**
     * Factory method
     * @note needed for c++03 compatibility (no uniform initialization
     * available)
     */
    static Triangle
    make(const array<VertInd, 3>& vertices, const array<TriInd, 3>& neighbors)
    {
        Triangle t = { vertices, neighbors };
        return t;
    }
};

typedef std::vector<Triangle> TriangleVec; ///< Vector of triangles

/// Advance vertex or neighbor index counter-clockwise
CDT_EXPORT Index ccw(Index i);

/// Advance vertex or neighbor index clockwise
CDT_EXPORT Index cw(Index i);

/// Location of point on a triangle
struct CDT_EXPORT PtTriLocation {
    /// Enum
    enum Enum {
        Inside,
        Outside,
        OnEdge1,
        OnEdge2,
        OnEdge3,
    };
};

/// Check if location is classified as on any of three edges
CDT_EXPORT bool isOnEdge(PtTriLocation::Enum location);

/// Neighbor index from a on-edge location
/// \note Call only if located on the edge!
CDT_EXPORT Index edgeNeighbor(PtTriLocation::Enum location);

/// Relative location of point to a line
struct CDT_EXPORT PtLineLocation {
    /// Enum
    enum Enum {
        Left,
        Right,
        OnLine,
    };
};

/// Orient p against line v1-v2 2D: robust geometric predicate
template <typename T>
CDT_EXPORT T orient2D(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2);

/// Check if point lies to the left of, to the right of, or on a line
template <typename T>
CDT_EXPORT PtLineLocation::Enum locatePointLine(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    T orientationTolerance = T(0));

/// Classify value of orient2d predicate
template <typename T>
CDT_EXPORT PtLineLocation::Enum
classifyOrientation(T orientation, T orientationTolerance = T(0));

/// Check if point a lies inside of, outside of, or on an edge of a triangle
template <typename T>
CDT_EXPORT PtTriLocation::Enum locatePointTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3);

/// Opposed neighbor index from vertex index
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY Index opoNbr(Index vertIndex);

/// Opposed vertex index from neighbor index
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY Index opoVrt(Index neighborIndex);

/// Index of triangle's neighbor opposed to a vertex
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY Index
opposedTriangleInd(const Triangle& tri, VertInd iVert);

/// Index of triangle's neighbor opposed to an edge
CDT_INLINE_IF_HEADER_ONLY Index
opposedTriangleInd(const Triangle& tri, VertInd iVedge1, VertInd iVedge2);

/// Index of triangle's vertex opposed to a triangle
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY Index
opposedVertexInd(const Triangle& tri, TriInd iTopo);

/// If triangle has a given neighbor return neighbor-index, throw otherwise
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY Index
neighborInd(const Triangle& tri, TriInd iTnbr);

/// If triangle has a given vertex return vertex-index, throw otherwise
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY Index
vertexInd(const Triangle& tri, VertInd iV);

/// Given triangle and a vertex find opposed triangle
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY TriInd
opposedTriangle(const Triangle& tri, VertInd iVert);

/// Given two triangles, return vertex of first triangle opposed to the second
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY VertInd
opposedVertex(const Triangle& tri, TriInd iTopo);

/// Test if point lies in a circumscribed circle of a triangle
template <typename T>
CDT_EXPORT bool isInCircumcircle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3);

/// Test if two vertices share at least one common triangle
CDT_EXPORT CDT_INLINE_IF_HEADER_ONLY bool
verticesShareEdge(const TriIndVec& aTris, const TriIndVec& bTris);

/// Distance between two 2D points
template <typename T>
CDT_EXPORT T distance(const V2d<T>& a, const V2d<T>& b);

/// Squared distance between two 2D points
template <typename T>
CDT_EXPORT T distanceSquared(const V2d<T>& a, const V2d<T>& b);

} // namespace CDT

#ifndef CDT_USE_AS_COMPILED_LIBRARY
#include "predicates.h" // robust predicates: orient, in-circle

#include <stdexcept>

namespace CDT {

//*****************************************************************************
// V2d
//*****************************************************************************
template <typename T>
V2d<T> V2d<T>::make(const T x, const T y)
{
    V2d<T> out = { x, y };
    return out;
}

//*****************************************************************************
// Box2d
//*****************************************************************************
template <typename T>
Box2d<T> envelopBox(const std::vector<V2d<T>>& vertices)
{
    return envelopBox<T>(
        vertices.begin(), vertices.end(), getX_V2d<T>, getY_V2d<T>);
}

//*****************************************************************************
// Edge
//*****************************************************************************
CDT_INLINE_IF_HEADER_ONLY Edge::Edge(VertInd iV1, VertInd iV2)
    : m_vertices(
        iV1 < iV2 ? std::make_pair(iV1, iV2) : std::make_pair(iV2, iV1))
{
}

CDT_INLINE_IF_HEADER_ONLY bool Edge::operator==(const Edge& other) const
{
    return m_vertices == other.m_vertices;
}

CDT_INLINE_IF_HEADER_ONLY bool Edge::operator!=(const Edge& other) const
{
    return !(this->operator==(other));
}

CDT_INLINE_IF_HEADER_ONLY VertInd Edge::v1() const
{
    return m_vertices.first;
}

CDT_INLINE_IF_HEADER_ONLY VertInd Edge::v2() const
{
    return m_vertices.second;
}

CDT_INLINE_IF_HEADER_ONLY const std::pair<VertInd, VertInd>& Edge::verts() const
{
    return m_vertices;
}

//*****************************************************************************
// Utility functions
//*****************************************************************************
CDT_INLINE_IF_HEADER_ONLY Index ccw(Index i)
{
    return Index((i + 1) % 3);
}

CDT_INLINE_IF_HEADER_ONLY Index cw(Index i)
{
    return Index((i + 2) % 3);
}

CDT_INLINE_IF_HEADER_ONLY bool isOnEdge(const PtTriLocation::Enum location)
{
    return location > PtTriLocation::Outside;
}

CDT_INLINE_IF_HEADER_ONLY Index edgeNeighbor(const PtTriLocation::Enum location)
{
    assert(location >= PtTriLocation::OnEdge1);
    return static_cast<Index>(location - PtTriLocation::OnEdge1);
}

template <typename T>
T orient2D(const V2d<T>& p, const V2d<T>& v1, const V2d<T>& v2)
{
    return predicates::adaptive::orient2d(v1.x, v1.y, v2.x, v2.y, p.x, p.y);
}

template <typename T>
PtLineLocation::Enum locatePointLine(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const T orientationTolerance)
{
    return classifyOrientation(orient2D(p, v1, v2), orientationTolerance);
}

template <typename T>
PtLineLocation::Enum
classifyOrientation(const T orientation, const T orientationTolerance)
{
    if (orientation < -orientationTolerance)
        return PtLineLocation::Right;
    if (orientation > orientationTolerance)
        return PtLineLocation::Left;
    return PtLineLocation::OnLine;
}

template <typename T>
PtTriLocation::Enum locatePointTriangle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    PtTriLocation::Enum result = PtTriLocation::Inside;
    PtLineLocation::Enum edgeCheck = locatePointLine(p, v1, v2);
    if (edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if (edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge1;
    edgeCheck = locatePointLine(p, v2, v3);
    if (edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if (edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge2;
    edgeCheck = locatePointLine(p, v3, v1);
    if (edgeCheck == PtLineLocation::Right)
        return PtTriLocation::Outside;
    if (edgeCheck == PtLineLocation::OnLine)
        result = PtTriLocation::OnEdge3;
    return result;
}

CDT_INLINE_IF_HEADER_ONLY Index opoNbr(const Index vertIndex)
{
    if (vertIndex == Index(0))
        return Index(1);
    if (vertIndex == Index(1))
        return Index(2);
    if (vertIndex == Index(2))
        return Index(0);
    throw std::runtime_error("Invalid vertex index");
}

CDT_INLINE_IF_HEADER_ONLY Index opoVrt(const Index neighborIndex)
{
    if (neighborIndex == Index(0))
        return Index(2);
    if (neighborIndex == Index(1))
        return Index(0);
    if (neighborIndex == Index(2))
        return Index(1);
    throw std::runtime_error("Invalid neighbor index");
}

CDT_INLINE_IF_HEADER_ONLY Index
opposedTriangleInd(const Triangle& tri, const VertInd iVert)
{
    for (Index vi = Index(0); vi < Index(3); ++vi)
        if (iVert == tri.vertices[vi])
            return opoNbr(vi);
    throw std::runtime_error("Could not find opposed triangle index");
}

CDT_INLINE_IF_HEADER_ONLY Index opposedTriangleInd(
    const Triangle& tri,
    const VertInd iVedge1,
    const VertInd iVedge2)
{
    for (Index vi = Index(0); vi < Index(3); ++vi) {
        const VertInd iVert = tri.vertices[vi];
        if (iVert != iVedge1 && iVert != iVedge2)
            return opoNbr(vi);
    }
    throw std::runtime_error("Could not find opposed-to-edge triangle index");
}

CDT_INLINE_IF_HEADER_ONLY Index
opposedVertexInd(const Triangle& tri, const TriInd iTopo)
{
    for (Index ni = Index(0); ni < Index(3); ++ni)
        if (iTopo == tri.neighbors[ni])
            return opoVrt(ni);
    throw std::runtime_error("Could not find opposed vertex index");
}

CDT_INLINE_IF_HEADER_ONLY Index
neighborInd(const Triangle& tri, const TriInd iTnbr)
{
    for (Index ni = Index(0); ni < Index(3); ++ni)
        if (iTnbr == tri.neighbors[ni])
            return ni;
    throw std::runtime_error("Could not find neighbor triangle index");
}

CDT_INLINE_IF_HEADER_ONLY Index vertexInd(const Triangle& tri, const VertInd iV)
{
    for (Index i = Index(0); i < Index(3); ++i)
        if (iV == tri.vertices[i])
            return i;
    throw std::runtime_error("Could not find vertex index in triangle");
}

CDT_INLINE_IF_HEADER_ONLY TriInd
opposedTriangle(const Triangle& tri, const VertInd iVert)
{
    return tri.neighbors[opposedTriangleInd(tri, iVert)];
}

CDT_INLINE_IF_HEADER_ONLY VertInd
opposedVertex(const Triangle& tri, const TriInd iTopo)
{
    return tri.vertices[opposedVertexInd(tri, iTopo)];
}

template <typename T>
bool isInCircumcircle(
    const V2d<T>& p,
    const V2d<T>& v1,
    const V2d<T>& v2,
    const V2d<T>& v3)
{
    using namespace predicates::adaptive;
    return incircle(v1.x, v1.y, v2.x, v2.y, v3.x, v3.y, p.x, p.y) > T(0);
}

CDT_INLINE_IF_HEADER_ONLY
bool verticesShareEdge(const TriIndVec& aTris, const TriIndVec& bTris)
{
    for (TriIndVec::const_iterator it = aTris.begin(); it != aTris.end(); ++it)
        if (std::find(bTris.begin(), bTris.end(), *it) != bTris.end())
            return true;
    return false;
}

template <typename T>
T distanceSquared(const T ax, const T ay, const T bx, const T by)
{
    const T dx = bx - ax;
    const T dy = by - ay;
    return dx * dx + dy * dy;
}

template <typename T>
T distance(const T ax, const T ay, const T bx, const T by)
{
    return std::sqrt(distanceSquared(ax, ay, bx, by));
}

template <typename T>
T distance(const V2d<T>& a, const V2d<T>& b)
{
    return distance(a.x, a.y, b.x, b.y);
}

template <typename T>
T distanceSquared(const V2d<T>& a, const V2d<T>& b)
{
    return distanceSquared(a.x, a.y, b.x, b.y);
}

} // namespace CDT

#endif

//*****************************************************************************
// Specialize hash functions
//*****************************************************************************
#ifdef CDT_CXX11_IS_SUPPORTED
namespace std
#else
namespace boost
#endif
{

#ifdef CDT_USE_STRONG_TYPING

/// Vertex index hasher
template <>
struct hash<CDT::VertInd> {
    /// Hash operator
    std::size_t operator()(const CDT::VertInd& vi) const
    {
        return std::hash<std::size_t>()(vi.t);
    }
};

/// Triangle index hasher
template <>
struct hash<CDT::TriInd> {
    /// Hash operator
    std::size_t operator()(const CDT::TriInd& vi) const
    {
        return std::hash<std::size_t>()(vi.t);
    }
};

#endif // CDT_USE_STRONG_TYPING

/// Edge hasher
template <>
struct hash<CDT::Edge> {
    /// Hash operator
    std::size_t operator()(const CDT::Edge& e) const
    {
        return hashEdge(e);
    }

private:
    static void hashCombine(std::size_t& seed, const CDT::VertInd& key)
    {
#ifdef CDT_CXX11_IS_SUPPORTED
        typedef std::hash<CDT::VertInd> Hasher;
#else
        typedef boost::hash<CDT::VertInd> Hasher;
#endif
        seed ^= Hasher()(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    static std::size_t hashEdge(const CDT::Edge& e)
    {
        const std::pair<CDT::VertInd, CDT::VertInd>& vv = e.verts();
        std::size_t seed1(0);
        hashCombine(seed1, vv.first);
        hashCombine(seed1, vv.second);
        std::size_t seed2(0);
        hashCombine(seed2, vv.second);
        hashCombine(seed2, vv.first);
        return std::min(seed1, seed2);
    }
};
} // namespace std/boost

#endif // header guard
