#ifndef CDT_POINTKDTREE_H
#define CDT_POINTKDTREE_H

#include "CDTUtils.h"
#include "KDTree.h"

namespace cdt {

/// KD-tree holding points
template <
    typename TCoordType,
    size_t NumVerticesInLeaf = 32,
    size_t InitialStackDepth = 32,
    size_t StackDepthIncrement = 32>
class LocatorKDTree {
public:
    /// Initialize KD-tree with points
    void initialize(const std::vector<vec2_<TCoordType>>& points)
    {
        vec2_<TCoordType> min = points.front();
        vec2_<TCoordType> max = min;
        typedef typename std::vector<vec2_<TCoordType>>::const_iterator Cit;
        for (Cit it = points.begin(); it != points.end(); ++it) {
            min = vec2_<TCoordType>::make(std::min(min.x(), it->x()), std::min(min.y(), it->y()));
            max = vec2_<TCoordType>::make(std::max(max.x(), it->x()), std::max(max.y(), it->y()));
        }
        m_kdTree = KDTree_t(min, max);
        for (std::uint32_t i = 0; i < points.size(); ++i) {
            m_kdTree.insert(i, points);
        }
    }
    /// Add point to KD-tree
    void addPoint(const std::uint32_t i, const std::vector<vec2_<TCoordType>>& points)
    {
        m_kdTree.insert(i, points);
    }
    /// Find nearest point using R-tree
    std::uint32_t nearPoint(
        const vec2_<TCoordType>& pos,
        const std::vector<vec2_<TCoordType>>& points) const
    {
        return m_kdTree.nearest(pos, points).second;
    }

private:
    typedef KDTree::KDTree<
        TCoordType,
        NumVerticesInLeaf,
        InitialStackDepth,
        StackDepthIncrement>
        KDTree_t;
    KDTree_t m_kdTree;
};

} // namespace cdt

#endif
