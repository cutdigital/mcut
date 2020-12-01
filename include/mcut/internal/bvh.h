#ifndef MCUT_BVH_H_
#define MCUT_BVH_H_

#include <cmath>

#ifdef _MSC_VER
#include <intrin.h>
#define __builtin_popcount __popcnt

// https://stackoverflow.com/questions/355967/how-to-use-msvc-intrinsics-to-get-the-equivalent-of-this-gcc-code
uint32_t __inline clz_(uint32_t value)
{
    unsigned long leading_zero = 0;

    if (_BitScanReverse(&leading_zero, value)) {
        return 31 - leading_zero;
    } else {
        // Same remarks as above
        return 32;
    }
}

#endif

// TODO: update this file

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

namespace mcut {
namespace bvh { // This code has not been thoroughly tested!!!

    typedef struct {
        int m_left; // node-A ID (implicit index)
        int m_right; // node-B ID (implicit index)
    } node_pair_t; // collision tree node

    unsigned int clz(unsigned int x) // stub
    {
#ifdef _MSC_VER
        return clz_(x);
#else
        return __builtin_clz(x); // only tested with gcc!!!
#endif
    }

    int next_power_of_two(int x)
    {
        x--;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        x++;
        return x;
    }

    bool is_power_of_two(int x)
    {
        return (x != 0) && !(x & (x - 1));
    }

    int ilog2(unsigned int x)
    {
        return sizeof(unsigned int) * CHAR_BIT - clz(x) - 1;
    }

    int get_leaf_level_from_real_leaf_count(const int t)
    {
        const int np2 = next_power_of_two(t); // todo
        const int tLeafLev = ilog2(np2);
        return tLeafLev;
    }

    int get_level_from_implicit_idx(const int bvhNodeImplicitIndex)
    {
        return ilog2(bvhNodeImplicitIndex + 1);
    }

    unsigned int flp2(unsigned int x) // prev pow2
    {
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        x = x | (x >> 16);
        return x - (x >> 1);
    }

    int get_ostensibly_implicit_bvh_size(const int t)
    {
        return 2 * t - 1 + __builtin_popcount(next_power_of_two(t) - t);
    }

    int get_level_leftmost_node(const int node_level)
    {
        return (1 << node_level) - 1;
    }

    int get_rightmost_real_leaf(const int bvhLeafLevelIndex, const int num_real_leaf_nodes_in_bvh)
    {
        return (get_level_leftmost_node(bvhLeafLevelIndex) + num_real_leaf_nodes_in_bvh) - 1;
    }

    bool is_real_implicit_tree_node_id(const int bvhNodeImplicitIndex, const int num_real_leaf_nodes_in_bvh)
    {

        const int t = num_real_leaf_nodes_in_bvh;
        //const int q = bvhNodeImplicitIndex; // queried node
        const int li = get_leaf_level_from_real_leaf_count(t);
        const int i = get_rightmost_real_leaf(li, t);
        const int lq = get_level_from_implicit_idx(bvhNodeImplicitIndex);
        const int p = (int)((1.0f / (1 << (li - lq))) + ((float)i / (1 << (li - lq))) - 1);

        return bvhNodeImplicitIndex <= p || p == 0; // and p is not the root
    }

    int get_level_rightmost_real_node(
        const int rightmostRealLeafNodeImplicitIndex,
        const int bvhLeafLevelIndex,
        const int ancestorLevelIndex)
    {
        using namespace std;
        const int level_dist = (bvhLeafLevelIndex - ancestorLevelIndex);
        const int implicit_index_of_ancestor = (int)((1.0f / (1 << level_dist)) + ((float)rightmostRealLeafNodeImplicitIndex / (1 << level_dist)) - 1);
        return implicit_index_of_ancestor;
    }

    int get_node_ancestor(
        const int nodeImplicitIndex,
        const int nodeLevelIndex,
        const int ancestorLevelIndex)
    {
        using namespace std;
        const int levelDistance = nodeLevelIndex - ancestorLevelIndex;
        return (int)((1.0f / (1 << levelDistance)) + ((float)nodeImplicitIndex / (1 << levelDistance)) - 1); /*trunc((1.0f / pow(bvhDegree, level_dist)) + (rightmostRealLeafNodeImplicitIndex / pow(bvhDegree, level_dist)) - 1)*/
    }

    int get_node_mem_index(
        const int nodeImplicitIndex,
        const int leftmostImplicitIndexOnNodeLevel,
        const int bvh_data_base_offset,
        const int rightmostRealNodeImplicitIndexOnNodeLevel)
    {
        return bvh_data_base_offset + get_ostensibly_implicit_bvh_size((rightmostRealNodeImplicitIndexOnNodeLevel - leftmostImplicitIndexOnNodeLevel) + 1) - 1 - (rightmostRealNodeImplicitIndexOnNodeLevel - nodeImplicitIndex);
    }

    unsigned int expandBits(unsigned int v)
    { ///< Expands a 10-bit integer into 30 bits by inserting 2 zeros after each
        ///< bit.
        v = (v * 0x00010001u) & 0xFF0000FFu;
        v = (v * 0x00000101u) & 0x0F00F00Fu;
        v = (v * 0x00000011u) & 0xC30C30C3u;
        v = (v * 0x00000005u) & 0x49249249u;
        return v;
    };

    ///< Calculates a 30-bit Morton code for the given 3D point located within the
    ///< unit cube [0,1].

    unsigned int morton3D(float x, float y, float z)
    {
        x = std::fmin(std::fmax(x * 1024.0f, 0.0f), 1023.0f);
        y = std::fmin(std::fmax(y * 1024.0f, 0.0f), 1023.0f);
        z = std::fmin(std::fmax(z * 1024.0f, 0.0f), 1023.0f);

        unsigned int xx = expandBits((unsigned int)x);
        unsigned int yy = expandBits((unsigned int)y);
        unsigned int zz = expandBits((unsigned int)z);

        return (xx * 4 + yy * 2 + zz);
    };
} // namespace bvh {
} // namespace mcut {

#endif // MCUT_BVH_H_
