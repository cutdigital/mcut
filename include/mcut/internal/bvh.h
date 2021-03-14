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
#ifndef MCUT_BVH_H_
#define MCUT_BVH_H_

namespace mcut {
namespace bvh {

    // TODO: just use std::pair
    typedef struct {
        int m_left; // node-A ID (implicit index)
        int m_right; // node-B ID (implicit index)
    } node_pair_t; // collision tree node

    // count leading zeros in 32 bit bitfield
    extern unsigned int clz(unsigned int x);

    // next power of two from x
    extern int next_power_of_two(int x);

    // check if "x" is a power of two
    extern bool is_power_of_two(int x);

    // compute log-base-2 of "x"
    extern int ilog2(unsigned int x);

    // compute index (0...N-1) of the leaf level from the number of leaves
    extern int get_leaf_level_from_real_leaf_count(const int t);

    // compute tree-level index from implicit index of a node
    extern int get_level_from_implicit_idx(const int bvhNodeImplicitIndex);

    // compute previous power of two
    extern unsigned int flp2(unsigned int x);

    // compute size of of Oi-BVH give number of triangles
    extern int get_ostensibly_implicit_bvh_size(const int t);

    // compute left-most node on a given level
    extern int get_level_leftmost_node(const int node_level);

    // compute right-most leaf node in tree
    extern int get_rightmost_real_leaf(const int bvhLeafLevelIndex, const int num_real_leaf_nodes_in_bvh);

    // check if node is a "real node"
    extern bool is_real_implicit_tree_node_id(const int bvhNodeImplicitIndex, const int num_real_leaf_nodes_in_bvh);

    // get the right most real node on a given tree level
    extern int get_level_rightmost_real_node(
        const int rightmostRealLeafNodeImplicitIndex,
        const int bvhLeafLevelIndex,
        const int ancestorLevelIndex);

    // compute implicit index of a node's ancestor
    extern int get_node_ancestor(
        const int nodeImplicitIndex,
        const int nodeLevelIndex,
        const int ancestorLevelIndex);

    // calculate linear memory index of a real node
    extern int get_node_mem_index(
        const int nodeImplicitIndex,
        const int leftmostImplicitIndexOnNodeLevel,
        const int bvh_data_base_offset,
        const int rightmostRealNodeImplicitIndexOnNodeLevel);

    // Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
    extern unsigned int expandBits(unsigned int v);

    // Calculates a 30-bit Morton code for the given 3D point located within the unit cube [0,1].
    extern unsigned int morton3D(float x, float y, float z);
} // namespace bvh {
} // namespace mcut {

#endif // MCUT_BVH_H_
