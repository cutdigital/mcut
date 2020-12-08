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

#include "mcut/internal/kernel.h"
#include "mcut/internal/bvh.h"
#include "mcut/internal/geom.h"
#include "mcut/internal/halfedge_mesh.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"
#include <algorithm>
#include <fstream>
#include <queue>
#include <set>
#include <string>
#include <tuple>

// keep around the intermediate meshes created during patch stitching (good for showing how code works)
#define MCUT_KEEP_TEMP_CCs_DURING_PATCH_STITCHING 1

// This macro enables dumping to the log information about meshes that are
// dumped to file via "dump_mesh(...)"
#ifndef MCUT_ENABLE_LOGGING_DUMPED_MESH_INFO
#define MCUT_ENABLE_LOGGING_DUMPED_MESH_INFO 0
#endif

// This macro for BVH-debugging purposes but can be excruciatingly slow when using exact numbers.
// #define MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE

namespace mcut {

logger_t* logger_ptr = nullptr;

std::string vstr(const vd_t& v, const std::string& pre = "", const std::string& post = "")
{
    std::stringstream ss;
    ss << pre << "v" << v << post;
    return ss.str();
}

std::string fstr(const fd_t& f, const std::string& pre = "", const std::string& post = "")
{
    std::stringstream ss;
    ss << pre << "f";
    if (f != mesh_t::null_face())
        ss << f;
    else
        ss << "-";
    ss << post;
    return ss.str();
}

std::string hstr_(const vd_t& s, const vd_t& t, const std::string& pre = "[", const std::string& post = "]", const std::string& delim = " ")
{
    std::stringstream ss;
    ss << pre << vstr(s) << delim << vstr(t) << post;
    return ss.str();
}

std::string hstr(const mesh_t& m, const hd_t& h)
{
    std::stringstream ss;
    ss << "h" << h << " ";
    return ss.str() + hstr_(m.source(h), m.target(h));
}

std::string estr(const mesh_t& m, const ed_t& e, const std::string& pre = "(", const std::string& post = ")", const std::string& delim = " ")
{
    std::stringstream ss;
    ss << "e" << e << " ";
    return ss.str() + hstr_(m.vertex(e, 0), m.vertex(e, 1), pre, post, delim);
}

std::string estr(const vd_t& s, const vd_t& t, const std::string& pre = "(", const std::string& post = ")", const std::string& delim = " ")
{
    std::stringstream ss;
    return ss.str() + hstr_(s, t, pre, post, delim);
}

std::string to_string(const connected_component_location_t& v)
{
    std::string s;
    switch (v) {
    case connected_component_location_t::ABOVE:
        s = "a";
        break;
    case connected_component_location_t::BELOW:
        s = "b";
        break;
    case connected_component_location_t::UNDEFINED:
        s = "u";
        break;
    }
    return s;
}

std::string to_string(const cut_surface_patch_location_t& v)
{
    std::string s;
    switch (v) {
    case cut_surface_patch_location_t::INSIDE:
        s = "i";
        break;
    case cut_surface_patch_location_t::OUTSIDE:
        s = "o";
        break;
    case cut_surface_patch_location_t::UNDEFINED:
        s = "u";
        break;
    }
    return s;
}

std::string to_string(const status_t& v)
{
    std::string s;
    switch (v) {
    case status_t::SUCCESS:
        s = "SUCCESS";
        break;
    case status_t::INVALID_SRC_MESH:
        s = "INVALID_SRC_MESH";
        break;
    case status_t::INVALID_CUT_MESH:
        s = "INVALID_CUT_MESH";
        break;
    case status_t::INVALID_MESH_INTERSECTION:
        s = "INVALID_MESH_INTERSECTION";
        break;
    case status_t::INVALID_BVH_INTERSECTION:
        s = "INVALID_BVH_INTERSECTION";
        break;
    }
    return s;
}

std::string to_string(const cut_surface_patch_winding_order_t& v)
{
    std::string s;
    switch (v) {
    case cut_surface_patch_winding_order_t::DEFAULT:
        s = "ccw";
        break;
    case cut_surface_patch_winding_order_t::REVERSE:
        s = "cw";
        break;
    }
    return s;
}

void dump_mesh(const mesh_t& mesh, const char* fbasename)
{
    const std::string name = std::string(fbasename) + ".off";

    (*logger_ptr) << "save " << name << std::endl;

#if !MCUT_ENABLE_LOGGING_DUMPED_MESH_INFO
    bool verb = logger_ptr->verbose();
    logger_ptr->set_verbose(false);
#endif

    (*logger_ptr) << "vertices = " << mesh.number_of_vertices() << std::endl;

    for (mesh_t::vertex_iterator_t v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        (*logger_ptr).indent();
        (*logger_ptr) << vstr(*v) << " (" << mesh.vertex(*v).x() << ", " << mesh.vertex(*v).y() << ", " << mesh.vertex(*v).z() << ")" << std::endl;
        (*logger_ptr).unindent();
    }

    (*logger_ptr) << "edges = " << mesh.number_of_edges() << std::endl;

    for (mesh_t::edge_iterator_t e = mesh.edges_begin(); e != mesh.edges_end(); ++e) {
        (*logger_ptr).indent();
        (*logger_ptr) << estr(mesh, *e) << std::endl;
        (*logger_ptr).unindent();
    }

    (*logger_ptr) << "halfedges = " << mesh.number_of_halfedges() << std::endl;

    for (mesh_t::halfedge_iterator_t h = mesh.halfedges_begin(); h != mesh.halfedges_end(); ++h) {
        (*logger_ptr).indent();
        (*logger_ptr) << hstr(mesh, *h) << std::endl;
        (*logger_ptr).unindent();
    }

    (*logger_ptr) << "faces = " << mesh.number_of_faces() << std::endl;

    for (mesh_t::face_iterator_t face_iter = mesh.faces_begin(); face_iter != mesh.faces_end(); ++face_iter) {
        (*logger_ptr).indent();
        (*logger_ptr) << "face " << *face_iter << std::endl;

        const std::vector<halfedge_descriptor_t>& halfedges_around_face = mesh.get_halfedges_around_face(*face_iter);

        (*logger_ptr).indent();

        int num_halfedges = (int)halfedges_around_face.size();
        MCUT_ASSERT(num_halfedges >= 3);

        (*logger_ptr).indent();
        (*logger_ptr) << "halfedges = " << num_halfedges << std::endl;
        for (std::vector<halfedge_descriptor_t>::const_iterator h = halfedges_around_face.cbegin();
             h != halfedges_around_face.cend();
             ++h) {
            (*logger_ptr).indent();
            (*logger_ptr) << hstr(mesh, *h) << std::endl;
            (*logger_ptr).unindent();
        }
        (*logger_ptr).unindent();

        (*logger_ptr).unindent();
        (*logger_ptr).unindent();
    }

#if !MCUT_ENABLE_LOGGING_DUMPED_MESH_INFO
    (*logger_ptr).set_verbose(verb);
#endif

    write_off(name.c_str(), mesh);
}

int find_connected_components(std::map<face_descriptor_t, int>& fccmap, const mesh_t& mesh)
{
    MCUT_ASSERT(mesh.number_of_vertices() >= 3);
    MCUT_ASSERT(mesh.number_of_edges() >= 3);
    MCUT_ASSERT(mesh.number_of_faces() >= 1);

    fccmap.clear();

    std::vector<std::vector<mcut::vertex_descriptor_t>> vertex_sets;
    std::map<mcut::vertex_descriptor_t, int> vertex_to_set_index;

    for (mcut::mesh_t::vertex_iterator_t v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        vertex_to_set_index[*v] = (int)std::distance(mesh.vertices_begin(), v);
        vertex_sets.push_back({ *v }); // make set of one
    }

    for (mcut::mesh_t::edge_iterator_t e = mesh.edges_begin(); e != mesh.edges_end(); ++e) {

        mcut::vertex_descriptor_t v0 = mesh.vertex(*e, 0);
        mcut::vertex_descriptor_t v1 = mesh.vertex(*e, 1);

        int v0_set_index = vertex_to_set_index[v0];
        int v1_set_index = vertex_to_set_index[v1];

        if (v0_set_index != v1_set_index) { // are they in different sets...?

            std::vector<std::vector<mcut::vertex_descriptor_t>>::iterator v0_set = vertex_sets.begin() + v0_set_index;
            std::vector<std::vector<mcut::vertex_descriptor_t>>::iterator v1_set = vertex_sets.begin() + v1_set_index;
            ;

            v0_set->insert(v0_set->cend(), v1_set->cbegin(), v1_set->cend()); // union/join sets

            // update the set index of all vertices in set of v1

            for (std::vector<mcut::vertex_descriptor_t>::const_iterator it = v1_set->cbegin();
                 it != v1_set->cend();
                 ++it) {
                vertex_to_set_index[*it] = v0_set_index; // update set index
            }

            v1_set->clear(); // wipe elements (no longer needed) but keep empty set
        }
    }

    // the sets which contain vertices that belong to separate connected components i.e. each set is a conn comp.
    // size of set is the number of connected components
    std::set<int> final_set_indices;
    std::map<int, int> final_set_index_to_vertex_count;

    for (mcut::mesh_t::vertex_iterator_t v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {

        const int vertex_set_index = vertex_to_set_index[*v];
        final_set_indices.insert(vertex_set_index); // NOTE: unique elements only

        if (final_set_index_to_vertex_count.count(vertex_set_index) == 0) {
            final_set_index_to_vertex_count[vertex_set_index] = 0; // init
        }

        final_set_index_to_vertex_count[vertex_set_index] += 1;
    }

    // map set index to a renormalized index (0 ,..., N-1)
    std::map<int, int> final_set_index_to_linear_index;

    for (std::set<int>::const_iterator set_index_iter = final_set_indices.cbegin(); set_index_iter != final_set_indices.cend(); set_index_iter++) {
        if (final_set_index_to_vertex_count[*set_index_iter] < 3) // 3 is min number of vertices which make up a "connected component" (triangle)
        {
            continue;
        }
        final_set_index_to_linear_index[*set_index_iter] = (int)final_set_index_to_linear_index.size();
    }

    // map each face to a connected component
    for (mesh_t::face_iterator_t f = mesh.faces_begin(); f != mesh.faces_end(); ++f) {
        const std::vector<vertex_descriptor_t> vertices = mesh.get_vertices_around_face(*f);

#ifndef NDEBUG

        // assert
        for (int i = 0; i < (int)vertices.size(); ++i) {
            for (int j = 0; j < (int)vertices.size(); ++j) {
                if (i != j) {
                    MCUT_ASSERT(vertex_to_set_index[vertices[i]] == vertex_to_set_index[vertices[j]]);
                }
            }
        }

#endif // !NDEBUG
        int set_idx = vertex_to_set_index[vertices.front()]; // all vertices belong to the same conn comp
        int descr = final_set_index_to_linear_index[set_idx];
        fccmap[*f] = descr;
    }

    return (int)final_set_index_to_linear_index.size();
}

struct connected_component_info_t {
    connected_component_location_t location; // above/ below
    // vertices along the cut path seam
    std::vector<vd_t> seam_vertices;
};

void mark_seam_vertices(
    std::map<vd_t, bool>& mesh_seam_vertices,
    mesh_t& mesh,
    const int ps_num_vertices,
    const int m1_num_vertices_after_srcmesh_partitioning = std::numeric_limits<int>::max())
{
    for (mesh_t::vertex_iterator_t i = mesh.vertices_begin();
         i != mesh.vertices_end();
         ++i) {
        const int idx = static_cast<int>(*i);
        mesh_seam_vertices[*i] = (idx >= ps_num_vertices && idx < m1_num_vertices_after_srcmesh_partitioning);
    }
}

// returns the unseparated/merged connected components
mesh_t extract_connected_components(
    // key = cc-id; value = list of cc copies each differing by one newly stitched polygon
    std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>& connected_components,
    const mesh_t& in,
    const std::vector<std::vector<hd_t>>& traced_polygons,
    const std::vector<int>& sm_polygons_below_cs,
    const std::vector<int>& sm_polygons_above_cs,
    const std::map<vd_t, bool>& mesh_vertex_to_seam_flag)
{
    (*logger_ptr) << "extract connected components" << std::endl;
    (*logger_ptr).indent();

    // the auxilliary halfedge mesh containing the vertices and edges
    // referenced by the traced polygons
    mesh_t mesh = in; // copy

    ///////////////////////////////////////////////////////////////////////////
    // Insert traced polygons into the auxilliary mesh
    ///////////////////////////////////////////////////////////////////////////

    (*logger_ptr) << "total polygons = " << traced_polygons.size() << std::endl;

    // for each traced polygon
    for (std::vector<std::vector<hd_t>>::const_iterator traced_sm_polygon_iter = traced_polygons.cbegin();
         traced_sm_polygon_iter != traced_polygons.cend();
         ++traced_sm_polygon_iter) {

        //const int polygon_idx = (int)std::distance(traced_polygons.cbegin(), traced_sm_polygon_iter);
        const std::vector<hd_t>& traced_polygon = *traced_sm_polygon_iter;

        //
        // gather polygon's vertices
        //

        std::vector<vd_t> polygon_vertices;

        // for each halfedge in polygon
        for (std::vector<hd_t>::const_iterator traced_sm_polygon_halfedge_iter = traced_polygon.cbegin();
             traced_sm_polygon_halfedge_iter != traced_polygon.cend();
             ++traced_sm_polygon_halfedge_iter) {
            polygon_vertices.push_back(mesh.source(*traced_sm_polygon_halfedge_iter));
        }

        // insert face into halfedge data structure
        const fd_t f = mesh.add_face(polygon_vertices);

        // we have violated halfedge data structure construction
        // rules probably because we are refering to a halfedge
        // and its opposite in one polygon
        MCUT_ASSERT(f != mesh_t::null_face());
    }

    ///////////////////////////////////////////////////////////////////////////
    // find connected components
    ///////////////////////////////////////////////////////////////////////////
    (*logger_ptr) << "search connected components" << std::endl;

    // connected components
    std::map<std::size_t, mesh_t> cc_to_mesh;
    // location of each connected component w.r.t cut-mesh (above | below | undefined)
    std::map<std::size_t, connected_component_location_t> cc_to_cs_descriptor;
    // for each component, we have a map which relates the vertex descriptors (indices) in the
    // auxilliary halfedge data structure "mesh" to the (local) vertex descriptors in
    // the connected-component.
    std::map<std::size_t, std::map<vd_t, vd_t>> cc_to_m1_to_cc_vertex;
    // the vertex descriptors [in the cc] which are seam vertices!
    std::map<std::size_t, std::vector<vd_t>> cc_to_seam_vertices;
    // here we create a map to tag each polygon in "mesh" with the connected component it belongs to.
    std::map<face_descriptor_t, int> fccmap;

    const std::size_t num = find_connected_components(fccmap, mesh);

    (*logger_ptr) << "connected components = " << num << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    // Map vertex descriptors to each connected component
    ///////////////////////////////////////////////////////////////////////////

    // NOTE: even if the number of connected components is one, we proceed anyway
    // because each connected connected excludes unused vertices

    // for each face in the auxilliary mesh (i.e. traced polygon)
    for (mesh_t::face_iterator_t face_iter = mesh.faces_begin(); face_iter != mesh.faces_end(); ++face_iter) {
        face_descriptor_t fd = *face_iter;
        const int face_cc_id = fccmap[fd]; // get connected component of face

        if (cc_to_mesh.find(face_cc_id) == cc_to_mesh.end()) {
            // create new mesh to store connected component
            cc_to_mesh.insert(std::make_pair(face_cc_id, mesh_t()));
        }

        if (cc_to_m1_to_cc_vertex.find(face_cc_id) == cc_to_m1_to_cc_vertex.end()) {
            // create new component descriptor map
            cc_to_m1_to_cc_vertex.insert(std::make_pair(face_cc_id, std::map<vd_t, vd_t>()));
        }

        //
        // Determine the location of the connected component w.r.t the cut-mesh
        //

        // check if the current face is already marked as "below" (w.r.t the cut-mesh).
        const bool cc_is_below_cs = std::find(sm_polygons_below_cs.cbegin(), sm_polygons_below_cs.cend(), static_cast<int>(fd)) != sm_polygons_below_cs.cend();

        if (cc_is_below_cs) {
            // try to save the fact that the current connected component is "below"
            std::pair<std::map<std::size_t, connected_component_location_t>::iterator, bool> p = cc_to_cs_descriptor.insert(std::make_pair(face_cc_id, connected_component_location_t::BELOW));
            // if 1) insertion did not take place (connected component already registered), and
            // 2) the existing connected component at that entry is marked as "above":
            //  --> partial cut: thus, the notion "above"/"below" is undefined
            if (p.second == false && p.first->second == connected_component_location_t::ABOVE) {
                // polygon classed as both above and below cs
                // this is because the connected component contains polygons which are both "above"
                // and "below" the cutting surface (we have a partial cut)
                p.first->second = connected_component_location_t::UNDEFINED;
            }
        }

        // check if connected component is marked as "above"
        const bool cc_is_above_cs = std::find(sm_polygons_above_cs.cbegin(), sm_polygons_above_cs.cend(), static_cast<int>(fd)) != sm_polygons_above_cs.cend();

        if (cc_is_above_cs) {
            // try to save the fact that the current connected component is tagged "above"
            std::pair<std::map<std::size_t, connected_component_location_t>::iterator, bool> p = cc_to_cs_descriptor.insert(std::make_pair(face_cc_id, connected_component_location_t::ABOVE));
            // if 1) insertion did not take place (connected component is already registered), and
            // 2) the existing connected component at that entry is marked as "below":
            //--> partial cut: connected component has polygon whch are both "above" and "below"
            if (p.second == false && p.first->second == connected_component_location_t::BELOW) {
                p.first->second = connected_component_location_t::UNDEFINED; // polygon classed as both above and below cs
            }
        }

        //
        // We now map the vertices of the current face from the auxilliary data
        // structure "mesh" to the (local) connected-component
        //

        // for each vertex around the current face
        const std::vector<vertex_descriptor_t> vertices_around_face = mesh.get_vertices_around_face(fd);
        for (std::vector<vertex_descriptor_t>::const_iterator face_vertex_iter = vertices_around_face.cbegin();
             face_vertex_iter != vertices_around_face.cend();
             ++face_vertex_iter) {

            MCUT_ASSERT(cc_to_m1_to_cc_vertex.find(face_cc_id) != cc_to_m1_to_cc_vertex.cend());

            // if vertex is not already mapped from "mesh" to connected component
            if (cc_to_m1_to_cc_vertex.at(face_cc_id).find(*face_vertex_iter) == cc_to_m1_to_cc_vertex.at(face_cc_id).end()) {

                MCUT_ASSERT(cc_to_mesh.find(face_cc_id) != cc_to_mesh.cend());

                // copy vertex from auxilliary data structure "mesh", add it into connected component mesh,
                // and save the vertex's descriptor in the conected component mesh.
                const vd_t cc_descriptor = cc_to_mesh.at(face_cc_id).add_vertex(mesh.vertex(*face_vertex_iter));

                // map vertex
                cc_to_m1_to_cc_vertex.at(face_cc_id).insert(std::make_pair(*face_vertex_iter, cc_descriptor));

                // check if we need to save vertex as being a seam vertex
                std::map<vd_t, bool>::const_iterator fiter = mesh_vertex_to_seam_flag.find(*face_vertex_iter);
                bool is_seam_vertex = fiter != mesh_vertex_to_seam_flag.cend() && fiter->second == true;
                if (is_seam_vertex) {
                    cc_to_seam_vertices[face_cc_id].push_back(cc_descriptor);
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // Insert faces into connected components using mapped vertex descriptors
    ///////////////////////////////////////////////////////////////////////////

    // for each face in the auxilliary data structure "mesh" (traced polygon)
    for (mesh_t::face_iterator_t face_iter = mesh.faces_begin(); face_iter != mesh.faces_end(); ++face_iter) {
        face_descriptor_t fd = *face_iter;
        const int cc_id = fccmap[fd]; // the connected component which contains the current face
        std::vector<vd_t> remapped_face; // using remapped cc descriptors

        // for each vertex around face
        const std::vector<vertex_descriptor_t> vertices_around_face = mesh.get_vertices_around_face(fd);
        for (std::vector<vertex_descriptor_t>::const_iterator face_vertex_iter = vertices_around_face.cbegin();
             face_vertex_iter != vertices_around_face.cend();
             ++face_vertex_iter) {
            MCUT_ASSERT(cc_to_m1_to_cc_vertex.find(cc_id) != cc_to_m1_to_cc_vertex.cend());

            const std::map<vd_t, vd_t>& vertex_map = cc_to_m1_to_cc_vertex.at(cc_id);
            const vd_t m1_sm_descr = *face_vertex_iter;

            MCUT_ASSERT(vertex_map.find(m1_sm_descr) != vertex_map.cend());

            const vd_t cc_descr = vertex_map.at(m1_sm_descr);
            remapped_face.push_back(cc_descr);
        }

        MCUT_ASSERT(cc_to_mesh.find(cc_id) != cc_to_mesh.cend());

        mesh_t& cc_mesh = cc_to_mesh.at(cc_id);
        fd_t f = cc_mesh.add_face(remapped_face); // insert the face

        MCUT_ASSERT(f != mesh_t::null_face());
    }

    // Note: at this stage we have our connected components (meshes) with their
    // vertices and faces defined

    MCUT_ASSERT(cc_to_mesh.size() == cc_to_m1_to_cc_vertex.size());

    ///////////////////////////////////////////////////////////////////////////
    // Save the output connected components marked with location
    ///////////////////////////////////////////////////////////////////////////

    // for each connected component
    for (std::map<std::size_t, mesh_t>::const_iterator cc_iter = cc_to_mesh.cbegin();
         cc_iter != cc_to_mesh.cend();
         ++cc_iter) {

        const std::size_t& cc_id = cc_iter->first;

        (*logger_ptr) << "connected component id = " << cc_id << std::endl;

        const mesh_t& cc = cc_iter->second;

        (*logger_ptr).indent();
        connected_component_location_t location = connected_component_location_t::UNDEFINED;

        if (!sm_polygons_below_cs.empty() && !sm_polygons_above_cs.empty()) {
            MCUT_ASSERT(cc_to_cs_descriptor.find(cc_id) != cc_to_cs_descriptor.cend());
            location = cc_to_cs_descriptor.at(cc_id);
        }

        // infer the seam vertices of the current cc
        // const std::map<vd_t, vd_t>& m1_to_cc_vertex = cc_to_m1_to_cc_vertex.at(cc_id);
        // mesh_vertex_to_seam_flag;

        connected_component_info_t ccinfo;
        ccinfo.location = location;
        ccinfo.seam_vertices = std::move(cc_to_seam_vertices[cc_id]);

        connected_components[cc_id].emplace_back(cc, std::move(ccinfo));

        (*logger_ptr) << "location = " << to_string(location) << std::endl;
        (*logger_ptr) << "vertices = " << cc.number_of_vertices() << std::endl;
        (*logger_ptr) << "edges = " << cc.number_of_edges() << std::endl;
        (*logger_ptr) << "halfedges = " << cc.number_of_halfedges() << std::endl;
        (*logger_ptr) << "faces = " << cc.number_of_faces() << std::endl;

        (*logger_ptr).unindent();
    }
    (*logger_ptr).unindent();

    return mesh;
}

// returns whether a polygon-soup vertex is an intersection vertex/point
bool m0_is_intersection_point(const vd_t& ps_vd, const int ps_vtx_cnt)
{
    return ((int)ps_vd) >= ps_vtx_cnt;
}

// returns whether a polygon-soup vertex belongs to the cut mesh
bool ps_is_cutmesh_vertex(const vd_t& ps_vd, const int sm_vtx_cnt)
{
    return ((int)ps_vd) >= sm_vtx_cnt;
}

bool ps_is_cutmesh_face(const fd_t& ps_fd, const int sm_face_count)
{
    return ((int)ps_fd) >= sm_face_count;
}

int wrap_integer(int x, const int lo, const int hi)
{
    const int range_size = hi - lo + 1;

    if (x < lo) {
        x += range_size * ((lo - x) / range_size + 1);
    }

    return lo + (x - lo) % range_size;
}

std::vector<vd_t> get_vertices_on_ps_edge(
    const ed_t ps_edge,
    const std::map<vd_t, hd_t>& m0_ivtx_to_ps_he,
    const mesh_t& ps,
    const std::map<vd_t, vd_t>& m0_to_ps_vtx)
{
    const vd_t ps_v0 = ps.vertex(ps_edge, 0);
    const vd_t ps_v1 = ps.vertex(ps_edge, 1);

    std::map<vd_t, vd_t>::const_iterator fiter = std::find_if(
        m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(),
        [&](const std::pair<vd_t, vd_t> p) { return ps_v0 == p.second; });

    MCUT_ASSERT(fiter != m0_to_ps_vtx.cend()); // ... because is an original vertex which means it should be in m0_to_ps_vtx

    const vd_t m0_v0 = fiter->first;
    fiter = std::find_if(m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(), [&](const std::pair<vd_t, vd_t> p) { return ps_v1 == p.second; });

    MCUT_ASSERT(fiter != m0_to_ps_vtx.cend()); // ... because is an original vertex which means it should be in m0_to_ps_vtx

    const vd_t m0_v1 = fiter->first;
    std::vector<vd_t> incident_vertices = { m0_v0, m0_v1 }; // add ps-vertices

    for (std::map<vd_t, hd_t>::const_iterator iter = m0_ivtx_to_ps_he.cbegin(); iter != m0_ivtx_to_ps_he.cend(); ++iter) {
        if (ps.edge(iter->second) == ps_edge) {
            incident_vertices.push_back(iter->first);
        }
    }

    return incident_vertices;
}

bool is_virtual_polygon(const fd_t& face)
{
    return (face == mesh_t::null_face());
}

/*
	@brief: Given a list of sorted vertices which belong to a histogram bin, check that a
	particular component (x, y, or z) of their coordinates is not the same amongst two or more vertices
	*/
bool have_same_coordinate(
    const std::vector<std::pair<vd_t, math::vec3>>& bin_vertices_sorted,
    const int coordinate_index = 0 // 0 = x, 1 = y, 2 = z component
)
{
    // for each vertex, compare to all others in vector (compare by given component)
    bool is_duplicate = false;
    for (std::vector<std::pair<vd_t, math::vec3>>::const_iterator i = bin_vertices_sorted.begin(); i != bin_vertices_sorted.end(); ++i) {
        const math::vec3& vertex_i_coordinates = i->second;
        const math::real_number_t vertex_i_coordinate = vertex_i_coordinates[coordinate_index];
        bool vertex_i_coordinate_is_duplicate = false;

        for (std::vector<std::pair<vd_t, math::vec3>>::const_iterator j = bin_vertices_sorted.begin(); j != bin_vertices_sorted.end(); ++j) {
            if (j == i) {
                continue; // same vertex, skip
            }

            const math::vec3& vertex_j_coordinates = j->second;
            const math::real_number_t vertex_j_coordinate = vertex_j_coordinates[coordinate_index];
            vertex_i_coordinate_is_duplicate = (vertex_i_coordinate == vertex_j_coordinate);

            if (vertex_i_coordinate_is_duplicate) {
                is_duplicate = true;
                break;
            }
        }

        if (is_duplicate) {
            break;
        }
    }

    return is_duplicate;
}

// point an intersection halfedge to the correct instance of an intersection point
vd_t resolve_intersection_point_descriptor(
    const mesh_t& ps,
    const mesh_t& m0,
    mesh_t& m1,
    const hd_t& m0_h,
    const vd_t& m0_h_tgt,
    const vd_t& m1_h_tgt,
    const bool m0_h_is_ox,
    const std::map<hd_t, std::vector<int>>& m0_h_to_ply,
    const std::map<vd_t, std::vector<hd_t>>& ivtx_to_incoming_hlist,
    const std::map<hd_t, bool>& m0_sm_ihe_to_flag,
    const std::map<vd_t, hd_t>& m0_ivtx_to_ps_he,
    const std::map<hd_t, hd_t>& m0_to_m1_ihe,
    const std::map<vd_t, vd_t>& m0_to_ps_vtx,
    const int ps_vtx_cnt,
    const int sm_vtx_cnt,
    const int sm_face_count)
{
    // the descriptor instance we want to return
    vd_t resolved_inst = m1_h_tgt;

    // First, we get list of all other halfedges in (in "m0") whose target-vertex
    // is the same as the target of the current halfedge
    const std::vector<hd_t>& incoming = ivtx_to_incoming_hlist.at(m0_h_tgt);

    // the minimum number of halfedges whose target is "m0_h_tgt"
    // this "minimum" case come from interior edges
    MCUT_ASSERT(incoming.size() >= 2);

    // Second, we will now filter "incoming" (those pointing to "m0_h_tgt") by
    // keeping only the halfedges which are:
    // 1) Processed/transformed (so that we can use it to infer what to do with "resolved_inst")
    // 2) are incident to a traced polygon, and
    // 3) used by a traced polygon of the src-mesh
    //
    // The remaining halfedges will be the ones we can use to infer the correct value of "resolved_inst"

    std::vector<hd_t> halfedges_across_cut_path = incoming;

    // for each halfedge across the cut-path
    for (std::vector<hd_t>::const_iterator halfedge_across_cut_path_iter = halfedges_across_cut_path.cbegin();
         halfedge_across_cut_path_iter != halfedges_across_cut_path.cend();) {

        const vd_t s = m0.source(*halfedge_across_cut_path_iter);
        const vd_t t = m0.target(*halfedge_across_cut_path_iter);
        const bool s_is_ivtx = m0_is_intersection_point(s, ps_vtx_cnt);
        const bool t_is_ivtx = m0_is_intersection_point(t, ps_vtx_cnt);

        // check if the halfedge is only next to the cut-mesh

        const bool is_ox_cs_h = (!s_is_ivtx && ps_is_cutmesh_vertex(m0_to_ps_vtx.at(s), sm_vtx_cnt));
        const bool is_xo_cs_h = (!t_is_ivtx && ps_is_cutmesh_vertex(m0_to_ps_vtx.at(t), sm_vtx_cnt));
        bool is_strictly_cs_h = is_ox_cs_h || is_xo_cs_h; // check if halfedge is used only by a cut-surface polygon
        const bool is_xx = s_is_ivtx && t_is_ivtx;

        if (!is_strictly_cs_h && is_xx) {
            const hd_t s_ps_h = m0_ivtx_to_ps_he.at(s);
            const hd_t t_ps_h = m0_ivtx_to_ps_he.at(t);
            const ed_t s_ps_e = ps.edge(s_ps_h);
            const ed_t t_ps_e = ps.edge(t_ps_h);
            const bool oh_is_exterior = (s_ps_e == t_ps_e); // lays on exterior of ps polygon

            if (oh_is_exterior) {
                is_strictly_cs_h = ps_is_cutmesh_face(ps.face(s_ps_h), sm_face_count); // could also use ps.face(t_ps_h) since both he's are part of same edge
            }
        }

        // if
        // 1) halfedge strictly belongs to the cut-mesh, OR
        // 2) halfedge is not used for tracing, OR
        // 3) halfedge has not been processed
        if (is_strictly_cs_h || //
            m0_h_to_ply.find(*halfedge_across_cut_path_iter) == m0_h_to_ply.end() || //
            m0_sm_ihe_to_flag.at(*halfedge_across_cut_path_iter) == false) { // is halfedge incident to a traced polygon and is it processed..?
            halfedge_across_cut_path_iter = halfedges_across_cut_path.erase(halfedge_across_cut_path_iter);
        } else {
            ++halfedge_across_cut_path_iter; // next
        }
    }

    // there exists not transformed halfedges connected to the current halfedge
    if (halfedges_across_cut_path.empty()) {
        return resolved_inst; // return the original descriptor
    }

    // At this, point we have found a number of halfedges which share "m0_h_tgt"
    // with the current halfedge. So we need to decide what value (instance) of
    // "m0_h_tgt" we should assign "resolved_inst"

    // We classify "halfedges_across_cut_path" into two sets:
    // 1) "halfedges_on_same_side" (... as m0_h )
    // 2) "halfedges_across_cut_path" (other-side)
    //

    std::vector<hd_t> halfedges_on_same_side;

    if (m0_h_is_ox) {
        //
        // check if the opposite halfedge has been transformed
        //

        // get opposite halfedge of the current halfedge (m0_h)
        const hd_t opp = m0.opposite(m0_h);

        if (m0_h_to_ply.find(opp) != m0_h_to_ply.end()) { // was the opposite halfedge used to trace a polygon
            // get the previous of the opposite halfedge (because it is one of the "incoming" halfedges)
            const hd_t prv_opp = m0.prev(opp);

            if (m0_sm_ihe_to_flag.at(prv_opp)) { // is halfedge processed
                halfedges_on_same_side.push_back(prv_opp);
                // prv_opp is guarranteed to be in halfedges_on_same_side becz halfedges_across_cut_path is simply a vec of all incoming hes
                halfedges_across_cut_path.erase(std::find(halfedges_across_cut_path.cbegin(), halfedges_across_cut_path.cend(), prv_opp));
            }
        }
    } else if (m0_h_to_ply.find(m0_h) == m0_h_to_ply.end()) // edge-case when src-mesh is not watertight (e.g. test 21)
    {
        MCUT_ASSERT(halfedges_across_cut_path.size() == 1);
        const hd_t& h = halfedges_across_cut_path.front();
        const hd_t& h_proc = m0_to_m1_ihe.at(h);
        vd_t h_tgt = m1.target(h_proc);
        const vd_t tgt_copy = m1.add_vertex(m1.vertex(h_tgt)); // make a copy

        (*logger_ptr) << "duplicate vertex : original=" << vstr(h_tgt) << " copy=" << vstr(tgt_copy) << std::endl;
        resolved_inst = tgt_copy;
    } else { // then halfedge is either xx or xo (see the conditions with which function is called)

        const hd_t nxt = m0.next(m0_h);
        const hd_t opp_nxt = m0.opposite(nxt);

        //MCUT_ASSERT(opp_nxt != mesh_t::null_halfedge());

        // if halfedge incident to traced polygon and is it processed
        if (m0_h_to_ply.find(opp_nxt) != m0_h_to_ply.end() && m0_sm_ihe_to_flag.at(opp_nxt)) {

            const vd_t nxt_src = m0_h_tgt; // i.e. m0.source(nxt);
            const vd_t nxt_tgt = m0.target(nxt);
            const bool nxt_src_is_itvx = m0_is_intersection_point(nxt_src, ps_vtx_cnt);
            const bool nxt_tgt_is_itvx = m0_is_intersection_point(nxt_tgt, ps_vtx_cnt);
            const bool nxt_is_xx = nxt_src_is_itvx && nxt_tgt_is_itvx;
            bool on_same_side = true;

            if (nxt_is_xx) {
                const hd_t nxt_src_ps_h = m0_ivtx_to_ps_he.at(nxt_src);
                const hd_t nxt_tgt_ps_h = m0_ivtx_to_ps_he.at(nxt_tgt);
                const ed_t nxt_src_ps_e = ps.edge(nxt_src_ps_h);
                const ed_t nxt_tgt_ps_e = ps.edge(nxt_tgt_ps_h);
                const bool nxt_is_exterior = (nxt_src_ps_e == nxt_tgt_ps_e); // lays on exterior of ps polygon
                on_same_side = nxt_is_exterior;
            }

            if (on_same_side) {
                halfedges_on_same_side.push_back(opp_nxt);
                halfedges_across_cut_path.erase(std::find(halfedges_across_cut_path.cbegin(), halfedges_across_cut_path.cend(), opp_nxt));
            }
        }
    }

    //
    // Decide what to do with target(h) i.e. determine the correct value for "resolved_inst"
    //

    if (!halfedges_on_same_side.empty()) { // do we already have a halfedge on the [same side] which is tranformed...?
        const hd_t& ss_h = halfedges_on_same_side.front(); // we can retrieve any one
        const hd_t& ss_h_proc = m0_to_m1_ihe.at(ss_h); // m1 version
        resolved_inst = m1.target(ss_h_proc); // update reference
    } else { // do we already have a halfedge on the [other side] which is tranformed...?

        MCUT_ASSERT(!halfedges_across_cut_path.empty());

        const hd_t& h = halfedges_across_cut_path.front();
        const hd_t& h_proc = m0_to_m1_ihe.at(h);
        vd_t h_tgt = m1.target(h_proc);
        const vd_t tgt_copy = m1.add_vertex(m1.vertex(h_tgt)); // make a copy

        (*logger_ptr) << "duplicate vertex : original=" << vstr(h_tgt) << " copy=" << vstr(tgt_copy) << std::endl;
        resolved_inst = tgt_copy;
    }

    return resolved_inst;
};

#if defined(MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE)
std::vector<vd_t> insert_bounding_box_mesh(mesh_t& bvh_mesh, const geom::bounding_box_t<math::fast_vec3>& bbox)
{
    math::fast_vec3 dim2 = ((bbox.maximum() - bbox.minimum()) / 2.0);
    math::fast_vec3 back_bottom_left(-dim2.x(), -dim2.y(), -dim2.z());
    math::fast_vec3 shift = (bbox.minimum() - back_bottom_left);
    math::fast_vec3 front_bl = (math::fast_vec3(-dim2.x(), -dim2.y(), dim2.z()) + shift);
    math::fast_vec3 front_br = (math::fast_vec3(dim2.x(), -dim2.y(), dim2.z()) + shift);
    math::fast_vec3 front_tr = (math::fast_vec3(dim2.x(), dim2.y(), dim2.z()) + shift);
    math::fast_vec3 front_tl = (math::fast_vec3(-dim2.x(), dim2.y(), dim2.z()) + shift);
    math::fast_vec3 back_bl = (back_bottom_left + shift);
    math::fast_vec3 back_br = (math::fast_vec3(dim2.x(), -dim2.y(), -dim2.z()) + shift);
    math::fast_vec3 back_tr = (math::fast_vec3(dim2.x(), dim2.y(), -dim2.z()) + shift);
    math::fast_vec3 back_tl = (math::fast_vec3(-dim2.x(), dim2.y(), -dim2.z()) + shift);

    std::vector<vd_t> v;
    v.resize(8);

    // front
    v[0] = bvh_mesh.add_vertex(front_bl); // bottom left
    MCUT_ASSERT(v[0] != mesh_t::null_vertex());
    v[1] = bvh_mesh.add_vertex(front_br); // bottom right
    MCUT_ASSERT(v[1] != mesh_t::null_vertex());
    v[2] = bvh_mesh.add_vertex(front_tr); // top right
    MCUT_ASSERT(v[2] != mesh_t::null_vertex());
    v[3] = bvh_mesh.add_vertex(front_tl); // top left
    MCUT_ASSERT(v[3] != mesh_t::null_vertex());
    // back
    v[4] = bvh_mesh.add_vertex(back_bl); // bottom left
    MCUT_ASSERT(v[4] != mesh_t::null_vertex());
    v[5] = bvh_mesh.add_vertex(back_br); // bottom right
    MCUT_ASSERT(v[5] != mesh_t::null_vertex());
    v[6] = bvh_mesh.add_vertex(back_tr); // top right
    MCUT_ASSERT(v[6] != mesh_t::null_vertex());
    v[7] = bvh_mesh.add_vertex(back_tl); // top left
    MCUT_ASSERT(v[7] != mesh_t::null_vertex());

    const std::vector<vd_t> face0 = { v[0], v[1], v[2], v[3] }; // front
    const fd_t f0 = bvh_mesh.add_face(face0);
    MCUT_ASSERT(f0 != mesh_t::null_face());

    const std::vector<vd_t> face1 = { v[7], v[6], v[5], v[4] }; //  back
    const fd_t f1 = bvh_mesh.add_face(face1);
    MCUT_ASSERT(f1 != mesh_t::null_face());

    const std::vector<vd_t> face2 = { v[1], v[5], v[6], v[2] }; // right
    const fd_t f2 = bvh_mesh.add_face(face2);
    MCUT_ASSERT(f2 != mesh_t::null_face());

    const std::vector<vd_t> face3 = { v[0], v[3], v[7], v[4] }; // left
    const fd_t f3 = bvh_mesh.add_face(face3);
    MCUT_ASSERT(f3 != mesh_t::null_face());

    const std::vector<vd_t> face4 = { v[3], v[2], v[6], v[7] }; // top
    const fd_t f4 = bvh_mesh.add_face(face4);
    MCUT_ASSERT(f4 != mesh_t::null_face());

    const std::vector<vd_t> face5 = { v[4], v[5], v[1], v[0] }; // bottom
    const fd_t f5 = bvh_mesh.add_face(face5);
    MCUT_ASSERT(f5 != mesh_t::null_face());
    return v;
}
#endif // #if defined(MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE)

bool check_input_mesh(const mesh_t& m)
{
    bool result = true;
    if (m.number_of_vertices() < 3) {
        (*logger_ptr).set_reason_for_failure("invalid input-mesh vertex count (" + std::to_string(m.number_of_vertices()) + ")");
        result = false;
    }

    if (m.number_of_faces() < 1) {
        (*logger_ptr).set_reason_for_failure("invalid input-mesh face count (" + std::to_string(m.number_of_faces()) + ")");
        result = false;
    }

    std::map<face_descriptor_t, int> fccmap;
    int n = find_connected_components(fccmap, m);

    if (n != 1) {
        (*logger_ptr).set_reason_for_failure("invalid number of connected components in input-mesh (" + std::to_string(n) + ")");
        result = false;
    }

    return result;
}

// TODO: replace with mesh.halfedge() function (which will need to be slightly extended)
// returns whether an edge connecting two  points has been created or not
bool interior_edge_exists(const mesh_t& m, const vd_t& src, const vd_t& tgt, const std::vector<ed_t>& m0_cutpath_edges)
{
    bool result = false;
    for (std::vector<ed_t>::const_iterator i = m0_cutpath_edges.cbegin(); i != m0_cutpath_edges.cend(); ++i) {
        if ((m.vertex(*i, 0) == src && m.vertex(*i, 1) == tgt) || (m.vertex(*i, 0) == tgt && m.vertex(*i, 1) == src)) {
            result = true;
        }
    }
    return result;
}

//
// update the edges coincident to an intersecting face of the polygon soup mesh
//
void update_neighouring_ps_iface_m0_edge_list(
    const vd_t& src_vertex,
    const vd_t& tgt_vertex,
    const mesh_t& ps,
    const fd_t sm_face,
    const fd_t cs_face,
    const std::map<vd_t, std::vector<fd_t>>& m0_ivtx_to_ps_faces,
    std::map<fd_t, std::vector<ed_t>>& ps_iface_to_m0_edge_list,
    const std::vector<ed_t>& m0_cutpath_edges)
{

    // for all neighbours of "sm_face" and "cs_face"
    //  if the face is in the registry of src and tgt vertex
    //      get list of face
    //      if list does not already contain new edge
    //          add new edge to list

    std::vector<fd_t> neighbouring_ifaces;
    for (auto neigh_face : { sm_face, cs_face }) {
        const std::vector<face_descriptor_t> faces_around_face = ps.get_faces_around_face(neigh_face);
        neighbouring_ifaces.insert(neighbouring_ifaces.end(), faces_around_face.cbegin(), faces_around_face.cend());
    }

    // for each face that is a neighbour to either sm-face or cm-face
    for (std::vector<fd_t>::const_iterator neigh_face_it = neighbouring_ifaces.cbegin();
         neigh_face_it != neighbouring_ifaces.cend();
         ++neigh_face_it) {
        const fd_t iface = *neigh_face_it;

        MCUT_ASSERT(iface != sm_face && iface != cs_face);

        const std::vector<fd_t>& src_registry = m0_ivtx_to_ps_faces.at(src_vertex);
        const std::vector<fd_t>& tgt_registry = m0_ivtx_to_ps_faces.at(tgt_vertex);

        const bool in_src_reg = std::find(src_registry.cbegin(), src_registry.cend(), iface) != src_registry.cend();
        const bool in_tgt_reg = std::find(tgt_registry.cbegin(), tgt_registry.cend(), iface) != tgt_registry.cend();

        if (in_src_reg && in_tgt_reg) {
            std::map<fd_t, std::vector<ed_t>>::iterator fiter = ps_iface_to_m0_edge_list.find(iface);
            bool iface_associated_with_some_edges = true;
            if (fiter == ps_iface_to_m0_edge_list.cend()) {
                // insert
                std::pair<std::map<fd_t, std::vector<ed_t>>::iterator, bool> p = ps_iface_to_m0_edge_list.insert(std::make_pair(iface, std::vector<ed_t>()));
                MCUT_ASSERT(p.second == true);
                fiter = p.first;
                iface_associated_with_some_edges = false;
            }

            MCUT_ASSERT(fiter != ps_iface_to_m0_edge_list.cend());

            std::vector<ed_t>& iface_m0_edge_list = fiter->second;

            bool associate_iface_with_edge = true;
            if (iface_associated_with_some_edges) {
                bool edge_already_associated_with_iface = std::find(iface_m0_edge_list.cbegin(), iface_m0_edge_list.cend(), m0_cutpath_edges.back()) != iface_m0_edge_list.cend();
                associate_iface_with_edge = !(edge_already_associated_with_iface);
            }

            if (associate_iface_with_edge) {
                iface_m0_edge_list.push_back(m0_cutpath_edges.back());
            }
        }
    }
}

// check if either "current_edge" or "next_edge" represents a false link
// i.e. due concave intersection (intersection registry representation is
// underdetermined)
// We do this by checking if an edge can be put between any two vertices of these
// two edges

bool cutpath_vertices_are_connectable(
    const ed_t& current_edge,
    const ed_t& next_edge,
    const mesh_t& m0,
    const std::map<vd_t, std::vector<fd_t>>& m0_ivtx_to_ps_faces,
    bool& registry_entries_match_by_3_real_faces,
    bool& registry_entries_differ_by_1_real_face_pc)
{
    registry_entries_match_by_3_real_faces = true;
    registry_entries_differ_by_1_real_face_pc = true;

    const vd_t current_edge_vertex0 = m0.vertex(current_edge, 0);
    const vd_t current_edge_vertex1 = m0.vertex(current_edge, 1);

    const vd_t next_edge_vertex0 = m0.vertex(next_edge, 0);
    const vd_t next_edge_vertex1 = m0.vertex(next_edge, 1);

    std::vector<vd_t> vertices_used_by_edges;

    // identify which vertex is not used by both edges (to prevent duplicates)
    if (current_edge_vertex0 == next_edge_vertex0 || current_edge_vertex0 == next_edge_vertex1) {
        vertices_used_by_edges.push_back(current_edge_vertex1);
    } else {
        vertices_used_by_edges.push_back(current_edge_vertex0);
    }

    vertices_used_by_edges.push_back(next_edge_vertex0);
    vertices_used_by_edges.push_back(next_edge_vertex1);

    // can I put an edge between these three vertices...? (i.e. do they share at least two
    // real faces in they registry entries)

    std::map<vd_t, std::vector<fd_t>>::const_iterator find_iter;
    find_iter = m0_ivtx_to_ps_faces.find(vertices_used_by_edges.front());
    MCUT_ASSERT(find_iter != m0_ivtx_to_ps_faces.cend());

    const std::vector<fd_t>& entry_faces_first = find_iter->second;

    int connectable_vertices = 0;
    int differing_real_faces = 0;
    // for every other vertex
    for (int i = 1; i < (int)vertices_used_by_edges.size(); ++i) {

        find_iter = m0_ivtx_to_ps_faces.find(vertices_used_by_edges.at(i));
        MCUT_ASSERT(find_iter != m0_ivtx_to_ps_faces.cend());
        const std::vector<fd_t>& entry_faces_other = find_iter->second;

        int matching_faces = 0;

        // for each face in the registry entry of first vertex
        for (std::vector<fd_t>::const_iterator face_iter = entry_faces_first.cbegin();
             face_iter != entry_faces_first.cend();
             ++face_iter) {

            if (is_virtual_polygon(*face_iter)) {
                continue; // we compare based on the number of real faces
            }

            bool match = std::find(entry_faces_other.cbegin(), entry_faces_other.cend(), *face_iter) != entry_faces_other.cend();

            if (match) {
                ++matching_faces;
            } else {
                ++differing_real_faces;
            }
        }

        MCUT_ASSERT(matching_faces <= 3);

        // can I put an edge between the first vertex and the current "other" vertex...?
        // i.e. based on our connectivity theory that "an edge is placed between two
        // intersection points if they share atleast two faces in their registry entry"

        bool connectable = (matching_faces >= 2);

        registry_entries_match_by_3_real_faces = (registry_entries_match_by_3_real_faces && (matching_faces == 3)); // all vertices!

        if (connectable) {
            ++connectable_vertices;
        }
    }

    MCUT_ASSERT(connectable_vertices <= 2);

    // used specifically for tet-triangle intersection problem
    registry_entries_differ_by_1_real_face_pc = (differing_real_faces == 2);

    bool all_vertices_are_connectable = (connectable_vertices == 2);
    return all_vertices_are_connectable;
}

typedef std::vector<hd_t> traced_polygon_t;

bool check_is_floating_patch(
    const traced_polygon_t& patch_poly,
    const mesh_t& m0,
    const mesh_t& ps,
    const std::map<vd_t, hd_t>& m0_ivtx_to_ps_he,
    const int ps_vtx_cnt,
    const int sm_face_count)
{
    std::vector<vd_t> patch_poly_vertices;

    for (traced_polygon_t::const_iterator patch_poly_he_iter = patch_poly.cbegin();
         patch_poly_he_iter != patch_poly.cend();
         ++patch_poly_he_iter) {
        const vd_t tgt = m0.target(*patch_poly_he_iter);
        // floating-patch graphs only have one polygon, where that polygon is defined
        // only by intersection vertices
        if (!m0_is_intersection_point(tgt, ps_vtx_cnt)) {
            return false; // is not an intersection point, therefore polygon cannot be patch!
        }
        patch_poly_vertices.push_back(tgt);
    }

    // now check if all vertices resulted from a src-mesh halfedge intersecting a cut-mesh halfedge
    bool all_vertices_from_src_mesh_he = true;
    for (std::vector<vd_t>::const_iterator viter = patch_poly_vertices.cbegin();
         viter != patch_poly_vertices.cend();
         viter++) {
        const hd_t& entry_ps_he = m0_ivtx_to_ps_he.at(*viter);
        const fd_t& entry_ps_he_face = ps.face(entry_ps_he);

        bool is_from_src_mesh_he = !ps_is_cutmesh_face(entry_ps_he_face, sm_face_count);

        if (!is_from_src_mesh_he) {
            all_vertices_from_src_mesh_he = false;
            break;
        }
    }

    return all_vertices_from_src_mesh_he;
}

bool mesh_is_closed(const mesh_t& mesh)
{
    bool all_halfedges_incident_to_face = true;
    for (mesh_t::halfedge_iterator_t iter = mesh.halfedges_begin(); iter != mesh.halfedges_end(); ++iter) {
        const fd_t f = mesh.face(*iter);
        if (f == mesh_t::null_face()) {
            all_halfedges_incident_to_face = false;
            break;
        }
    }
    return all_halfedges_incident_to_face;
}

//
// entry point
//
void dispatch(output_t& output, const input_t& input)
{
    logger_t& lg = output.logger;
    logger_ptr = &output.logger;
    lg.reset();
    lg.set_verbose(input.verbose);

    const mesh_t& sm = (*input.src_mesh);
    const mesh_t& cs = (*input.cut_mesh);

    ///////////////////////////////////////////////////////////////////////////
    // check input meshes for errors
    ///////////////////////////////////////////////////////////////////////////

    lg << "check src-mesh." << std::endl;
    if (check_input_mesh(sm) == false) {
        output.status = status_t::INVALID_SRC_MESH;
        return;
    }

    if(input.verbose){
        dump_mesh(sm, "src-mesh");
    }

    lg << "check cut-mesh." << std::endl;
    if (check_input_mesh(cs) == false) {
        output.status = status_t::INVALID_CUT_MESH;
        return;
    }
    if(input.verbose){
        dump_mesh(cs, "cut-mesh");
    }

    const int sm_vtx_cnt = sm.number_of_vertices();
    const int sm_face_count = sm.number_of_faces();
    const int cs_face_count = cs.number_of_faces();

    const bool sm_is_watertight = mesh_is_closed(sm);
    lg << "src-mesh is watertight = " << sm_is_watertight << std::endl;

    const bool cs_is_watertight = mesh_is_closed(cs);
    lg << "cut-mesh is watertight = " << cs_is_watertight << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    // create polygon soup
    ///////////////////////////////////////////////////////////////////////////

    lg << "create polygon soup (ps)" << std::endl;

    mesh_t ps = sm;

    std::map<vd_t, vd_t> cs_to_ps_vtx; // vmap

    // merge cm vertices
    for (auto i = cs.vertices_begin(); i != cs.vertices_end(); ++i) {
        const vd_t v = ps.add_vertex(cs.vertex(*i));

        MCUT_ASSERT(v != mesh_t::null_vertex());

        cs_to_ps_vtx.insert(std::make_pair(*i, v));
    }

    // merge cm faces
    for (mesh_t::face_iterator_t i = cs.faces_begin(); i != cs.faces_end(); ++i) {

        //std::vector<vd_t> fv = get_vertices_on_face(cs, *i);
        std::vector<vd_t> fv = cs.get_vertices_around_face(*i);
        std::vector<vd_t> remapped_face_vertices;

        for (std::vector<vd_t>::const_iterator j = fv.cbegin(); j != fv.cend(); ++j) {
            remapped_face_vertices.push_back(cs_to_ps_vtx.at(*j));
        }

        const fd_t f = ps.add_face(remapped_face_vertices);

        MCUT_ASSERT(f != mesh_t::null_face());
    }

    cs_to_ps_vtx.clear(); 

    if(input.verbose){
        dump_mesh(ps, "polygon-soup");
    }

    const int ps_vtx_cnt = ps.number_of_vertices();
    const int ps_face_cnt = ps.number_of_faces();

    lg << "polygon-soup vertices = " << ps_vtx_cnt << std::endl;
    lg << "polygon-soup faces = " << ps_face_cnt << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    // create the first auxilliary halfedge data structure ("m0")
    ///////////////////////////////////////////////////////////////////////////

    lg << "create auxiliary mesh (`m0`)" << std::endl;

    // The auxilliary data structure stores:
    // 1) vertices of the polygon-soup, including new intersection points
    // 2) Non-intersecting edges of the polygon-soup
    // 3) New edges created from intersection points
    mesh_t m0;

    // copy ps vertices into the auxilliary mesh (map is used to maintain original vertex order)
    std::map<vd_t, vd_t> m0_to_ps_vtx;

    for (auto i = ps.vertices_begin(); i != ps.vertices_end(); ++i) {
        const vd_t v = m0.add_vertex(ps.vertex(*i));

        MCUT_ASSERT(v != mesh_t::null_vertex());

        m0_to_ps_vtx.emplace(v, *i);
    }

    MCUT_ASSERT(m0.number_of_vertices() == ps.number_of_vertices()); // ... because we have only copied vertices

    ///////////////////////////////////////////////////////////////////////////
    // calculate BVHs
    ///////////////////////////////////////////////////////////////////////////

    lg << "calculate bounding volume hierarchies (BVH)" << std::endl;

    std::map<fd_t, geom::bounding_box_t<math::fast_vec3>> ps_face_bboxes;
    std::map<fd_t, math::fast_vec3> ps_face_bbox_centres;

    for (mesh_t::face_iterator_t i = ps.faces_begin(); i != ps.faces_end(); ++i) {

        //const std::vector<vd_t> vertices_on_face = get_vertices_on_face(ps, *i);
        const std::vector<vd_t> vertices_on_face = ps.get_vertices_around_face(*i);

        for (std::vector<vd_t>::const_iterator face_vertex_iter = vertices_on_face.cbegin(); face_vertex_iter != vertices_on_face.cend(); ++face_vertex_iter) {
            const math::fast_vec3 face_vertex_coords = ps.vertex(*face_vertex_iter);
            ps_face_bboxes[*i].expand(face_vertex_coords);
        }

        //const double eps = 1e-6; // enlarge Bbox by small epsilon
        // ps_face_bboxes[*i] = CGAL::Bbox_3(minx, miny, minz, maxx, maxy, maxz);

        const geom::bounding_box_t<math::fast_vec3>& bbox = ps_face_bboxes[*i];
        // calculate bbox centre for morton code evaluation
        ps_face_bbox_centres[*i] = (bbox.minimum() + bbox.maximum()) / 2;
    }

    std::map<std::string, std::vector<geom::bounding_box_t<math::fast_vec3>>> bvh_internal_nodes_array = {
        { "sm", {} },
        { "cs", {} },
    };

    std::map<std::string, std::vector<std::pair<fd_t, unsigned int>>> bvh_leaf_nodes_array = {
        { "sm", {} },
        { "cs", {} },
    };

    for (std::map<std::string, std::vector<geom::bounding_box_t<math::fast_vec3>>>::iterator mesh_bvh_iter = bvh_internal_nodes_array.begin();
         mesh_bvh_iter != bvh_internal_nodes_array.end();
         ++mesh_bvh_iter) {
        lg.indent();
        const std::string mesh_name = mesh_bvh_iter->first;

        lg << "mesh = " << mesh_name << std::endl;
        lg.indent();
        std::vector<geom::bounding_box_t<math::fast_vec3>>& bvh = mesh_bvh_iter->second; // my ostensibly implicit BVH

        std::vector<std::pair<fd_t, unsigned int>> unsorted_morton_codes;
        std::map<fd_t, math::fast_vec3>::const_iterator ps_face_bbox_centres_iter_cbegin; // sm face come first
        std::map<fd_t, math::fast_vec3>::const_iterator ps_face_bbox_centres_iter_cend;

        if (mesh_name == "sm") {
            ps_face_bbox_centres_iter_cbegin = ps_face_bbox_centres.cbegin();
            ps_face_bbox_centres_iter_cend = ps_face_bbox_centres.cbegin();
            std::advance(ps_face_bbox_centres_iter_cend, sm_face_count); // offset to start of cs faces
        } else { // cs faces
            ps_face_bbox_centres_iter_cbegin = ps_face_bbox_centres.cbegin();
            std::advance(ps_face_bbox_centres_iter_cbegin, sm_face_count); // offset to start of cs faces
            ps_face_bbox_centres_iter_cend = ps_face_bbox_centres.cend();
        }

        lg << "calculate mesh bounding box" << std::endl;

        geom::bounding_box_t<math::fast_vec3>& meshBbox = ps_face_bboxes.at(ps_face_bbox_centres_iter_cbegin->first); // root bounding box

        for (std::map<fd_t, math::fast_vec3>::const_iterator ps_face_bbox_centres_iter = ps_face_bbox_centres_iter_cbegin;
             ps_face_bbox_centres_iter != ps_face_bbox_centres_iter_cend;
             ++ps_face_bbox_centres_iter) {

            if (ps_face_bbox_centres_iter == ps_face_bbox_centres_iter_cbegin) {
                continue;
            }

            //meshBbox += ps_face_bboxes.at(ps_face_bbox_centres_iter->first);
            meshBbox.expand(ps_face_bboxes.at(ps_face_bbox_centres_iter->first));
        }

        lg << "calculate morton codes" << std::endl;

        for (std::map<fd_t, math::fast_vec3>::const_iterator ps_face_bbox_centres_iter = ps_face_bbox_centres_iter_cbegin;
             ps_face_bbox_centres_iter != ps_face_bbox_centres_iter_cend;
             ++ps_face_bbox_centres_iter) {

            const fd_t& face_id = ps_face_bbox_centres_iter->first;
            const math::fast_vec3& face_aabb_centre = ps_face_bbox_centres_iter->second;
            math::fast_vec3 offset = face_aabb_centre - meshBbox.minimum();
            math::fast_vec3 dims = meshBbox.maximum() - meshBbox.minimum();

            const unsigned int mortion_code = bvh::morton3D(
                static_cast<float>(offset.x() / dims.x()),
                static_cast<float>(offset.y() / dims.y()),
                static_cast<float>(offset.z() / dims.z()));

            unsorted_morton_codes.emplace_back(face_id, mortion_code);
        }

        lg << "sort morton codes" << std::endl;

        bvh_leaf_nodes_array.at(mesh_name).insert(bvh_leaf_nodes_array.at(mesh_name).begin(), unsorted_morton_codes.cbegin(), unsorted_morton_codes.cend()); // copy

        unsorted_morton_codes.clear(); // free

        std::sort(
            bvh_leaf_nodes_array.at(mesh_name).begin(),
            bvh_leaf_nodes_array.at(mesh_name).end(),
            [](const std::pair<fd_t, unsigned int>& a, const std::pair<fd_t, unsigned int>& b) {
                return a.second < b.second;
            });

        const int real_leaf_node_count = mesh_name == "sm" ? sm_face_count : cs_face_count;
        const int bvh_real_node_count = bvh::get_ostensibly_implicit_bvh_size(real_leaf_node_count);

        lg << "build BVH (" << bvh_real_node_count << " nodes)" << std::endl;

        bvh.resize(bvh_real_node_count - real_leaf_node_count); // internal nodes stored separately from leafs

        const int leaf_level_index = bvh::get_leaf_level_from_real_leaf_count(real_leaf_node_count);

        // process internal nodes

        for (int level_index = leaf_level_index - 1; level_index >= 0; --level_index) {

            const int rightmost_real_leaf = bvh::get_rightmost_real_leaf(leaf_level_index, real_leaf_node_count);
            const int rightmost_real_node_on_level = bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, level_index);
            const int leftmost_real_node_on_level = bvh::get_level_leftmost_node(level_index);
            const int number_of_real_nodes_on_level = (rightmost_real_node_on_level - leftmost_real_node_on_level) + 1;

            for (int level_node_idx_iter = 0; level_node_idx_iter < number_of_real_nodes_on_level; ++level_node_idx_iter) {

                const int node_implicit_idx = leftmost_real_node_on_level + level_node_idx_iter;
                const int left_child_implicit_idx = (node_implicit_idx * 2) + 1;
                const int right_child_implicit_idx = (node_implicit_idx * 2) + 2;
                const bool is_penultimate_level = (level_index == (leaf_level_index - 1));
                const int rightmost_real_node_on_child_level = bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, level_index + 1);
                const int leftmost_real_node_on_child_level = bvh::get_level_leftmost_node(level_index + 1);
                const bool right_child_exists = (right_child_implicit_idx <= rightmost_real_node_on_child_level);

                geom::bounding_box_t<math::fast_vec3> node_bbox;

                if (is_penultimate_level) {

                    const int left_child_index_on_level = left_child_implicit_idx - leftmost_real_node_on_child_level;
                    const fd_t& left_child_face = bvh_leaf_nodes_array.at(mesh_name).at(left_child_index_on_level).first;
                    const geom::bounding_box_t<math::fast_vec3>& left_child_bbox = ps_face_bboxes.at(left_child_face);
                    //node_bbox = left_child_bbox;
                    node_bbox.expand(left_child_bbox);

                    if (right_child_exists) {
                        const int right_child_index_on_level = right_child_implicit_idx - leftmost_real_node_on_child_level;
                        const fd_t& right_child_face = bvh_leaf_nodes_array.at(mesh_name).at(right_child_index_on_level).first;
                        const geom::bounding_box_t<math::fast_vec3>& right_child_bbox = ps_face_bboxes.at(right_child_face);
                        node_bbox.expand(right_child_bbox);
                    }
                } else { // remaining internal node levels

                    const int left_child_memory_idx = bvh::get_node_mem_index(
                        left_child_implicit_idx,
                        leftmost_real_node_on_child_level,
                        0,
                        rightmost_real_node_on_child_level);
                    const geom::bounding_box_t<math::fast_vec3>& left_child_bbox = bvh.at(left_child_memory_idx);
                    node_bbox.expand(left_child_bbox);

                    if (right_child_exists) {
                        const int right_child_memory_idx = bvh::get_node_mem_index(
                            right_child_implicit_idx,
                            leftmost_real_node_on_child_level,
                            0,
                            rightmost_real_node_on_child_level);
                        const geom::bounding_box_t<math::fast_vec3>& right_child_bbox = bvh.at(right_child_memory_idx);
                        node_bbox.expand(right_child_bbox);
                    }
                }

                const int node_memory_idx = bvh::get_node_mem_index(
                    node_implicit_idx,
                    leftmost_real_node_on_level,
                    0,
                    rightmost_real_node_on_level);

                bvh.at(node_memory_idx) = node_bbox;
            } // for each real node on level
        } // for each internal level
        lg.unindent();
        lg.unindent();
    } // for each mesh BVH

    ps_face_bbox_centres.clear();

#if defined(MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE)
    ///////////////////////////////////////////////////////////////////////////
    // generate BVH meshes
    ///////////////////////////////////////////////////////////////////////////

    if (input.verbose) {
        lg << "create BVH meshes\n";

        for (std::map<std::string, std::vector<geom::bounding_box_t<math::fast_vec3>>>::iterator mesh_bvh_iter = bvh_internal_nodes_array.begin();
             mesh_bvh_iter != bvh_internal_nodes_array.end();
             ++mesh_bvh_iter) {

            const std::string mesh_name = mesh_bvh_iter->first;
            const std::vector<geom::bounding_box_t<math::fast_vec3>>& internal_nodes_array = mesh_bvh_iter->second;
            const std::vector<std::pair<fd_t, unsigned int>>& leaf_nodes_array = bvh_leaf_nodes_array.at(mesh_name);
            const int real_leaf_node_count = (int)leaf_nodes_array.size();
            //const int bvh_real_node_count = bvh::get_ostensibly_implicit_bvh_size(real_leaf_node_count);
            const int leaf_level_index = bvh::get_leaf_level_from_real_leaf_count(real_leaf_node_count);

            mesh_t bvh_mesh;

            // internal levels
            for (int level_idx = 0; level_idx <= leaf_level_index; ++level_idx) {

                const int rightmost_real_leaf = bvh::get_rightmost_real_leaf(leaf_level_index, real_leaf_node_count);
                const int rightmost_real_node_on_level = bvh::get_level_rightmost_real_node(rightmost_real_leaf, leaf_level_index, level_idx);
                const int leftmost_real_node_on_level = bvh::get_level_leftmost_node(level_idx);
                const int number_of_real_nodes_on_level = (rightmost_real_node_on_level - leftmost_real_node_on_level) + 1;

                for (int level_node_idx_iter = 0; level_node_idx_iter < number_of_real_nodes_on_level; ++level_node_idx_iter) {

                    geom::bounding_box_t<math::fast_vec3> node_bbox;
                    const bool is_leaf_level = (level_idx == leaf_level_index);

                    if (is_leaf_level) {
                        const fd_t leaf_node_face = leaf_nodes_array.at(level_node_idx_iter).first;
                        //node_bbox = ps_face_bboxes.at(leaf_node_face);
                        node_bbox.expand(ps_face_bboxes.at(leaf_node_face));
                    } else {
                        const int node_implicit_idx = leftmost_real_node_on_level + level_node_idx_iter;
                        const int node_memory_idx = bvh::get_node_mem_index(
                            node_implicit_idx,
                            leftmost_real_node_on_level,
                            0,
                            rightmost_real_node_on_level);
                        node_bbox.expand(internal_nodes_array.at(node_memory_idx));
                    }

                    std::vector<vd_t> node_bbox_vertices = insert_bounding_box_mesh(bvh_mesh, node_bbox);
                }
            }

            dump_mesh(bvh_mesh, (mesh_name + ".bvh").c_str());
        }

    } // if (input.verbose)
#endif // #if defined(MCUT_DUMP_BVH_MESH_IN_DEBUG_MODE)

    ///////////////////////////////////////////////////////////////////////////
    // Evaluate BVHs
    ///////////////////////////////////////////////////////////////////////////

    lg << "find potentially-intersecting polygons" << std::endl;

    std::vector<std::pair<fd_t, fd_t>> intersecting_sm_cs_face_pairs;

    // simultaneuosly traverse both BVHs to find intersecting pairs
    std::queue<bvh::node_pair_t> oibvh_traversal_queue;
    oibvh_traversal_queue.push({ 0, 0 }); // left = sm; right = cs

    const int sm_bvh_leaf_level_idx = bvh::get_leaf_level_from_real_leaf_count(sm_face_count);
    const int cs_bvh_leaf_level_idx = bvh::get_leaf_level_from_real_leaf_count(cs_face_count);

    const int cs_bvh_rightmost_real_leaf = bvh::get_rightmost_real_leaf(cs_bvh_leaf_level_idx, cs_face_count);
    const int sm_bvh_rightmost_real_leaf = bvh::get_rightmost_real_leaf(sm_bvh_leaf_level_idx, sm_face_count);

    do {
        bvh::node_pair_t ct_front_node = oibvh_traversal_queue.front();

        geom::bounding_box_t<math::fast_vec3> sm_bvh_node_bbox;
        geom::bounding_box_t<math::fast_vec3> cs_bvh_node_bbox;

        // sm
        const int sm_bvh_node_implicit_idx = ct_front_node.m_left;
        const int sm_bvh_node_level_idx = bvh::get_level_from_implicit_idx(sm_bvh_node_implicit_idx);
        const bool sm_bvh_node_is_leaf = sm_bvh_node_level_idx == sm_bvh_leaf_level_idx;
        const int sm_bvh_node_level_leftmost_node = bvh::get_level_leftmost_node(sm_bvh_node_level_idx);
        fd_t sm_node_face = mesh_t::null_face();

        if (sm_bvh_node_is_leaf) {
            const int sm_bvh_node_idx_on_level = sm_bvh_node_implicit_idx - sm_bvh_node_level_leftmost_node;
            sm_node_face = bvh_leaf_nodes_array.at("sm").at(sm_bvh_node_idx_on_level).first;
            sm_bvh_node_bbox = ps_face_bboxes.at(sm_node_face);
        } else {

            const int sm_bvh_node_level_rightmost_node = bvh::get_level_rightmost_real_node(sm_bvh_rightmost_real_leaf, sm_bvh_leaf_level_idx, sm_bvh_node_level_idx);
            const int sm_bvh_node_mem_idx = bvh::get_node_mem_index(
                sm_bvh_node_implicit_idx,
                sm_bvh_node_level_leftmost_node,
                0,
                sm_bvh_node_level_rightmost_node);
            sm_bvh_node_bbox = bvh_internal_nodes_array.at("sm").at(sm_bvh_node_mem_idx);
        }

        // cs
        const int cs_bvh_node_implicit_idx = ct_front_node.m_right;
        const int cs_bvh_node_level_idx = bvh::get_level_from_implicit_idx(cs_bvh_node_implicit_idx);
        const int cs_bvh_node_level_leftmost_node = bvh::get_level_leftmost_node(cs_bvh_node_level_idx);
        const bool cs_bvh_node_is_leaf = cs_bvh_node_level_idx == cs_bvh_leaf_level_idx;
        fd_t cs_node_face = mesh_t::null_face();

        if (cs_bvh_node_is_leaf) {

            const int cs_bvh_node_idx_on_level = cs_bvh_node_implicit_idx - cs_bvh_node_level_leftmost_node;
            cs_node_face = bvh_leaf_nodes_array.at("cs").at(cs_bvh_node_idx_on_level).first;
            cs_bvh_node_bbox = ps_face_bboxes.at(cs_node_face);

        } else {

            const int cs_bvh_node_level_rightmost_node = bvh::get_level_rightmost_real_node(cs_bvh_rightmost_real_leaf, cs_bvh_leaf_level_idx, cs_bvh_node_level_idx);
            const int cs_bvh_node_mem_idx = bvh::get_node_mem_index(
                cs_bvh_node_implicit_idx,
                cs_bvh_node_level_leftmost_node,
                0,
                cs_bvh_node_level_rightmost_node);
            cs_bvh_node_bbox = bvh_internal_nodes_array.at("cs").at(cs_bvh_node_mem_idx);
        }

        if (intersect_bounding_boxes(sm_bvh_node_bbox, cs_bvh_node_bbox)) {

            if (cs_bvh_node_is_leaf && sm_bvh_node_is_leaf) {
                MCUT_ASSERT(cs_node_face != mesh_t::null_face());
                MCUT_ASSERT(sm_node_face != mesh_t::null_face());

                intersecting_sm_cs_face_pairs.emplace_back(sm_node_face, cs_node_face);
            } else if (sm_bvh_node_is_leaf && !cs_bvh_node_is_leaf) {
                MCUT_ASSERT(cs_node_face == mesh_t::null_face());
                MCUT_ASSERT(sm_node_face != mesh_t::null_face());

                const int cs_bvh_node_left_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 1;
                const int cs_bvh_node_right_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 2;

                const int rightmost_real_node_on_child_level = bvh::get_level_rightmost_real_node(cs_bvh_rightmost_real_leaf, cs_bvh_leaf_level_idx, cs_bvh_node_level_idx + 1);
                const bool right_child_is_real = cs_bvh_node_right_child_implicit_idx <= rightmost_real_node_on_child_level;

                oibvh_traversal_queue.push({ sm_bvh_node_implicit_idx, cs_bvh_node_left_child_implicit_idx });

                if (right_child_is_real) {
                    oibvh_traversal_queue.push({ sm_bvh_node_implicit_idx, cs_bvh_node_right_child_implicit_idx });
                }
            } else if (!sm_bvh_node_is_leaf && cs_bvh_node_is_leaf) {

                MCUT_ASSERT(cs_node_face != mesh_t::null_face());
                MCUT_ASSERT(sm_node_face == mesh_t::null_face());

                const int sm_bvh_node_left_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 1;
                const int sm_bvh_node_right_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 2;

                const int rightmost_real_node_on_child_level = bvh::get_level_rightmost_real_node(sm_bvh_rightmost_real_leaf, sm_bvh_leaf_level_idx, sm_bvh_node_level_idx + 1);
                const bool right_child_is_real = sm_bvh_node_right_child_implicit_idx <= rightmost_real_node_on_child_level;

                oibvh_traversal_queue.push({ sm_bvh_node_left_child_implicit_idx, cs_bvh_node_implicit_idx });

                if (right_child_is_real) {
                    oibvh_traversal_queue.push({ sm_bvh_node_right_child_implicit_idx, cs_bvh_node_implicit_idx });
                }

            } else { // both nodes are internal
                MCUT_ASSERT(cs_node_face == mesh_t::null_face());
                MCUT_ASSERT(sm_node_face == mesh_t::null_face());

                const int sm_bvh_node_left_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 1;
                const int sm_bvh_node_right_child_implicit_idx = (sm_bvh_node_implicit_idx * 2) + 2;

                const int cs_bvh_node_left_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 1;
                const int cs_bvh_node_right_child_implicit_idx = (cs_bvh_node_implicit_idx * 2) + 2;

                const int sm_rightmost_real_node_on_child_level = bvh::get_level_rightmost_real_node(sm_bvh_rightmost_real_leaf, sm_bvh_leaf_level_idx, sm_bvh_node_level_idx + 1);
                const bool sm_right_child_is_real = sm_bvh_node_right_child_implicit_idx <= sm_rightmost_real_node_on_child_level;

                const int cs_rightmost_real_node_on_child_level = bvh::get_level_rightmost_real_node(cs_bvh_rightmost_real_leaf, cs_bvh_leaf_level_idx, cs_bvh_node_level_idx + 1);
                const bool cs_right_child_is_real = cs_bvh_node_right_child_implicit_idx <= cs_rightmost_real_node_on_child_level;

                oibvh_traversal_queue.push({ sm_bvh_node_left_child_implicit_idx, cs_bvh_node_left_child_implicit_idx });

                if (cs_right_child_is_real) {
                    oibvh_traversal_queue.push({ sm_bvh_node_left_child_implicit_idx, cs_bvh_node_right_child_implicit_idx });
                }

                if (sm_right_child_is_real) {
                    oibvh_traversal_queue.push({ sm_bvh_node_right_child_implicit_idx, cs_bvh_node_left_child_implicit_idx });

                    if (cs_right_child_is_real) {
                        oibvh_traversal_queue.push({ sm_bvh_node_right_child_implicit_idx, cs_bvh_node_right_child_implicit_idx });
                    }
                }
            }
        }

        oibvh_traversal_queue.pop(); // rm ct_front_node
    } while (!oibvh_traversal_queue.empty());

    ps_face_bboxes.clear();
    bvh_internal_nodes_array.clear();
    bvh_leaf_nodes_array.clear();

    lg << "polygon-pairs found = " << intersecting_sm_cs_face_pairs.size() << std::endl;

    const bool bvh_intersection_exists = !intersecting_sm_cs_face_pairs.empty();

    if (!bvh_intersection_exists) {
        lg.set_reason_for_failure("meshes are too far apart.");
        output.status = status_t::INVALID_BVH_INTERSECTION;
        return;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Calculate polygon intersection points
    ///////////////////////////////////////////////////////////////////////////

    lg << "calculate intersection-points" << std::endl;

    // intersection registry
    std::map<vd_t, std::vector<fd_t>> m0_ivtx_to_ps_faces;
    std::map<vd_t, hd_t> m0_ivtx_to_ps_he;

    // cut surface tip re-entrant vertices
    std::vector<vd_t> cs_tip_reentrant_ivtx_list;

    // only need to be computed if input mesh is watertight
    // key=ivertex; value=[intersected-smface-normal;cs-halfedge-to-normal scalar product]
    std::map<vd_t, std::pair<math::vec3, math::real_number_t>> cs_reg_reentrant_ivtx_list;

    // Always needs to be computed (i.e. whether input mesh is watertight or not)
    // key=ivertex; value=[intersected-csface-normal;sm-halfedge-to-normal scalar product]
    std::map<vd_t, std::pair<math::vec3, math::real_number_t>> sm_reg_reentrant_ivtx_list;

    // edges of the polygon soup mesh which intersect a polygon
    std::vector<ed_t> ps_intersecting_edges;

    // mapping from an intersecting ps-face to the edges. These edges are those whose
    // src and tgt vertex contains the respective face in their registry entry
    // (note: all or some may be used for used to clip the face).
    std::map<fd_t, std::vector<ed_t>> ps_iface_to_m0_edge_list;

    // CGAL surface_mesh does not allow check if a half edge already exists (TODO: use a bool matrix instead)
    std::vector<ed_t> m0_cutpath_edges;

    // key=intersection-point; value=[list of halfedges whose target is the [key]-vertex]
    std::map<vd_t, std::vector<hd_t>> ivtx_to_incoming_hlist;

    // for each pair of polygons to be tested for intersection
    for (std::vector<std::pair<fd_t, fd_t>>::const_iterator i = intersecting_sm_cs_face_pairs.cbegin();
         i != intersecting_sm_cs_face_pairs.cend();
         ++i) {
        lg.indent();
        const std::pair<fd_t, fd_t>& intersection_pair = *i;
        const fd_t& sm_face = intersection_pair.first;
        const fd_t& cs_face = intersection_pair.second;

        lg << "src-mesh polygon = " << fstr(sm_face) << std::endl;
        lg << "cut-mesh polygon = " << fstr(cs_face) << std::endl;

        const std::vector<std::pair<fd_t, fd_t>> test_permutations = { { sm_face, cs_face }, { cs_face, sm_face } };
        //const int new_ivertex_start_offset = ps.number_of_vertices() + m0_ivtx_to_ps_faces.size();

        std::vector<vd_t> test_ivertex_collector; // returns all the intersection points from the current poly intersection test

        // for each test permutation (A -> B, B -> A)
        for (std::vector<std::pair<fd_t, fd_t>>::const_iterator j = test_permutations.cbegin();
             j != test_permutations.cend();
             ++j) {

            const fd_t& face_A = j->first;
            const fd_t& face_B = j->second;

            lg.indent();

            lg << "test " << fstr(face_A) << " & " << fstr(face_B) << std::endl;

            // test A vs B
            std::vector<vd_t> face_B_descriptors = ps.get_vertices_around_face(face_B);
            std::vector<math::vec3> face_B_vertices;
            for (std::vector<vd_t>::const_iterator it = face_B_descriptors.cbegin(); it != face_B_descriptors.cend(); ++it) {
                const math::vec3& vertex = ps.vertex(*it);
                lg << "vertex: " << vertex << std::endl;
                face_B_vertices.push_back(vertex);
            }
            //CGAL::Vertex_around_face_iterator<mesh_t> vbegin, vend;

            //for (boost::tie(vbegin, vend) = vertices_around_face(ps.halfedge(face_B), ps); vbegin != vend; ++vbegin) {
            //   face_B_descriptors.push_back(*vbegin);
            // }

            // compute the plane of face B
            // const Exact_kernel::Plane_3 plane = [&]() {
            //     int index = 2; // pick the triplet of vertices which are not colinear
            //     while (CGAL::collinear(ps.point(face_B_descriptors[0]), ps.point(face_B_descriptors[1]), ps.point(face_B_descriptors[index])) && index < face_B_descriptors.size()) {
            //         ++index;
            //     }
            //     return Exact_kernel::Plane_3(ps.point(face_B_descriptors[0]), ps.point(face_B_descriptors[1]), ps.point(face_B_descriptors[index]));
            // }();

            math::vec3 face_B_plane_normal;
            geom::polygon_normal(face_B_plane_normal, face_B_vertices.data(), (int)face_B_vertices.size());

            lg << "plane normal: " << face_B_plane_normal << std::endl;

            math::real_number_t face_B_plane_param_d;
            geom::polygon_plane_d_param(face_B_plane_param_d, face_B_plane_normal, face_B_vertices.data(), (int)face_B_vertices.size());

            lg << "plane d: " << face_B_plane_param_d << std::endl;

            //CGAL::Halfedge_around_face_iterator<mesh_t> hbegin, hend;

            // stores the new intersection-point registry entries for the current intersection test
            std::map<vd_t, hd_t> ivtx_to_he_entry_collector;
            std::map<vd_t, std::vector<fd_t>> ivtx_to_face_entry_collector;

            std::vector<halfedge_descriptor_t> halfedges_around_face_A = ps.get_halfedges_around_face(face_A);
            // for each halfedge of Face A
            //for (boost::tie(hbegin, hend) = halfedges_around_face(ps.halfedge(face_A), ps); hbegin != hend; ++hbegin) {
            for (std::vector<halfedge_descriptor_t>::const_iterator face_A_halfedge_iter = halfedges_around_face_A.cbegin();
                 face_A_halfedge_iter != halfedges_around_face_A.cend();
                 ++face_A_halfedge_iter) {

                lg.indent();
                lg << "test " << hstr(ps, *face_A_halfedge_iter) << " & " << fstr(face_B) << std::endl;

                const bool halfedge_belongs_to_cs = ps_is_cutmesh_face(face_A, sm_face_count); // based on the fact that sm faces come first inside ps
                //Exact_kernel::Segment_3 segment(ps.point(ps.source(*hbegin)), ps.point(ps.target(*hbegin)));

                const vertex_descriptor_t face_A_halfedge_source_descr = ps.source(*face_A_halfedge_iter);
                const vertex_descriptor_t face_A_halfedge_target_descr = ps.target(*face_A_halfedge_iter);
                const math::vec3& face_A_halfedge_source_vertex = ps.vertex(face_A_halfedge_source_descr);
                const math::vec3& face_A_halfedge_target_vertex = ps.vertex(face_A_halfedge_target_descr);
                //auto result = CGAL::intersection(segment, plane);
                math::vec3 intersection_point;
                math::real_number_t tparam = 0.0; // how far along the halfedge the intersection point is.

                bool have_plane_intersection = geom::intersect_plane_with_segment(
                    intersection_point,
                    tparam,
                    // plane
                    face_B_plane_normal,
                    face_B_plane_param_d,
                    // segment
                    face_A_halfedge_source_vertex,
                    face_A_halfedge_target_vertex);

                lg << "plane intersection exists = " << std::boolalpha << (bool)have_plane_intersection << std::endl;

                if (have_plane_intersection) {
                    lg.indent();

                    lg << "intersection point: " << intersection_point << std::endl;
#if 1
                    bool have_point_in_polygon = geom::point_in_polygon(intersection_point, face_B_plane_normal, face_B_vertices.data(), (int)face_B_vertices.size());

                    lg << "point in polygon = " << std::boolalpha << have_point_in_polygon << std::endl;

                    if (have_point_in_polygon) {
                        if (tparam == math::real_number_t(0.0) || tparam == math::real_number_t(1.0)) {
                            // NOTE: if t == 0 or t == 1, then we need to terminate with an error.
                            // This is because 1) our intersection registry formulation requires that
                            // edges completely penetrate/intersect through polygon's area. Also, 2) If any
                            // one of a segment's vertices only touch (i.e. lie on) the polygon (in 3D)
                            // then that implies a situation of cutting an edge along the vertex
                            // which is undefined (topologically).
                            lg.set_reason_for_failure("invalid polygon intersection (vertex lies on face)");
                            output.status = status_t::INVALID_MESH_INTERSECTION;
                            return;
                        }

                        vd_t pre_existing_copy = mesh_t::null_vertex(); // set to correct value if vertex is pre-existing
                        //const vd_t new_copy = add_vertex(intersection_vertex_coords, pre_existing_copy);
                        vd_t new_copy;

                        { //add vertex  if it does not exist.

                            hd_t halfedge_pq = *face_A_halfedge_iter; // the halfedge which is intersected with polygon
                            fd_t face_pqr = face_A; // the face which is incident to halfedge-pq
                            fd_t face_xyz = face_B; // the face which is intersected with halfedge-pq
                            fd_t face_pqs = mesh_t::null_face(); // the face which is incident to the halfedge opposite to halfedge-pq
                            fd_t face_pqX = mesh_t::null_face(); // a virtual face pqX (where X denotes an unspecified auxiliary point)

                            // find face in polygon-soup which is incident to edge containing pq (excl' pqr) i.e. find face_pqs

                            // TODO: the following for-loop can be improved/replaced by simply querying the halfedge mesh "m_ps".
                            // We can do this by:
                            // 1) query the opposite halfedge of "halfedge_pq" in "m_ps".
                            // 2) query the face incident to 1)
                            for (mesh_t::face_iterator_t face_iter = ps.faces_begin(); face_iter != ps.faces_end(); face_iter++) {

                                if (*face_iter == face_pqr) {
                                    continue; // we want other faces
                                }

                                // CGAL::Halfedge_around_face_iterator<mesh_t> hbegin, hend;
                                std::vector<halfedge_descriptor_t> halfedges_around_face = ps.get_halfedges_around_face(*face_iter);
                                //for (boost::tie(hbegin, hend) = halfedges_around_face(m_ps.halfedge(*face_iter), m_ps); hbegin != hend; ++hbegin) {
                                for (std::vector<halfedge_descriptor_t>::const_iterator it = halfedges_around_face.cbegin(); it != halfedges_around_face.cend(); ++it) {
                                    if ((*it) == ps.opposite(halfedge_pq)) {
                                        face_pqs = *face_iter;
                                        break;
                                    }
                                }

                                if (face_pqs != mesh_t::null_face()) {
                                    break;
                                }
                            }

                            const bool pq_is_indicent_on_pqr_and_pqs = (face_pqs != mesh_t::null_face()); // pq is common to faces pqr and pqs
                            std::vector<fd_t> new_vertex_incident_ps_faces; // the list of faces which are incident to our intersection point
                            bool new_intersection_vertex_exists = false; // NOTE: Two intersection vertices are same if they are incident on the same faces

                            if (pq_is_indicent_on_pqr_and_pqs) {

                                new_vertex_incident_ps_faces.push_back(face_pqr);
                                new_vertex_incident_ps_faces.push_back(face_pqs);
                                new_vertex_incident_ps_faces.push_back(face_xyz);

                                // check if vertex is new

                                bool found_opposite_of_halfedge_pq = false;

                                for (std::map<vd_t, std::vector<fd_t>>::const_iterator itr = m0_ivtx_to_ps_faces.cbegin(); itr != m0_ivtx_to_ps_faces.cend(); ++itr) {

                                    // we want to compare against only intersection points whose registries contain 3 real faces.
                                    // This is because if an intersection point has two real faces in its registry then the intersecting halfedge in its registry is on the border.
                                    // When this is the case, there will only ever be one occasion when that intersection point will be computed.
                                    const bool is_three_face_intersection_point = std::find(itr->second.cbegin(), itr->second.cend(), mesh_t::null_face()) == itr->second.cend();

                                    if (!is_three_face_intersection_point) {
                                        continue;
                                    }

                                    //const bool registries_match = intersection_registries_match(new_vertex_incident_ps_faces, i->second); // exact same faces!
                                    bool registries_match = true;

                                    for (std::vector<fd_t>::const_iterator it = new_vertex_incident_ps_faces.cbegin(); it != new_vertex_incident_ps_faces.cend(); ++it) {
                                        registries_match = std::find(itr->second.cbegin(), itr->second.cend(), *it) != itr->second.cend();
                                        if (!registries_match) {
                                            break;
                                        }
                                    }

                                    if (registries_match) {
                                        const hd_t& other_halfedge = m0_ivtx_to_ps_he.at(itr->first);
                                        // We also check if the halfedge in the matched registry is the opposite of "halfedge_pq", which may be false during a concave polygon intersection.
                                        // The reason the result may be false is that some concave polygon intersection points may indeed match by their faces in the registry but
                                        // this distinction is insufficient because its based only on topology (i.e. based on faces).
                                        // When more than two intersection points result from an intersection test between two polygons, the faces in their registries are exactly the same.
                                        // What distinguishes them is the additional halfedge in their registry - hence the check below.
                                        found_opposite_of_halfedge_pq = other_halfedge == ps.opposite(halfedge_pq);
                                        if (found_opposite_of_halfedge_pq) {
                                            pre_existing_copy = itr->first; // the intersection point already exists.
                                            break;
                                        }
                                    }

                                    if (found_opposite_of_halfedge_pq) {
                                        break;
                                    }
                                }

                                new_intersection_vertex_exists = (found_opposite_of_halfedge_pq == true);

                            }

                            else { // pqr is the only face incident to pq
                                new_vertex_incident_ps_faces.push_back(face_pqr);
                                new_vertex_incident_ps_faces.push_back(face_pqX); // virtual face
                                new_vertex_incident_ps_faces.push_back(face_xyz);
                            }

                            vd_t new_vertex = mesh_t::null_vertex();

                            if (new_intersection_vertex_exists == false) {

                                new_vertex = m0.add_vertex(intersection_point);

                                lg << "add vertex" << std::endl;
                                lg.indent();
                                lg << "position = (" << intersection_point << ")" << std::endl;
                                lg << "descriptor = " << vstr(new_vertex) << std::endl;
                                lg << "registry-entry" << std::endl;
                                lg.indent();
                                lg << "faces = [" << fstr(new_vertex_incident_ps_faces.at(0)) << ", " << fstr(new_vertex_incident_ps_faces.at(1)) << ", " << fstr(new_vertex_incident_ps_faces.at(2)) << "]" << std::endl;
                                lg << "halfedge = " << hstr(ps, halfedge_pq) << std::endl;
                                lg.unindent();
                                lg.unindent();

                                ivtx_to_face_entry_collector.insert(std::make_pair(new_vertex, new_vertex_incident_ps_faces));
                                ivtx_to_he_entry_collector.insert(std::make_pair(new_vertex, halfedge_pq));

                                ed_t e = ps.edge(halfedge_pq);
                                bool edge_registered_as_intersecting = std::find(ps_intersecting_edges.cbegin(), ps_intersecting_edges.cend(), e) != ps_intersecting_edges.cend();

                                if (edge_registered_as_intersecting == false) {
                                    ps_intersecting_edges.push_back(e);
                                }

                                test_ivertex_collector.push_back(new_vertex);

                            } else {
                                lg << "pre-existing instance = " << vstr(pre_existing_copy) << std::endl;
                                test_ivertex_collector.push_back(pre_existing_copy);
                            }
                            new_copy = new_vertex;
                        } // add vertex
                        const bool vertex_is_new = new_copy != mesh_t::null_vertex();

                        // NOTE: we are not guarranteed to find all cs-regular re-entrant vertices.
                        // This is because of the order-dependent nature of the halfedge-face intersection tests (combined with the fact that vertex_is_new is not always true).
                        // Also, the winding order will be an arbiter when determing which cs-halfedge edges lead to the identification of a re-entrant vertices.
                        // This is not a problem however, because
                        // 1) For cs-re-entrant vertices (tip or regular) we only need a small subset of the each type in order to a) classify cs-patches
                        // as interior or exterior (regular re-entrant vertex) and b) to find the cs-border polygons which allows us to find the border vetrices that we do
                        // not need to duplicte when there is a partial cut (tip re-entrant vertex).
                        // 2) For sm-re-entrant vertices (regular re-entrant vertex) we only need to know one such vertex (along the border of each cs interior-patch) in order for us
                        // to be able to distinguish the connected components adjacent to each interior-patch as above or below. When the current halfedge is an
                        // sm halfedge, we ignore vertex_is_new in order to capture all sm re-entrant vertices. Note: if we do not ignore vertex_is_new, we are [not] guarranteed to
                        // find atleast one sm-regular re-entrant vertex in the vicinity of each cs interior-patch because if vertex_is_new is not always true it implies that a vertex
                        // may already have been registered but with an sm-halfedge that fails our is_reentrance condition (positive scalar product) because this sm-halfedge points
                        // in the same direction as the cs-face it intersected.).
                        //
                        const bool is_intersecting_halfedge_of_sm = !halfedge_belongs_to_cs; //!m_face_i_he_belongs_to_cs;

                        if (vertex_is_new || is_intersecting_halfedge_of_sm) {

                            const math::real_number_t scalar_prod = math::dot_product(face_B_plane_normal, face_A_halfedge_target_vertex - face_A_halfedge_source_vertex /*m_ihalfedge_segment.to_vector()*/);

                            const bool is_reentrant = (scalar_prod < math::real_number_t(0.0));

                            if (is_reentrant) {
                                const bool intersection_registry_exists = pre_existing_copy != mesh_t::null_vertex(); // vertex already registered in intersection registries

                                lg << "vertex is a re-entrant point (dot = " << scalar_prod << ")" << std::endl;
                                //lg << "intersection-registry exists=" << std::boolalpha << intersection_registry_exists << std::endl;
                                lg << "mesh = " << (is_intersecting_halfedge_of_sm ? "sm" : "cs") << std::endl;

                                if (!is_intersecting_halfedge_of_sm /*&& m_sm_is_watertight*/) // NOTE: 2nd condition pertains to the fact that cs-re-entrant vertices are necessary only if sm is watertight
                                {
                                    const bool is_tip_ivertex = ps.is_border(ps.edge(*face_A_halfedge_iter));

                                    if (is_tip_ivertex) {
                                        lg << "vertex is tip re-entrant point" << std::endl;
                                        cs_tip_reentrant_ivtx_list.push_back(new_copy);
                                    } else // is regular
                                    {
                                        std::pair<std::map<vd_t, std::pair<math::vec3, math::real_number_t>>::const_iterator, bool> m_cs_regular_reentrant_ivertices_insertion = cs_reg_reentrant_ivtx_list.insert(
                                            std::make_pair(new_copy, std::make_pair(face_B_plane_normal, scalar_prod)));
                                        MCUT_ASSERT(m_cs_regular_reentrant_ivertices_insertion.second == true);
                                    }
                                } else { // m_face_i_halfedge belongs to sm (i.e. we have an sm halfedge intersecting an cs face)

                                    vd_t sm_regular_reentrant_vertex = new_copy;

                                    if (intersection_registry_exists) {
                                        MCUT_ASSERT(new_copy == mesh_t::null_vertex());
                                        sm_regular_reentrant_vertex = pre_existing_copy;
                                    }

                                    std::pair<std::map<vd_t, std::pair<math::vec3, math::real_number_t>>::const_iterator, bool> m_sm_regular_reentrant_ivertices_insertion = sm_reg_reentrant_ivtx_list.insert(
                                        std::make_pair(sm_regular_reentrant_vertex, std::make_pair(face_B_plane_normal, scalar_prod)));
                                    MCUT_ASSERT(m_sm_regular_reentrant_ivertices_insertion.second == true); // TODO: may fail (can't remember why!)
                                }
                            }
                        }
                    } // if (have_point_in_polygon)
#else
                    boost::apply_visitor(
                        intersection_visitor_t(
                            m0_ivtx_to_ps_faces,
                            m0_ivtx_to_ps_he,
                            m0,
                            *hbegin,
                            face_A,
                            face_B,
                            ps,
                            face_B_descriptors,
                            segment,
                            cs_tip_reentrant_ivtx_list,
                            cs_reg_reentrant_ivtx_list,
                            halfedge_belongs_to_cs,
                            sm_reg_reentrant_ivtx_list,
                            sm_is_watertight,
                            ivtx_to_he_entry_collector,
                            ivtx_to_face_entry_collector,
                            test_ivertex_collector,
                            ps_intersecting_edges),
                        *result);
#endif
                    lg.unindent();
                } // if (have_plane_intersection) {

                lg.unindent();
            }

            m0_ivtx_to_ps_faces.insert(ivtx_to_face_entry_collector.cbegin(), ivtx_to_face_entry_collector.cend());
            m0_ivtx_to_ps_he.insert(ivtx_to_he_entry_collector.cbegin(), ivtx_to_he_entry_collector.cend());

            lg.unindent();
        } // for (std::vector<std::pair<fd_t, fd_t> >::const_iterator j = test_permutations.cbegin(); j != test_permutations.cend(); ++j) {

        const int new_ivertices_count = (int)test_ivertex_collector.size();

        lg << "create edge(s) from " << new_ivertices_count << " intersection points" << std::endl;

        if (new_ivertices_count == 2) {
            vd_t first_new_ivertex = test_ivertex_collector.front();
            vd_t second_new_ivertex = test_ivertex_collector.back();

            MCUT_ASSERT(m0_is_intersection_point(first_new_ivertex, ps_vtx_cnt));
            MCUT_ASSERT(m0_is_intersection_point(second_new_ivertex, ps_vtx_cnt));

            if (!interior_edge_exists(m0, first_new_ivertex, second_new_ivertex, m0_cutpath_edges)) {
                lg << "add edge (xx) : " << estr(first_new_ivertex, second_new_ivertex) << std::endl;

                hd_t h = m0.add_edge(first_new_ivertex, second_new_ivertex);
                MCUT_ASSERT(h != mesh_t::null_halfedge());

                m0_cutpath_edges.emplace_back(m0.edge(h));

                // all newly created edges will lie on both face A and B since intersection points lie on a line which is the intersection of the two planes of face A and B
                ps_iface_to_m0_edge_list[sm_face].emplace_back(m0_cutpath_edges.back());
                ps_iface_to_m0_edge_list[cs_face].emplace_back(m0_cutpath_edges.back());

                update_neighouring_ps_iface_m0_edge_list(first_new_ivertex, second_new_ivertex, ps,
                    sm_face,
                    cs_face,
                    m0_ivtx_to_ps_faces,
                    ps_iface_to_m0_edge_list,
                    m0_cutpath_edges);

                MCUT_ASSERT(m0.target(h) == second_new_ivertex);
                ivtx_to_incoming_hlist[second_new_ivertex].push_back(h);

                MCUT_ASSERT(m0.target(m0.opposite(h)) == first_new_ivertex);
                ivtx_to_incoming_hlist[first_new_ivertex].push_back(m0.opposite(h));
            }

        } else if (new_ivertices_count > 2) {
            MCUT_ASSERT(new_ivertices_count >= 4); // concave polygon intersection produce a minimum of 4 intersection point if not 2

            std::vector<std::pair<vd_t, math::vec3>> ivertex_coords;

            for (int v = 0; v < new_ivertices_count; ++v) {
                vd_t new_ivertex_descr = test_ivertex_collector.at(v);
                MCUT_ASSERT(m0_is_intersection_point(new_ivertex_descr, ps_vtx_cnt));
                ivertex_coords.emplace_back(new_ivertex_descr, m0.vertex(new_ivertex_descr));
            }

            // since all points are on straight line, we sort them by x-coord and by y-coord if x-coord is the same for all vertices
            std::sort(ivertex_coords.begin(), ivertex_coords.end(),
                [&](const std::pair<vd_t, math::vec3>& a,
                    const std::pair<vd_t, math::vec3>& b) {
                    {
                        return (a.second.x() < b.second.x());
                    }
                });

            const bool x_coordinate_is_same = have_same_coordinate(ivertex_coords, 0);

            if (x_coordinate_is_same) {
                // ... then  sort on y-coord
                std::sort(ivertex_coords.begin(), ivertex_coords.end(),
                    [&](const std::pair<vd_t, math::vec3>& a,
                        const std::pair<vd_t, math::vec3>& b) {
                        {
                            return (a.second.y() < b.second.y());
                        }
                    });
            }

            //int counter = 0;
            for (std::vector<std::pair<vd_t, math::vec3>>::const_iterator iter = ivertex_coords.cbegin() + 1; iter != ivertex_coords.cend(); ++iter) {

                const vd_t src_vertex = (iter - 1)->first;
                const vd_t tgt_vertex = (iter)->first;

                if (!interior_edge_exists(m0, src_vertex, tgt_vertex, m0_cutpath_edges)) {

                    lg << "add edge (xx) : " << estr(src_vertex, tgt_vertex) << "(from concave intersection)" << std::endl;

                    const hd_t h = m0.add_edge(src_vertex, tgt_vertex); // insert segment!

                    MCUT_ASSERT(h != mesh_t::null_halfedge());
                    m0_cutpath_edges.emplace_back(m0.edge(h));

                    // NOTE: here we add all edge without assuming anything about which of the will be used to clip either polygon
                    ps_iface_to_m0_edge_list[sm_face].emplace_back(m0_cutpath_edges.back());
                    ps_iface_to_m0_edge_list[cs_face].emplace_back(m0_cutpath_edges.back());

                    update_neighouring_ps_iface_m0_edge_list(src_vertex, tgt_vertex, ps,
                        sm_face,
                        cs_face,
                        m0_ivtx_to_ps_faces,
                        ps_iface_to_m0_edge_list,
                        m0_cutpath_edges);

                    MCUT_ASSERT(m0.target(h) == tgt_vertex);
                    ivtx_to_incoming_hlist[tgt_vertex].push_back(h);

                    MCUT_ASSERT(m0.target(m0.opposite(h)) == src_vertex);
                    ivtx_to_incoming_hlist[src_vertex].push_back(m0.opposite(h));
                }
            }
        } else {
            if (new_ivertices_count != 0) // edge-case scenario: an edge intersects with another edge exactly
            {
                lg.set_reason_for_failure("unresolvable edge-case: (vertex-face OR edge-edge intersection)");
                output.status = status_t::INVALID_MESH_INTERSECTION;
                return;
            }
        } // else if (new_ivertices_count > 2) {
        lg.unindent();
    } // for (std::vector<std::pair<fd_t, fd_t> >::const_iterator i = intersecting_sm_cs_face_pairs.cbegin(); i != intersecting_sm_cs_face_pairs.cend(); ++i) {

    intersecting_sm_cs_face_pairs.clear(); // free

    if (m0_ivtx_to_ps_faces.empty()) {
        lg.set_reason_for_failure("polygons do not intersect");
        output.status = status_t::INVALID_MESH_INTERSECTION;
        return;
    }

    lg << "total intersection-points = " << m0_ivtx_to_ps_faces.size() << std::endl;

    // A partial cut intersection exists when there exists at-least one intersection point
    // whose registry has a halfedge from the cut-surface, where this halfedge is a border halfedge.
    bool partial_cut_detected = false;

    for (std::map<vd_t, std::vector<fd_t>>::const_iterator entry_it = m0_ivtx_to_ps_faces.cbegin(); entry_it != m0_ivtx_to_ps_faces.cend(); ++entry_it) {

        const vd_t& ipoint_descr = entry_it->first;
        const hd_t& ipoint_ihalfedge = m0_ivtx_to_ps_he.at(ipoint_descr);
        const vd_t tgt = ps.target(ipoint_ihalfedge);
        const bool is_cs_halfedge = ps_is_cutmesh_vertex(tgt, sm_vtx_cnt);

        if (is_cs_halfedge) {

            const bool is_border = ps.is_border(ps.opposite(ipoint_ihalfedge));

            if (is_border) {
                partial_cut_detected = true;
            }
        }
    }

    lg << "partial cut = " << std::boolalpha << partial_cut_detected << std::endl;

    if (partial_cut_detected && cs_tip_reentrant_ivtx_list.size() == 0) {
        // can happen with case when both the input mesh and cut surface are not watertight
        lg << "note: did not find cut-surface tip re-entrant vertices." << std::endl;
    }
    if(input.verbose){
    dump_mesh(m0, "m0.v"); // containing only vertices (polygon soup vertices and intersection points)
    }
    if (partial_cut_detected) {

        MCUT_ASSERT(!cs_is_watertight);
        MCUT_ASSERT(!m0_ivtx_to_ps_faces.empty());
        MCUT_ASSERT(!m0_ivtx_to_ps_he.empty());

        if (input.require_looped_cutpaths) {
            output.status = status_t::SUCCESS;
            return;
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // Check for degenerate mesh intersections
    ///////////////////////////////////////////////////////////////////////////

    lg << "detect degeneracies" << std::endl;

    mesh_t::vertex_iterator_t m0_ivtx_iter_begin = m0.vertices_begin();
    std::advance(m0_ivtx_iter_begin, ps_vtx_cnt); // offset to start of intersection vertices in mesh_t (note: internal mesh data stored consecutively)

    //
    // check if at-least one input mesh edge intersects any face of the cs.
    //

    // TODO: stab test
    /*for each intersection cs face
        for each halfedge of face        
            if halfedge intersects an im face and halfedge is a border halfedge
                1) find all other border halfedges of faces which also intersect im face
                if (1) > 0
                    check to ensure that at least one halfedge of the im face intersects the cs face 
    */

    // This check prevents malformed configurations where a cs face might stab a face of
    // the im face while correctly intersecting another im face on the "other side".
    // (Think of a wedge-like triangle stabbing a tet face while intersecting [two] other faces by cutting an edge in the tet)
    bool atleast_one_sm_edge_intersects_an_cs_face = false;

    for (mesh_t::vertex_iterator_t i = m0_ivtx_iter_begin; i != m0.vertices_end(); ++i) {

        const hd_t& ps_halfedge = m0_ivtx_to_ps_he.at(*i);
        const vd_t ps_halfedge_tgt = ps.target(ps_halfedge);
        const bool is_sm_halfedge = !ps_is_cutmesh_vertex(ps_halfedge_tgt, sm_vtx_cnt);

        if (is_sm_halfedge) {
            atleast_one_sm_edge_intersects_an_cs_face = true;
            break;
        }
    }

    if (!atleast_one_sm_edge_intersects_an_cs_face) {
        // NOTE: the sm must intersect at least one face of the cs to allow for an opening on the sm boundary.
        lg.set_reason_for_failure("no edge in the input mesh intersects a cut surface polygon.");
        output.status = status_t::INVALID_MESH_INTERSECTION;
        return;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Create new edges partitioning intersecting ps edges (2-part process)
    ///////////////////////////////////////////////////////////////////////////

    lg << "create polygon-exterior edges" << std::endl;
    lg.indent();
    // Part 1
    //
    // Here, we identify ps-edges with more than 3 coincident m0-vertices (ps-
    // and intersection points)
    //
    // Task: 1) find ps-edges with more than 3 coincident m0-vertices, 2)
    // sort these vertices along the ps-edge 3) connect sorted point by
    // creating edges in "m0"
    //
    // Brief: ps-edges with more than 3 coincident vertices arise during a
    // partial-cut (3d polyhedron) and/or concave cs-sm polygon intersection.
    // For each such edges, there will be 2 ps-vertices and the rest are
    // intersection points. (Sorting requires numerical calculation).
    //

    // We also create a mapping between each polygon-exterior interior-edge
    // vertex and its multiple copies which will be used for connected component
    // separation and sealing (hole filling).
    // NOTE: an exterior edge is one which is lies on the boundary of a ps-polygon.
    // Conversely, an interior edge lies within the polygon (path along which
    // polygon is clipped).

    //std::map<vd_t, std::vector<vd_t>> m0_to_m1_poly_ext_int_edge_vertex;

    std::map<ed_t, std::vector<std::pair<vd_t, math::vec3>>> ps_edge_to_vertices_sorted; // stores ps-edges with more-than 3 coincident vertices

    for (std::vector<ed_t>::const_iterator iter_ps_edge = ps_intersecting_edges.cbegin(); iter_ps_edge != ps_intersecting_edges.cend(); ++iter_ps_edge) {
        lg.indent();

        const std::vector<vd_t> vertices_on_ps_edge = get_vertices_on_ps_edge(*iter_ps_edge, m0_ivtx_to_ps_he, ps, m0_to_ps_vtx);

        if (vertices_on_ps_edge.size() > 3) {

            lg << "ps-edge " << estr(ps, *iter_ps_edge) << " : ";

            MCUT_ASSERT(ps_edge_to_vertices_sorted.find(*iter_ps_edge) == ps_edge_to_vertices_sorted.end()); // edge cannot have been traversed before!

            ps_edge_to_vertices_sorted.insert(std::make_pair(*iter_ps_edge, std::vector<std::pair<vd_t, math::vec3>>()));

            for (std::vector<vd_t>::const_iterator it = vertices_on_ps_edge.cbegin(); it != vertices_on_ps_edge.cend(); ++it) {

                lg << vstr(*it) << " ";

                const math::vec3& vertex_coordinates = m0.vertex(*it); // get the coordinates (for sorting)
                ps_edge_to_vertices_sorted.at(*iter_ps_edge).push_back(std::make_pair(*it, vertex_coordinates));

                if (m0_is_intersection_point(*it, ps_vtx_cnt)) { // is intersection point
                    //m0_to_m1_poly_ext_int_edge_vertex.insert(std::make_pair(*it, std::vector<vd_t>()));
                }
            }
            lg << std::endl;
        }

        lg.unindent();
    }

    ps_intersecting_edges.clear();

    lg << "ps-edges with > 3 coincident vertices = " << ps_edge_to_vertices_sorted.size() << std::endl;

    // In the next for-loop, we sort each list of vertices on each ps-edge
    // which more than 3 coincident vertices

    for (std::map<ed_t, std::vector<std::pair<vd_t, math::vec3>>>::iterator edge_vertices_iter = ps_edge_to_vertices_sorted.begin(); edge_vertices_iter != ps_edge_to_vertices_sorted.end(); ++edge_vertices_iter) {
        std::vector<std::pair<vd_t, math::vec3>>& incident_vertices = edge_vertices_iter->second;

        // since all points are on straight line, we sort them by x-coord and by y-coord if x-coord is the same for all vertices
        std::sort(incident_vertices.begin(), incident_vertices.end(),
            [&](const std::pair<vd_t, math::vec3>& a, const std::pair<vd_t, math::vec3>& b) {
                return (a.second.x() < b.second.x());
            });

        const bool x_coordinate_is_same = have_same_coordinate(incident_vertices, 0);

        if (x_coordinate_is_same) {
            // ... then  sort on y-coord
            std::sort(incident_vertices.begin(), incident_vertices.end(),
                [&](const std::pair<vd_t, math::vec3>& a, const std::pair<vd_t, math::vec3>& b) {
                    return (a.second.y() < b.second.y());
                });

            const bool y_coordinate_is_same = have_same_coordinate(incident_vertices, 1);

            if (y_coordinate_is_same) {
                // ... then  sort on z-coord
                std::sort(incident_vertices.begin(), incident_vertices.end(),
                    [&](const std::pair<vd_t, math::vec3>& a, const std::pair<vd_t, math::vec3>& b) {
                        return (a.second.z() < b.second.z());
                    });
            }
        }
    }

    //
    // Now we, create edges between the sorted vertices that are coincident
    // on the same ps-edge that has more-than 3 incident vertices.
    //
    // This step will create class-1 (o==>x), class-2 (o==>x),
    // and class-3 (x==>x) which are the so called "polygon-exterior
    // interior-iedges".

    std::map<ed_t, std::vector<ed_t>> ps_to_m0_edges; // key=ps-edge;value=list of m0-edges which lay on ps-edge

    // for each ps-edge with more than 3 coicindent vertices
    for (std::map<ed_t, std::vector<std::pair<vd_t, math::vec3>>>::const_iterator ps_edge_coincident_vertices_iter = ps_edge_to_vertices_sorted.begin();
         ps_edge_coincident_vertices_iter != ps_edge_to_vertices_sorted.end();
         ++ps_edge_coincident_vertices_iter) {

        lg.indent();

        // get sorted list of vertices on edge
        const std::vector<std::pair<vd_t, math::vec3>>& coincident_sorted_vertices = ps_edge_coincident_vertices_iter->second;

        MCUT_ASSERT(coincident_sorted_vertices.size() > 3); // we are only dealing with ps-edges with more than 3 coicindent vertices

        // first vertex must not be an intersection point, because all vertices lie on a
        // ps-edge to be partitioned into new edges, thus the first vertex must not
        // be an intersection point: [*]===========[*] --> [*]===*==*======[*]
        MCUT_ASSERT(!m0_is_intersection_point(coincident_sorted_vertices.front().first, ps_vtx_cnt));
        MCUT_ASSERT(m0_is_intersection_point((*(coincident_sorted_vertices.cbegin() + 1)).first, ps_vtx_cnt));

        MCUT_ASSERT(m0_is_intersection_point((*(coincident_sorted_vertices.cend() - 2)).first, ps_vtx_cnt));
        MCUT_ASSERT(!m0_is_intersection_point(coincident_sorted_vertices.back().first, ps_vtx_cnt)); // likewise, last vertex must not be an intersection point

        // for each sorted vertex on ps-edge (starting from the second in the list)
        for (std::vector<std::pair<vd_t, math::vec3>>::const_iterator iter = coincident_sorted_vertices.cbegin() + 1; iter != coincident_sorted_vertices.cend(); ++iter) {

            const vd_t src_vertex = (iter - 1)->first;
            const vd_t tgt_vertex = (iter)->first;
            const hd_t h = m0.add_edge(src_vertex, tgt_vertex); // create edge!

            lg << "add edge : " << estr(src_vertex, tgt_vertex) << std::endl;

            MCUT_ASSERT(h != mesh_t::null_halfedge());

            const ed_t new_edge = m0.edge(h);
            MCUT_ASSERT(new_edge != mesh_t::null_edge());

            // map original ps-edge to list of "child" edges which lie on it
            ps_to_m0_edges[ps_edge_coincident_vertices_iter->first].push_back(new_edge);

            // Here we save the "incoming" halfedge for each vertex of the created edge,
            // if the vertex is an intersection point. An incoming halfedge is a one
            // whose target is the vertex.
            // We will using this information when splitting the input mesh along the
            // cut surface (when duplicating intersection points to create holes).

            if ((iter - 1) == coincident_sorted_vertices.cbegin()) // first iteration
            {
                MCUT_ASSERT(m0.target(h) == tgt_vertex);
                ivtx_to_incoming_hlist[tgt_vertex].push_back(h);
            } else if ((std::size_t)std::distance(coincident_sorted_vertices.cbegin(), iter) == coincident_sorted_vertices.size() - 1) // last iterator
            {
                MCUT_ASSERT(m0.target(m0.opposite(h)) == src_vertex);
                ivtx_to_incoming_hlist[src_vertex].push_back(m0.opposite(h));
            } else {
                MCUT_ASSERT(m0.target(h) == tgt_vertex);
                ivtx_to_incoming_hlist[tgt_vertex].push_back(h);

                MCUT_ASSERT(m0.target(m0.opposite(h)) == src_vertex);
                ivtx_to_incoming_hlist[src_vertex].push_back(m0.opposite(h));
            }

            // Here, we also associate the new edge with an intersecting ps-face.
            // Note: since the new edge here will lie on the polygon exterior, its associated intersecting ps-face(s) will
            // be those which are incident to the parent ps-edge
            const ed_t ps_edge = ps_edge_coincident_vertices_iter->first;

            for (int i = 0; i < 2; ++i) {
                const hd_t ps_edge_h = ps.halfedge(ps_edge, i);

                if (ps_edge_h != mesh_t::null_halfedge()) {
                    const fd_t f = ps.face(ps_edge_h);
                    if (f != mesh_t::null_face()) // ps_edge could be on the border!
                    {
                        ps_iface_to_m0_edge_list[f].emplace_back(new_edge);
                    }
                }
            }
        }

        lg.unindent();
    }

    // Part 2
    //
    // We will now create edges between vertices that lie on the same ps-edge
    // which has 2 or 3 coincident vertices. Note that in the case of 2 coincident
    // vertices, the created edge is the same as the original ps-edge.
    //
    // Brief: the order of m0-vertices along the ps-edge is inferred since the number
    // of vertices is small enough (unlike Part 1).
    // So we have two simple cases:
    // a) ps-edge is coincident on two m0-vertices which are not intersection points
    // b) ps-edge is coincident on three m0-vertices such that one is an intersection point
    //

    lg << "create edges on ps-edges with 2 (oo) or 3 vertices (ox or xo)" << std::endl;

    // a map between edge ids in "ps" and in "m0", which is the data structure we are progressively
    // defining to hold data for the new mesh containing clipped polygons
    std::map<ed_t, ed_t> ps_to_m0_non_intersecting_edge;

    // for each ps-edge
    for (mesh_t::edge_iterator_t iter_ps_edge = ps.edges_begin(); iter_ps_edge != ps.edges_end(); ++iter_ps_edge) {
        lg.indent();

        if (ps_edge_to_vertices_sorted.find(*iter_ps_edge) != ps_edge_to_vertices_sorted.end()) {
            continue; // the case of more than 3 vertices (handled in Step 6.2 and Step 6.3)
        }

        const ed_t ps_edge = *iter_ps_edge; // edge handle

        std::vector<vd_t> vertices_on_ps_edge = get_vertices_on_ps_edge(*iter_ps_edge, m0_ivtx_to_ps_he, ps, m0_to_ps_vtx);

        if (vertices_on_ps_edge.size() == 2) // simple case (edge did not intersect with any polygon)
        {
            lg << "add edge (oo) : (" << vertices_on_ps_edge.back() << ", " << vertices_on_ps_edge.front() << ")" << std::endl;

            const hd_t h = m0.add_edge(vertices_on_ps_edge.back(), vertices_on_ps_edge.front());

            MCUT_ASSERT(h != mesh_t::null_halfedge());

            const ed_t edge = m0.edge(h);
            ps_to_m0_non_intersecting_edge[ps_edge] = edge; // associate

            // similar to Part 1, we also associate the new edge with an intersecting ps-face.
            for (int i = 0; i < 2; ++i) {
                const hd_t ps_edge_h = ps.halfedge(ps_edge, i);
                if (ps_edge_h != mesh_t::null_halfedge()) { // note: ps_edge could be on the border!
                    const fd_t f = ps.face(ps_edge_h);
                    bool is_iface = f != mesh_t::null_face() && ps_iface_to_m0_edge_list.find(f) != ps_iface_to_m0_edge_list.cend();
                    if (is_iface) {
                        ps_iface_to_m0_edge_list[f].emplace_back(edge);
                    }
                }
            }
        } else { // this is the more complex case where we add minimal set of non overlapping edges between 3 vertices

            MCUT_ASSERT(vertices_on_ps_edge.size() == 3);

            const vd_t first = vertices_on_ps_edge.front();
            const vd_t second = *(vertices_on_ps_edge.begin() + 1);
            const vd_t third = vertices_on_ps_edge.back();

            hd_t h0;
            hd_t h1;

            if (!m0_is_intersection_point(first, ps.number_of_vertices())) { // o-->...
                if (m0_is_intersection_point(second, ps.number_of_vertices())) {
                    //
                    // o x o
                    //
                    lg << "add edge (ox) : " << estr(first, second) << std::endl;
                    h0 = m0.add_edge(first, second);
                    MCUT_ASSERT(h0 != mesh_t::null_halfedge());

                    MCUT_ASSERT(m0.target(h0) == second);
                    ivtx_to_incoming_hlist[second].push_back(h0);

                    lg << "add edge (xo) : " << estr(second, third) << std::endl;
                    h1 = m0.add_edge(second, third);
                    MCUT_ASSERT(h1 != mesh_t::null_halfedge());

                    MCUT_ASSERT(m0.target(m0.opposite(h1)) == second);
                    ivtx_to_incoming_hlist[second].push_back(m0.opposite(h1));
                } else {
                    //
                    //  o o x
                    //
                    lg << "add edge (ox) : " << estr(first, third) << std::endl;
                    h0 = m0.add_edge(first, third);
                    MCUT_ASSERT(h0 != mesh_t::null_halfedge());
                    ivtx_to_incoming_hlist[third].push_back(h0);

                    lg << "add edge (xo) : " << estr(third, second) << std::endl;
                    h1 = m0.add_edge(third, second);
                    MCUT_ASSERT(h1 != mesh_t::null_halfedge());
                    ivtx_to_incoming_hlist[third].push_back(m0.opposite(h1));
                }
            } else {
                //
                // x o o
                //
                lg << "add edge (ox) : " << estr(second, first) << std::endl;
                h0 = m0.add_edge(second, first); // o-->x
                MCUT_ASSERT(h0 != mesh_t::null_halfedge());
                ivtx_to_incoming_hlist[first].push_back(h0);

                MCUT_ASSERT(m0.target(m0.opposite(h0)) == second);

                lg << "add edge (xo) : " << estr(first, third) << std::endl;
                h1 = m0.add_edge(first, third); // x-->o
                MCUT_ASSERT(h1 != mesh_t::null_halfedge());

                MCUT_ASSERT(m0.target(m0.opposite(h1)) == first);
                ivtx_to_incoming_hlist[first].push_back(m0.opposite(h1));
            }

            // // associate the new edge with an intersecting ps-face
            for (int i = 0; i < 2; ++i) {
                const hd_t ps_edge_h = ps.halfedge(ps_edge, i);

                if (ps_edge_h != mesh_t::null_halfedge()) {
                    const fd_t f = ps.face(ps_edge_h);
                    if (f != mesh_t::null_face()) // ps_edge could be on the border!
                    {
                        ps_iface_to_m0_edge_list[f].emplace_back(m0.edge(h0));
                        ps_iface_to_m0_edge_list[f].emplace_back(m0.edge(h1));
                    }
                }
            }
        }
        lg.unindent();
    }
    lg.unindent();

    ps_edge_to_vertices_sorted.clear(); //free

    // NOTE: at this stage we have all vertices ad edges which are needed to clip
    // intersecting polygon in the polygon-soup ("ps").
if(input.verbose){
    dump_mesh(m0, "m0.v.e"); // containing only vertices & edges
}
    ///////////////////////////////////////////////////////////////////////////
    // Find cut-paths (src-mesh openings/holes)
    ///////////////////////////////////////////////////////////////////////////

    lg << "find cut-paths" << std::endl;

    lg.indent();

    // We are now going to search for all the cut-paths created from the intersection
    // between src- and cut-mesh faces. Some of these cut-paths identify holes to be
    // filled while others indentify separation/slitting of the src-mesh.

    // We start off by creating "bins" : each bin corresonds to an intersection point
    // and the values/elements in that bin are the [cut-path edges] connected to it.

    std::map<vd_t, std::vector<ed_t>> m0_ivtx_to_cutpath_edges;

    for (std::vector<ed_t>::const_iterator cutpath_edge_iter = m0_cutpath_edges.cbegin();
         cutpath_edge_iter != m0_cutpath_edges.cend();
         ++cutpath_edge_iter) {
        const ed_t& edge = *cutpath_edge_iter;
        const vd_t vertex0 = m0.vertex(edge, 0);
        const vd_t vertex1 = m0.vertex(edge, 1);
        m0_ivtx_to_cutpath_edges[vertex0].push_back(edge);
        m0_ivtx_to_cutpath_edges[vertex1].push_back(edge);
    }

    //... every intersection point is connected to at least onecut-path edge
    MCUT_ASSERT(m0_ivtx_to_cutpath_edges.empty() == false);

    // build implicit cut-path sequences (a sorted set of connected edges)
    // -----------------------------------------------------------------------

    lg << "build implicit cut-path sequences" << std::endl;

    // An "implicit" cut-path sequence is a list of cut-path edges that are sorted (i.e.
    // in memory, edges are placed next to others which share a vertex).
    //
    // The notion is "implicitness" here means that the sequence may include cut-path edges
    // which are on (but pass outside) a border face of the src-mesh. Thus an implicit
    // cut-path may contain either one explicit cut-path (itself) or more explicit cut-paths.
    //
    // DETAIL
    // The instance of having an implicit cut-path mapping/containing to one explicit
    // (actual/real) cut-path is the normal case. This is because most polygon intersections
    // tend produce at-most, 2 intersection points. However, when a polygon intersection
    // produced more that 2 intersection points (min=4), an implicit cut-path will maps to
    // at least two explicit cut-paths.
    //

    // contains implicit cut-path sequences. "disjoint" because some cut-paths (namely, those
    // which are "linear") may be discovered in two parts due to how we search/build for implicit
    // cut-paths from "m0_ivtx_to_cutpath_edges".
    // The truly disjoint sequences will have spliced/joined at a later stage (see below).
    std::vector<std::vector<ed_t>> m0_implicit_cutpath_sequences;
    // MapKey=intersection point
    // MapValue=index of containing disjoint implicit cut-path in "m0_implicit_cutpath_sequences"
    std::map<vd_t, int> m0_ivtx_to_disjoint_implicit_cutpath_sequence;
    // MapKey=cut-path edge
    // MapValue=index of containing disjoint implicit cut-path in "m0_implicit_cutpath_sequences"
    std::map<ed_t, int> m0_edge_to_disjoint_implicit_cutpath_sequence;

    // Here, we now build all the disjoint implicit cut-path sequences.

    // NOTE: some discovered implicit cut-path sequences will be complete, meaning that
    // they don't require splicing later

    do { // an iteration will build a disjoint implicit cut-path sequence

        const int diff = (int)m0_ivtx_to_cutpath_edges.size() - (int)m0_ivtx_to_disjoint_implicit_cutpath_sequence.size();
        MCUT_ASSERT(diff >= 2); // need a minimum of 2 intersection points (one edge) to form a sequence
        lg.indent();
        int current_disjoint_implicit_cutpath_sequence_index = (int)m0_implicit_cutpath_sequences.size();

        lg << "current disjoint implicit cut-path sequence = " << current_disjoint_implicit_cutpath_sequence_index << std::endl;
        lg.indent();
        // start from an intersection point that is not yet mapped-to/associated-with a
        // disjoint implicit cut-path sequence in "current_disjoint_implicit_cutpath_sequence"
        // pick the vertex which is a terminal vertex (to begin search from beginning of sequence)
        // or any vertex (if there are not termal vertices)

        // if find any vertex which is not associated with a cut-path and is connected to one edge
        std::map<vd_t, std::vector<ed_t>>::const_iterator m0_ivtx_to_cutpath_edges_iter = std::find_if(
            m0_ivtx_to_cutpath_edges.cbegin(), m0_ivtx_to_cutpath_edges.cend(),
            [&](const std::pair<vd_t, std::vector<ed_t>>& elem) {
                bool is_mapped = m0_ivtx_to_disjoint_implicit_cutpath_sequence.find(elem.first) != m0_ivtx_to_disjoint_implicit_cutpath_sequence.cend();
                return !is_mapped && elem.second.size() == 1;
            });

        // if find any vertex which is not associated with a cut-path
        if (m0_ivtx_to_cutpath_edges_iter == m0_ivtx_to_cutpath_edges.cend()) {
            //m0_ivtx_to_cutpath_edges_iter = m0_ivtx_to_cutpath_edges.cbegin();
            m0_ivtx_to_cutpath_edges_iter = std::find_if(
                m0_ivtx_to_cutpath_edges.cbegin(), m0_ivtx_to_cutpath_edges.cend(),
                [&](const std::pair<vd_t, std::vector<ed_t>>& elem) {
                    bool is_mapped = m0_ivtx_to_disjoint_implicit_cutpath_sequence.find(elem.first) != m0_ivtx_to_disjoint_implicit_cutpath_sequence.cend();
                    return !is_mapped;
                });
        }

        if (m0_ivtx_to_cutpath_edges_iter == m0_ivtx_to_cutpath_edges.cend()) {
            break; // done
        }

        // start new sequence
        m0_implicit_cutpath_sequences.emplace_back(std::vector<ed_t>());
        std::vector<ed_t>& current_disjoint_implicit_cutpath_sequence = m0_implicit_cutpath_sequences.back();

        const vd_t& first_vertex_of_sequence = m0_ivtx_to_cutpath_edges_iter->first;
        // get the edges connected to our first intersection point
        const std::vector<ed_t>& cutpath_edges_connected_to_first_vertex = m0_ivtx_to_cutpath_edges_iter->second;
        // pick the edge that is not yet mapped-to/associated-with a disjoint implicit cut-path
        // sequence in "current_disjoint_implicit_cutpath_sequence". Note: if the current sequence
        // is really a disjoint sequence (i.e. linear), then there is the possibility that one of the edges
        // in "cutpath_edges_connected_to_first_vertex" has already been mapped-to/associated-with
        // a disjoint implicit cut-path sequence in "current_disjoint_implicit_cutpath_sequence".
        // This is because sequence discovery can start from any edge, and so we want to pick the
        // we are certain has not been mapped.
        std::vector<ed_t>::const_iterator incident_edge_find_iter = std::find_if(
            cutpath_edges_connected_to_first_vertex.cbegin(),
            cutpath_edges_connected_to_first_vertex.cend(),
            [&](const ed_t& incident_edge) {
                return m0_edge_to_disjoint_implicit_cutpath_sequence.find(incident_edge) == m0_edge_to_disjoint_implicit_cutpath_sequence.cend();
            });

        const ed_t& first_edge = *incident_edge_find_iter;

        // now we will iterative add edges into the current sequence, starting from "first_edge".
        // the next added edge is alway one which share's the "next_vertex" with the current.

        vd_t current_vertex = mesh_t::null_vertex();
        ed_t current_edge = mesh_t::null_edge();
        vd_t next_vertex = first_vertex_of_sequence; //m0_ivtx_to_cutpath_edges_iter->first; // ... initial intersection point
        ed_t next_edge = first_edge;

        do { // an iteration will add an edge to the current disjoint implicit cut-path sequence
            lg.indent();

            // update state
            current_vertex = next_vertex;
            current_edge = next_edge;

            lg << "current vertex = " << current_vertex << std::endl;
            lg << "current edge = " << current_edge << std::endl;

            // add edge
            current_disjoint_implicit_cutpath_sequence.emplace_back(current_edge);

            // map vertex to current disjoint implicit cut-path sequence
            std::pair<std::map<vd_t, int>::const_iterator, bool> ret0 = m0_ivtx_to_disjoint_implicit_cutpath_sequence.emplace(current_vertex, current_disjoint_implicit_cutpath_sequence_index);
            MCUT_ASSERT(ret0.second == true);

            // map edge to current disjoint implicit cut-path sequence
            std::pair<std::map<ed_t, int>::const_iterator, bool> ret1 = m0_edge_to_disjoint_implicit_cutpath_sequence.emplace(current_edge, current_disjoint_implicit_cutpath_sequence_index);
            MCUT_ASSERT(ret1.second == true);

            // reset state
            next_vertex = mesh_t::null_vertex();
            next_edge = mesh_t::null_edge();

            // resolve next vertex (..since we don't know whether vertex0 or vertex0 is "current_vertex")
            const vd_t current_edge_vertex0 = m0.vertex(current_edge, 0);
            const vd_t current_edge_vertex1 = m0.vertex(current_edge, 1);

            // "next_vertex" is whichever vertex of the current edge that is not
            // equal to the "current_vertex"
            if (current_vertex == current_edge_vertex0) {
                next_vertex = current_edge_vertex1;
            } else {
                next_vertex = current_edge_vertex0;
            }

            // now that we have the next vertex, we can determin the next edge

            // check if next vertex has already been associated with the disjoint
            // implicit cut-path sequence.
            bool reached_end_of_sequence = m0_ivtx_to_disjoint_implicit_cutpath_sequence.find(next_vertex) != m0_ivtx_to_disjoint_implicit_cutpath_sequence.cend();

            if (!reached_end_of_sequence) {
                // get the other edge connected to "next_vertex" i.e. the edge which is not
                // the "current_edge"
                m0_ivtx_to_cutpath_edges_iter = m0_ivtx_to_cutpath_edges.find(next_vertex);
                MCUT_ASSERT(m0_ivtx_to_cutpath_edges_iter != m0_ivtx_to_cutpath_edges.cend());

                const std::vector<ed_t>& cutpath_edges_connected_to_next_vertex = m0_ivtx_to_cutpath_edges_iter->second;
                MCUT_ASSERT(cutpath_edges_connected_to_next_vertex.size() <= 2);

                bool current_edge_is_terminal = (cutpath_edges_connected_to_next_vertex.size() == 1);

                if (current_edge_is_terminal == false) {
                    const ed_t& edge0 = cutpath_edges_connected_to_next_vertex.front();
                    const ed_t& edge1 = cutpath_edges_connected_to_next_vertex.back();
                    const ed_t& other_edge = (current_edge == edge0) ? edge1 : edge0;

                    // check that "other_edge" has not already been mapped to a disjoint implicit cutpath sequence
                    std::map<ed_t, int>::const_iterator find_iter = m0_edge_to_disjoint_implicit_cutpath_sequence.find(other_edge);
                    bool other_edge_is_already_mapped = (find_iter != m0_edge_to_disjoint_implicit_cutpath_sequence.cend());

                    if (other_edge_is_already_mapped == false) {
                        next_edge = other_edge; // set sext edge
                    } else {
                        // reached end of sequence
                        ret0 = m0_ivtx_to_disjoint_implicit_cutpath_sequence.emplace(next_vertex, current_disjoint_implicit_cutpath_sequence_index);
                        MCUT_ASSERT(ret0.second == true);
                    }
                }
            }

            lg.unindent();

            // while there is another edge to added to the current disjoint implicit cutpath sequence
        } while (next_edge != mesh_t::null_edge());

        lg << "disjoint implicit cut-path sequence size = " << current_disjoint_implicit_cutpath_sequence.size() << std::endl;

        lg.unindent();
        lg.unindent();
        // while not all intersection-points have been mapped to a disjoint implicit cutpath sequence
    } while (m0_edge_to_disjoint_implicit_cutpath_sequence.size() != m0_cutpath_edges.size());

    lg << "total disjoint implicit cut-path sequences = " << m0_implicit_cutpath_sequences.size() << std::endl;

    MCUT_ASSERT(m0_implicit_cutpath_sequences.empty() == false);

    // delink the implicit cut-path sequences to create the
    // final explicit cut-path sequences
    // -----------------------------------------------------------------------

    m0_cutpath_edges.clear();
    m0_ivtx_to_disjoint_implicit_cutpath_sequence.clear();
    m0_edge_to_disjoint_implicit_cutpath_sequence.clear();

    lg << "create explicit cut-path sequences" << std::endl;

    std::vector<std::vector<ed_t>> m0_explicit_cutpath_sequences;

    for (std::vector<std::vector<ed_t>>::const_iterator iter = m0_implicit_cutpath_sequences.cbegin();
         iter != m0_implicit_cutpath_sequences.cend();
         ++iter) {
        lg.indent();
        const int implicit_cutpath_sequence_index = (int)std::distance(m0_implicit_cutpath_sequences.cbegin(), iter);
        lg << "spliced implicit cut-path sequence = " << implicit_cutpath_sequence_index << std::endl;

        lg.indent();

        const std::vector<ed_t>& implicit_cutpath_sequence = *iter;
        const int implicit_sequence_size = (int)implicit_cutpath_sequence.size();

        lg << "total edges = " << implicit_sequence_size << std::endl;

        // we will now work our way through the vertices of the current sequence,
        // incrementally building sub-sequences. These sub-sequences contain
        // connected/sorted edges which [pass through] intersecting src-mesh
        // polygons. Basically, we want to remove any so-called "exterior
        // interior-edges", which are edges on the cut-path that topologically
        // (according to registry rules) lie on a border-concave src-mesh polygon
        // but do not actually pass inside to cut that polygon (i.e. they are not used
        // for clipping).
        // NOTE: this unique scenario occurs if 1) the src-mesh is not
        // watertight, and 2) it has concave polygons on the border.
        //
        // Each sub-sequence will form an explicit (and final) cut path that we
        // will use in subsequent cutting stages.

        // start new sub-sequence
        m0_explicit_cutpath_sequences.push_back(std::vector<ed_t>());
        std::vector<ed_t>* m0_explicit_cutpath_subsequence = &m0_explicit_cutpath_sequences.back();

        if (implicit_sequence_size <= 2) { // simple linear cut-path case
            lg << "simple linear cut-path" << std::endl;
            m0_explicit_cutpath_subsequence->insert(
                m0_explicit_cutpath_subsequence->cend(),
                implicit_cutpath_sequence.cbegin(),
                implicit_cutpath_sequence.cend());
        } else { // complex linear an circular implicit cut-paths
            lg << "complex cut-path (linear/circular)" << std::endl;

            // Our iterator, counting how many edges that have been traversed so far
            // NOTE: initial value of zero does not imply the beginning of "m0_explicit_cutpath_subsequence".
            // Rather, it implies the first edge traversed in the sequence (we are not guarranteed to
            // start from beginning of "m0_explicit_cutpath_subsequence")
            int implicit_sequence_edge_iter = 0;

            // pick the first edge to start from (in the spliced implicit cut-path sequence)
            // according to the following conditions:
            // 1) is 1st edge of sequence AND is terminal; OR 2) any edge whose
            // vertices have registry entries whose faces differ by one real face
            for (int edge_iter = 0; edge_iter < implicit_sequence_size; ++edge_iter) {

                // check if edge is a terminal edge (i.e. beginning of linear cut-path)
                const ed_t& edge = implicit_cutpath_sequence.at(edge_iter);
                const vd_t edge_vertex0 = m0.vertex(edge, 0);
                const vd_t edge_vertex1 = m0.vertex(edge, 1);
                bool edge_vertex0_is_terminal = (m0_ivtx_to_cutpath_edges.at(edge_vertex0).size() == 1);
                bool edge_vertex1_is_terminal = (m0_ivtx_to_cutpath_edges.at(edge_vertex1).size() == 1);
                bool edge_is_terminal = (edge_vertex0_is_terminal || edge_vertex1_is_terminal);

                // check first condition
                if (edge_iter == 0 && edge_is_terminal) { // .. found!
                    implicit_sequence_edge_iter = edge_iter; // 0
                    break;
                } else { // check second condition
                    std::map<vd_t, std::vector<fd_t>>::const_iterator find_iter;

                    // get intersection-registry faces of first vertex
                    find_iter = m0_ivtx_to_ps_faces.find(edge_vertex0);
                    MCUT_ASSERT(find_iter != m0_ivtx_to_ps_faces.cend());
                    const std::vector<fd_t>& entry0_faces = find_iter->second;

                    // get intersection-registry faces of second vertex
                    find_iter = m0_ivtx_to_ps_faces.find(edge_vertex1);
                    MCUT_ASSERT(find_iter != m0_ivtx_to_ps_faces.cend());
                    const std::vector<fd_t>& entry1_faces = find_iter->second;

                    int diff = 0;
                    // for each face in the registry-entry of first vertex
                    for (std::vector<fd_t>::const_iterator face_iter = entry0_faces.cbegin();
                         face_iter != entry0_faces.cend();
                         ++face_iter) {

                        if (is_virtual_polygon(*face_iter)) {
                            // we compare based on the number of real faces since ambiguities may arise
                            // in the specific cave of border cancave polygons, whose cut-path problem
                            // we are trying to solve right now.
                            continue;
                        }

                        // if the face is not contained in the other vertex' registry entry
                        bool match = std::find(entry1_faces.cbegin(), entry1_faces.cend(), *face_iter) != entry1_faces.cend();

                        if (match == false) {
                            ++diff;
                        }
                    }

                    MCUT_ASSERT(diff <= 1); // ... based on edge placement rules!

                    if (diff == 1) {
                        implicit_sequence_edge_iter = edge_iter;
                        break;
                    }
                }
            }

            do { // each iteration adds an edge to an explicit cut-path sequence, or creates a new explicit cut-path sequence

                // *--*--*--*--*--*--*--* <-- example sequence
                //    |_____| <- example sliding window starting from 2nd vertex/ 2nd edge
                int sliding_window_width = 2;

                int cur_edge_offset = implicit_sequence_edge_iter; // offset of the first edge
                MCUT_ASSERT(cur_edge_offset < (int)implicit_cutpath_sequence.size());
                int next_edge_offset_tmp = (implicit_sequence_edge_iter + (sliding_window_width - 1)) % implicit_sequence_size;
                int next_edge_offset = wrap_integer(next_edge_offset_tmp, 0, implicit_sequence_size - 1);
                MCUT_ASSERT(next_edge_offset < (int)implicit_cutpath_sequence.size());

                const ed_t& current_edge = implicit_cutpath_sequence.at(cur_edge_offset);
                const ed_t& next_edge = implicit_cutpath_sequence.at(next_edge_offset);

                //
                // An implicit cut-path which has at least 4 vertices passing through the same src-mesh polygon
                // needs to be trimmed ([at least] one edge has to be removed to partition the sequence). These
                // vertices (>=4) which pass through the same src-mesh polygon have the property that their
                // intersection registry-rentries match by at-least 2 faces, making them connectable.
                //
                bool registry_entries_match_by_3_real_faces_cn = false;
                bool registry_entries_differ_by_1_real_face_cn = false;
                bool all_vertices_are_connectable = cutpath_vertices_are_connectable(
                    current_edge, next_edge, m0, m0_ivtx_to_ps_faces, registry_entries_match_by_3_real_faces_cn, registry_entries_differ_by_1_real_face_cn);

                // if 1) the vertices are connectable, and 2) their intersection registry-entries [do not] match by 3 faces,
                // and 3) the current sequence is not empty
                // NOTE: condition 2) is used to prevent trimming at the current position if the intersection points are
                // not actually on a border of either the src-mesh or cut-mesh (e.g. simple tet-triangle test)
                // "registry_entries_differ_by_1_real_face_cn" used specifically for tet-triangle intersection (complete cut)
                if (all_vertices_are_connectable && m0_explicit_cutpath_subsequence->size() > 0) { // ... reached the end of an explicit sequence, start a new one

                    if (registry_entries_match_by_3_real_faces_cn || registry_entries_differ_by_1_real_face_cn) {
                        m0_explicit_cutpath_subsequence->push_back(current_edge); // add current edge to sequence
                    } else {
                        // edge passes through a border concave polygon, which implies that
                        // registry entries (for vertices of cur + next edge ) differ by 1 virtual
                        // face

                        MCUT_ASSERT( // .. this is obvious it but aides understanding
                            registry_entries_match_by_3_real_faces_cn == false && registry_entries_differ_by_1_real_face_cn == false);

                        // get edge at the front of current explicit sequence i.e. previously added
                        const ed_t& previous_added_edge = m0_explicit_cutpath_subsequence->back();
                        // get the vertex from previous edge which is not shared with current edge

                        // check if they are all connectable
                        bool registry_entries_match_by_3_real_faces_pc = false; // unused
                        bool registry_entries_differ_by_1_real_face_pc = false; // unused
                        bool cur_edge_vertices_are_connectable_with_previous = cutpath_vertices_are_connectable(
                            previous_added_edge, current_edge, m0, m0_ivtx_to_ps_faces, registry_entries_match_by_3_real_faces_pc, registry_entries_differ_by_1_real_face_pc);
                        bool current_edge_is_last = (implicit_sequence_edge_iter == (implicit_sequence_size - 1));

                        // NOTE: to understand the logic here, refer to test 7 & 25 (src-mesh = quad & concave-teeth)
                        //
                        if (cur_edge_vertices_are_connectable_with_previous && //
                            (!registry_entries_match_by_3_real_faces_pc /*&& registry_entries_differ_by_1_real_face_pc*/) && //
                            !current_edge_is_last) {
                            // begin new explicit sequence, but do not add current edge (it has been removed)
                            m0_explicit_cutpath_sequences.push_back(std::vector<ed_t>());
                            m0_explicit_cutpath_subsequence = &m0_explicit_cutpath_sequences.back();
                        } else {
                            m0_explicit_cutpath_subsequence->push_back(current_edge); // add current edge to sequence
                        }
                    }
                } else { // .. then we can add the current edge as part of the current explicit sequence
                    m0_explicit_cutpath_subsequence->push_back(current_edge);
                }

            } while (++implicit_sequence_edge_iter != implicit_sequence_size);
        }

        lg.unindent();
        lg.unindent();
    }

    m0_ivtx_to_cutpath_edges.clear();
    m0_implicit_cutpath_sequences.clear();

    lg << "total explicit cut-path sequences = " << m0_explicit_cutpath_sequences.size() << std::endl;

    MCUT_ASSERT(m0_explicit_cutpath_sequences.empty() == false);

    const int num_explicit_cutpath_sequences = (int)m0_explicit_cutpath_sequences.size();

    // save explicit cut-path sequence properties (linear/circular;is_hole)
    // -----------------------------------------------------------------------

    // first we need to find all intersection point which have a border src-mesh
    // halfedge in their intersection registry
    //

    lg << "find border intersection points" << std::endl;

    // NOTE: we need this data structure to allow us to determine the
    // properties of the explicit cut-paths
    //
    // MapKey=intersection point on a border halfedge of either the src-mesh or cut-mesh
    // MapValue=pointer entry in "m0_ivtx_to_ps_he"
    std::map<vd_t, std::map<vd_t, hd_t>::const_iterator> m0_explicit_cutpath_terminal_vertices;

    for (std::map<vd_t, hd_t>::const_iterator iter = m0_ivtx_to_ps_he.cbegin();
         iter != m0_ivtx_to_ps_he.cend();
         ++iter) {
        lg.indent();
        const vd_t& ivtx = iter->first;
        const hd_t& ivtx_ps_he = iter->second;

        MCUT_ASSERT(ivtx_ps_he != mesh_t::null_halfedge());

        const ed_t edge_of_ivtx_ps_he = ps.edge(ivtx_ps_he);
        // check that "ivtx_ps_he" is a border halfedge
        if (ps.is_border(edge_of_ivtx_ps_he)) {
            // we have found a terminal vertex
            std::pair<std::map<vd_t, std::map<vd_t, hd_t>::const_iterator>::const_iterator, bool> ret = m0_explicit_cutpath_terminal_vertices.insert(std::make_pair(ivtx, iter));
            MCUT_ASSERT(ret.second == true);
            lg << vstr(ivtx) << std::endl;
        }

        lg.unindent();
    }

    lg << "total border intersection points = " << m0_explicit_cutpath_terminal_vertices.size() << std::endl;

    lg << "infer cut-path properties" << std::endl;

    // MapKey=index of an explicit cutpath in  m0_explicit_cutpath_sequences
    // MapValue=a pair of boolean properties (is_linear, is_hole, is_srcmesh_severing).
    //
    // if is_linear is false, then the cut path is "circular" and "is_hole" will
    // always be true in this specific case.
    // if is_linear is true, then the cutpath may or may not be a srcmesh severing cutpath (depends on
    // whether we have a partial cut)
    // if is_circular is true, then the cutpath is always severing.
    std::map<int, std::tuple<bool, bool, bool>> m0_explicit_cutpath_sequence_to_properties;

    for (std::vector<std::vector<ed_t>>::const_iterator iter = m0_explicit_cutpath_sequences.cbegin();
         iter != m0_explicit_cutpath_sequences.cend();
         ++iter) {
        lg.indent();
        const int cutpath_index = (int)std::distance(m0_explicit_cutpath_sequences.cbegin(), iter);

        lg << "explicit cut-path = " << cutpath_index << std::endl;

        lg.indent();
        const std::vector<ed_t>& cutpath = *iter;

        std::pair<std::map<int, std::tuple<bool, bool, bool>>::iterator, bool> inserted = m0_explicit_cutpath_sequence_to_properties.insert(std::make_pair(cutpath_index, std::tuple<bool, bool, bool>()));

        MCUT_ASSERT(inserted.second == true);

        std::tuple<bool, bool, bool>& properties = inserted.first->second;
        bool& cutpath_is_linear = std::get<0>(properties);
        bool& cutpath_is_hole = std::get<1>(properties);
        bool& cutpath_is_srcmesh_severing = std::get<2>(properties); // i.e. the cutpath severs/partitions the src-mesh into two parts

        cutpath_is_linear = false;
        cutpath_is_hole = false;
        cutpath_is_srcmesh_severing = true;

        // check if it is a linear cut path

        const ed_t& first_edge = cutpath.front();
        const vd_t first_edge_vertex0 = m0.vertex(first_edge, 0);
        const vd_t first_edge_vertex1 = m0.vertex(first_edge, 1);
        bool first_edge_vertex0_is_terminal = m0_explicit_cutpath_terminal_vertices.find(first_edge_vertex0) != m0_explicit_cutpath_terminal_vertices.cend();

        bool first_edge_is_terminal = first_edge_vertex0_is_terminal;

        if (first_edge_vertex0_is_terminal == false) {
            // check if vertex1 is terminal
            bool first_edge_vertex1_is_terminal = m0_explicit_cutpath_terminal_vertices.find(first_edge_vertex1) != m0_explicit_cutpath_terminal_vertices.cend();
            first_edge_is_terminal = first_edge_vertex1_is_terminal;
        }

        // note: by construction, if the first edge is terminal then the
        // last edge will also be terminal (thus we could have used the
        // last edge for the above tests too!)
        cutpath_is_linear = first_edge_is_terminal;

        bool cutpath_is_circular = !cutpath_is_linear;
        // check if a hole is created by the cutpath (which will need sealing later)
        if (cutpath_is_circular) {
            cutpath_is_hole = true;
        } else {
            // current cut path is [linear]. it creates a hole if both terminal vertices
            // have a cut-mesh halfedge in there registry

            const vd_t& first_edge_terminal_vertex = (first_edge_vertex0_is_terminal ? first_edge_vertex0 : first_edge_vertex1);

            // get the halfedge and check where is comes from (cut-mesh/src-mesh)

            std::map<vd_t, std::map<vd_t, hd_t>::const_iterator>::const_iterator find_iter = m0_explicit_cutpath_terminal_vertices.cend();
            find_iter = m0_explicit_cutpath_terminal_vertices.find(first_edge_terminal_vertex);

            MCUT_ASSERT(find_iter != m0_explicit_cutpath_terminal_vertices.cend());

            const hd_t& first_edge_terminal_vertex_he = find_iter->second->second;
            const fd_t ps_face_of_first_edge_terminal_vertex_he = ps.face(first_edge_terminal_vertex_he);

            // must exist because "ivtx_ps_he" came from an intersecting face in the
            // polygon soup
            MCUT_ASSERT(ps_face_of_first_edge_terminal_vertex_he != mesh_t::null_face());

            bool is_from_cut_mesh = ps_is_cutmesh_face(ps_face_of_first_edge_terminal_vertex_he, sm_face_count);
            bool is_from_src_mesh = !is_from_cut_mesh;

            const bool first_vtx_is_from_src_mesh = is_from_src_mesh;
            /*
            if (is_from_src_mesh) {
                cutpath_is_hole = false;
            }
            else
            {*/
            // ... so the halfedge in the registry of "first_edge_terminal_vertex"
            // belongs to the cut-mesh. Now let us repeat the same test but this
            // time for the "last_edge_terminal_vertex"

            const ed_t& last_edge = cutpath.back();
            const vd_t last_edge_vertex0 = m0.vertex(last_edge, 0);
            const vd_t last_edge_vertex1 = m0.vertex(last_edge, 1);
            bool last_edge_vertex0_is_terminal = m0_explicit_cutpath_terminal_vertices.find(last_edge_vertex0) != m0_explicit_cutpath_terminal_vertices.cend();

            bool last_edge_is_terminal = last_edge_vertex0_is_terminal;

            if (last_edge_vertex0_is_terminal == false) {
                // check if vertex1 is terminal
                bool last_edge_vertex1_is_terminal = m0_explicit_cutpath_terminal_vertices.find(last_edge_vertex1) != m0_explicit_cutpath_terminal_vertices.cend();
                last_edge_is_terminal = last_edge_vertex1_is_terminal;
            }

            bool last_vtx_is_from_src_mesh = first_vtx_is_from_src_mesh; // ... we will use this to determine whether we have a severing cutpath or not (the current one)

            MCUT_ASSERT(last_edge_is_terminal); // i.e. we have a linear cut path
            vd_t last_edge_terminal_vertex = mesh_t::null_vertex();

            if (last_edge == first_edge) // sequence has one edge
            {
                last_edge_terminal_vertex = (first_edge_terminal_vertex == last_edge_vertex0) ? last_edge_vertex1 : last_edge_vertex0;
            } else {
                last_edge_terminal_vertex = (last_edge_vertex0_is_terminal ? last_edge_vertex0 : last_edge_vertex1);
            }

            // get the halfedge and check where is comes from (cut-mesh/src-mesh)

            //std::map<vd_t, std::map<vd_t, hd_t>::const_iterator>::const_iterator find_iter = m0_explicit_cutpath_terminal_vertices.cend();
            find_iter = m0_explicit_cutpath_terminal_vertices.find(last_edge_terminal_vertex);
            MCUT_ASSERT(find_iter != m0_explicit_cutpath_terminal_vertices.cend());
            const hd_t& last_edge_terminal_vertex_he = find_iter->second->second;

            const fd_t ps_face_of_last_edge_terminal_vertex_he = ps.face(last_edge_terminal_vertex_he);

            // must exist because "ivtx_ps_he" came from an intersecting face in the
            // polygon soup
            MCUT_ASSERT(ps_face_of_last_edge_terminal_vertex_he != mesh_t::null_face());

            is_from_cut_mesh = ps_is_cutmesh_face(ps_face_of_last_edge_terminal_vertex_he, sm_face_count);
            is_from_src_mesh = !is_from_cut_mesh;

            last_vtx_is_from_src_mesh = is_from_src_mesh;

            //if (is_from_cut_mesh) {
            //    cutpath_is_hole = true;
            //}
            // }
            cutpath_is_hole = (!last_vtx_is_from_src_mesh && !first_vtx_is_from_src_mesh);
            cutpath_is_srcmesh_severing = first_vtx_is_from_src_mesh == true && (last_vtx_is_from_src_mesh == first_vtx_is_from_src_mesh);
            //}
        }

        lg << "type = `" << (cutpath_is_linear ? "linear" : "circular") << "`" << std::endl;
        lg << "hole = " << std::boolalpha << cutpath_is_hole << std::endl;

        lg.unindent();
        lg.unindent();
    }

    m0_explicit_cutpath_terminal_vertices.clear();

    int num_explicit_linear_cutpaths = 0;
    int num_explicit_circular_cutpaths = 0;
    std::vector<int> explicit_cutpaths_making_holes;
    std::vector<int> explicit_cutpaths_severing_srcmesh;

    for (std::map<int, std::tuple<bool, bool, bool>>::const_iterator iter = m0_explicit_cutpath_sequence_to_properties.cbegin();
         iter != m0_explicit_cutpath_sequence_to_properties.cend(); ++iter) {
        const int& explicit_cutpath_index = iter->first;
        const std::tuple<bool, bool, bool>& properties = iter->second;

        const bool& is_linear = std::get<0>(properties);
        const bool& is_hole = std::get<1>(properties);
        const bool& is_srcmesh_severing = std::get<2>(properties);

        if (is_linear) {
            num_explicit_linear_cutpaths += 1;
        } else {
            num_explicit_circular_cutpaths += 1;
        }

        if (is_hole) {
            explicit_cutpaths_making_holes.push_back(explicit_cutpath_index);
        }

        if (is_srcmesh_severing) {
            explicit_cutpaths_severing_srcmesh.push_back(explicit_cutpath_index);
        }
    }

    lg << "total explicit linear cutpaths = " << num_explicit_linear_cutpaths << std::endl;
    lg << "total explicit circular cutpaths = " << num_explicit_circular_cutpaths << std::endl;
    lg << "total explicit cutpaths making holes = " << explicit_cutpaths_making_holes.size() << std::endl;
    lg << "total explict cutpaths severing the srcmesh = " << explicit_cutpaths_severing_srcmesh.size() << std::endl;

    lg.unindent(); // end of cutpath sequence detection code

    m0_explicit_cutpath_sequence_to_properties.clear();

    // NOTE:    at this point we have all vertices, edges, and the lists of
    //          edge sequences identifying the cutpaths

    // associate/map intersection points to their explicit cut-path sequences
    std::map<vd_t, int> m0_ivtx_to_explicit_cutpath_sequence;
    for (std::vector<std::vector<ed_t>>::const_iterator iter = m0_explicit_cutpath_sequences.cbegin();
         iter != m0_explicit_cutpath_sequences.cend();
         ++iter) {

        const int cutpath_index = (int)std::distance(m0_explicit_cutpath_sequences.cbegin(), iter);

        for (std::vector<ed_t>::const_iterator it = iter->cbegin(); it != iter->cend(); ++it) {
            const ed_t& edge = *it;
            const vd_t vertex0 = m0.vertex(edge, 0);
            const vd_t vertex1 = m0.vertex(edge, 1);

            std::pair<std::map<vd_t, int>::const_iterator, bool> insertion;

            // insert vertex0
            insertion = m0_ivtx_to_explicit_cutpath_sequence.insert(std::make_pair(vertex0, cutpath_index));
            if (insertion.second == false) {
                MCUT_ASSERT(insertion.first->second == cutpath_index);
            }

            // insert vertex1
            insertion = m0_ivtx_to_explicit_cutpath_sequence.insert(std::make_pair(vertex1, cutpath_index));
            if (insertion.second == false) {
                MCUT_ASSERT(insertion.first->second == cutpath_index);
            }
        }
    }

    // Detect degeneracy (see note "limitations")
    //------------------

    bool have_more_than_one_explicit_cutpath = (m0_explicit_cutpath_sequences.size() > 0);
    if (have_more_than_one_explicit_cutpath) {
        bool atleast_one_explicit_cutpath_makes_a_hole = !explicit_cutpaths_making_holes.empty();
        //bool atleast_one_explicit_cutpath_is_linear = num_explicit_linear_cutpaths;
        if (atleast_one_explicit_cutpath_makes_a_hole) {
            // TODO
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // Gather intersection vertices on each intersecting polygon
    ///////////////////////////////////////////////////////////////////////////

    lg << "associate intersecting faces to intersection-points" << std::endl;

    std::map<fd_t, std::vector<vd_t>> ps_iface_to_ivtx_list; // faces which intersect with another

    for (std::map<vd_t, std::vector<fd_t>>::const_iterator ireg_entry_iter = m0_ivtx_to_ps_faces.cbegin();
         ireg_entry_iter != m0_ivtx_to_ps_faces.cend();
         ++ireg_entry_iter) { // for each intersection registry entry

        const vd_t& entry_vertex = ireg_entry_iter->first;
        const std::vector<fd_t>& entry_faces = ireg_entry_iter->second;

        // update face vertex-registry

        for (std::vector<fd_t>::const_iterator entry_face_iter = entry_faces.cbegin();
             entry_face_iter != entry_faces.cend();
             ++entry_face_iter) {

            if (is_virtual_polygon(*entry_face_iter)) {
                continue; // virtal faces are simply placeholders
            }

            std::map<fd_t, std::vector<vd_t>>::iterator find_iter = ps_iface_to_ivtx_list.find(*entry_face_iter);
            const bool face_vertex_registery_exists = find_iter != ps_iface_to_ivtx_list.cend();

            if (face_vertex_registery_exists) {
                std::vector<vd_t>& face_vertex_registry = find_iter->second;
                const bool vertex_registered = std::find(face_vertex_registry.cbegin(), face_vertex_registry.cend(), entry_vertex) != face_vertex_registry.cend();

                if (!vertex_registered) {
                    face_vertex_registry.push_back(entry_vertex);
                }
            } else {
                // add face-registry and register vertex
                std::pair<std::map<fd_t, std::vector<vd_t>>::iterator, bool> pair = ps_iface_to_ivtx_list.insert(std::make_pair(*entry_face_iter, std::vector<vd_t>()));
                MCUT_ASSERT(pair.second == true);

                find_iter = pair.first;
                std::vector<vd_t>& face_vertex_registry = find_iter->second;
                face_vertex_registry.push_back(entry_vertex);
            }
        }
    }

    // dump
    for (std::map<fd_t, std::vector<vd_t>>::const_iterator i = ps_iface_to_ivtx_list.cbegin(); i != ps_iface_to_ivtx_list.cend(); ++i) {
        lg.indent();
        lg << "face " << i->first << std::endl;
        lg.indent();
        for (std::vector<vd_t>::const_iterator j = i->second.cbegin(); j != i->second.cend(); ++j) {
            lg << vstr(*j) << " ";
        }
        lg << std::endl;
        lg.unindent();

        lg.unindent();
    }

    ///////////////////////////////////////////////////////////////////////////
    // Polygon tracing (clipping of intersecting polygon-soup faces)
    ///////////////////////////////////////////////////////////////////////////

    lg << "clip intersecting polygons" << std::endl;

    std::vector<traced_polygon_t> m0_polygons;

    int traced_sm_polygon_count = 0;

    for (mesh_t::face_iterator_t ps_face_iter = ps.faces_begin(); ps_face_iter != ps.faces_end(); ++ps_face_iter) {
        lg.indent();

        const fd_t& ps_face = *ps_face_iter;

        // reference to all the edges which lie on the face
        std::map<fd_t, std::vector<ed_t>>::iterator ps_iface_to_m0_edge_list_fiter = ps_iface_to_m0_edge_list.find(ps_face);
        bool is_iface = ps_iface_to_m0_edge_list_fiter != ps_iface_to_m0_edge_list.end();
        bool is_from_cut_mesh = ps_is_cutmesh_face(ps_face, sm_face_count);

        lg << "face " << ps_face << std::endl;

        std::vector<traced_polygon_t> child_polygons; // new polygons traced on current face

        lg.indent();

        lg << "mesh = " << (is_from_cut_mesh ? "cut-mesh" : "source-mesh") << std::endl;
        lg << "is intersecting face = " << std::boolalpha << is_iface << std::endl;

        if (is_iface == false) { // non-intersecting face

            // NOTE: here we just copy the polygon as-is
            traced_polygon_t retraced_poly;
            //CGAL::Halfedge_around_face_iterator<mesh_t> hbegin, hend;
            //int num_halfedges = 0;
            lg << "polygon" << std::endl;
            //for (boost::tie(hbegin, hend) = halfedges_around_face(ps.halfedge(ps_face), ps); hbegin != hend; ++hbegin) {
            std::vector<hd_t> halfedges_around_face = ps.get_halfedges_around_face(ps_face);
            for (std::vector<hd_t>::const_iterator hbegin = halfedges_around_face.cbegin(); hbegin != halfedges_around_face.cend(); ++hbegin) {
                lg.indent();

                const vd_t ps_h_src = ps.source(*hbegin);
                const vd_t ps_h_tgt = ps.target(*hbegin);
                std::map<vd_t, vd_t>::const_iterator ps_h_src_fiter = std::find_if(m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(),
                    [&](const std::pair<vd_t, vd_t>& e) -> bool { return e.second == ps_h_src; });

                MCUT_ASSERT(ps_h_src_fiter != m0_to_ps_vtx.cend());

                std::map<vd_t, vd_t>::const_iterator ps_h_tgt_fiter = std::find_if(m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(),
                    [&](const std::pair<vd_t, vd_t>& e) -> bool { return e.second == ps_h_tgt; });

                MCUT_ASSERT(ps_h_tgt_fiter != m0_to_ps_vtx.cend());

                const vd_t m0_h_src = ps_h_src_fiter->first;
                const vd_t m0_h_tgt = ps_h_tgt_fiter->first;
                const ed_t ps_edge = ps.edge(*hbegin);
                const ed_t m0_edge = ps_to_m0_non_intersecting_edge.at(ps_edge);
                const hd_t m0_edge_h0 = m0.halfedge(m0_edge, 0);
                const hd_t m0_edge_h1 = m0.halfedge(m0_edge, 1);

                if (m0.source(m0_edge_h0) == m0_h_src && m0.target(m0_edge_h0) == m0_h_tgt) {
                    lg << hstr(m0, m0_edge_h0) << std::endl;
                    retraced_poly.emplace_back(m0_edge_h0);
                } else {
                    lg << hstr(m0, m0_edge_h1) << std::endl;
                    retraced_poly.emplace_back(m0_edge_h1);
                }

                //num_halfedges++;
                lg.unindent();
            }

            child_polygons.emplace_back(retraced_poly);

        } else {

            // list of edges which lie on the face
            const std::vector<ed_t>& ps_iface_m0_edge_list = ps_iface_to_m0_edge_list_fiter->second;

            //
            // gather vertices on face (including intersection points)
            //

            //std::vector<vd_t> ps_coincident_vertices = get_vertices_on_face(ps, ps_face);
            std::vector<vd_t> ps_coincident_vertices = ps.get_vertices_around_face(ps_face);
            std::vector<vd_t> coincident_vertices; // those stored in "m0"

            // gather the original (m0) vertices on the face
            for (int i = 0; i < (int)ps_coincident_vertices.size(); ++i) {

                std::map<vd_t, vd_t>::const_iterator m0_to_ps_vtx_fiter = std::find_if(
                    m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(),
                    [&](const std::pair<vd_t, vd_t>& e) -> bool {
                        return e.second == ps_coincident_vertices.at(i);
                    });

                MCUT_ASSERT(m0_to_ps_vtx_fiter != m0_to_ps_vtx.end());

                coincident_vertices.emplace_back(m0_to_ps_vtx_fiter->first);
            }

            // now gather the intersection-points on the face
            const std::map<fd_t, std::vector<vd_t>>::const_iterator ireg_entry_iter = ps_iface_to_ivtx_list.find(ps_face);

            MCUT_ASSERT(ireg_entry_iter != ps_iface_to_ivtx_list.cend());

            const int coincident_ps_vertex_count = (int)coincident_vertices.size();
            const std::vector<vd_t>& intersection_points_on_face = ireg_entry_iter->second;
            coincident_vertices.insert(coincident_vertices.end(), intersection_points_on_face.cbegin(), intersection_points_on_face.cend());

            lg << "intersection points = " << intersection_points_on_face.size() << std::endl;

            MCUT_ASSERT(intersection_points_on_face.size() >= 2);

            // dump
            if (input.verbose) {
                lg << "coincident vertices = " << coincident_vertices.size() << std::endl;
                lg.indent();
                for (std::vector<vd_t>::const_iterator j = coincident_vertices.cbegin(); j != coincident_vertices.cend(); ++j) {
                    lg << vstr(*j) << " ";
                }
                lg << std::endl;
                lg.unindent();
            }

            //
            // gather edges on face
            //

            // first, we gather the edges on the exterior (boundary) of the face
            std::vector<ed_t> incident_edges;
#if 1
            incident_edges = ps_iface_m0_edge_list;
#else
            // for each edge on the current face
            for (std::vector<ed_t>::const_iterator aux_edge_iter = ps_iface_m0_edge_list.cbegin(); aux_edge_iter != ps_iface_m0_edge_list.cend(); ++aux_edge_iter) {

                const ed_t& edge = (*aux_edge_iter);
                const vd_t v0 = m0.vertex(edge, 0);
                const vd_t v1 = m0.vertex(edge, 1);
                const bool v0_is_ivtx = m0_is_intersection_point(v0, ps_vtx_cnt);
                const bool v1_is_ivtx = m0_is_intersection_point(v1, ps_vtx_cnt);
                const bool is_ambiguious_exterior_edge_case = v0_is_ivtx && v1_is_ivtx;
                bool is_valid_ambiguious_exterior_edge = false;

                if (is_ambiguious_exterior_edge_case) { // denotes an exterior edge defined by two intersection vertices which is an ambigious case arising from concave cutting problem

                    // get the halfedges which where traversed to calculate v0 and v1 as intersection points
                    const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v0);
                    const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v1);

                    // get their edges
                    const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                    const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);

                    // when the two halfedges belongs to the same edge, v0 and v1 where produced by multiple intersections of one edge "edge" with two faces which is a scenario which will arise from a concave cutting problem
                    is_valid_ambiguious_exterior_edge = (v0_ps_edge == v1_ps_edge); //
                }

                if ((!is_ambiguious_exterior_edge_case || is_valid_ambiguious_exterior_edge)) {
                    incident_edges.push_back(edge);
                }
            }

            const int incident_exterior_edge_count = incident_edges.size(); // used to calculate an offset into "incident_edges" for determining that starting position of interior edges

            // dump
            lg << "exterior edges = " << incident_exterior_edge_count << " :";
            for (std::vector<ed_t>::const_iterator exterior_edge_iter = incident_edges.cbegin(); exterior_edge_iter != incident_edges.cend(); ++exterior_edge_iter) {
                lg << estr(m0, *exterior_edge_iter) << std::endl; // " ext-edge=" << *exterior_edge_iter << " : " << m0.vertex(*exterior_edge_iter, 0) << " " << m0.vertex(*exterior_edge_iter, 1) << std::endl;
            }
            lg << std::endl;

            MCUT_ASSERT(incident_exterior_edge_count >= 3); // minimum is 3 edge which is for a triangle

            // now that we have gathered the exterior edges, the next step is to gather
            // interior edges on the face
            //
            //  an incident interior edge must satisfy 3 conditions
            //  1) edge defined by incident vertices
            //  2) these vertices are intersection vertices
            //  3) the coincident ps-halfedges of these vertices are different (i.e. the inverse rule which is used to resolve the ambiguity of exterior edges defined by two edges).

            // for each edge on face
            for (std::vector<ed_t>::const_iterator aux_edge_iter = ps_iface_m0_edge_list.cbegin(); aux_edge_iter != ps_iface_m0_edge_list.cend(); ++aux_edge_iter) {

                const ed_t& edge = *aux_edge_iter;
                const vd_t v0 = m0.vertex(edge, 0);
                const vd_t v1 = m0.vertex(edge, 1);
                const bool v0_is_ivtx = m0_is_intersection_point(v0, ps_vtx_cnt);
                const bool v1_is_ivtx = m0_is_intersection_point(v1, ps_vtx_cnt);
                const bool is_ambiguious_interior_edge_case = v0_is_ivtx && v1_is_ivtx;
                bool is_valid_ambiguious_interior_edge = false;

                if (is_ambiguious_interior_edge_case) {
                    const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v0);
                    const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v1);
                    const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                    const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);

                    is_valid_ambiguious_interior_edge = (v0_ps_edge != v1_ps_edge); // must be different!

                    if (is_valid_ambiguious_interior_edge) {
                        incident_edges.push_back(edge);
                        lg << estr(m0, edge) << std::endl; //"interior edge = "  << edge << " " << m0.vertex(edge, 0) << " " << m0.vertex(edge, 1) << std::endl;
                    }
                }
            }
#endif
            int incident_exterior_edge_count = 0;
            // at this point we want to partition the list of edges into exterior and interior
            // i.e. exterior edges come first. We do this because it makes it easier for us
            // to filter interior edges when they are consecutive in the list
            std::partition(incident_edges.begin(), incident_edges.end(),
                [&](const ed_t& e) { // calculate if edge is exterior
                    const vd_t v0 = m0.vertex(e, 0);
                    const vd_t v1 = m0.vertex(e, 1);
                    const bool v0_is_ivtx = m0_is_intersection_point(v0, ps_vtx_cnt);
                    const bool v1_is_ivtx = m0_is_intersection_point(v1, ps_vtx_cnt);
                    const bool is_ambiguious_exterior_edge_case = v0_is_ivtx && v1_is_ivtx;
                    bool is_valid_ambiguious_exterior_edge = false;

                    if (is_ambiguious_exterior_edge_case) { // denotes an exterior edge defined by two intersection vertices which is an ambigious case arising from concave cutting problem

                        // get the halfedges which where traversed to calculate v0 and v1 as intersection points
                        const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v0);
                        const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v1);

                        // get their edges
                        const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                        const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);

                        // when the two halfedges belongs to the same edge, v0 and v1 where produced by multiple intersections of one edge "edge" with two faces which is a scenario which will arise from a concave cutting problem
                        is_valid_ambiguious_exterior_edge = (v0_ps_edge == v1_ps_edge); //
                    }

                    bool is_exterior_edge = (!is_ambiguious_exterior_edge_case || is_valid_ambiguious_exterior_edge);
                    if (is_exterior_edge) {
                        incident_exterior_edge_count++; // count
                    }
                    return is_exterior_edge;
                });

            // dump
            lg << "exterior edges = " << incident_exterior_edge_count << std::endl;
            for (std::vector<ed_t>::const_iterator exterior_edge_iter = incident_edges.cbegin();
                 exterior_edge_iter != incident_edges.cbegin() + incident_exterior_edge_count;
                 ++exterior_edge_iter) {
                lg.indent();
                lg << estr(m0, *exterior_edge_iter) << std::endl; // " ext-edge=" << *exterior_edge_iter << " : " << m0.vertex(*exterior_edge_iter, 0) << " " << m0.vertex(*exterior_edge_iter, 1) << std::endl;
                lg.unindent();
            }

            MCUT_ASSERT(incident_exterior_edge_count >= 3); // minimum is 3 edge which is for a triangle

            const int interior_edges_on_face = (int)incident_edges.size() - incident_exterior_edge_count;
            lg << "interior edges (unfiltered) = " << interior_edges_on_face << std::endl;
            for (std::vector<ed_t>::const_iterator interior_edge_iter = incident_edges.cbegin() + incident_exterior_edge_count;
                 interior_edge_iter != incident_edges.cend();
                 ++interior_edge_iter) {
                lg.indent();
                lg << estr(m0, *interior_edge_iter) << std::endl; // " ext-edge=" << *exterior_edge_iter << " : " << m0.vertex(*exterior_edge_iter, 0) << " " << m0.vertex(*exterior_edge_iter, 1) << std::endl;
                lg.unindent();
            }

            MCUT_ASSERT(interior_edges_on_face >= 1); // minimum possible number (cae of two intersection points)

            // Having partitioned the list of edges on the current face, we will now filter out the
            // redundant [exterior interior-edges]. An exterior interior-edge is one whose vertices
            // lie on the clipped polygon but the line connecting them passes outside that polygon.
            // such edges arise due to concave polygon intersections i.e. the currently clipped
            // polygon (ps_face) intersected a concave polygon.
            //
            // For why we need filtering, we need it because our goal is to be able to trace
            // child-polygons without of numerical (geometry) operations. Specifically, filtering
            // is necessary to avoid certain ambiguities which arise when tracing child-polygons,
            // and these ambiguity come from the fact that we will be using only halfedges to trace.
            // (See tracing steps below!)
            lg << "filter exterior interior-edges" << std::endl;

            /*
                Here we identify [sets] of edges that are 1) defined using intersection points, and 2) each 
                set has edges with the (grouping) property that: intersection registry entries match by at 
                least two real faces

                1. gather all intersection points (on current face) 
                2. gather all iedges which are connected to vertices in step (1)
                3. classify the edges from step (2) into sets using our set property `S` # (registry entry matching by 2 faces)
                    a. while there is a non-classified edge, `i`
                        b. for each set `s`
                            c. pick any edge in `s`, `e` # (since all in the set edges will share the set property)
                            d. IF the property of `e` matches `i`
                                e. insert `i` into `s`
                        d. IF `i` not added to any set
                            e. create new set and insert `i`
                4. for each set in `S`, `s`
                    a. IF `s` has 3 or more edges
                        b. apply filtering

                see code below for filtering algorithm
            */

            // 2. gather all interior edges (those connected to intesection points)
            std::deque<ed_t> incident_iedges_set_classification_queue(
                incident_edges.cbegin() + incident_exterior_edge_count, // offset to start of interior edges
                incident_edges.cend()); // dirty copy

            std::map<int, std::set<ed_t>> iedge_sets;

            // 3. classify the edges into sets according to our set property (incident vertex registry matching by 2 faces)

            // a. while there is a non-classified interior edges
            while (!incident_iedges_set_classification_queue.empty()) {

                const ed_t& unclassified_iedge = incident_iedges_set_classification_queue.front();
                const vd_t unclassified_iedge_v0 = m0.vertex(unclassified_iedge, 0);
                const vd_t unclassified_iedge_v1 = m0.vertex(unclassified_iedge, 1);

                // proceed only if unclassified_iedge is not a polygon-exterior interior-iedge : x-->x where o==>x==>x==>o

                const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(unclassified_iedge_v0);
                const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(unclassified_iedge_v1);
                const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);

                const bool is_valid_ambiguious_exterior_edge = (v0_ps_edge == v1_ps_edge); // must be different!

                if (is_valid_ambiguious_exterior_edge) {
                    continue; // is exterior!
                }

                const std::vector<fd_t>& unclassified_iedge_v0_registry = m0_ivtx_to_ps_faces.at(unclassified_iedge_v0);
                const std::vector<fd_t>& unclassified_iedge_v1_registry = m0_ivtx_to_ps_faces.at(unclassified_iedge_v1);

                bool inserted_into_preexisting_set = false;

                // b. for each set
                for (std::map<int, std::set<ed_t>>::iterator iedge_sets_iter = iedge_sets.begin(); iedge_sets_iter != iedge_sets.end(); ++iedge_sets_iter) {

                    std::set<ed_t>& iedge_set = iedge_sets_iter->second;
                    // c. pick any edge in <b> # (since all edges will share the set property)
                    const ed_t set_edge0 = *iedge_set.cbegin();

                    // d. IF the property of <c> matches with <a>
                    const vd_t set_edge0_v0 = m0.vertex(set_edge0, 0);
                    const vd_t set_edge0_v1 = m0.vertex(set_edge0, 1);

                    // NOTE: we need to compare against both vertices' registries (set_edge0_v0 and set_edge0_v1) since second vertex may not share the two same faces we are trying to match with
                    const std::vector<fd_t>& set_edge0_v0_registry = m0_ivtx_to_ps_faces.at(set_edge0_v0);

                    int unclassified_iedge_v0_match_count = 0;
                    int unclassified_iedge_v1_match_count = 0;

                    for (std::vector<fd_t>::const_iterator set_edge0_v0_registry_iter = set_edge0_v0_registry.cbegin(); set_edge0_v0_registry_iter != set_edge0_v0_registry.cend(); ++set_edge0_v0_registry_iter) {
                        if (is_virtual_polygon(*set_edge0_v0_registry_iter)) {
                            continue; // we want to count the number of real faces
                        }

                        unclassified_iedge_v0_match_count += (std::find(unclassified_iedge_v0_registry.cbegin(), unclassified_iedge_v0_registry.cend(), *set_edge0_v0_registry_iter) != unclassified_iedge_v0_registry.cend()) ? 1 : 0;
                        unclassified_iedge_v1_match_count += (std::find(unclassified_iedge_v1_registry.cbegin(), unclassified_iedge_v1_registry.cend(), *set_edge0_v0_registry_iter) != unclassified_iedge_v1_registry.cend()) ? 1 : 0;
                    }

                    const bool vertices_match_set = unclassified_iedge_v0_match_count >= 2 && unclassified_iedge_v1_match_count >= 2;

                    if (vertices_match_set) {

                        // repeat : now match with set_edge0_v1

                        const std::vector<fd_t>& set_edge0_v1_registry = m0_ivtx_to_ps_faces.at(set_edge0_v1);

                        unclassified_iedge_v0_match_count = 0;
                        unclassified_iedge_v1_match_count = 0;

                        for (std::vector<fd_t>::const_iterator set_edge0_v1_registry_iter = set_edge0_v1_registry.cbegin(); set_edge0_v1_registry_iter != set_edge0_v1_registry.cend(); ++set_edge0_v1_registry_iter) {
                            if (is_virtual_polygon(*set_edge0_v1_registry_iter)) {
                                continue; // we want to count the number of real faces
                            }

                            unclassified_iedge_v0_match_count += (std::find(unclassified_iedge_v0_registry.cbegin(), unclassified_iedge_v0_registry.cend(), *set_edge0_v1_registry_iter) != unclassified_iedge_v0_registry.cend()) ? 1 : 0;
                            unclassified_iedge_v1_match_count += (std::find(unclassified_iedge_v1_registry.cbegin(), unclassified_iedge_v1_registry.cend(), *set_edge0_v1_registry_iter) != unclassified_iedge_v1_registry.cend()) ? 1 : 0;
                        }

                        const bool vertices_still_match_set = unclassified_iedge_v0_match_count >= 2 && unclassified_iedge_v1_match_count >= 2;
                        if (vertices_still_match_set) {
                            iedge_set.insert(unclassified_iedge); // NOTE: insertion invalidates set iterators
                            inserted_into_preexisting_set = true;
                        }
                    }
                }

                // d. IF <a> not added to any set
                if (!inserted_into_preexisting_set) {
                    // e. create new set and insert <a>
                    std::pair<std::map<int, std::set<ed_t>>::iterator, bool> iedge_sets_insertion = iedge_sets.insert(std::make_pair((int)iedge_sets.size(), std::set<ed_t>()));

                    MCUT_ASSERT(iedge_sets_insertion.second == true);

                    iedge_sets_insertion.first->second.insert(unclassified_iedge);
                }

                incident_iedges_set_classification_queue.pop_front(); // rm unclassified_iedge
            }

            if (!iedge_sets.empty()) {
                // normally we just get one set
                lg << "interior edge sets = " << iedge_sets.at(0).size() << std::endl;
            }

            // 4. for each set in <3>
            for (std::map<int, std::set<ed_t>>::const_iterator iedge_sets_iter = iedge_sets.cbegin(); iedge_sets_iter != iedge_sets.cend(); ++iedge_sets_iter) {

                const std::set<ed_t>& iedge_set = iedge_sets_iter->second;

                // a. IF set has 3 or more edges
                // Note: The minimum possible number of interior halfedges necessary to
                // apply filtering is 3. ** think 2-teeth concave polygon where both teeth
                // are partially cut , as shown below:
                //   
                //    _     _
                //   / \   / \.
                //  /*-*--*-*\.
                // /    \/   \.
                //|          |  <-- two-teeth concave polygon
                //------------
                //
                //the asterisk represents intersection vertices

                bool apply_filtering = iedge_set.size() >= 3;

                if (apply_filtering) {

                    // b. apply filtering

                    // first, we gather all vertices used by iedges in the current set

                    std::vector<vd_t> iedge_set_vertices;

                    // TODO: we can use the already-known interrior edges from above!!
                    for (std::set<ed_t>::const_iterator iedge_set_iter = iedge_set.cbegin();
                         iedge_set_iter != iedge_set.cend();
                         ++iedge_set_iter) {

                        const vd_t iedge_v0 = m0.vertex(*iedge_set_iter, 0);
                        const vd_t iedge_v1 = m0.vertex(*iedge_set_iter, 1);
                        const bool v0_contained = std::find(iedge_set_vertices.cbegin(), iedge_set_vertices.cend(), iedge_v0) != iedge_set_vertices.cend();
                        const bool v1_contained = std::find(iedge_set_vertices.cbegin(), iedge_set_vertices.cend(), iedge_v1) != iedge_set_vertices.cend();

                        if (!v0_contained) {
                            iedge_set_vertices.push_back(iedge_v0);
                        }

                        if (!v1_contained) {
                            iedge_set_vertices.push_back(iedge_v1);
                        }
                    }

                    // Here, we check that there is at-least one of the intersection
                    // points (connected to by the edges in the set) that lies on a halfedge
                    // of the currently clipped face (ps_face). If this is false, then we
                    // can't do filtering. Alternative, we also check is exactly two such vertices
                    // lie on an edge of the clipped polygon
                    //
                    // This is necessary if the clipped face (ps_face) was intersected by
                    // another jagged/concave face

                    bool atleast_one_set_ivertex_on_ps_face_he = false;
                    int num_set_ivertices_on_a_ps_face_he = 0;
                    for (std::vector<vd_t>::const_iterator iedge_set_vertices_iter = iedge_set_vertices.cbegin();
                         iedge_set_vertices_iter != iedge_set_vertices.cend();
                         ++iedge_set_vertices_iter) {

                        const vd_t ivertex = *iedge_set_vertices_iter;
                        // get halfedge which intersected at ivertex
                        const hd_t coincident_ps_halfedge = m0_ivtx_to_ps_he.at(ivertex);
                        const ed_t coincident_ps_edge = ps.edge(coincident_ps_halfedge);
                        // halfedges of the clipped face
                        //std::vector<hd_t> ps_face_halfedges = get_halfedges_on_face(ps, ps_face);
                        std::vector<hd_t> ps_face_halfedges = ps.get_halfedges_around_face(ps_face);

                        // check if any halfedge of coincident_ps_edge belongs to ps_face
                        for (int i = 0; i < 2; ++i) {
                            // Note: an alternative solution here is to query the face and compare to ps_face i.e `ps.face(he) == ps_face`
                            hd_t he = ps.halfedge(coincident_ps_edge, i);
                            bool he_on_face = std::find(ps_face_halfedges.cbegin(), ps_face_halfedges.cend(), he) != ps_face_halfedges.cend();
                            if (he_on_face) {
                                num_set_ivertices_on_a_ps_face_he++;
                                if (!atleast_one_set_ivertex_on_ps_face_he) {
                                    atleast_one_set_ivertex_on_ps_face_he = true;
                                }
                                // break;
                            }
                        }
                        //if (atleast_one_set_ivertex_on_ps_face_he) {
                        //    break;
                        //}
                    }

                    if (!atleast_one_set_ivertex_on_ps_face_he || num_set_ivertices_on_a_ps_face_he == 2) {
                        // we cannot do filtering because
                        // 1) the edges in the set lie on/inside ps_face and without touching its bounding halfedges (e.g. tet vs tri complete cut)
                        // 2) the edges partition the current polygon into two parts (e.g. test 7)
                        continue;
                    }

                    // Now we are going to create a bin for each intersection point which connected
                    // to by an edge in the set. a bin contains the edges that are connected to the
                    // corresponding vertex.

                    std::map<vd_t, std::vector<ed_t>> iedge_set_vertex_to_iedge_set_edges; // bins

                    for (std::vector<vd_t>::const_iterator iedge_set_vertices_iter = iedge_set_vertices.cbegin();
                         iedge_set_vertices_iter != iedge_set_vertices.cend();
                         ++iedge_set_vertices_iter) {

                        const vd_t& bin_vertex = *iedge_set_vertices_iter;
                        std::pair<std::map<vd_t, std::vector<ed_t>>::const_iterator, bool> pair = iedge_set_vertex_to_iedge_set_edges.insert(std::make_pair(bin_vertex, std::vector<ed_t>()));

                        MCUT_ASSERT(pair.second == true);
                    }

                    // populate bins (if the vertex of bin is used by edge, then add edge to bin)

                    for (std::set<ed_t>::const_iterator iedge_set_iter = iedge_set.cbegin();
                         iedge_set_iter != iedge_set.cend();
                         ++iedge_set_iter) {

                        const ed_t& edge = (*iedge_set_iter);
                        const vd_t v0 = m0.vertex(edge, 0);
                        const vd_t v1 = m0.vertex(edge, 1);
                        const std::map<vd_t, std::vector<ed_t>>::iterator find_v0 = iedge_set_vertex_to_iedge_set_edges.find(v0);

                        MCUT_ASSERT(find_v0 != iedge_set_vertex_to_iedge_set_edges.cend());

                        const bool edge_exists_in_v0_bin = std::find(find_v0->second.cbegin(), find_v0->second.cend(), edge) != find_v0->second.cend();

                        MCUT_ASSERT(!edge_exists_in_v0_bin);

                        find_v0->second.push_back(edge);

                        const std::map<vd_t, std::vector<ed_t>>::iterator find_v1 = iedge_set_vertex_to_iedge_set_edges.find(v1);

                        MCUT_ASSERT(find_v1 != iedge_set_vertex_to_iedge_set_edges.cend());

                        const bool edge_exists_in_v1_bin = std::find(find_v1->second.cbegin(), find_v1->second.cend(), edge) != find_v1->second.cend();

                        MCUT_ASSERT(!edge_exists_in_v1_bin);

                        find_v1->second.push_back(edge);
                    }

                    // Now we are going to `sort` edges of the set into a sequence using the bins.
                    // Note that the sorting is not numerical but is akin to re-ordering the edges
                    // according to the vertices they connect

                    // First, we must find a bin (any) with 1 edge. This is a case where the vertex
                    // (of the bin) is the first or last vertex of the final sequence - a `terminal`
                    // vertex.

                    const int terminal_vertex_count = (int)std::count_if(
                        iedge_set_vertex_to_iedge_set_edges.cbegin(),
                        iedge_set_vertex_to_iedge_set_edges.cend(),
                        [&](const std::pair<vd_t, std::vector<ed_t>>& e) {
                            return e.second.size() == 1;
                        });

                    if (terminal_vertex_count == 0) {
                        // NOTE: this signifies an that we have encountered `floating patch`.
                        // A `floating patch` is arises when our edges in the set do not have a terminal
                        // point. Example: "ps_face" is a triangle of the cut-mesh whereby this
                        // triangle intersects a tetrahedron source-mesh to cut it in half. When a floating patch
                        // arises, we need to keep all of the edges of the sorted sequence.
                        // (see also below: when we seal connected components)
                        lg << "skip filtering : interior edge sequence forms loop (floating patch)." << std::endl;
                        continue;
                    }

                    MCUT_ASSERT(terminal_vertex_count == 2); // verify that there are exactly two vertices associated with only one edge each

                    // get any terminal vertex
                    std::map<vd_t, std::vector<ed_t>>::const_iterator first_vertex_of_sequence_iter = std::find_if(
                        iedge_set_vertex_to_iedge_set_edges.cbegin(),
                        iedge_set_vertex_to_iedge_set_edges.cend(),
                        [&](const std::pair<vd_t, std::vector<ed_t>>& e) {
                            return e.second.size() == 1;
                        });

                    MCUT_ASSERT(first_vertex_of_sequence_iter != iedge_set_vertex_to_iedge_set_edges.cend());

                    const ed_t first_edge_of_sequence = first_vertex_of_sequence_iter->second.front();
                    iedge_set_vertex_to_iedge_set_edges.erase(first_vertex_of_sequence_iter); // remove

                    // Now we are going to re-order/sort the sequence using the bins.
                    // O(n) since the do-while loop can iterate at most "n-1" times where n is the number of bins

                    std::vector<ed_t> iedge_set_sequence;
                    ed_t current_edge = mesh_t::null_edge();
                    ed_t next_edge = first_edge_of_sequence;

                    do {

                        current_edge = next_edge;
                        iedge_set_sequence.push_back(current_edge);
                        next_edge = mesh_t::null_edge();

                        // find next bin which contains the edge that can be connected to the current
                        std::map<vd_t, std::vector<ed_t>>::const_iterator next_edge_sequencing_histogram_bin_iter = std::find_if(
                            iedge_set_vertex_to_iedge_set_edges.cbegin(), iedge_set_vertex_to_iedge_set_edges.cend(),
                            [&](const std::pair<vd_t, std::vector<ed_t>>& e) {
                                const std::vector<ed_t>& bin_edges = e.second;
                                const std::vector<ed_t>::const_iterator f_iter = std::find(bin_edges.cbegin(), bin_edges.cend(), current_edge);
                                const bool vertex_bin_contains_edge = f_iter != bin_edges.cend();

                                return vertex_bin_contains_edge;
                            });

                        MCUT_ASSERT(next_edge_sequencing_histogram_bin_iter != iedge_set_vertex_to_iedge_set_edges.cend());

                        const std::vector<ed_t>& next_edge_sequencing_histogram_bin = next_edge_sequencing_histogram_bin_iter->second;
                        // set the next edge to the one which has not be used
                        next_edge = next_edge_sequencing_histogram_bin.front() == current_edge ? next_edge_sequencing_histogram_bin.back() : next_edge_sequencing_histogram_bin.front();
                        iedge_set_vertex_to_iedge_set_edges.erase(next_edge_sequencing_histogram_bin_iter); // remove (no longer needed)

                    } while (!iedge_set_vertex_to_iedge_set_edges.empty());

                    // dump
                    lg << "re-ordered edge sequence : ";
                    for (std::vector<ed_t>::const_iterator i = iedge_set_sequence.begin(); i != iedge_set_sequence.end(); ++i) {
                        lg << " <" << *i << ">";
                    }
                    lg << std::endl;

                    // Now that we have sorted the sequence, we can then systemically filter out
                    // the exterior interior-edges.
                    // It so happens that every second edge in the sorted sequence will be a exterior
                    // interior-edge :)

                    lg << "filtered exterior interior-iedges : ";

                    // for each edge in the sorted sequence (starting from the second)
                    for (int i = 1; i < (int)iedge_set_sequence.size(); i += 2) {

                        const ed_t& edge = iedge_set_sequence.at(i);
                        std::vector<ed_t>::const_iterator find_iter = std::find(incident_edges.cbegin(), incident_edges.cend(), edge);

                        MCUT_ASSERT(find_iter != incident_edges.cend()); // the exterior interior-edge should exist because it has not yet been filtered until now

                        lg << " <" << *find_iter << ">";

                        incident_edges.erase(find_iter); // remove exterior interior-iedge
                    }
                    lg << std::endl;

                } // end of edge filtering
            }

            // dump

            lg << "final edges on face = " << incident_edges.size() << std::endl;

            for (std::vector<ed_t>::const_iterator j = incident_edges.cbegin(); j != incident_edges.cend(); ++j) {
                lg << estr(m0, *j) << std::endl;
            }

            //
            // Now that we have the essential set of edges which describe the clipping, the next step
            // is to gather the halfedges on the clipped face from these edges.
            //
            // Note that the gathered set of halfedges will contain some halfedges which are redundant.
            // These redundant halfedges are those which lie on the exterior of the clipped polygon and
            // have a winding order which is opposite to input meshes (cut-mesh or source-mesh) i.e. cw
            // (clockwise) order. Thus, we need one more filtering step which will remove these redundant
            // halfedges from the gather set.

            lg << "gather exterior halfedges on face" << std::endl;

            std::vector<hd_t> incident_halfedges;

            // 1. find an exterior halfedge (any)
            hd_t first_exterior_halfedge = mesh_t::null_halfedge();

            // for each edge on clipped polygon, (i.e. from the filtered set)
            for (std::vector<ed_t>::const_iterator incident_edge_iter = incident_edges.cbegin();
                 incident_edge_iter != incident_edges.cend();
                 ++incident_edge_iter) {

                const int incident_edge_idx = (int)std::distance(incident_edges.cbegin(), incident_edge_iter);

                if (incident_edge_idx >= incident_exterior_edge_count) {
                    continue; // we only want exterior edge
                }

                const ed_t& edge = (*incident_edge_iter);

                for (int edge_he_iter = 0; edge_he_iter < 2; ++edge_he_iter) {

                    const hd_t m0_edge_he = m0.halfedge(edge, edge_he_iter);
                    const vd_t m0_edge_he_src = m0.source(m0_edge_he);
                    const vd_t m0_edge_he_tgt = m0.target(m0_edge_he);
                    const bool m0_edge_he_src_is_ivertex = m0_is_intersection_point(m0_edge_he_src, ps_vtx_cnt);
                    const bool m0_edge_he_tgt_is_ivertex = m0_is_intersection_point(m0_edge_he_tgt, ps_vtx_cnt);

                    if (!m0_edge_he_src_is_ivertex && !m0_edge_he_tgt_is_ivertex) { // o-->o

                        const vd_t ps_he_src = m0_to_ps_vtx.at(m0_edge_he_src);
                        const vd_t ps_he_tgt = m0_to_ps_vtx.at(m0_edge_he_tgt);
                        const hd_t ps_he = ps.halfedge(ps_he_src, ps_he_tgt);

                        if (ps_he == mesh_t::null_halfedge()) {
                            continue; // opposite of border halfedge
                        }

                        if (ps.face(ps_he) == ps_face) {
                            first_exterior_halfedge = m0_edge_he;
                            break;
                        }
                    } else { // x-->x o-->x x-->o

                        // o-->x : We want the ihalfedges which point into the sm whose tgt lays on the
                        // sm-face of tgt (they have an opposite direction wrt the face normal)
                        const bool is_ox = (!m0_edge_he_src_is_ivertex && m0_edge_he_tgt_is_ivertex);

                        if (is_ox) {

                            // get the incident ps-halfedge of tgt
                            hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0_edge_he_tgt);
                            //hd_t ps_halfedge_of_face = tgt_ps_h;
                            if (ps.face(tgt_ps_h) != ps_face) {
                                tgt_ps_h = ps.opposite(tgt_ps_h);
                                MCUT_ASSERT(tgt_ps_h != mesh_t::null_halfedge()); // must be true if the current face is exists.
                            }

                            const vd_t& m0_edge_he_src_as_ps_vertex = m0_to_ps_vtx.at(m0_edge_he_src);

                            if (m0_edge_he_src_as_ps_vertex == ps.source(tgt_ps_h)) { // is counter clock-wise halfedge
                                first_exterior_halfedge = m0_edge_he;
                                break;
                            }
                        } else {

                            const bool is_xx = m0_edge_he_src_is_ivertex && m0_edge_he_tgt_is_ivertex;

                            if (is_xx) { // exterior interior-iedge

                                const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0_edge_he_src);
                                const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0_edge_he_tgt);
                                const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                                const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                                const bool is_exterior_ih = (src_ps_edge == tgt_ps_edge);

                                if (!is_exterior_ih) {
                                    continue; // interior ihalfedges cannot be used as the first exterior ihalfedge
                                }

                                /*
                                    At this point, vertex information alone is insufficient to select the correct
                                    ihalfedge from the edge. This is because the two vertices are topologically equivalent 
                                    if just try to distinguish them following similar rules as other iedge types.
                                    To solve this problem, we must instead used on the connectivity information of 
                                    the polygon soup by using the incident ps-halfedge common to both vertices 
                                    (in their registry entries). The steps:

                                    1. get incident ps-halfedge incident to both src and tgt
                                    2. get m0 edges incident to the edge on <1>
                                    3. sort <2> with the first edge containing the source of <1>
                                    4. get the halfedge sequence in <3> where src of the first he is the src of <1> and the tgt of the last he is the tgt of <1>
                                    5. get the halfedge in <4> which is incident to the same ivertices as the current potential first-exterior halfedge  
                                    6. if the ps-face of <1> is the same as the current face
                                        7. set first polygon-exterior halfedge as <5> 
                                    8. else
                                        9. set first polygon-exterior halfedge as opposite of <5> 
                                */

                                hd_t ps_halfedge_of_face = tgt_ps_h;

                                if (ps.face(ps_halfedge_of_face) != ps_face) {
                                    ps_halfedge_of_face = ps.opposite(ps_halfedge_of_face);
                                    MCUT_ASSERT(ps_halfedge_of_face != mesh_t::null_halfedge()); // guarranteed to exist since we have a poly=exterior interior ihalfedge
                                }

                                const ed_t& incident_ps_edge = ps.edge(ps_halfedge_of_face);
                                std::map<ed_t, std::vector<ed_t>>::const_iterator ps_to_m0_edges_find_iter = ps_to_m0_edges.find(incident_ps_edge);
                                MCUT_ASSERT(ps_to_m0_edges_find_iter != ps_to_m0_edges.cend()); // because incident_ps_edge contains a polygon exterior interior ihalfedge

                                const std::vector<ed_t>& sorted_m0_edges = ps_to_m0_edges_find_iter->second;

                                // 4. get the halfedge sequence in <3> where src of the first he is the src of <1> and the tgt of the last he is the tgt of <1>
                                std::vector<hd_t> halfedge_sequence;

                                // add the first halfedge (its sourc emust be an original vertex)
                                const ed_t& first_e = sorted_m0_edges.front();
                                hd_t first_he = m0.halfedge(first_e, 0);
                                vd_t first_he_src = m0.source(first_he);

                                if (m0_is_intersection_point(first_he_src, ps_vtx_cnt)) {
                                    first_he = m0.halfedge(first_e, 1);
                                    first_he_src = m0.source(first_he);
                                    MCUT_ASSERT(!m0_is_intersection_point(first_he_src, ps_vtx_cnt)); // expect original vertex since halfedge edge is the first in sequence
                                }

                                halfedge_sequence.push_back(first_he);

                                // get the remaining halfedge of sequence
                                for (std::vector<ed_t>::const_iterator seq_edge_iter = sorted_m0_edges.cbegin(); seq_edge_iter != sorted_m0_edges.cend(); ++seq_edge_iter) {

                                    if (seq_edge_iter == sorted_m0_edges.cbegin()) {
                                        continue; // we have already added the first halfedge
                                    }

                                    const ed_t& e = *seq_edge_iter;
                                    const hd_t h0 = m0.halfedge(e, 0);

                                    if (m0.source(h0) == m0.target(halfedge_sequence.back())) {
                                        halfedge_sequence.push_back(h0);
                                    } else {
                                        const hd_t h1 = m0.halfedge(e, 1);
                                        halfedge_sequence.push_back(h1);
                                    }
                                }

                                MCUT_ASSERT(halfedge_sequence.size() == sorted_m0_edges.size());

                                const vd_t& first_he_src_as_ps_vertex = m0_to_ps_vtx.at(first_he_src); // first he of sequence

                                if (first_he_src_as_ps_vertex != ps.source(ps_halfedge_of_face)) {

                                    std::for_each(
                                        halfedge_sequence.begin(),
                                        halfedge_sequence.end(),
                                        [&](hd_t& he) {
                                            he = m0.opposite(he);
                                        }); // flip seq
                                }

                                // 5. get the halfedge in <4> which is incident to the same ivertices as the current potential first-exterior halfedge

                                std::vector<hd_t>::const_iterator matching_he_find_iter = std::find_if(
                                    halfedge_sequence.cbegin(),
                                    halfedge_sequence.cend(),
                                    [&](const hd_t& he) {
                                        const vd_t& he_src = m0.source(he);
                                        const vd_t& he_tgt = m0.target(he);

                                        return (he_src == m0_edge_he_src && he_tgt == m0_edge_he_tgt /*|| (he_src == m0_edge_he_tgt && he_tgt == m0_edge_he_src*/);
                                    });

                                if (matching_he_find_iter != halfedge_sequence.cend()) // does the potential halfedge actually point in the correct direction or not
                                {
                                    // 6. if the ps-face of <1> is the same as the current face
                                    //      7. set first polygon-exterior halfedge as <5>
                                    // 8. else
                                    //      9. set first polygon-exterior halfedge as opposite of <5>
                                    // if (ps.face(incident_ps_halfedge) == ps_face) {
                                    first_exterior_halfedge = *matching_he_find_iter;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (first_exterior_halfedge != mesh_t::null_halfedge()) {
                    break; // done
                }
            }

            MCUT_ASSERT(first_exterior_halfedge != mesh_t::null_halfedge());

            // Now that we halfedge a halfedge which lies on the exterior of the clipped polygon,
            // we will traverse/walk the clipped polygon's [exterior] to collect all other exterior halfedges
            // that share the same winding order as the input meshes (src-mesh and cut-mesh).

            hd_t current_exterior_halfedge = mesh_t::null_halfedge();
            hd_t next_exterior_halfedge = first_exterior_halfedge;

            do {
                lg.indent();

                current_exterior_halfedge = next_exterior_halfedge;
                incident_halfedges.push_back(current_exterior_halfedge);

                lg << hstr(m0, current_exterior_halfedge) << std::endl;

                const vd_t current_tgt = m0.target(current_exterior_halfedge);
                next_exterior_halfedge = mesh_t::null_halfedge(); // reset

                // find next halfedge from incident edges
                for (std::vector<ed_t>::const_iterator incident_edge_iter = incident_edges.cbegin(); incident_edge_iter != incident_edges.cend(); ++incident_edge_iter) {
                    const int incident_edge_idx = (int)std::distance(incident_edges.cbegin(), incident_edge_iter);
                    if (incident_edge_idx >= incident_exterior_edge_count) {
                        continue; // we only want exterior halfedge
                    }

                    const ed_t& edge = *incident_edge_iter;
                    bool edge_walked = std::find_if(
                                           incident_halfedges.cbegin(),
                                           incident_halfedges.cend(),
                                           [&](const hd_t& e) {
                                               const hd_t h0 = m0.halfedge(edge, 0);
                                               const hd_t h1 = m0.halfedge(edge, 1);
                                               return (e == h0 || e == h1);
                                           })
                        != incident_halfedges.cend();

                    if (edge_walked) {
                        continue; // skip edge is walked already
                    }

                    const vd_t v0 = m0.vertex(edge, 0);
                    const vd_t v1 = m0.vertex(edge, 1);

                    if (v0 == current_tgt || v1 == current_tgt) // check if connected to current (i.e. they share one vertex)
                    {
                        const bool v0_is_ivtx = m0_is_intersection_point(v0, ps_vtx_cnt);
                        const bool v1_is_ivtx = m0_is_intersection_point(v1, ps_vtx_cnt);
                        bool is_ambiguious_exterior_edge_case = v0_is_ivtx && v1_is_ivtx;
                        bool is_valid_ambiguious_exterior_edge = false;

                        if (is_ambiguious_exterior_edge_case) { // exterior edge with two intersection vertices (ambigious case arising from concave polyhedron cut)

                            const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v0);
                            const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v1);
                            const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                            const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);
                            is_valid_ambiguious_exterior_edge = (v0_ps_edge == v1_ps_edge); // see also above when gathering exterior incident edges
                        }

                        if (!is_ambiguious_exterior_edge_case || is_valid_ambiguious_exterior_edge) {
                            const hd_t h0 = m0.halfedge(edge, 0);
                            const hd_t h1 = m0.halfedge(edge, 1);

                            next_exterior_halfedge = h0;

                            if (m0.source(h0) != current_tgt) // h0 is facing the opposite dir
                            {
                                next_exterior_halfedge = h1;
                            }

                            //lg << "h=" << next_exterior_halfedge << " " << m0.source(next_exterior_halfedge) << ", " << m0.target(next_exterior_halfedge) << std::endl;

                            break; // found
                        }
                    }
                }
                lg.unindent();
            } while (next_exterior_halfedge != mesh_t::null_halfedge() /*first_exterior_halfedge*/);

            MCUT_ASSERT(incident_halfedges.size() >= 3); // minumum i.e. for a triangles!

            // Note: at this stage we have gathered all of the [exterior] halfedges needed to traced child polygons

            const int exterior_halfedge_count = (int)incident_halfedges.size();

            lg << "exterior halfedges on face = " << exterior_halfedge_count << std::endl;

            MCUT_ASSERT(exterior_halfedge_count == incident_exterior_edge_count);

            // Now we going to also gather interior halfedges (those defined only by intersection points
            // where the src and tgt vertex do not share the same incident ihalfedge in their registry entry.

            for (std::vector<ed_t>::const_iterator incident_edge_iter = incident_edges.cbegin(); incident_edge_iter != incident_edges.cend(); ++incident_edge_iter) {

                const ed_t& edge = (*incident_edge_iter);
                const vd_t v0 = m0.vertex(edge, 0);
                const vd_t v1 = m0.vertex(edge, 1);

                const bool v0_is_ivtx = m0_is_intersection_point(v0, ps_vtx_cnt);
                const bool v1_is_ivtx = m0_is_intersection_point(v1, ps_vtx_cnt);

                const hd_t h0 = m0.halfedge(edge, 0);
                const hd_t h1 = m0.halfedge(edge, 1);

                if (v0_is_ivtx && v1_is_ivtx) {

                    const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v0);
                    const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v1);
                    const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                    const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);
                    const bool is_valid_interior_edge = (v0_ps_edge != v1_ps_edge); // NOTE: by construction, if this condition is false then "edge" is exterior

                    if (is_valid_interior_edge) {
                        //MCUT_ASSERT(v0_ps_edge != edge);

                        incident_halfedges.push_back(h0);
                        incident_halfedges.push_back(h1);
                    }
                }
            }

            // dump
            if (input.verbose) {
                lg << "halfedges on face : ";
                for (std::vector<hd_t>::const_iterator j = incident_halfedges.cbegin(); j != incident_halfedges.cend(); ++j) {
                    lg << " <" << *j << ">";
                }
                lg << std::endl;
            }

            // Note: at this stage, we have all the halfedges that we need to trace child polygons.
            // Thus, the next step is tracing

            //
            // Trace child polygons on face to clip it
            //

            std::vector<hd_t> incident_halfedges_to_be_walked(incident_halfedges.cbegin(), incident_halfedges.cend()); // dirty copy

            do { // each iteration traces a child polygon

                traced_polygon_t child_polygon;
                hd_t current_halfedge = mesh_t::null_halfedge();
                hd_t next_halfedge = incident_halfedges_to_be_walked.front(); // can be any halfedge in vector

                MCUT_ASSERT(incident_halfedges_to_be_walked.size() >= 2);

                lg << "polygon " << m0_polygons.size() + child_polygons.size() << std::endl;

                bool is_valid_polygon = false;
                do { // each iteration walks a halfedge to incremetally trace a child polygon
                    lg.indent();

                    // 1. update state
                    current_halfedge = next_halfedge;

                    lg << hstr(m0, current_halfedge) << std::endl;

                    child_polygon.push_back(current_halfedge);
                    const vd_t current_halfedge_target = m0.target(current_halfedge);
                    next_halfedge = mesh_t::null_halfedge(); // reset
                    // remove next halfedge so that we dont walk it again
                    std::vector<hd_t>::const_iterator find_iter = std::find(incident_halfedges_to_be_walked.cbegin(), incident_halfedges_to_be_walked.cend(), current_halfedge);

                    MCUT_ASSERT(find_iter != incident_halfedges_to_be_walked.cend());

                    incident_halfedges_to_be_walked.erase(find_iter); // remove

                    if (child_polygon.size() >= 3) { // minimum halfedge count (triangle)

                        // source of first halfedge is target of last
                        if (m0.source(child_polygon.front()) == m0.target(child_polygon.back())) {

                            // the current halfedge is [not] the opposite of the first halfedge in list
                            // This is an important edge case for when you walk a halfedge connecting two vertices not in alpha (intersection).
                            // Example: case of tracing a polygon analogous to a cheek slash.
                            if (current_halfedge != m0.opposite(child_polygon.front())) {
                                is_valid_polygon = true;
                                lg.unindent();
                                break;
                            } else { // ... if the current halfedge is the opposite of the first halfedge in list

                                // peak forward to see what the next halfedge will be (if the next halfedge is in "child_polygon" then we are done)
                                std::vector<hd_t> premptive_candidate_halfedges;
                                for (std::vector<hd_t>::const_iterator incident_halfedges_to_be_walked_iter = incident_halfedges_to_be_walked.cbegin(); incident_halfedges_to_be_walked_iter != incident_halfedges_to_be_walked.cend(); ++incident_halfedges_to_be_walked_iter) {
                                    const hd_t& potential_candidate = *incident_halfedges_to_be_walked_iter;
                                    if (m0.source(potential_candidate) == current_halfedge_target) {
                                        premptive_candidate_halfedges.push_back(potential_candidate);
                                    }
                                }

                                hd_t prime_candidate = mesh_t::null_halfedge();
                                if (premptive_candidate_halfedges.size() == 2) {

                                    const std::vector<hd_t>::const_iterator current_halfedge_find_iter = std::find(incident_halfedges.cbegin(), incident_halfedges.cend(), current_halfedge);
                                    MCUT_ASSERT(current_halfedge_find_iter != incident_halfedges.cend());
                                    const bool current_halfedge_is_exterior = std::distance(incident_halfedges.cbegin(), current_halfedge_find_iter) < exterior_halfedge_count;

                                    if (current_halfedge_is_exterior) {

                                        // pick interior candidate
                                        const std::vector<hd_t>::const_iterator prime_candidate_find_iter = std::find(incident_halfedges.cbegin(), incident_halfedges.cend(), current_halfedge);
                                        MCUT_ASSERT(prime_candidate_find_iter != incident_halfedges.cend());
                                        const bool prime_candidate_is_exterior = std::distance(incident_halfedges.cbegin(), prime_candidate_find_iter) < exterior_halfedge_count;

                                        if (prime_candidate_is_exterior) {
                                            prime_candidate = premptive_candidate_halfedges.back(); // select correct interior halfedge
                                        }
                                    } else { // interior

                                        // pick non-opposite
                                        const bool prime_candidate_is_opposite = m0.opposite(current_halfedge) == prime_candidate;

                                        if (prime_candidate_is_opposite) {
                                            prime_candidate = premptive_candidate_halfedges.back(); // select correct non-opposite halfedge
                                        }
                                    }
                                }

                                const hd_t premptive_next = prime_candidate;

                                MCUT_ASSERT(premptive_candidate_halfedges.size() <= 2);

                                if (std::find(child_polygon.cbegin(), child_polygon.cend(), premptive_next) != child_polygon.cend()) {
                                    is_valid_polygon = true;
                                    lg.unindent();
                                    break;
                                }
                            }
                        }
                    }

                    // 2. find next halfedge

                    // 2.1. get candidates (halfedges whose source vertex is the target of current)
                    std::vector<hd_t> candidate_halfedges;
                    candidate_halfedges.reserve(2); // two candidates at most because we filtered out exterior interior-halfedges as well as clockwise (cw) halfedge

                    for (std::vector<hd_t>::const_iterator incident_halfedges_to_be_walked_iter = incident_halfedges_to_be_walked.cbegin(); incident_halfedges_to_be_walked_iter != incident_halfedges_to_be_walked.cend(); ++incident_halfedges_to_be_walked_iter) {
                        const hd_t& potential_candidate = *incident_halfedges_to_be_walked_iter;
                        if (m0.source(potential_candidate) == current_halfedge_target) {
                            candidate_halfedges.push_back(potential_candidate);
                        }
                    }

                    MCUT_ASSERT(candidate_halfedges.size() <= 2);

                    // 2.2. select prime candidate
                    hd_t prime_candidate = mesh_t::null_halfedge();

                    if (!candidate_halfedges.empty()) {
                        prime_candidate = candidate_halfedges.front(); // assuming: candidate_halfedges.size() == 1
                    }

                    if (candidate_halfedges.size() == 2) {

                        const std::vector<hd_t>::const_iterator current_halfedge_find_iter = std::find(incident_halfedges.cbegin(), incident_halfedges.cend(), current_halfedge);
                        MCUT_ASSERT(current_halfedge_find_iter != incident_halfedges.cend());
                        const bool current_halfedge_is_exterior = std::distance(incident_halfedges.cbegin(), current_halfedge_find_iter) < exterior_halfedge_count;

                        if (current_halfedge_is_exterior) {

                            // pick interior candidate
                            const std::vector<hd_t>::const_iterator prime_candidate_find_iter = std::find(incident_halfedges.cbegin(), incident_halfedges.cend(), current_halfedge);
                            MCUT_ASSERT(prime_candidate_find_iter != incident_halfedges.cend());
                            const bool prime_candidate_is_exterior = std::distance(incident_halfedges.cbegin(), prime_candidate_find_iter) < exterior_halfedge_count;

                            if (prime_candidate_is_exterior) {
                                prime_candidate = candidate_halfedges.back(); // select correct interior halfedge
                            }
                        } else { // interior

                            // pick non-opposite
                            const bool prime_candidate_is_opposite = m0.opposite(current_halfedge) == prime_candidate;

                            if (prime_candidate_is_opposite) {
                                prime_candidate = candidate_halfedges.back(); // select correct non-opposite halfedge
                            }
                        }
                    }

                    next_halfedge = prime_candidate;
                    lg.unindent();
                } while (next_halfedge != mesh_t::null_halfedge());

                //MCUT_ASSERT(is_valid_polygon);

                if (is_valid_polygon) {
                    lg << "valid" << std::endl;
                    child_polygons.emplace_back(child_polygon);
                }

            } while (!incident_halfedges_to_be_walked.empty());
        } // if (!is_iface) {

        lg << "traced polygons on face = " << child_polygons.size() << std::endl;

        m0_polygons.insert(m0_polygons.end(), child_polygons.cbegin(), child_polygons.cend());

        if (!is_from_cut_mesh /*!ps_is_cutmesh_face(ps_face, sm_face_count)*/) {
            traced_sm_polygon_count += (int)child_polygons.size();
        }

        lg.unindent();
        lg.unindent();
    } // for each ps-face to trace

    m0_ivtx_to_ps_faces.clear(); // free
    ps_iface_to_m0_edge_list.clear(); 
    ps_to_m0_edges.clear();
    ps_to_m0_non_intersecting_edge.clear();
    ps_iface_to_ivtx_list.clear();

    // Note: at this stage, we have traced all polygons. This means that any intersecting face in the polygon
    // soup will also now have been clipped.
    //
    // The connectivity of all traced polygons are represented a lists of halfedges for each
    // traced polygon. The halfedge data structure (i.e. "m0") still holds the underlying mesh data
    // over-which we are abstracting the connectivity i.e. it stores vertices (like intersection
    // points), edges, and halfeges.
    //
    // The lists of halfedges that we are using to represent the traced polygons avoid "2-manifold restrictions".
    // Storing the traced polygons inside a halfedge data structure is not possible because we would violate the
    // prinicipal rule that an edge must be incident to at most 2 faces (2-manifold surface mesh rule).
    //
    // There is a other benefit to using lists: it makes for a more logical implementation for the remainder of the
    // cutting algorithm i.e when duplicating intersection points, creating cut-mesh patches, stitching (hole
    // filling), and more.

    lg << "traced polygons = " << m0_polygons.size() << " (source-mesh = " << traced_sm_polygon_count << ")" << std::endl;

    MCUT_ASSERT((int)m0_polygons.size() >= ps.number_of_faces());

    const std::vector<traced_polygon_t>::iterator traced_sm_polygons_iter_end = m0_polygons.begin() + traced_sm_polygon_count;
    //const std::vector<traced_polygon_t>::iterator& traced_cs_polygons_iter_begin = traced_sm_polygons_iter_end;
    const std::vector<traced_polygon_t>::const_iterator m0_traced_sm_polygons_iter_cend = m0_polygons.cbegin() + traced_sm_polygon_count;
    const std::vector<traced_polygon_t>::const_iterator& traced_cs_polygons_iter_cbegin = traced_sm_polygons_iter_end;

    // extract the seam vertices
    std::map<vd_t, bool> m0_vertex_to_seam_flag;
    mark_seam_vertices(m0_vertex_to_seam_flag, m0, ps.number_of_vertices());

    MCUT_ASSERT(!m0_vertex_to_seam_flag.empty());

    ///////////////////////////////////////////////////////////////////////////
    // Dump meshes for src-mesh and cut-mesh using the traced polygons
    ///////////////////////////////////////////////////////////////////////////

    //
    // NOTE: we cannot always create meshes using the traced polygons because of
    // a violation of the surface mesh contruction rules. Basically, we cannot
    // reference a halfedge and its opposite in the same face because it violates
    // halfedge construction rules (2-manifold surface mesh). This issue occurs
    // whenever ps polygon is partially cut.
    //
    // Thus, we will only dump meshes if can gaurranteed not to violate halfedge
    // mesh rules (which can crash the program).
    //

    lg << "dump traced-polygons if possible" << std::endl;

    // dump traced src-mesh polygons.

    // dump traced polygons only if the cut paths are circular or complete linear cuts (prevents us
    // from violating halfedge construction rules, which would happen otherwise)
    bool all_cutpaths_are_circular = (num_explicit_circular_cutpaths == num_explicit_cutpath_sequences);
    bool all_cutpaths_linear_and_without_making_holes = (num_explicit_circular_cutpaths == 0) && ((int)explicit_cutpaths_severing_srcmesh.size() == num_explicit_linear_cutpaths);

    if (cs_is_watertight || (all_cutpaths_are_circular || all_cutpaths_linear_and_without_making_holes)) {

        std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>> separated_src_mesh_fragments;

        // NOTE: The result is a mesh identical to the original except at the edges introduced by the cut..
        extract_connected_components(
            separated_src_mesh_fragments,
            m0,
            std::vector<traced_polygon_t>(m0_polygons.begin(), traced_sm_polygons_iter_end),
            std::vector<int>(), // sm_polygons_below_cs
            std::vector<int>(), // sm_polygons_above_cs
            m0_vertex_to_seam_flag);

        MCUT_ASSERT(separated_src_mesh_fragments.size() == 1); // one cc
        MCUT_ASSERT(separated_src_mesh_fragments.cbegin()->second.size() == 1); // one instance
        output.seamed_src_mesh.mesh = std::move(separated_src_mesh_fragments.begin()->second.front().first);
        output.seamed_src_mesh.seam_vertices = std::move(separated_src_mesh_fragments.begin()->second.front().second.seam_vertices);

        if (input.verbose) {
            dump_mesh(output.seamed_src_mesh.mesh, "src-mesh-traced-poly");
        }
    }

    // dump traced cut-mesh polygons

    bool all_cutpaths_linear_and_make_holes = (num_explicit_circular_cutpaths == 0) && (explicit_cutpaths_severing_srcmesh.size() == 0);

    if (sm_is_watertight || (all_cutpaths_are_circular || all_cutpaths_linear_and_make_holes)) {
        std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>> separated_cut_mesh_fragments;

        mesh_t merged = extract_connected_components(
            separated_cut_mesh_fragments,
            m0,
            std::vector<traced_polygon_t>(traced_cs_polygons_iter_cbegin, m0_polygons.cend()),
            std::vector<int>(),
            std::vector<int>(),
            m0_vertex_to_seam_flag);

        if (separated_cut_mesh_fragments.size() == 1) { // usual case
            MCUT_ASSERT(separated_cut_mesh_fragments.cbegin()->second.size() == 1); // one instance
            output.seamed_cut_mesh.mesh = std::move(separated_cut_mesh_fragments.begin()->second.front().first);
            output.seamed_cut_mesh.seam_vertices = std::move(separated_cut_mesh_fragments.begin()->second.front().second.seam_vertices);
        } else // cutmesh is a single (large) polygon [and] we have multiple patches
        {
            // here we create a new mesh containing only the polygons of the cut-mesh [and] where every vertex is referenced
            // by a face (i.e. we remove those vertices used only by the source mesh in "m0")
            mesh_t m; // output (seamed connected component)
            std::vector<vd_t> remapped_seam_vertices;
            std::map<vd_t, vd_t> vmap_mesh_vertices;
            for (mesh_t::face_iterator_t f = merged.faces_begin(); f != merged.faces_end(); ++f) {
                const std::vector<vd_t> vertices_around_face = merged.get_vertices_around_face(*f);
                std::vector<vd_t> remapped_face;
                for (std::vector<vd_t>::const_iterator v = vertices_around_face.cbegin(); v != vertices_around_face.cend(); ++v) {
                    if (vmap_mesh_vertices.count(*v) == 0) { // not registered
                        vmap_mesh_vertices[*v] = m.add_vertex(merged.vertex(*v)); // add vertex and save it new descriptor

                        if (m0_vertex_to_seam_flag.at(*v) == true) { // is it a seam vertex? (i.e. on a cutpath)
                            remapped_seam_vertices.push_back(vmap_mesh_vertices[*v]);
                        }
                    }
                    remapped_face.push_back(vmap_mesh_vertices[*v]);
                }

                fd_t fd = m.add_face(remapped_face);
                MCUT_ASSERT(fd != mesh_t::null_face());
            }

            output.seamed_cut_mesh.mesh = std::move(m);
            output.seamed_cut_mesh.seam_vertices = std::move(remapped_seam_vertices);
        }

        if (input.verbose) {
            dump_mesh(output.seamed_src_mesh.mesh, "cut-mesh-traced-poly");
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // Map each halfedge to the traced polygons that use it
    ///////////////////////////////////////////////////////////////////////////

    lg << "map halfedges to polygons" << std::endl;

    // We now need to manually maintain halfedge incidence (i.e. "used-by") information since
    // the traced-polygon connectivity is not stored inside of a halfedge mesh data structure.
    // The halfedge data structure would normally store such incidence information for us but this is
    // no longer possible for reasons mentioned above (see long comment after tracing loop).
    //
    // So the first incidence information that we need to keep around is the mapping from every
    // halfedge (in "m0") which is used to trace a polygon, to the traced polygon(s) that use
    // that halfedge. Thus, halfedges which are not used for tracing [at all] do not have an entry
    // in this map. We will use this information later, like to stitch cut-mesh patches to src-mesh
    // fragments.

    // key=halfedge used to traced-polygon face; value=index of traced polygon which is coincident to halfedge <key>
    std::map<hd_t, std::vector<int>> m0_h_to_ply;

    // for each traced polygon
    for (std::vector<traced_polygon_t>::const_iterator traced_polygon_iter = m0_polygons.cbegin();
         traced_polygon_iter != m0_polygons.cend();
         ++traced_polygon_iter) {
        const traced_polygon_t& traced_polygon = *traced_polygon_iter;
        const int traced_polygon_index = (int)std::distance(m0_polygons.cbegin(), traced_polygon_iter);

        lg << "polygon = " << traced_polygon_index << " :";

        // for each halfedge in polygon
        for (traced_polygon_t::const_iterator traced_polygon_halfedge_iter = traced_polygon.cbegin();
             traced_polygon_halfedge_iter != traced_polygon.cend();
             ++traced_polygon_halfedge_iter) {

            const hd_t& traced_polygon_halfedge = *traced_polygon_halfedge_iter;

            lg << " " << hstr(m0, traced_polygon_halfedge);

            std::pair<std::map<hd_t, std::vector<int>>::iterator, bool> pair = m0_h_to_ply.insert(std::make_pair(traced_polygon_halfedge, std::vector<int>()));

            if (pair.second == false) // element exists (m0 halfedges (only interior ihalfedges) can be reused by more than one polygon. upto two polygons!)
            {
                MCUT_ASSERT(!pair.first->second.empty());
                MCUT_ASSERT(std::find(pair.first->second.cbegin(), pair.first->second.cend(), traced_polygon_index) == pair.first->second.cend());
            }

            pair.first->second.push_back(traced_polygon_index);
            MCUT_ASSERT(pair.first->second.size() <= 2);
        }
        lg << std::endl;
    }

#if 0
    // dump
    for (std::map<hd_t, std::vector<int>>::const_iterator i = m0_h_to_ply.cbegin(); i != m0_h_to_ply.cend(); ++i) {
        lg << "halfedge " << i->first << ": ";
        for (std::vector<int>::const_iterator j = i->second.cbegin(); j != i->second.cend(); ++j) {
            lg.indent();
            lg << " <" << *j << ">";
            lg.unindent();
        }
        lg << std::endl;
    }
#endif

    // bool all_cutpaths_make_holes = ((int)explicit_cutpaths_making_holes.size() == num_explicit_cutpath_sequences);

    ///////////////////////////////////////////////////////////////////////////
    // Find all cut-mesh polygons which are "exterior" relative to the src-mesh
    ///////////////////////////////////////////////////////////////////////////

    // Here we will explicitly find a subset of the traced cut-mesh polygons which lie
    // outside/exterior w.r.t the src-mesh. We find these polygons using the
    // "re-entrant" vertices that where identified while calculating intersection
    // points. These will be used (later) to mark cut-mesh patches as either interior
    // or exterior w.r.t the src-mesh.
    //
    // Note that these polygons will be the new "child polygons" which are new as a result of
    // the intersection. Bare in mind that not all such child polygon can be found due to
    // the order-dependant nature of doing halfedge-polygon intersection tests earlier. But
    // the subset we find is sufficient for us in later tasks

    // An element here represents the index of an exterior cut-mesh polygon, and the index of
    // halfedge which touches the src-mesh and points torward the interior (inside) of the src-mesh.
    std::vector<std::pair<int /*poly*/, int /*he idx*/>> known_exterior_cs_polygons;

    //if (sm_is_watertight /*no holes*/ || all_cut_paths_form_loops /*i.e. complete cut*/) { // TODO: this will need update once we have fixed the "all_cut_paths_form_loops" problem above
    if (explicit_cutpaths_making_holes.size() > 0) { // atleast one cut-path makes a hole to be sealed later
        lg << "find known exterior cut-mesh polygons" << std::endl;

        // for each traced cut-mesh polygon
        for (std::vector<traced_polygon_t>::const_iterator cs_poly_iter = traced_cs_polygons_iter_cbegin;
             cs_poly_iter != m0_polygons.cend();
             ++cs_poly_iter) {

            const traced_polygon_t& cs_poly = *cs_poly_iter;
            const int cs_poly_idx = (int)std::distance(m0_polygons.cbegin(), cs_poly_iter);

            // for each halfedge of polygon
            for (traced_polygon_t::const_iterator cs_poly_he_iter = cs_poly.cbegin();
                 cs_poly_he_iter != cs_poly.cend();
                 ++cs_poly_he_iter) {

                // we want to use class-1 ihalfedges : o-->x. This type of halfedge was the
                // one used to calculate re-entrant vertices
                const hd_t& cs_poly_he = *cs_poly_he_iter;
                const vd_t cs_poly_he_src = m0.source(cs_poly_he);
                const vd_t cs_poly_he_tgt = m0.target(cs_poly_he);
                const bool tgt_is_ivertex = m0_is_intersection_point(cs_poly_he_tgt, ps_vtx_cnt);
                const bool src_is_ivertex = m0_is_intersection_point(cs_poly_he_src, ps_vtx_cnt);

                if (!tgt_is_ivertex) {
                    continue; // either class-0 or class-2
                }

                // check that the target vertex is along a cut-path making a hole
                const int tgt_explicit_cutpath_sequence_idx = m0_ivtx_to_explicit_cutpath_sequence.at(cs_poly_he_tgt);
                bool cutpath_makes_a_hole = std::find(explicit_cutpaths_making_holes.cbegin(),
                                                explicit_cutpaths_making_holes.cend(),
                                                tgt_explicit_cutpath_sequence_idx)
                    != explicit_cutpaths_making_holes.cend();

                if (cutpath_makes_a_hole == false) {
                    // skip because the patch of the curent polygon will not be used
                    // for sealing/stitching holes. Thus, there is no need to tag the
                    // polygon as being either interior or exterior. That is, its
                    // adjacent cutpath does not make a hole!
                    continue;
                }

                // get the intersection info which was calculated earlier (src-mesh normal vector, and dot product)
                const std::map<vd_t, std::pair<math::vec3, math::real_number_t>>::const_iterator cs_regular_reentrant_ivertices_find_iter = cs_reg_reentrant_ivtx_list.find(cs_poly_he_tgt);
                const bool tgt_is_regular_reentrant_vertex = cs_regular_reentrant_ivertices_find_iter != cs_reg_reentrant_ivtx_list.cend();
                std::vector<vd_t>::const_iterator tip_reentrant_vertex_find_iter = std::find(cs_tip_reentrant_ivtx_list.cbegin(), cs_tip_reentrant_ivtx_list.cend(), cs_poly_he_tgt);
                const bool tgt_is_tip_reentrant_vertex = tip_reentrant_vertex_find_iter != cs_tip_reentrant_ivtx_list.cend();

                MCUT_ASSERT(!(tgt_is_regular_reentrant_vertex && tgt_is_tip_reentrant_vertex)); // a re-entrant vertex cannot be both a tip and regular

                if (!tgt_is_regular_reentrant_vertex && !tgt_is_tip_reentrant_vertex) {
                    continue; // cs_poly_he_tgt is an ivertex but it is not a regular re-entrant vertex ( was not saved as one)
                }

                // o-->x : We want the ihalfedges which point "into" the src-mesh, i.e. whose tgt is on
                // the src-mesh face of tgt (found in the registry entry). This implies that the current
                // cut-mesh halfedge must have an opposite direction w.r.t the normal of the src-mesh face.
                const bool is_ox = (!src_is_ivertex && tgt_is_ivertex);
                bool is_exterior_ih = false; // i.e. is and intersecting halfedge

                if (src_is_ivertex && tgt_is_ivertex) {
                    const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0.source(cs_poly_he));
                    const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0.target(cs_poly_he));
                    const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                    const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                    is_exterior_ih = (src_ps_edge == tgt_ps_edge);
                }

                if (!(is_ox || is_exterior_ih)) {
                    continue; // is interior-ihalfedge
                }

                // if tgt is a tip re-entrant vertex then the polygon is (by definition) exterior
                // given what constitutes an tip re-entrant vertex. Basically, this is due to the
                // type of halfedge that we are looking for and the fact that tip re-entrant vertices
                // occur only on the border of the cut-mesh.
                bool is_exterior_polygon = tgt_is_tip_reentrant_vertex;

                if (!is_exterior_polygon && tgt_is_regular_reentrant_vertex) {
                    // Re-calculate the geometry operation as as we did before we calculated the
                    // tgt-ivertex (i.e. with scalar product) using the halfedge's src and tgt
                    // coordinates and the normal of the face which was intersected to produce
                    // the tgt vertex.
                    const std::pair<math::vec3, math::real_number_t>& geometric_data = cs_regular_reentrant_ivertices_find_iter->second;
                    const math::vec3& polygon_normal = geometric_data.first; // src-mesh face normal
                    const math::real_number_t& orig_scalar_prod = geometric_data.second; // the dot product result we computed earlier

                    // the original ps-halfedge was "incoming" (pointing inwards) and gave a
                    // negative scalar-product with the src-mesh face normal.
                    MCUT_ASSERT(math::sign(orig_scalar_prod) == math::NEGATIVE);

                    // calculate the vector represented by the current halfedge
                    const math::vec3 cs_poly_he_vector = m0.vertex(cs_poly_he_tgt) - m0.vertex(cs_poly_he_src);
                    // calculate dot product with the src-mesh normal
                    const math::real_number_t scalar_prod = math::dot_product(polygon_normal, cs_poly_he_vector);
                    // check that it is the same
                    // Note: we want the same sign (i.e. cs_poly_he_vector has negative scalar-product)
                    // because we want the class-1 ihalfedge which is exterior but points inside the src-mesh
                    is_exterior_polygon = (math::sign(scalar_prod) == math::sign(orig_scalar_prod));
                }

                if (is_exterior_polygon) { // the current halfedge passed the sign test
                    known_exterior_cs_polygons.push_back(
                        std::make_pair(cs_poly_idx, (int)std::distance(cs_poly.cbegin(), cs_poly_he_iter)));
                }
            }
        }
    }

    cs_reg_reentrant_ivtx_list.clear(); // free
    m0_ivtx_to_explicit_cutpath_sequence.clear();

    ///////////////////////////////////////////////////////////////////////////
    // Find the src-mesh polygons (next to cut) which are above and below
    ///////////////////////////////////////////////////////////////////////////

    //
    // We are search through all of the traced src-mesh polygons to find those
    // which adjacent to the cut path. We then identify them as being either
    // "above" or "below" the cut-mesh which we do using the src-mesh re-entrant
    // vertices.
    //

    std::vector<int> sm_polygons_below_cs;
    std::vector<int> sm_polygons_above_cs;

    // for each traced src-mesh polygon
    for (std::vector<traced_polygon_t>::const_iterator sm_poly_iter = m0_polygons.cbegin();
         sm_poly_iter != traced_sm_polygons_iter_end;
         ++sm_poly_iter) {
        const traced_polygon_t& sm_poly = *sm_poly_iter;
        const int sm_poly_idx = (int)std::distance(m0_polygons.cbegin(), sm_poly_iter);

        // for each halfedge of polygon
        for (traced_polygon_t::const_iterator sm_poly_he_iter = sm_poly.cbegin();
             sm_poly_he_iter != sm_poly.cend();
             ++sm_poly_he_iter) {

            const hd_t& sm_poly_he = *sm_poly_he_iter; // we want class-1 ihalfedges : o-->x
            const vd_t sm_poly_he_src = m0.source(sm_poly_he);
            const vd_t sm_poly_he_tgt = m0.target(sm_poly_he);
            const bool tgt_is_ivertex = m0_is_intersection_point(sm_poly_he_tgt, ps_vtx_cnt);
            const bool src_is_ivertex = m0_is_intersection_point(sm_poly_he_src, ps_vtx_cnt);

            if (!tgt_is_ivertex) {
                continue; // either class-0 (o-->o) or class-2 (x-->o)
            }

            const std::map<vd_t, std::pair<math::vec3, math::real_number_t>>::const_iterator sm_regular_reentrant_ivertices_find_iter = sm_reg_reentrant_ivtx_list.find(sm_poly_he_tgt);
            const bool tgt_is_sm_regular_reentrant_vertex = sm_regular_reentrant_ivertices_find_iter != sm_reg_reentrant_ivtx_list.cend();

            // NOTE: we do not need src-mesh tip re-entrant vertices because they are not useful for the
            // determining whether src-mesh faces are above or below the cut-mesh. The notion
            // of above or below is defined only for the src-mesh w.r.t. the cut-mesh. The is because
            // we are only interested in manipulating (cutting/partitioning) the src-mesh and not the cut-mesh
            //
            // PERSONAL NOTE (TODO?): it may be okay to enable this code. I think it would allow us
            // to avoid the edge case arising when the src-mesh has only one face (see below). --> "if (sm_polygons_above_cs.empty() && sm_polygons_below_cs.empty())"

            //std::vector<vd_t>::const_iterator sm_tip_reentrant_vertex_find_iter = std::find(sm_tip_reentrant_ivertices.cbegin(), sm_tip_reentrant_ivertices.cend(), sm_poly_he_tgt);
            //const bool tgt_is_sm_tip_reentrant_vertex = sm_tip_reentrant_vertex_find_iter != sm_tip_reentrant_ivertices.cend();
            //MCUT_ASSERT(!(tgt_is_sm_regular_reentrant_vertex && tgt_is_sm_tip_reentrant_vertex)); // a re-entrant vertex cannot be both a tip and regular

            if (!tgt_is_sm_regular_reentrant_vertex) {
                continue; // cs_poly_he_tgt is an ivertex but it is not a regular re-entrant vertex
            }

            // o-->x : We want the ihalfedges which point into the cut-mesh whose tgt lays on the cut-mesh face of tgt
            // (they have an opposite direction wrt the face normal)
            const bool is_ox = (!src_is_ivertex && tgt_is_ivertex);

            bool is_exterior_ih = false;

            if (src_is_ivertex && tgt_is_ivertex) {
                const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(sm_poly_he_src);
                const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(sm_poly_he_tgt);
                const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                is_exterior_ih = (src_ps_edge == tgt_ps_edge);
            }

            if (!(is_ox || is_exterior_ih)) {
                continue;
            }

            // Re-calculate the exact same geometry operation as the time we calculated
            // the tgt-ivertex (scalar product using the halfedge's src and tgt coordinates
            // and and the normal of the cut-mesh face that was intersected to produce
            // the tgt vertex).
            const std::pair<math::vec3, math::real_number_t>& geometric_data = sm_regular_reentrant_ivertices_find_iter->second;
            const math::vec3& polygon_normal = geometric_data.first;
            const math::real_number_t& orig_scalar_prod = geometric_data.second;

            // Again, the notion of exterior is denoted by a negative dot-product.
            // Original ps-halfedge was "incoming" and gave a negative scalar-product
            // with the cut-mesh face normal.
            MCUT_ASSERT(math::sign(orig_scalar_prod) == math::NEGATIVE);

            const math::vec3 sm_poly_he_vector = m0.vertex(sm_poly_he_tgt) - m0.vertex(sm_poly_he_src);
            const math::real_number_t scalar_prod = math::dot_product(polygon_normal, sm_poly_he_vector);

            // We want the same sign (i.e. cs_poly_he_vector has negative scalar-product) because we want
            // the class-1 ihalfedge which is exterior but points "inside" the cut-mesh (i.e. torward
            // the negative side)
            if (math::sign(scalar_prod) == math::sign(orig_scalar_prod)) {

                // At this point, we have found our class-1 (or class 3, x-->x) src-mesh halfedge
                // from which we can infer whether the current polygon is "above" (outside) or
                // "below" (inside) the cut-mesh.
                // Also, by using the traced halfedge connectivity, we can determine the adjacent polygon
                // which shares an edge with the curent polygon. This shared edge is the edge of the
                // "next" halfedge (of the current halfedge) and it is always an interior edge.

                // Check if the current polygon is already found to be "above" the cut-mesh
                const bool cur_poly_already_marked_as_above = std::find(sm_polygons_above_cs.cbegin(), sm_polygons_above_cs.cend(), sm_poly_idx) != sm_polygons_above_cs.cend();

                if (!cur_poly_already_marked_as_above) {
                    sm_polygons_above_cs.push_back(sm_poly_idx);
                }

                // Here we can conviently find and save the the neighbouring polygon that is on the
                // other side i.e. "below" the cut-mesh. This is made possible because we can easily
                // search through the halfedge connectivity.

                // index of current halfedge in the current polygon
                const int sm_poly_he_idx = (int)std::distance(sm_poly.cbegin(), sm_poly_he_iter);
                // index of the "next" halfedge in the current polygon
                const int sm_poly_next_he_idx = wrap_integer(sm_poly_he_idx + 1, 0, (int)sm_poly.size() - 1);
                // the handle of the next halfedge in the current polygon
                const hd_t& sm_poly_next_he = sm_poly.at(sm_poly_next_he_idx);
                // now we query the handle of the opposite-halfedge of the next-halfedge.
                // This is facilitated by the incidence information that is maintained inside
                // our halfedge data structure "m0" which stores our vertices (including intersection
                // points) and edges that we calculated in earlier stages of the pipeline).
                const hd_t opp_of_sm_poly_next_he = m0.opposite(sm_poly_next_he);
                // using our halfedge-to-traced-polygon map, we then get the polygon index of the
                // opposite-halfedge
                const std::vector<int>& coincident_polys = m0_h_to_ply.at(opp_of_sm_poly_next_he); // coincident polygons (one cs and one sm)
                const std::vector<int>::const_iterator find_iter = std::find_if(
                    coincident_polys.cbegin(), coincident_polys.cend(),
                    [&](const int& e) { return (e < traced_sm_polygon_count); });

                // must always exist since "opp_of_sm_poly_next_he" is an interior ihalfedge
                MCUT_ASSERT(find_iter != coincident_polys.cend());

                // we have found the other src-mesh polygon which is "below" (inside) the cut-mesh
                const int coincident_sm_poly_idx = *find_iter;
                const bool neigh_poly_already_marked_as_below = std::find(sm_polygons_below_cs.cbegin(), sm_polygons_below_cs.cend(), sm_poly_idx) != sm_polygons_below_cs.cend();

                if (!neigh_poly_already_marked_as_below) {
                    sm_polygons_below_cs.push_back(coincident_sm_poly_idx);
                }
            }
        }
    }

    sm_reg_reentrant_ivtx_list.clear();

    // Here, we check for the unique case in which we could not find any traced src-mesh
    // polygons along the cut path which could be identified as either "above" (outside)
    // or "below" (inside).
    // Such a situation is rare and happens when the src-mesh has one face where the
    // intersection with the src-mesh is a partial cut
    //
    // PERSONAL NOTE (TODO?): This may not be necessary if we just enable also using src-mesh
    // tip re-entrant vertices when searching for above and below polygons.

    if (sm_polygons_above_cs.empty() && sm_polygons_below_cs.empty()) {
        MCUT_ASSERT(sm_face_count == 1);
        sm_polygons_above_cs.push_back(0); // sm polygons are stored first theirfore sm polygon will ccse first (see "ps" definition)
        sm_polygons_below_cs.push_back(0);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Map src-mesh intersection halfedges to a boolean
    ///////////////////////////////////////////////////////////////////////////

    //
    // Here we will map every src-mesh halfedge connected to an intersection point to a boolean.
    // This boolean value indicates if the halfedge has been `transformed`. The notion
    // of "transformation" is used to indicate whether a halfedge has been "processed"
    // to assign it to a distinct connected component.
    //
    // We call a halfedge connected to at-least one intersection point note an "intersection
    // halfedge"
    //

    // key=intersection halfedge which is used for tracing; value=flag for indicating if halfedge has been transformed
    std::map<hd_t, bool> m0_sm_ihe_to_flag;

    // TODO: this can be improved so that we do not iterate over all edges.
    // I.e. we could iterate over the cutpath edge and the edges we create ealier that connect to atleast one intersection point
    for (mesh_t::edge_iterator_t edge_iter = m0.edges_begin(); edge_iter != m0.edges_end(); ++edge_iter) {
        const ed_t& edge = (*edge_iter);
        const vd_t v0 = m0.vertex(edge, 0);
        const vd_t v1 = m0.vertex(edge, 1);

        const bool v0_is_ivtx = m0_is_intersection_point(v0, ps_vtx_cnt);
        const bool v1_is_ivtx = m0_is_intersection_point(v1, ps_vtx_cnt);

        if (!v0_is_ivtx && !v1_is_ivtx) { // o-->o
            // we only want halfedges with an intersection point
            continue;
        }

        //
        // check if current edges is a cut-mesh edge
        //

        if (v0_is_ivtx && !v1_is_ivtx) { // x-->o

            // get the polygon-soup version of tgt descriptor
            std::map<vd_t, vd_t>::const_iterator m0_to_ps_vtx_find_v1_iter = std::find_if(
                m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(),
                [&](const std::pair<vd_t, vd_t>& e) { return e.first == v1; });

            MCUT_ASSERT(m0_to_ps_vtx_find_v1_iter != m0_to_ps_vtx.cend());

            const vd_t& ps_v1 = m0_to_ps_vtx_find_v1_iter->second;

            if (ps_is_cutmesh_vertex(ps_v1, sm_vtx_cnt)) { // is it a cut-mesh vertex..?
                // we want only src-mesh edges
                continue;
            }
        }

        if (!v0_is_ivtx && v1_is_ivtx) { // o-->x
            std::map<vd_t, vd_t>::const_iterator m0_to_ps_vtx_find_v0_iter = std::find_if(
                m0_to_ps_vtx.cbegin(), m0_to_ps_vtx.cend(),
                [&](const std::pair<vd_t, vd_t>& e) { return e.first == v0; });

            MCUT_ASSERT(m0_to_ps_vtx_find_v0_iter != m0_to_ps_vtx.cend());

            const vd_t& ps_v0 = m0_to_ps_vtx_find_v0_iter->second;

            if (ps_is_cutmesh_vertex(ps_v0, sm_vtx_cnt)) {
                continue; // is a cut-mesh edge
            }
        }

        // TODO: we also need to check for cut-mesh edges of the form x-->x [but only the polygon boundary type]
        // At the moment, "m0_sm_ihe_to_flag" will also include those cut-mesh halfedges!
        //
        // ** I'm not convinced that this is a problem

#if 0
        bool is_ambiguious_exterior_edge_case = v0_is_ivtx && v1_is_ivtx;

        if (is_ambiguious_exterior_edge_case) { // exterior edge with two intersection vertices (ambigious case arising from concave polyhedron cut)

            const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v0);
            const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(v1);
            const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
            const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);
            bool is_valid_ambiguious_exterior_edge = (v0_ps_edge == v1_ps_edge); // see also above when gathering exterior incident edges
            if (is_valid_ambiguious_exterior_edge && !cs_is_watertight) // x-->x where o-->x-->x-->o
            {
                // Exterior ihalfedges (and hence their respective halfedges) are not transformed.
                // Only interior ihalfedges need to be transformed to create incisions that allow openings of the sm via transformations.
                // NOTE: when cs is watertight we still include polygon-exterior interior-ihalfedge because they a needed to "bite" a chuck out of the cs (see example 19)
                continue;
            }
        }
#endif

        //
        // save the halfegdes of the current edge (if they are used to trace a polygon)
        //

        const hd_t h0 = m0.halfedge(edge, 0);

        if (m0_h_to_ply.find(h0) != m0_h_to_ply.end()) { // check if used to trace polygon
            std::pair<std::map<hd_t, bool>::const_iterator, bool> pair0 = m0_sm_ihe_to_flag.insert(std::make_pair(h0, false));
            MCUT_ASSERT(pair0.second == true);
        }

        const hd_t h1 = m0.halfedge(edge, 1);

        if (m0_h_to_ply.find(h1) != m0_h_to_ply.end()) { // check id used to trace polygon
            std::pair<std::map<hd_t, bool>::const_iterator, bool> pair1 = m0_sm_ihe_to_flag.insert(std::make_pair(h1, false));
            MCUT_ASSERT(pair1.second == true);
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // create the second auxilliary halfedge data structure ("m1")
    ///////////////////////////////////////////////////////////////////////////

    //
    // At this point, we create another auxilliary halfedge data structure.
    // It will store the vertices and edges like "m0" but will also include the
    // duplicate copy of (most/all) intersection points, as well as some new edges. The new
    // edges are created to partition the src-mesh as we process intersection
    // halfedges by assigning them the correct copy of dupicated intersection
    // points. Thus along the cut path, we will create new connectivity that
    // allows us to partition the src-mesh along this path.
    //

    // store's the (unsealed) connected components
    mesh_t m1;

    // copy vertices from m0 t0 m1 (and save mapping to avoid assumptions).

    std::map<vd_t, vd_t> m0_to_m1_vtx;

    for (mesh_t::vertex_iterator_t v = m0.vertices_begin(); v != m0.vertices_end(); ++v) {
        const vd_t m1_vd = m1.add_vertex(m0.vertex(*v));
        MCUT_ASSERT(m1_vd != mesh_t::null_vertex());
        std::pair<std::map<vd_t, vd_t>::const_iterator, bool> pair = m0_to_m1_vtx.insert(std::make_pair(*v, m1_vd));
        MCUT_ASSERT(pair.second == true);
    }

    MCUT_ASSERT(m1.number_of_vertices() == m0.number_of_vertices());

    // copy m0 edges and halfedges [which are not intersection-halfedges] and
    // build a mapping between m0 and m1. This mapping is needed because as we
    // begin to transform halfedges incident to the cut-path, some of their opposites
    // will become invalidated. This is because for each interior halfedge we will
    // essentially create a new edge.
    // We must also relate halfedges (in "m1") to their opposites explicitly (essentially
    // copying the information already stored in "m0"). Because this information will be
    // lost after duplicating intersection points and transforming all halfedges
    // along the cut-path.

    std::map<hd_t, hd_t> m0_to_m1_he;

    for (mesh_t::edge_iterator_t e = m0.edges_begin(); e != m0.edges_end(); ++e) {
        const ed_t& m0_edge = (*e);
        const vd_t m0_v0 = m0.vertex(m0_edge, 0);
        const vd_t m0_v1 = m0.vertex(m0_edge, 1);
        //const bool m0_v0_is_ivtx = m0_is_intersection_point(m0_v0, ps_vtx_cnt);
        //const bool m0_v1_is_ivtx = m0_is_intersection_point(m0_v1, ps_vtx_cnt);

        if (!(m0_is_intersection_point(m0_v0, ps_vtx_cnt) || m0_is_intersection_point(m0_v1, ps_vtx_cnt))) { // not coincident to an intersection vertex (i.e. is class-0 edge)

            const vd_t m1_v0 = m0_to_m1_vtx.at(m0_v0);
            const vd_t m1_v1 = m0_to_m1_vtx.at(m0_v1);
            const hd_t m1_halfedge = m1.add_edge(m1_v0, m1_v1); // add m1
            const hd_t m0_h0 = m0.halfedge(m0_edge, 0);
            const vd_t m0_h0_src = m0.source(m0_h0);
            const hd_t m0_h1 = m0.halfedge(m0_edge, 1);
            const vd_t m1_halfedge_src = m1.source(m1_halfedge);
            const vd_t m1_halfedge_tgt = m1.target(m1_halfedge);

            if (m0_to_m1_vtx.at(m0_h0_src) == m1_halfedge_src) { // i.e. "is the m0_h0 equivalent to m1_halfedge?"
                m0_to_m1_he.insert(std::make_pair(m0_h0, m1_halfedge));
                m0_to_m1_he.insert(std::make_pair(m0_h1, m1.opposite(m1_halfedge)));
            } else {
                m0_to_m1_he.insert(std::make_pair(m0_h1, m1_halfedge));
                m0_to_m1_he.insert(std::make_pair(m0_h0, m1.opposite(m1_halfedge)));
            }
        }
    }

    //
    // For each src-mesh halfedge we store "next-halfedge" state for quick-lookup in "m0".
    // We store this information in "m0" because it allows for a more expedient state-lookup during
    // connected-component re-assignment.
    //
    // Note: this is only made possible because 1) from this point onwards "m0" will not be modified
    // at all, and 2) we can safely assume that its okay to store "next-halfedge" state without `technically`
    // violating 2-manifold rules since this information is only for the src-mesh polygons.
    //
    // We will ultimately use this saved state to extract the intersection-halfedges which are coincident
    // to intersection-vertices during connected-component re-assignment.
    //

    // for each src-mesh polygon
    for (std::vector<traced_polygon_t>::const_iterator traced_sm_polygon_iter = m0_polygons.cbegin();
         traced_sm_polygon_iter != m0_traced_sm_polygons_iter_cend;
         ++traced_sm_polygon_iter) {
        const traced_polygon_t& traced_sm_polygon = *traced_sm_polygon_iter;

        // for each halfedge of polygon
        for (traced_polygon_t::const_iterator traced_sm_polygon_halfedge_iter = traced_sm_polygon.cbegin();
             traced_sm_polygon_halfedge_iter != traced_sm_polygon.cend();
             ++traced_sm_polygon_halfedge_iter) {
            const int i = (int)std::distance(traced_sm_polygon.cbegin(), traced_sm_polygon_halfedge_iter);

            const hd_t& cur = traced_sm_polygon.at(i);
            const hd_t& next = traced_sm_polygon.at((i + 1) % traced_sm_polygon.size());
            m0.set_next(cur, next); // update state
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // src-mesh partitioning
    ///////////////////////////////////////////////////////////////////////////

    //
    // Here we partition the traced src-mesh polygons into connected components
    // by circulating around each intersection point and creating a copy for
    // each distinct connected component seen. In the case of a complete cut,
    // all intersection points are duplicated. However, in the case of a partial
    // cut, we only duplicate the intersection points which are along the cut path
    // but exclude those which are the terminal vertices of a bounding sequence.
    // A bounding sequence is an ordered list of edges passing through intersection
    // points (along the cut path). In the case of a partial cut, a bounding sequence
    // does not form a loop.
    //

    /*
      Goal: 
      Assign the correct intersection vertex instance (descriptor) to each intersection halfedge.

      For reference there following are the types of halfedges we have

      class : instance : definition (what will happen)

      0 : o-->o : Exterior (nothing, we already copied this type of edge into m1)
      1 : o-->x : Exterior (tgt may be assigned duplicate instance of descriptor)
      2 : x-->o : Exterior (src may be assigned duplicate instance of descriptor)
      3 : x-->x : Interior OR Exterior (tgt and src may be assigned duplicate instance of descriptor)

      o - original-vertex
      x - intersection-vertex
    */

    lg << "re-assign intersection-halfedges to distinct connected components" << std::endl;

    // This data structure will map the descriptors of intersection-halfedges in "m0"
    // to their descriptor in "m1". Thus, some halfedges (in "m0") will be mapped to
    // new halfedges which are not in "m0" but will be added into "m1".
    std::map<hd_t, hd_t> m0_to_m1_ihe;

    // our routine will start from an untransformed class-1 ihalfedge. We do this because
    // it makes transformation process easier for us.
    std::map<hd_t, bool>::iterator m0_1st_sm_ihe_fiter = std::find_if( // for each src-mesh intersection halfedge
        m0_sm_ihe_to_flag.begin(),
        m0_sm_ihe_to_flag.end(),
        // lamda checks the intersection halfedge that has not been transformed/processed already
        [&](const std::pair<hd_t, bool>& e) {
            const hd_t& m0_ihe = e.first;
            const vd_t m0_ihe_src_vertex = m0.source(m0_ihe);
            const bool src_is_ivertex = m0_is_intersection_point(m0_ihe_src_vertex, ps_vtx_cnt);

            if (src_is_ivertex) {
                return false; // has to be original vertex
            }

            const bool is_transformed = e.second;

            if (is_transformed) {
                return false; // cannot have been transformed already.
            }

            const vd_t& m0_ihe_tgt_vertex = m0.target(m0_ihe);
            const bool tgt_is_ivertex = m0_is_intersection_point(m0_ihe_tgt_vertex, ps_vtx_cnt);

            if (tgt_is_ivertex) { // tgt is an intersection point

                // is the current halfedge used to traced a polygon (i.e. those we stored in "m0")
                const bool is_incident_to_traced_polygon = m0_h_to_ply.find(m0_ihe) != m0_h_to_ply.end();

                if (is_incident_to_traced_polygon) {
                    //
                    // We now need to make sure that the preceeding (prevous) halfedge of the
                    // current (in its polygon) is class0 or class2
                    //

                    // find coincident polygon
                    const std::vector<int>& incident_polys = m0_h_to_ply.at(m0_ihe);

                    MCUT_ASSERT(incident_polys.size() == 1); // class-1 halfedges are incident to exactly one polygon

                    const int incident_poly_idx = incident_polys.front();
                    const traced_polygon_t& incident_poly = m0_polygons.at(incident_poly_idx);
                    // find the reference to the current halfedde (in the traced polygon)
                    traced_polygon_t::const_iterator he_find_iter = std::find(incident_poly.cbegin(), incident_poly.cend(), m0_ihe);

                    MCUT_ASSERT(he_find_iter != incident_poly.cend()); // if its incident to a polygon then that polygon must have it!

                    // halfedge index in polygon
                    const int he_index = (int)std::distance(incident_poly.cbegin(), he_find_iter);
                    // index of previous halfedge in polygon
                    const int preceeding_he_idx = wrap_integer(he_index - 1, 0, (int)incident_poly.size() - 1);
                    const hd_t& preceeding_he = incident_poly.at(preceeding_he_idx);
                    const vd_t preceeding_he_src = m0.source(preceeding_he);
                    const vd_t preceeding_he_tgt = m0.target(preceeding_he);
                    const bool preceeding_he_src_is_ivertex = m0_is_intersection_point(preceeding_he_src, ps_vtx_cnt);
                    const bool preceeding_he_tgt_is_ivertex = m0_is_intersection_point(preceeding_he_tgt, ps_vtx_cnt);
                    // classify preceeding halfedge
                    const bool preceeding_he_is_class0 = !preceeding_he_src_is_ivertex && !preceeding_he_tgt_is_ivertex; // o-->o
                    const bool preceeding_he_is_class2 = preceeding_he_src_is_ivertex && !preceeding_he_tgt_is_ivertex; // x-->o
                    // count the original vertices that are contained in the polygon the current halfedge.
                    // we need this check to detect a special edge case.
                    const int overtices_in_poly = (int)std::count_if(
                        incident_poly.cbegin(), incident_poly.cend(),
                        [&](const hd_t& e) { return !m0_is_intersection_point(m0.target(e), ps_vtx_cnt); });

                    return (preceeding_he_is_class0 || (preceeding_he_is_class2 && overtices_in_poly == 1));
                }
            }

            return false;
        });

    // Here we have queue of intersection halfedges which will be used to begin a transformation walk/traversal
    // around the polygon of each contained halfedge. For each polygon along the cut path there will ever be at
    // most one of its halfedges in this queue.
    // To assign a polygon along the cut-path to the correct connected component, we will traverse a subset (or full set)
    // of it halfedges in order to assign the correct instance of each referenced intersection point to the traversed
    // halfedge (s).
    std::vector<hd_t> m0_ox_hlist;

    // used to specifically prevent duplicate ox and xo halfedges
    // (CGAL surface mesh does not allow checking if halfede exists)
    // Map-key=an intersection-vertex in m1;
    // Map-value=list of (vertex,halfedge) pairs.
    //  The first elem in a pair is a vertex connected to [Map-key].
    //  The second element in a pair is the halfedge connecting [Map-key] and the first element in the pair
    //
    // NOTE: [Map-key] is always the src vertex of the halfedge which is the second element in a pair of [Map-value]
    std::map<vd_t, std::vector<std::pair<vd_t, hd_t>>> m1_ivtx_to_h;

    // At this point we also introduce the notion of a so-called "strongly connected border set" (SCBS).
    // An SCBS is set of adjacent src-mesh polygons along the partitioned cut-path (partitioned implies that the
    // src-mesh polygons along the cut path are no longer "topologically connected").
    int strongly_connected_sm_boundary_seq_iter_id = -1;

    // for each strongly-connected set of sm-boundary sequences.
    // one iteration pertains to a transformation of a set of sm-boundary sequences which all belong to the same connected ccsponent.
    // sets of sm-boundary sequences which belong to the same (sm) connected ccsponent may be produced different iterations.

    do {
        lg << "SCBS iteration: " << ++strongly_connected_sm_boundary_seq_iter_id << std::endl;

        MCUT_ASSERT((m0_1st_sm_ihe_fiter != m0_sm_ihe_to_flag.end())); // their must be at least one halfedge from which we can start walking!

        m0_ox_hlist.push_back(m0_1st_sm_ihe_fiter->first); // add to queue

        // The following do-while loop will transform/process the halfedges which belong
        // to exactly one SCBS
        do {

            lg.indent();

            hd_t m0_cur_h = mesh_t::null_halfedge();
            // get first intersection halfedge which determine's the first polygon of the current SCBS.
            // Note that the current SCBS is determined implicitely from this initial intersection halfedge.
            const hd_t seq_init_ihe = m0_ox_hlist.back();

            lg << "first SCBS halfedge  = " << seq_init_ihe << std::endl;

            m0_ox_hlist.pop_back(); // remove "seq_init_ihe" from queue
            hd_t m0_nxt_h = seq_init_ihe;

            // The following do-while loop will transform/process the halfedges which belong
            // to [a part] of the current SCBS (i.e. a swept surface of polygons next to the
            // partitioned cut-path).
            do { // process ih sequence starting from "seq_init_ihe"

                lg.indent();

                m0_cur_h = m0_nxt_h; // current
                lg << "curr(m0)  = " << hstr(m0, m0_cur_h) << std::endl;
                m0_nxt_h = m0.next(m0_cur_h); // next

                const vd_t m0_cur_h_src = m0.source(m0_cur_h);
                const vd_t m0_cur_h_tgt = m0.target(m0_cur_h);
                const bool m0_cur_h_src_is_ivtx = m0_is_intersection_point(m0_cur_h_src, ps_vtx_cnt);
                const bool m0_cur_h_tgt_is_ivtx = m0_is_intersection_point(m0_cur_h_tgt, ps_vtx_cnt);
                const bool m0_cur_h_is_ox = !m0_cur_h_src_is_ivtx && m0_cur_h_tgt_is_ivtx; // o-->x

                MCUT_ASSERT(m0_to_m1_vtx.find(m0_cur_h_src) != m0_to_m1_vtx.cend());

                vd_t m1_cur_h_src = m0_to_m1_vtx.at(m0_cur_h_src); // from m0 to m1 descriptor

                MCUT_ASSERT(m0_to_m1_vtx.find(m0_cur_h_tgt) != m0_to_m1_vtx.cend());

                vd_t m1_cur_h_tgt = m0_to_m1_vtx.at(m0_cur_h_tgt); // from m0 to m1 descriptor

                // o-->x OR x-->x
                if (m0_cur_h_tgt_is_ivtx) { // tgt vertex of current halfedge is an intersection point

                    //
                    // check if the next halfedge has been processed/transformed
                    //

                    // can we find the m1 version of the next halfedge
                    std::map<hd_t, hd_t>::const_iterator m1_nxt_h_fiter = m0_to_m1_ihe.find(m0_nxt_h);
                    const bool nxt_is_processed = m1_nxt_h_fiter != m0_to_m1_ihe.cend();

                    if (nxt_is_processed) {
                        // Since the next halfedge has been processed, we can simply set
                        // target vertex instance of the current ("m1") halfedge to the source of the
                        // next halfedge
                        m1_cur_h_tgt = m1.source(m1_nxt_h_fiter->second);
                    } else {
                        // otherwise, we need to determined the correct instance of the tgt
                        // vertex to be used (see paper for details)
                        m1_cur_h_tgt = resolve_intersection_point_descriptor(ps, m0, m1, m0_cur_h, m0_cur_h_tgt, m1_cur_h_tgt, m0_cur_h_is_ox,
                            m0_h_to_ply, ivtx_to_incoming_hlist, m0_sm_ihe_to_flag, m0_ivtx_to_ps_he, m0_to_m1_ihe, m0_to_ps_vtx, ps_vtx_cnt, sm_vtx_cnt, sm_face_count);
                    }
                }

                // x-->o OR x-->x
                if (m0_cur_h_src_is_ivtx) { // src vertex of current halfedge is an intersection point
                    if (m0_cur_h == seq_init_ihe) // is it the first halfedge of the current SCBS?
                    {
                        // get the opposite halfedge
                        const hd_t opp = m0.opposite(m0_cur_h); // NOTE: m0_cur_h_src == target(opp)
                        // we need to determined the correct instance of the src vertex to be used (see paper for details)
                        m1_cur_h_src = resolve_intersection_point_descriptor(ps, m0, m1, opp, m0_cur_h_src, m1_cur_h_src, m0_cur_h_is_ox,
                            m0_h_to_ply, ivtx_to_incoming_hlist, m0_sm_ihe_to_flag, m0_ivtx_to_ps_he, m0_to_m1_ihe, m0_to_ps_vtx, ps_vtx_cnt, sm_vtx_cnt, sm_face_count);
                    } else { // current halfedge is not the first halfedge of the current SCBS

                        // get the previous halfedge
                        const hd_t m0_prv_h = m0.prev(m0_cur_h);

                        // The previous halfedge must have been transformed since the current halfedge
                        // is not the first halfedge of the current SCBS. TODO: explain further for why
                        // this is true when halfedge is x-->x (i.e. exterior, and from scoop cut)
                        MCUT_ASSERT(m0_to_m1_ihe.find(m0_prv_h) != m0_to_m1_ihe.cend());

                        // get transformed instance of previous halfedge ("m1" version )
                        const hd_t m1_prv_h = m0_to_m1_ihe.at(m0_prv_h);
                        // Since the previous halfedge has been processed, we can simply set
                        // src vertex instance of the current ("m1") halfedge to the source of the
                        // next halfedge
                        const vd_t m1_prv_h_tgt = m1.target(m1_prv_h);
                        m1_cur_h_src = m1_prv_h_tgt;
                    }
                }

                //
                // Now that we have resolved the correct descriptor instance to use for the src and/or tgt vertex,
                // we will create an edge in halfedge data structure ("m1") connecting "m1_cur_h_src" and
                // "m1_cur_h_tgt" if this edge does not already exist.
                //

                // bool m1_cur_h_exists = false;
                const bool m0_cur_h_is_xx = m0_cur_h_src_is_ivtx && m0_cur_h_tgt_is_ivtx;
                //std::map<vd_t, std::vector<std::pair<vd_t, hd_t> > >::iterator fiter = m1_ivtx_to_h.end();

                bool m0_cur_h_is_exterior = true;

                if (m0_cur_h_is_xx) {
                    const hd_t m0_cur_h_src_ps_h = m0_ivtx_to_ps_he.at(m0_cur_h_src);
                    const hd_t m0_cur_h_tgt_ps_h = m0_ivtx_to_ps_he.at(m0_cur_h_tgt);
                    const ed_t m0_cur_h_src_ps_e = ps.edge(m0_cur_h_src_ps_h);
                    const ed_t m0_cur_h_tgt_ps_e = ps.edge(m0_cur_h_tgt_ps_h);
                    m0_cur_h_is_exterior = (m0_cur_h_src_ps_e == m0_cur_h_tgt_ps_e);
                }

                // get the opposite of the current halfedge ("m0")
                const hd_t opp = m0.opposite(m0_cur_h);
                // check if this opposite halfedge was used to traced a polygon
                const bool opp_used_for_tracing = m0_h_to_ply.find(opp) != m0_h_to_ply.end();

                // if 1) the current halfedge is an interior halfedge (x-->x), OR
                // 2) the  current halfedge is an exterior halfedge AND it has not been processed
                if (!m0_cur_h_is_exterior || (m0_cur_h_is_exterior && m0_to_m1_ihe.find(m0_cur_h) == m0_to_m1_ihe.cend())) {

                    lg << "create edge = " << estr(m1_cur_h_src, m1_cur_h_tgt) << std::endl;

                    // create processed version (i.e. "m1" version) of "m0_cur_h"
                    hd_t m1_cur_h = m1.add_edge(m1_cur_h_src, m1_cur_h_tgt);

                    MCUT_ASSERT(m1_cur_h != mesh_t::null_halfedge());

                    //
                    // here, we update the list containing the vertices and halfedge that are connected
                    // to the "m1_cur_h_src" and "m1_cur_h_tgt" i.e. adjacency information
                    //

                    if (m0_cur_h_is_exterior) { // is the current halfedge an exterior halfedge..?

                        // find entry in the adjancency vector

                        // src
                        std::map<vd_t, std::vector<std::pair<vd_t, hd_t>>>::iterator fiter = m1_ivtx_to_h.find(m1_cur_h_src);

                        if (fiter != m1_ivtx_to_h.cend()) // check src's entry exists
                        {
                            MCUT_ASSERT(std::find_if(fiter->second.cbegin(), fiter->second.cend(), [&](const std::pair<vd_t, hd_t>& p) { return p.first == m1_cur_h_tgt; }) == fiter->second.cend());
                            fiter->second.emplace_back(m1_cur_h_tgt, m1_cur_h); // record connection and save connecting halfedge
                        } else {
                            std::pair<std::map<vd_t, std::vector<std::pair<vd_t, hd_t>>>::iterator, bool> p = m1_ivtx_to_h.emplace(m1_cur_h_src, std::vector<std::pair<vd_t, hd_t>>());
                            MCUT_ASSERT(p.second == true);
                            fiter = p.first;
                            fiter->second.emplace_back(m1_cur_h_tgt, m1_cur_h);
                        }

                        // repeat for tgt

                        fiter = m1_ivtx_to_h.find(m1_cur_h_tgt); // check if the tgt's entry exists and update it

                        if (fiter != m1_ivtx_to_h.cend()) // check tgt's entry exists
                        {
                            MCUT_ASSERT(std::find_if(fiter->second.cbegin(), fiter->second.cend(), [&](const std::pair<vd_t, hd_t>& p) { return p.first == m1_cur_h_src; }) == fiter->second.cend());
                            fiter->second.emplace_back(m1_cur_h_src, m1.opposite(m1_cur_h)); // record that it is connected to src (by the opp he of m1_cur_h)
                        } else {
                            std::pair<std::map<vd_t, std::vector<std::pair<vd_t, hd_t>>>::iterator, bool> p = m1_ivtx_to_h.emplace(m1_cur_h_tgt, std::vector<std::pair<vd_t, hd_t>>());
                            MCUT_ASSERT(p.second == true);
                            fiter = p.first;
                            fiter->second.emplace_back(m1_cur_h_src, m1.opposite(m1_cur_h)); // record that it is connected to src (by the opp he of m1_cur_h)
                        }
                    }

                    // map m0 to m1 version of current halfedge
                    m0_to_m1_ihe.emplace(m0_cur_h, m1_cur_h);

                    // if 1) the current halfedge is an exterior halfedge, AND
                    // 2) the opposite of the current halfedge was used to trace a polygon
                    if (m0_cur_h_is_exterior && opp_used_for_tracing) {
                        // Thanks of the halfedge data structure (each edge has 2 halfedges),
                        // we also have the m1 version/copy of opposite halfedge.
                        // NOTE however, the opposite halfedge it is still not "processed", and
                        // we will do so only when we traverse/walk it!
                        m0_to_m1_ihe.emplace(opp, m1.opposite(m1_cur_h));
                    }

                    // NOTE: keep in mind that two opposite halfedges which are interior halfedges
                    // will belong to separate connected components after all processing is complete.
                    // (exterior halfedges on the other hand will share the same connected component
                    // as their opposites).

                } else {

                    //
                    // here, we have an exterior halfedge whose "m1" version has already been created.
                    //
                    MCUT_ASSERT(m0_to_m1_ihe.find(m0_cur_h) != m0_to_m1_ihe.cend());
                    const hd_t m1_cur_h = m0_to_m1_ihe.at(m0_cur_h);

                    lg << "recycled halfedge = " << hstr(m1, m1_cur_h) << std::endl;
                }

                //
                // update queue of ox halfedges which will be the initial halfedges of
                // (potentially parts of) SCBS's to be processed
                //

                // x-->o
                const bool m0_cur_h_is_xo = m0_cur_h_src_is_ivtx && !m0_cur_h_tgt_is_ivtx;

                // if 1) curreent halfedge is x-->o AND 2) it's opposites has been using to trace a polygon, AND
                // 3) this opposite has not already been processed
                if (m0_cur_h_is_xo && opp_used_for_tracing && !m0_sm_ihe_to_flag.at(opp)) {

                    // get the next halfedge
                    const hd_t nxt = m0.next(m0_cur_h);
                    // is the next halfedge an intersection halfedge "o-->x"
                    const bool nxt_is_ih = m0_is_intersection_point(m0.target(nxt), ps_vtx_cnt); // check if is last halfedge

                    if ((nxt_is_ih && nxt == seq_init_ihe) || !nxt_is_ih) {
                        // here we add the next SCBS's first halfedge from which SCBS processing will begin.
                        m0_ox_hlist.push_back(opp);
                    }
                }

                MCUT_ASSERT(m0_sm_ihe_to_flag.find(m0_cur_h) != m0_sm_ihe_to_flag.cend());

                m0_sm_ihe_to_flag.at(m0_cur_h) = true; // mark as "processed"

                lg.unindent();
            } while (
                // "next" is ihalfedge
                (m0_is_intersection_point(m0.source(m0_nxt_h), ps_vtx_cnt) || m0_is_intersection_point(m0.target(m0_nxt_h), ps_vtx_cnt)) &&
                // "next" is not transformed. For case when ihalfedge-sequence forms a loop.
                m0_sm_ihe_to_flag.at(m0_nxt_h) == false); // TODO: I think this last condition is the same as "m0_nxt_h" == "seq_init_ihe" (try it bcz using m0_sm_ihe_to_flag will be slower)

            lg.unindent();
        } while (!m0_ox_hlist.empty());

        //
        // find next class1 halfedge which has not been transformed ( possibly in the same connected ccsponent )
        //

        m0_1st_sm_ihe_fiter = std::find_if( // find o-->x halfedge
            m0_sm_ihe_to_flag.begin(),
            m0_sm_ihe_to_flag.end(),
            // TODO: this lambda is exactly the same as the one above (the out-scope do-while loop above)
            // consider refactoring
            [&](const std::pair<hd_t, bool>& e) {
                const hd_t& m0_ihe = e.first;

                const vd_t m0_ihe_src_vertex = m0.source(m0_ihe);
                const bool src_is_ivertex = m0_is_intersection_point(m0_ihe_src_vertex, ps_vtx_cnt);
                if (src_is_ivertex) {
                    return false; // has to be original
                }

                const bool is_transformed = e.second;

                if (is_transformed) {
                    return false;
                }

                const vd_t m0_ihe_tgt_vertex = m0.target(m0_ihe);
                const bool tgt_is_ivertex = m0_is_intersection_point(m0_ihe_tgt_vertex, ps_vtx_cnt);

                if (tgt_is_ivertex) {
                    const bool is_incident_to_traced_polygon = m0_h_to_ply.find(m0_ihe) != m0_h_to_ply.end();

                    if (is_incident_to_traced_polygon) {

                        // now make sure that the preceeding halfedge of the current (in its polygon) is class0 or class two

                        // find coincident polygon
                        const std::vector<int>& incident_polys = m0_h_to_ply.at(m0_ihe);

                        MCUT_ASSERT(incident_polys.size() == 1); // class-one halfedge are incident to one polygon

                        const int incident_poly_idx = incident_polys.front();
                        const traced_polygon_t& incident_poly = m0_polygons.at(incident_poly_idx);
                        traced_polygon_t::const_iterator he_find_iter = std::find(incident_poly.cbegin(), incident_poly.cend(), m0_ihe);

                        MCUT_ASSERT(he_find_iter != incident_poly.cend());

                        const int he_index = (int)std::distance(incident_poly.cbegin(), he_find_iter);
                        const int preceeding_he_idx = wrap_integer(he_index - 1, 0, (int)incident_poly.size() - 1);
                        const hd_t& preceeding_he = incident_poly.at(preceeding_he_idx);
                        const vd_t preceeding_he_src = m0.source(preceeding_he);
                        const vd_t preceeding_he_tgt = m0.target(preceeding_he);
                        const bool preceeding_he_src_is_ivertex = m0_is_intersection_point(preceeding_he_src, ps_vtx_cnt);
                        const bool preceeding_he_tgt_is_ivertex = m0_is_intersection_point(preceeding_he_tgt, ps_vtx_cnt);

                        // check preceeding halfedge is class0 or class two

                        const bool preceeding_he_is_class0 = !preceeding_he_src_is_ivertex && !preceeding_he_tgt_is_ivertex; // o-->o
                        const bool preceeding_he_is_class2 = preceeding_he_src_is_ivertex && !preceeding_he_tgt_is_ivertex; // x-->o
                        const int overtices_in_poly = (int)std::count_if(incident_poly.cbegin(), incident_poly.cend(),
                            [&](const hd_t& e) {
                                return !m0_is_intersection_point(m0.target(e), ps_vtx_cnt);
                            });
                        return (preceeding_he_is_class0 || (preceeding_he_is_class2 && overtices_in_poly == 1));
                    }
                }

                return false;
            });

        // True only if there exists a src-mesh ps-edge which has [at least] two intersection points
        // This means that the src-mesh has a scoop cut (see example 19)
        const bool class1_ihalfedge_found = (m0_1st_sm_ihe_fiter != m0_sm_ihe_to_flag.end());

        if (!class1_ihalfedge_found) { // The above search failed to find an untransformed class-1 halfedge.

            //
            // So now we instead try to search an untransformed polygon-exterior interior-ihalfedge (x-->x).
            //

            m0_1st_sm_ihe_fiter = std::find_if( // for each intersection halfedge
                m0_sm_ihe_to_flag.begin(), m0_sm_ihe_to_flag.end(),
                [&](const std::pair<hd_t, bool>& e) {
                    const bool is_transformed = e.second; // has it already been transformed..?

                    if (is_transformed) {
                        return false; // we want only the transformed intersection halfedges
                    }

                    const hd_t& m0_ihe = e.first;

                    const vd_t m0_ihe_src_vertex = m0.source(m0_ihe);
                    const bool src_is_ivertex = m0_is_intersection_point(m0_ihe_src_vertex, ps_vtx_cnt);
                    const vd_t m0_ihe_tgt_vertex = m0.target(m0_ihe);
                    const bool tgt_is_ivertex = m0_is_intersection_point(m0_ihe_tgt_vertex, ps_vtx_cnt);

                    if (!(src_is_ivertex && tgt_is_ivertex)) {
                        return false; // we want only class-3 intersection halfedges (x-->x)
                    }

                    //
                    // checf is halfedge is really an exterior one (ambigious case arising from concave polyhedron cut)
                    //
                    const hd_t v0_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0_ihe_src_vertex);
                    const hd_t v1_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0_ihe_tgt_vertex);
                    const ed_t v0_ps_edge = ps.edge(v0_coincident_ps_halfedge);
                    const ed_t v1_ps_edge = ps.edge(v1_coincident_ps_halfedge);
                    const bool is_poly_exterior_interior_ihalfedge = (v0_ps_edge == v1_ps_edge);

                    if (is_poly_exterior_interior_ihalfedge) { // we want only polygon-exterior interior ihalfedges

                        // get the traced polygon which uses the current halfedge
                        const std::map<hd_t, std::vector<int>>::const_iterator coincident_poly_find_iter = m0_h_to_ply.find(m0_ihe);

                        MCUT_ASSERT(coincident_poly_find_iter != m0_h_to_ply.end());
                        MCUT_ASSERT(coincident_poly_find_iter->second.size() == 1); // polygon-exterior interior-ihalfedges are incident to exactly one polygon

                        const bool is_used_to_trace_src_mesh_polygon = (coincident_poly_find_iter->second.front() < traced_sm_polygon_count);

                        return (is_used_to_trace_src_mesh_polygon);
                    } else {
                        return false;
                    }
                });
        }

        // loop while there exists a "non-transformed" exterior intersection-halfedge
        // from which we can start building a SCBS
    } while (m0_1st_sm_ihe_fiter != m0_sm_ihe_to_flag.end());

#if 0
    // dump
    lg << "m0 to m1 ihalfedge" << std::endl;

    for (std::map<hd_t, hd_t>::const_iterator i = m0_to_m1_ihe.cbegin(); i != m0_to_m1_ihe.cend(); ++i) {
        lg << "<" << i->first << ", " << i->second << ">" << std::endl;
    }
#endif

    m0_to_ps_vtx.clear();
    ivtx_to_incoming_hlist.clear();
    m0_sm_ihe_to_flag.clear();
    m0_to_m1_vtx.clear();
    m0_ox_hlist.clear();
    m1_ivtx_to_h.clear();

    //
    // NOTE: at this stage, we have calculated all the vertices, edges, halfedges and meta-data
    // which describes the connectivity of the partitioned (topologically split) src-mesh along
    // the cut-path.
    //

    ///////////////////////////////////////////////////////////////////////////
    // Save the number of vertices in "m1" after src-mesh partitioning
    ///////////////////////////////////////////////////////////////////////////

    // saving the number of vertices here will allow us to infer exactly which vertices
    // lie on the seam.

    const int m1_num_vertices_after_srcmesh_partitioning = m1.number_of_vertices();

    ///////////////////////////////////////////////////////////////////////////
    // Update the traced polygons to represent the partitioned src-mesh
    ///////////////////////////////////////////////////////////////////////////

    //
    // We are basically re-tracing the polygons that we traced earlier (in "m0").
    // These retraced polygon are stored in "m1". The re-traced polygons which
    // where next to teh cut-path will now reside (i.e. reference vertices and
    // halfedges) in the correct connected component to separate the mesh.
    //

    // the updated polygons (with the partitioning)
    std::vector<traced_polygon_t> m1_polygons;
    m1_polygons.resize(traced_sm_polygon_count); // resize to match

    // for each traced polygon (in "m0")
    for (std::vector<traced_polygon_t>::const_iterator m0_traced_sm_polygon_iter = m0_polygons.cbegin();
         m0_traced_sm_polygon_iter != m0_traced_sm_polygons_iter_cend;
         ++m0_traced_sm_polygon_iter) {
        const traced_polygon_t& m0_sm_polygon = *m0_traced_sm_polygon_iter; // m0 version (unpartitioned)
        // get index of polygon
        const int polygon_index = (int)std::distance(m0_polygons.cbegin(), m0_traced_sm_polygon_iter);

        MCUT_ASSERT(polygon_index < (int)m1_polygons.size()); // sanity check

        traced_polygon_t& m1_sm_polygon = m1_polygons.at(polygon_index); // m1 version (partitioned)
        m1_sm_polygon.resize(m0_sm_polygon.size()); // resize to match

        // for each halfedge of current polygon
        for (traced_polygon_t::const_iterator m0_traced_sm_polygon_halfedge_iter = m0_sm_polygon.cbegin();
             m0_traced_sm_polygon_halfedge_iter != m0_sm_polygon.cend();
             ++m0_traced_sm_polygon_halfedge_iter) {

            const hd_t& m0_he = *m0_traced_sm_polygon_halfedge_iter;
            const bool m0_he_src_is_ivertex = m0_is_intersection_point(m0.source(m0_he), ps_vtx_cnt);
            const bool m0_he_tgt_is_ivertex = m0_is_intersection_point(m0.target(m0_he), ps_vtx_cnt);
            // is the halfedge connected to an intersection point...?
            const bool is_ihalfedge = m0_he_src_is_ivertex || m0_he_tgt_is_ivertex;

            hd_t m1_he = mesh_t::null_halfedge();

            if (is_ihalfedge) { // its an intersection halfedge
                MCUT_ASSERT(m0_to_m1_ihe.find(m0_he) != m0_to_m1_ihe.cend()); // must have been walked/traversed

                m1_he = m0_to_m1_ihe.at(m0_he); // m1 version
            } else {
                MCUT_ASSERT(m0_to_m1_he.find(m0_he) != m0_to_m1_he.cend());

                m1_he = m0_to_m1_he.at(m0_he); // m1 version
            }

            // get halfedge index in polygon
            const int halfedge_index = (int)std::distance(m0_sm_polygon.cbegin(), m0_traced_sm_polygon_halfedge_iter);

            MCUT_ASSERT(halfedge_index < (int)m1_sm_polygon.size()); // array was resized with the same capacity as m0 polygon

            m1_sm_polygon.at(halfedge_index) = m1_he;
        }
    }

    m0_to_m1_he.clear();

    // NOTE: at this stage "m1_polygons" contains only src-mesh polygons.

    // extract the seam vertices
    std::map<vd_t, bool> m1_vertex_to_seam_flag;
    mark_seam_vertices(m1_vertex_to_seam_flag, m1, ps.number_of_vertices(), m1_num_vertices_after_srcmesh_partitioning);

    MCUT_ASSERT(!m1_vertex_to_seam_flag.empty());

    ///////////////////////////////////////////////////////////////////////////
    // Extract the partitioned connected components for output
    ///////////////////////////////////////////////////////////////////////////

    std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>> unsealed_connected_components;

    extract_connected_components(unsealed_connected_components, m1, m1_polygons, sm_polygons_below_cs, sm_polygons_above_cs, m1_vertex_to_seam_flag);

    // for each connected component (i.e. mesh)
    for (std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>::const_iterator cc_iter = unsealed_connected_components.cbegin();
         cc_iter != unsealed_connected_components.cend();
         ++cc_iter) {
        const int cc_id = static_cast<int>(cc_iter->first);
        const std::vector<std::pair<mesh_t, connected_component_info_t>>& mesh_data = cc_iter->second;

        // there will only be one element of the mesh since "unsealed_connected_components"
        // is empty before calling "extract_connected_components"
        MCUT_ASSERT(mesh_data.size() == 1);
        if(input.verbose){
        dump_mesh(mesh_data.front().first, ("fragment.unsealed." + std::to_string(cc_id) + "." + to_string(mesh_data.front().second.location)).c_str());
        }
        const std::pair<mesh_t, connected_component_info_t>& md = mesh_data.front();
        output_mesh_info_t omi;
        omi.mesh = md.first;
        omi.seam_vertices = std::move(md.second.seam_vertices);
        output.unsealed_cc[md.second.location].emplace_back(std::move(omi));
    }

    unsealed_connected_components.clear();

    ///////////////////////////////////////////////////////////////////////////
    // Check if the pipeline needs to terminate at this point
    ///////////////////////////////////////////////////////////////////////////

    //
    // that  we do not ever continue to patch fill holes if the cut-mesh cuts
    // the src-mesh multiple times where most cuts are complete but there is at-least one partial.
    // So long as there is a partial cut and the input mesh is not water tight, we wont patch.
    // This is because patching becomes complex as we then need to account for skipping the task of stitching patches which are incident to hole-bounding-sequences which are not loops.
    // Maybe future work..?
    const bool proceed_to_fill_holes = explicit_cutpaths_making_holes.size() == m0_explicit_cutpath_sequences.size();

    //
    // The pipeline stops here if there are no holes to fill.
    //
    // NOTE: 2D non-watertight meshes with a complete cut by a single cut-surface polygon cannot be "sealed"
    // since no edges of the cs polygon will be intersected.
    //

    if (proceed_to_fill_holes == false) {
        lg << "no holes to fill" << std::endl;

        lg << "end" << std::endl;
        return; // exit
    }

    m0_explicit_cutpath_sequences.clear(); // free, no longer needed.

    //lg << "finish before sealing!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    //std::exit(0);

    ///////////////////////////////////////////////////////////////////////////
    // The remainder of the pipeline performs hole-filling if there exist
    // atleast one from circular/linear cut-path which creates a hole in the
    // src-mesh.
    // Specifically, we will seal/patch each connected component by
    // identifying and then "stitching" patches of cut-mesh polygons to the
    // src-mesh connected components
    ///////////////////////////////////////////////////////////////////////////

    // Please refer to: `docs/notes.md`, section "Hole Filling"

    ///////////////////////////////////////////////////////////////////////////
    // Gather a primary intersection-halfedge for each patch (i.e. cut-path)
    ///////////////////////////////////////////////////////////////////////////

    // Here, we gather one interior intersection-halfedge (in "m0") for each patch,
    // (this halfedge is used to trace a cut-mesh polygon). We will use these
    // halfedges to iteratively build patches. Building a patch is analogous to
    // labelling each cut-mesh polygon with a patch id. We use halfedge along
    // cut paths since they mark/represent the border/boundary between patches.
    //
    // Thus, in the following std::vector, each element is a pair of a cut-mesh polygon index
    // and the index of a halfedge (on a cut-path) in that polygon ("m0" version).
    // Note that the referenced polygon will (by construction) be adjacent to the
    // border/boundary of the patch.
    //
    // The elements of this vector are used to initiate our graph search (building
    // the patch-graph), where a graph is a collection of one or more cut-mesh polygon
    // patches which are adjacent (i.e. sharing a cut-path).
    std::vector<std::pair<int, int>> primary_interior_ihalfedge_pool; // NOTE: pertains to all graphs

    // for each traced cut-mesh polygon (TODO: clean this up)
    for (std::vector<traced_polygon_t>::const_iterator cs_poly_iter = traced_cs_polygons_iter_cbegin;
         cs_poly_iter != m0_polygons.cend();
         ++cs_poly_iter) {
        const traced_polygon_t& cs_poly = *cs_poly_iter;
        const int cs_poly_idx = (int)std::distance(m0_polygons.cbegin(), cs_poly_iter);

        for (traced_polygon_t::const_iterator cs_poly_he_iter = cs_poly.cbegin();
             cs_poly_he_iter != cs_poly.cend();
             ++cs_poly_he_iter) {

            const hd_t& cs_poly_he = *cs_poly_he_iter;
            vd_t s = m0.source(cs_poly_he);
            vd_t t = m0.target(cs_poly_he);
            const bool is_ihalfedge = m0_is_intersection_point(s, ps_vtx_cnt) && m0_is_intersection_point(t, ps_vtx_cnt);
            if (!is_ihalfedge) {
                continue; // NOTE: This is how we ensure that the original cs-polygon is never referenced during the degenerate case of tet v tri (ccsplete cut)
            }

            bool is_ambiguious_interior_edge_case = m0_is_intersection_point(m0.source(cs_poly_he), ps_vtx_cnt) && m0_is_intersection_point(m0.target(cs_poly_he), ps_vtx_cnt);
            bool is_valid_ambiguious_interior_edge = false;

            if (is_ambiguious_interior_edge_case) { // exterior edge with two intersection vertices (ambigious case arising from concave polyhedron cut)

                const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0.source(cs_poly_he));
                const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0.target(cs_poly_he));
                const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                is_valid_ambiguious_interior_edge = (src_ps_edge != tgt_ps_edge); // see also above when gathering exterior incident edges
            }

            // TODO: use "cut-paths" to ensure that we store just one primary ihalfedge for each
            // cut-path ( at the moment its a extremely redundant and very slow to store all ihalfedges into the pool)
            if (!is_ambiguious_interior_edge_case || is_valid_ambiguious_interior_edge) { // check is interior ihalfedge (minor ambiguity which may arise with exterior interior halfedges x-->x)
                primary_interior_ihalfedge_pool.emplace_back(cs_poly_idx, (int)std::distance(cs_poly.cbegin(), cs_poly_he_iter));
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // Find graph(s) and build patches
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // Note that the built patches in this stage will have the same winding
    // order (e.g. counter-clock-wise ) as the input cut-mesh. The patches with
    // reversed winding order will be created later (once all patches are
    // identified).
    //
    // There is `normally` one graph which arises from the intersection of the
    // src-mesh with the cut-mesh. In this `normal` case, the graph is planar
    // and there is exactly one node with e.g. color "A" and the rest are "B", or
    // vice-versa. When visualised, it looks like a star, and this graph topology
    // is bipartite. We call this a "strongly-connected set" (SCS) because there
    // is a path from one graph node to another.
    //
    // An abnormality can also occurs when, we find more than one SCS. The SCSs
    // happen if there exists one or more floating patches. Thus, in this case,
    // each floating patch forms an SCS of its own because we cannot establish
    // adjacency with another (i.e. the exterior) patch.
    //
    // MapKey=patch index
    // MapValue=patch polygon-indices
    std::map<int, std::vector<int>> patches;
    // MapKey=traced cut-mesh polygon index
    // MapValue=patch index
    std::map<int, int> cs_poly_to_patch_idx;
    // This map stores an interior intersection-halfedge for each patch. This
    // halfedge also represents the traced cut-mesh polygon from which we will
    // start stitching/glueing the correspnding patch to a connected component.
    // This connected component will be one which has the same winding  as the
    // patch
    //
    // MapKey=patch index
    // MapValue=interior intersection-halfedge from which patch-stitching into
    //          a connected component will start from
    std::map<int, int> patch_to_seed_interior_ihalfedge_idx;
    //
    // MapKey=patch index
    // MapValue=index of patch-polygon that will be stitched first (from "patch_to_seed_interior_ihalfedge_idx[patch index]")
    std::map<int, int> patch_to_seed_poly_idx;
    // Interior patches must be stitched into separate connected component
    // as exterior patches so we create two versions of "m1" for that.
    //
    // MapKey=color value (representing the notion of "interior"/"exterior")
    // MapValue=the mesh (copy of "m1") to which corresponding patch(es) will be stitched
    std::map<char, mesh_t> color_to_m1 = { { 'A' /*e.g. "red"*/, m1 }, { 'B' /*e.g. "blue"*/, m1 } };
    //TODO: m1.free_data();

    // Patch (node) colors
    // NOTE: we use the letters 'A' and 'B' just to ensure lexicographical order
    // when we iterate over color_to_patch
    //
    // MapKey=color value
    // MapValue=list of patches of that color
    std::map<char, std::vector<int>> color_to_patch = { { 'A', std::vector<int>() }, { 'B', std::vector<int>() } };
    // We also tag each patch, identifying whether is it a floating patch or not.
    // All patches have an entry, including the reversed patches that are created
    // later.
    //
    // MapKey=patch id
    // MapValue=flag to indicate of patch is a floating patch.
    std::map<int, bool> patch_to_floating_flag;

    // tracks how many cs polygon have been stitched. Used only for naming damped meshes
    int global_cs_poly_stitch_counter = 0;
    // keeps track of the total number of CCW patches which has been identified
    // NOTE: not all will be CCW if we have floating patches (in this case winding
    // could be flipped)
    int total_ccw_patch_count = 0;

    // Each iteration of the following do-while loop will discover a strong-connected
    // set (SCS) of patches (i.e. our graph of nodes which will also be colored).
    //
    // Note: The total number of SCS's is unknown beforehand (typically one). Thus,
    // our goal in the following loop will be to find each SCS.
    do {
        ///////////////////////////////////////////////////////////////////////////
        // Associate cut-mesh polygons with patches of the current SCS
        ///////////////////////////////////////////////////////////////////////////

        // index of the first patch discovered
        const int cur_scs_1st_patch_idx = total_ccw_patch_count;
        // index of last-discovered patch
        int cur_scs_prev_patch_idx = cur_scs_1st_patch_idx;
        // index of the current patch
        int cur_scs_cur_patch_idx = cur_scs_1st_patch_idx;
        // counter to keep track of the number of patches discovered for
        // the current SCS
        int cur_scs_patch_counter = 0;
        // MapKey=patch index
        // MapValue=adjacent patches (i.e. sharing a cut-path)
        std::map<int, std::vector<int>> cur_scs_patch_to_adj_list;

        //
        // Here, we will pick an interior intersection halfedge (and its polygons) from
        // which we can indentify the [first patch] of the current SCS. This halfedge
        // and its polygon are called "seeds" because they seed the polygon search.
        //
        // This vector stores interior intersection-halfedges in the current
        // SCS whose cut-mesh polygons have not been associated with a patch.
        // Each element is defined as:
        // <patch index, cut-mesh polygon index, halfedge index>
        std::vector<std::tuple<int, int, int>> cur_scs_interior_ihalfedge_pool;

        // find the first interior intersection-halfedge of a cut-mesh polygon
        // which has not already been associated with a patch
        while (cur_scs_interior_ihalfedge_pool.empty() && !primary_interior_ihalfedge_pool.empty()) { // while seeds are not found

            // pull an interior intersection-halfedge from the queue
            std::vector<std::pair<int, int>>::const_iterator primary_interior_ihalfedge_pool_citer = primary_interior_ihalfedge_pool.cend() - 1; // last element
            // halfedge polygon index
            const int& potential_seed_poly_idx = primary_interior_ihalfedge_pool_citer->first;
            // halfedge index in polygon
            const int& potential_seed_poly_he_idx = primary_interior_ihalfedge_pool_citer->second;
            // check if the polygon has already been associated with a patch
            const bool poly_patch_is_known = cs_poly_to_patch_idx.find(potential_seed_poly_idx) != cs_poly_to_patch_idx.cend();

            if (!poly_patch_is_known) {

                // we can use the halfedge as a seed from which to starting point to build [a] patch
                cur_scs_interior_ihalfedge_pool.emplace_back(
                    cur_scs_cur_patch_idx,
                    potential_seed_poly_idx,
                    potential_seed_poly_he_idx);
            }

            // remove from the potential set
            primary_interior_ihalfedge_pool_citer = primary_interior_ihalfedge_pool.erase(primary_interior_ihalfedge_pool_citer);
        }

        // NOTE: an iteration of the following loop will build a patch/node.
        //
        // Cases when an iteration does not build a patch:
        //  1. if the elements in cur_scs_interior_ihalfedge_pool are interior-intersection
        // halfedges whose opposite halfedge is used to trace a cut-mesh polygon defining a
        // floating patch, and the reversed version of this cut-mesh polygon has already been
        // associated with a patch.
        while (!cur_scs_interior_ihalfedge_pool.empty()) { // TODO: this loop may have to go (see TODO comments at the end. Algorithm needs revision)

            // stores all ihalfedges (borders) of current patch
            // <patch idx, cs-polygon idx, he idx>halfedges which mark shared borders between patches
            std::vector<std::tuple<int, int, int>> cur_scs_patch_interior_ihalfedges;

            //
            // Find the seeds from which to begin building the current patch with flood-fill
            // A seed is an interior intersection-halfedge and together with its coincident cut-mesh polygon.
            //
            bool cur_scs_patch_seeds_found = false;
            // These are the seed variables
            int cur_scs_patch_seed_poly_he_idx = -1; // an interior ihalfedge
            int cur_scs_patch_seed_poly_idx = -1; // index of the cs polygon coincident to seed_he

            while (!cur_scs_patch_seeds_found && !cur_scs_interior_ihalfedge_pool.empty()) {
                // for each untested interior ihalfedge of last-built patch.

                // get element from pool of interior ihalfedges
                std::vector<std::tuple<int, int, int>>::iterator cur_scs_interior_ihalfedge_pool_elem = cur_scs_interior_ihalfedge_pool.end() - 1;

                // patch index
                const int pool_elem_patch_idx = std::get<0>(*cur_scs_interior_ihalfedge_pool_elem);
                // polygon index
                const int pool_elem_poly_idx = std::get<1>(*cur_scs_interior_ihalfedge_pool_elem);
                // halfedge index in polygon
                const int pool_elem_poly_he_idx = std::get<2>(*cur_scs_interior_ihalfedge_pool_elem);
                // the traced polygon
                const traced_polygon_t& pool_elem_poly = m0_polygons.at(pool_elem_poly_idx);
                // polygon halfedge
                const hd_t& pool_elem_poly_he = pool_elem_poly.at(pool_elem_poly_he_idx);

                // The following halfedge is an interior intersection-halfedge which belongs to a
                // patch that is [assumed] to have already been built and is adjacent to the new
                // patch that we are about to build. (hence the name "..._opp").
                //
                // This halfedge belongs to a polygon which may potentially be the seed if
                // the following conditions are held
                const hd_t pool_elem_poly_he_opp = m0.opposite(pool_elem_poly_he);
                vd_t s = m0.source(pool_elem_poly_he_opp);
                vd_t t = m0.target(pool_elem_poly_he_opp);
                //const bool is_ihalfedge = m0_is_intersection_point(s, ps_vtx_cnt) || m0_is_intersection_point(t, ps_vtx_cnt);
                //
                // We are now going to find the cut-mesh polygon which is traced with "pool_elem_poly_he_opp"
                //
                const std::vector<int>& coincident_polys = m0_h_to_ply.at(pool_elem_poly_he_opp); // coincident polygons (always one cut-mesh and one src-mesh)
                const std::vector<int>::const_iterator find_iter = std::find_if(coincident_polys.cbegin(), coincident_polys.cend(),
                    [&](const int& e) {
                        return (e >= traced_sm_polygon_count); // our condition for "cut-mesh" polygons
                    });

                // coincident cut-mesh polygon must exist because "pool_elem_poly_he_opp" is
                // an interior intersection-halfedge, and such halfedges are used to traced
                // both src-mesh and cut-mesh polygons
                MCUT_ASSERT(find_iter != coincident_polys.cend());

                // Now that we have found the cut-mesh polygon traced with "pool_elem_poly_he_opp",
                // the next step is to check that:
                // 1)  this polygon polygon is a not floating patch (in which case it opposite
                // polygon will have already been identified a forming a patch)
                const int potential_seed_poly_idx = *find_iter; // potential seed (starting) polygon for the new patch we want to build
                // has it already been associated with a patch..?
                const bool coincident_cs_poly_patch_is_known = cs_poly_to_patch_idx.find(potential_seed_poly_idx) != cs_poly_to_patch_idx.cend();

                if (!coincident_cs_poly_patch_is_known) {

                    // check if the coincident cut-mesh polygon is a floating-patch polygon because
                    // such polygons will already have their opposite traced too. The opposite polygon
                    // is already traced because our polygon tracing cannot distinguish the winding-order
                    // (CCW or CW) of polygons incident only to interior intersection-halfedges. So,
                    // the polygon-tracing routine will have produced both orientations of such polygons.
                    // bool is_floating_patch = true;
                    const traced_polygon_t& potential_seed_poly = m0_polygons.at(potential_seed_poly_idx);
                    bool is_floating_patch = check_is_floating_patch(potential_seed_poly, m0, ps, m0_ivtx_to_ps_he, ps_vtx_cnt, sm_face_count);
#if 0
                    // for each halfdge in the potential seed polygon
                    for (traced_polygon_t::const_iterator potential_seed_poly_he_iter = potential_seed_poly.cbegin();
                         potential_seed_poly_he_iter != potential_seed_poly.cend();
                         ++potential_seed_poly_he_iter) {

                        const vd_t src_vertex = m0.source(*potential_seed_poly_he_iter);
                        const vd_t tgt_vertex = m0.target(*potential_seed_poly_he_iter);
                        const bool is_ihalfedge = m0_is_intersection_point(src_vertex, ps_vtx_cnt) || m0_is_intersection_point(tgt_vertex, ps_vtx_cnt);

                        if (!is_ihalfedge) {
                            is_floating_patch = false;
                            // its an original halfedge : o-->o OR o<--o
                            // keep in mind that floating-patches are defined using only
                            // interior intersection-halfedges
                            break;
                        }

                        // check to make sure that the current halfedge is an interior
                        // intersection-halfedge i.e. it doesn't actually lie on the exterior
                        // of the current polygon

                        bool is_ambiguious_interior_edge_case = m0_is_intersection_point(src_vertex, ps_vtx_cnt) && m0_is_intersection_point(tgt_vertex, ps_vtx_cnt);
                        bool is_valid_ambiguious_interior_edge = false;

                        if (is_ambiguious_interior_edge_case) { // exterior edge with two intersection vertices (ambigious case arising from concave polyhedron cut)
                            // use the intersection registries to verify the edge
                            const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0.source(*potential_seed_poly_he_iter));
                            const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0.target(*potential_seed_poly_he_iter));
                            const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                            const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                            is_valid_ambiguious_interior_edge = (src_ps_edge != tgt_ps_edge);

                            if (!is_valid_ambiguious_interior_edge) {
                                // the current halfedge is "polygon-exterior interior intersection-halfedge"  x-->x
                                is_floating_patch = false;
                                break;
                            }
                        } else { // is exterior ihalfedge : o-->x OR x-->o
                            is_floating_patch = false;
                            break;
                        }
                    }
#endif
                    // at this stage, the potential seed polygon will be a true seed from which to start
                    // building a patch if the following conditions hold:
                    // 1) the potential polygon is a floating patch, which is the first built in the current SCS.
                    //  This is what prevents degeneracy of the adjacency matrix of the current SCS. OR
                    // 2) the potential seed polygon is not a floating patch

                    // NOTE: the condition "(cur_scs_patch_counter == 0)" below is needed to indicate
                    // that the a patch containing reverse  polygon of potential_seed_poly has not been created.
                    if ((is_floating_patch && (cur_scs_patch_counter == 0)) || !is_floating_patch) {

                        cur_scs_patch_seeds_found = true; // we have found the seed polygon of the current patch
                        cur_scs_patch_seed_poly_idx = potential_seed_poly_idx;
                        // get the seed polygon
                        const traced_polygon_t& cur_scs_patch_seed_poly = m0_polygons.at(cur_scs_patch_seed_poly_idx);
                        // get index of the halfedge we used to find the seed polygon
                        cur_scs_patch_seed_poly_he_idx = (int)std::distance(
                            cur_scs_patch_seed_poly.cbegin(),
                            std::find(cur_scs_patch_seed_poly.cbegin(), cur_scs_patch_seed_poly.cend(), pool_elem_poly_he_opp));

                        // update patch adjacency list for the current SCS if necessary.
                        if (cur_scs_cur_patch_idx != pool_elem_patch_idx) {
                            if (std::find(cur_scs_patch_to_adj_list[cur_scs_cur_patch_idx].cbegin(), cur_scs_patch_to_adj_list[cur_scs_cur_patch_idx].cend(), pool_elem_patch_idx) == cur_scs_patch_to_adj_list[cur_scs_cur_patch_idx].cend()) {
                                cur_scs_patch_to_adj_list[cur_scs_cur_patch_idx].push_back(pool_elem_patch_idx);
                            }

                            if (std::find(cur_scs_patch_to_adj_list[pool_elem_patch_idx].cbegin(), cur_scs_patch_to_adj_list[pool_elem_patch_idx].cend(), cur_scs_cur_patch_idx) == cur_scs_patch_to_adj_list[pool_elem_patch_idx].cend()) {
                                cur_scs_patch_to_adj_list[pool_elem_patch_idx].push_back(cur_scs_cur_patch_idx);
                            }
                        }

                        std::pair<std::map<int, bool>::const_iterator, bool> patch_to_floating_flag_insertion = patch_to_floating_flag.insert(std::make_pair(cur_scs_cur_patch_idx, is_floating_patch));
                        MCUT_ASSERT(patch_to_floating_flag_insertion.second == true);
                    }
                }

                cur_scs_interior_ihalfedge_pool_elem = cur_scs_interior_ihalfedge_pool.erase(cur_scs_interior_ihalfedge_pool_elem);

            } // while not found seeds

            if (!cur_scs_patch_seeds_found) {
                // done. We have identified all nodes (patches)
                // of the current SCS (or we simply couldn't find any
                // based on initial ihalfedges in the local pool)
                break;
            }

            MCUT_ASSERT(cur_scs_patch_seed_poly_he_idx != -1);

            ///////////////////////////////////////////////////////////////////////////
            // build the patch by flood-fill (BFS)
            ///////////////////////////////////////////////////////////////////////////

            // create patch entry
            std::pair<std::map<int, std::vector<int>>::iterator, bool> patch_insertion = patches.insert(std::make_pair(cur_scs_cur_patch_idx, std::vector<int>()));
            MCUT_ASSERT(patch_insertion.second == true);

            std::pair<std::map<int, int>::const_iterator, bool> seed_interior_ihalfedge_idx_insertion = patch_to_seed_interior_ihalfedge_idx.insert(std::make_pair(cur_scs_cur_patch_idx, cur_scs_patch_seed_poly_he_idx));
            MCUT_ASSERT(seed_interior_ihalfedge_idx_insertion.second == true);

            std::pair<std::map<int, int>::const_iterator, bool> seed_poly_idx_insertion = patch_to_seed_poly_idx.insert(std::make_pair(cur_scs_cur_patch_idx, cur_scs_patch_seed_poly_idx));
            MCUT_ASSERT(seed_poly_idx_insertion.second == true);

            std::vector<int>& patch = patch_insertion.first->second; // polygons of patch
            std::deque<int> flood_fill_queue; // for building patch using BFS
            flood_fill_queue.push_back(cur_scs_patch_seed_poly_idx); // first polygon

            do {

                // get the polygon at the front of the queue
                const int cur_scs_patch_poly_idx = flood_fill_queue.front();
                // add polygon to patch
                patch.push_back(cur_scs_patch_poly_idx);

                // relate polygon to patch
                std::pair<std::map<int, int>::const_iterator, bool> pair = cs_poly_to_patch_idx.insert(std::make_pair(cur_scs_patch_poly_idx, cur_scs_cur_patch_idx)); // signifies that polygon has been associated with a patch
                MCUT_ASSERT(pair.second == true);

                //
                // find adjacent polygons which share class 0,1,2 (o-->o, o-->x, x-->o) halfedges, and
                // the class 3 (x-->x) halfedges which are [exterior] intersection-halfedges.
                //

                // adjacent polygons to the current
                std::vector<int> adj_polys;
                // the current polygon
                const traced_polygon_t& cur_scs_patch_poly = m0_polygons.at(cur_scs_patch_poly_idx);

                // for each halfedge of the current polygon
                for (traced_polygon_t::const_iterator poly_he_iter = cur_scs_patch_poly.cbegin();
                     poly_he_iter != cur_scs_patch_poly.cend();
                     ++poly_he_iter) {

                    const vd_t src_vertex = m0.source(*poly_he_iter);
                    const vd_t tgt_vertex = m0.target(*poly_he_iter);
                    bool is_ambiguious_exterior_edge_case = m0_is_intersection_point(src_vertex, ps_vtx_cnt) && m0_is_intersection_point(tgt_vertex, ps_vtx_cnt);
                    bool is_valid_ambiguious_exterior_edge = false;

                    if (is_ambiguious_exterior_edge_case) { // exterior edge with two intersection vertices (ambigious case arising from concave polyhedron cut)

                        const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(src_vertex);
                        const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(tgt_vertex);
                        const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                        const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                        is_valid_ambiguious_exterior_edge = (src_ps_edge == tgt_ps_edge);
                    }

                    // "is the halfdge not along the cut-path"
                    if (!is_ambiguious_exterior_edge_case || is_valid_ambiguious_exterior_edge) {

                        // get the opposite halfedge which is used to trace the adjacent polygon
                        const hd_t poly_he_opp = m0.opposite(*poly_he_iter);
                        // get the coincident polygons (if any)
                        std::map<hd_t, std::vector<int>>::const_iterator find_iter = m0_h_to_ply.find(poly_he_opp);

                        // check if "poly_he_opp" is used to trace a polygon i.e its not a border halfedge (think partal cut tip)
                        if (find_iter != m0_h_to_ply.cend()) {

                            MCUT_ASSERT(find_iter->second.size() == 1); // only used to trace a CCW cut-mesh polygon

                            // get the polygon which is traced with "poly_he_opp" i.e. the adjacent polygon we are looking for!
                            const int incident_poly = find_iter->second.front();

                            // must cut-mesh polygon since we are only dealing with such polygons when building patches
                            MCUT_ASSERT(incident_poly >= traced_sm_polygon_count);

                            // does the patch already contain the polygon ...?
                            // TODO: search "cs_poly_to_patch_idx" which has O(log N) compared to O(N) here
                            const bool poly_already_in_patch = std::find(patch.cbegin(), patch.cend(), incident_poly) != patch.cend();

                            if (!poly_already_in_patch) {
                                const bool poly_already_in_queue = std::find(flood_fill_queue.cbegin(), flood_fill_queue.cend(), incident_poly) != flood_fill_queue.cend();
                                if (!poly_already_in_queue) {
                                    flood_fill_queue.push_back(incident_poly); // add adjacent polygon to bfs-queue
                                }
                            }
                        }
                    } else { // we have an interior-ihalfedge (i.e a border between patches)
                        const int cur_scs_patch_poly_he_idx = (int)std::distance(cur_scs_patch_poly.cbegin(), poly_he_iter);
                        // TODO: I think there is a logical innefficency in our path identification algorithm which
                        // leads to what we are doing here. I should not need to add [every] border/boundary halfedge
                        // in order to search for the adjacent patch. This may be why the algorithm does not scale
                        // so well.
                        //
                        // The algorithm need to be fixed to so that we instead just rely on the information we
                        // already have in "primary_interior_ihalfedge_pool" which contains at-least one
                        // border/boundary halfedge of each patch!!!
                        cur_scs_patch_interior_ihalfedges.emplace_back(cur_scs_cur_patch_idx, cur_scs_patch_poly_idx, cur_scs_patch_poly_he_idx); // NOTE: patches only join at interior ihalfedges
                    }
                }

                flood_fill_queue.pop_front(); // cur_scs_patch_poly_idx

            } while (!flood_fill_queue.empty()); // while there are more adjacent polygons for building current patch

            MCUT_ASSERT(!patch.empty()); // there has to be at least one polygon

            ++cur_scs_patch_counter;
            cur_scs_prev_patch_idx = cur_scs_cur_patch_idx;
            ++cur_scs_cur_patch_idx;

            // update the SCS pool with the interior ihalfedges (borders) of the current patch
            // TODO: again, this feel sub-optimal!!! (see TODO comment a few line above)
            cur_scs_interior_ihalfedge_pool.insert(cur_scs_interior_ihalfedge_pool.end(), cur_scs_patch_interior_ihalfedges.cbegin(), cur_scs_patch_interior_ihalfedges.cend());
            cur_scs_patch_interior_ihalfedges.clear();

        } // while there are more adjacent patches

        // NOTE: At this stage, we have identified all patches of the current graph

        if (cur_scs_patch_counter == 0) {
            // could not identitfy a patch with the halfedges which where available in the primary_interior_ihalfedge_pool.
            continue; // pick another ihalfedge from primary_interior_ihalfedge_pool
        }

        total_ccw_patch_count += cur_scs_patch_counter;
        const bool cur_scs_is_floating_patch = cur_scs_patch_counter == 1 && patch_to_floating_flag.at(cur_scs_prev_patch_idx) == true; // last created patch

        ///////////////////////////////////////////////////////////////////////////
        // Identify which patches are interior and which are exterior (coloring)
        ///////////////////////////////////////////////////////////////////////////

        //
        // We will now sort the patches into two sets - interior and exterior.
        // We do this by building an adjacency matrix and its square to produce
        // a bipartite graph via coloring. The adjacency matrix represents the
        // adjacency between patches (sharing a cut path).
        //

        math::matrix_t scs_adj_matrix(total_ccw_patch_count, total_ccw_patch_count); // square

        for (std::map<int, std::vector<int>>::const_iterator patch_iter = cur_scs_patch_to_adj_list.cbegin();
             patch_iter != cur_scs_patch_to_adj_list.cend();
             ++patch_iter) {

            const int row_id = patch_iter->first; // same as patch index

            for (std::vector<int>::const_iterator adj_patch_iter = patch_iter->second.cbegin();
                 adj_patch_iter != patch_iter->second.cend();
                 ++adj_patch_iter) {

                const int col_id = *adj_patch_iter;

                if (row_id == col_id) {
                    // our adjacency matrix is no self referent because patches
                    // do not connect to themselves!
                    continue;
                }

                scs_adj_matrix(row_id, col_id) = 1; // mark adjacent
            }
        }

        lg << "patch-graph adjacency matrix:\n"
           << scs_adj_matrix << std::endl;

        const math::matrix_t scs_adj_matrix_sqrd = scs_adj_matrix * scs_adj_matrix;

        lg << "squared:\n"
           << scs_adj_matrix_sqrd << std::endl;

        const bool is_1st_colored_scs = (cur_scs_1st_patch_idx == 0); // the first scs whose patches/nodes are to be colored

        if (is_1st_colored_scs) {

            // Here we do graph coloring using BFS
            // NOTE: coloring is used to mark patches as either interior or exterior.
            // Be aware that since we work only with the topology (connectivity), the
            // notion color itself will not tell us whether a patch is interior or
            // exterior. The coloring simply tells us that a patch belongs to one
            // group or the other. One exception is when we have a floating-patch
            // in which case it is possible to infer that the patch is interior.
            // This is because floating patches are always defined by interior
            // intersection-halfedges.

            std::deque<int> cur_scs_patch_coloring_queue;
            // start coloring with the first patch
            cur_scs_patch_coloring_queue.push_back(cur_scs_1st_patch_idx);
            // red chosen arbitrarilly
            std::vector<int>& red_nodes = color_to_patch.at('A');

            do { // color the current node/patch of the red set
                const int cur_scs_cur_colored_patch_idx = cur_scs_patch_coloring_queue.front();
                red_nodes.push_back(cur_scs_cur_colored_patch_idx);

                const int row_id = cur_scs_cur_colored_patch_idx; // NOTE: no need to account for the fact that the number of patches accumulates since cur_scs_1st_patch_idx == 0

                // find adjacent patch using A^2 and push adj patch onto queue (if not already colored)
                for (int col_id = 0; col_id < scs_adj_matrix_sqrd.cols(); ++col_id) {

                    if (row_id == col_id) {
                        continue; // we dont care about two-walks from a node back to itself
                    }

                    const unsigned int entry = scs_adj_matrix_sqrd(row_id, col_id);

                    if (entry > 0) // two-walk exists
                    {
                        const int cur_scs_next_colored_patch_idx = col_id;

                        if ( // not already colored
                            std::find(red_nodes.cbegin(), red_nodes.cend(), cur_scs_next_colored_patch_idx) == red_nodes.cend() &&
                            // not in queue
                            std::find(cur_scs_patch_coloring_queue.cbegin(), cur_scs_patch_coloring_queue.cend(), cur_scs_next_colored_patch_idx) == cur_scs_patch_coloring_queue.cend()) {
                            cur_scs_patch_coloring_queue.push_back(cur_scs_next_colored_patch_idx);
                        }
                    }
                }

                cur_scs_patch_coloring_queue.pop_front(); // rm cur_scs_cur_colored_patch_idx

            } while (!cur_scs_patch_coloring_queue.empty());

            // color the remaining uncolored nodes
            std::vector<int>& blue_nodes = color_to_patch.at('B'); // i.e. blue patches

            for (std::map<int, std::vector<int>>::const_iterator patch_iter = cur_scs_patch_to_adj_list.cbegin();
                 patch_iter != cur_scs_patch_to_adj_list.cend();
                 ++patch_iter) {

                const bool is_red = std::find(red_nodes.cbegin(), red_nodes.cend(), patch_iter->first) != red_nodes.cend();

                if (!is_red) {
                    blue_nodes.push_back(patch_iter->first);
                }
            }
        } else { // current SCS is not the first SCS to be colored (rarely happens but crucial)

            /*
                Objective: infering color value from an already-colored scs

                if the [1st patch of the current scs] is a [floating patch]
                    #
                    # Then any already-color scs will be either 
                    # 1) normal scs with at-least one interior patch and one exterior patch OR 
                    # 2) another floating patch
                    #

                    1. extract the vertices of the 1st patch of the current scs
                    2. find any already-colored scs
                    3. if <2> is a floating-patch scs
                    4.      infer color from <2> by using "color_to_patch" (same color because <2> is a floating patch and floating patches are always interior)
                    5. ELSE # <2> is a normal scs
                            error: we do not handle this case due ambiguities that arise because we only rely on topological information  e
                                
                else // the 1st patch of the current scs is a normal patch
                    #
                    # Any already-colored scs will be a floating patch.
                    # This stipulation follows one reason: 
                    #  - The current (to-be colored) scs has one or more patches: Floating-patches arise simply because their one & only polygon cannot have a topological connection to the normal scs. 
                    # 
                    error: we do not handle this case due ambiguities that arise because we only rely on topological information   

            */

            if (cur_scs_is_floating_patch) { // TODO: calculate "cur_scs_is_floating_patch" with "check_is_floating_patch(...)"
                // 1 get patch polygon vertices
                const std::vector<int>& first_patch_of_cur_graph = patches.at(cur_scs_1st_patch_idx);

                MCUT_ASSERT(first_patch_of_cur_graph.size() == 1); // ... floating patch

                // get index of the only polygon of patch
                const int& patch_poly_idx = first_patch_of_cur_graph.front();
                const traced_polygon_t& new_patch_poly = m0_polygons.at(patch_poly_idx); // the patch polygon
                std::vector<vd_t> new_polygon_vertices; // vertices of the polygon

                for (traced_polygon_t::const_iterator patch_poly_he_iter = new_patch_poly.cbegin();
                     patch_poly_he_iter != new_patch_poly.cend();
                     ++patch_poly_he_iter) {
                    const vd_t tgt = m0.target(*patch_poly_he_iter);
                    // floating-patch graphs only have one polygon, where that polygon is defined
                    // only by intersection vertices (think tet ccsplete cut example)
                    MCUT_ASSERT(m0_is_intersection_point(tgt, ps_vtx_cnt));
                    new_polygon_vertices.push_back(tgt);
                }

                // 2. find any already-colored scs
                std::map<int, std::vector<int>>::const_iterator colored_patch_find_iter = std::find_if(
                    patches.cbegin(), patches.cend(),
                    [&](const std::pair<int, std::vector<int>>& e) {
                        // we want to compare with the patches/nodes of the other scs (not the current scs)
                        // path indices of current (just created) scs will be bigger than patches of preceeding
                        // scs's
                        return e.first < cur_scs_1st_patch_idx;
                    });

                // must exist because the current scs is not the first (hence we why we are in this scope)
                MCUT_ASSERT(colored_patch_find_iter != patches.cend());

                // if <2> is a floating-patch scs
                //const bool colored_patch_is_floating_patch = colored_patch_find_iter->second.size() == 1;
                //const bool patch_has_one_polygon = colored_patch_find_iter->second.size() == 1;
                bool colored_patch_is_floating_patch = patch_to_floating_flag.at(colored_patch_find_iter->first) == true;

                //if (patch_has_one_polygon) {
                //     const int patch_poly_idx = colored_patch_find_iter->second.front();
                //     const traced_polygon_t& patch_poly = m0_polygons.at(patch_poly_idx);
                //    colored_patch_is_floating_patch = check_is_floating_patch(patch_poly, m0, ps, m0_ivtx_to_ps_he, ps_vtx_cnt, sm_face_count);
                //}

                if (colored_patch_is_floating_patch) {
                    // 4. infer color from <2> by using "color_to_patch"
                    // (same color because <2> is a floating patch and floating patches are always interior)
                    std::map<char, std::vector<int>>::const_iterator color_to_patch_idx_find_iter = std::find_if(
                        color_to_patch.cbegin(), color_to_patch.cend(),
                        [&](const std::pair<char, std::vector<int>>& e) {
                            return std::find(e.second.cbegin(), e.second.cend(), colored_patch_find_iter->first) != e.second.cend();
                        });

                    // if it is pre-existing then it should be assigned a colour already!
                    MCUT_ASSERT(color_to_patch_idx_find_iter != color_to_patch.cend());

                    // same color because they are both the new scs patch and the already-colored one are both floating patch scs's
                    color_to_patch.at(color_to_patch_idx_find_iter->first).push_back(cur_scs_1st_patch_idx);
                } else { // colored_patch_is_floating_patch == false

                    // NOTE: Keep in mind at this point that our objective have been to infer the color of a
                    // floating patch from an SCS which has >= 1 polygon i.e. its not a floating patch itself

                    lg.set_reason_for_failure("ambiguous configuraton: floating patch found with pre-existing regular patch(es)");
                    output.status = status_t::INVALID_MESH_INTERSECTION;
                    return; // exit
                } // if(colored_patch_is_floating_patch){

                // NOTE: at the end of this scope, we have inferred the color of the floating-patch in the current scs
            } else {
                // NOTE: Keep in mind at this point that our objective have been to infer the color of a
                // normal SCS from a floating patch

                lg.set_reason_for_failure("ambiguous configuraton: regular patch found with pre-existing floating patch(es)");
                output.status = status_t::INVALID_MESH_INTERSECTION;
                return; // exit
            } // if (cur_scs_is_floating_patch) {
        } // if (is_1st_colored_scs) {

        // NOTE: at this stage, all patches/nodes of the current graph have been coloured i.e. we have bipartite graph of the patches (except if there is only one patch i.e. a floating patch).
    } while (!primary_interior_ihalfedge_pool.empty()); // while there are more interior ihalfedges coincident to polygons which have not been associated with a patch (hence, there are remaining graphs of patches to be stitched)

    // NOTE: at this stage, all strongly-connected-sets have been identified and colored (i.e via coloring, all nodes/patches have been associated with a side : interior or exterior)

    MCUT_ASSERT(!patches.empty());

    primary_interior_ihalfedge_pool.clear();

    ///////////////////////////////////////////////////////////////////////////
    // check for further degeneracy before we proceed further
    ///////////////////////////////////////////////////////////////////////////

    //
    // Here look for cut-mesh patches whose boundary/border is not defined by
    // an intersection point with a src-mesh halfedge in its registry.
    // (stab cut)
    //

    if (sm_is_watertight && !cs_is_watertight) // The only scenario in which we may get a degenerate (partial cut) intersection
    {
        bool all_patches_okay = true;
        // for each color
        for (std::map<char, std::vector<int>>::const_iterator color_to_ccw_patches_iter = color_to_patch.cbegin();
             color_to_ccw_patches_iter != color_to_patch.cend();
             ++color_to_ccw_patches_iter) {

            //const char color_label = color_to_ccw_patches_iter->first;
            const std::vector<int>& colored_patches = color_to_ccw_patches_iter->second;

            // for each patch
            for (std::vector<int>::const_iterator patch_idx_iter = colored_patches.cbegin();
                 patch_idx_iter != colored_patches.cend();
                 ++patch_idx_iter) {

                const int& patch_idx = *patch_idx_iter;
                const std::vector<int>& patch = patches.at(patch_idx);
                bool patch_is_good = false;

                // for each polygon
                for (std::vector<int>::const_iterator patch_poly_idx_iter = patch.cbegin();
                     patch_poly_idx_iter != patch.cend();
                     ++patch_poly_idx_iter) {

                    const int& patch_poly_idx = *patch_poly_idx_iter;
                    const traced_polygon_t& patch_poly = m0_polygons.at(patch_poly_idx);

                    // for each halfedge of polygon
                    for (traced_polygon_t::const_iterator patch_poly_he_iter = patch_poly.cbegin(); patch_poly_he_iter != patch_poly.cend(); ++patch_poly_he_iter) {

                        const hd_t& patch_poly_he = *patch_poly_he_iter;
                        const vd_t patch_poly_he_src = m0.source(patch_poly_he);
                        const vd_t patch_poly_he_tgt = m0.target(patch_poly_he);
                        const bool patch_poly_he_src_is_ivertex = m0_is_intersection_point(patch_poly_he_src, ps_vtx_cnt);
                        const bool patch_poly_he_tgt_is_ivertex = m0_is_intersection_point(patch_poly_he_tgt, ps_vtx_cnt);

                        if (patch_poly_he_src_is_ivertex) {

                            const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(patch_poly_he_src);
                            const bool src_coincident_ps_halfedge_is_sm_halfedge = !ps_is_cutmesh_vertex(ps.target(src_coincident_ps_halfedge), sm_vtx_cnt);

                            if (src_coincident_ps_halfedge_is_sm_halfedge) {
                                patch_is_good = true;
                                break;
                            }
                        } else if (patch_poly_he_tgt_is_ivertex) {
                            const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(patch_poly_he_tgt);
                            const bool tgt_coincident_ps_halfedge_is_sm_halfedge = !ps_is_cutmesh_vertex(ps.target(tgt_ps_h), sm_vtx_cnt);
                            if (tgt_coincident_ps_halfedge_is_sm_halfedge) {
                                patch_is_good = true;
                                break;
                            }
                        }
                    }
                    if (patch_is_good) {
                        break;
                    }
                }

                if (!patch_is_good) {
                    all_patches_okay = false;
                    break;
                }
            }

            if (!all_patches_okay) {
                break;
            }
        }

        if (!all_patches_okay) {
            lg << "found dangling cs patch" << std::endl;
            output.status = status_t::INVALID_MESH_INTERSECTION; // ... which cannot be connected to sm because the cs is intersecting an sm face without cutting its edges
            return; // exit
        }
    } // if (sm_is_watertight && !cs_watertight )

    ///////////////////////////////////////////////////////////////////////////
    // Find the cut-mesh vertices that must not be duplicated
    ///////////////////////////////////////////////////////////////////////////

    // In the case of a partial cut, the o-vertices of the cut-mesh are not duplicated
    // e.g. those which reside interior to the sm
    std::vector<vd_t> sm_interior_cs_border_vertices;

    if (partial_cut_detected) {
        lg << "save non-duplicate intersection points" << std::endl;
        //
        // Here we save the cut-mesh border vertices (non-intersection points) which
        // are on the interior (inside) of the src-mesh.
        // These are needed for calculating properly-sealed connected components which
        // have been partially cut. We use this informaion determine which vertices of
        // the cut-mesh to not duplicate while allowing for the openings in the sealed
        // the connected components.
        //
        MCUT_ASSERT(!cs_is_watertight);

        /*
            1. do while there exists a re-entrant vertex which has not been used to find an interior cut-mesh border vertices
                a. get the ihalfedge whose source is <1> and it is used for tracing
                b.  do while target of next halfedge along border is not an intersection point
                    c. save target of current halfedge as one which we will not duplicate 
                    d. go to next halfedge along border 
        */

        // populate the queue with all cut-mesh tip re-entrant vertices
        std::deque<vd_t> reentrant_ivertex_queue(cs_tip_reentrant_ivtx_list.cbegin(), cs_tip_reentrant_ivtx_list.cend());

        do {

            // pull any re-entrant vertex from queue
            const vd_t current_reentrant_ivertex = reentrant_ivertex_queue.front();

            hd_t current_cs_border_he = mesh_t::null_halfedge();
            hd_t next_cs_border_he = mesh_t::null_halfedge();
            int current_cs_border_he_idx = -1;
            int next_cs_border_he_idx = -1;

            //
            // find polygon, and its halfedge whose src vertex is the current re-entrant vertex
            // "current_reentrant_ivertex", and the target is not an intersection point
            // keep in mind that we are looking for the non-intersection points that lie
            // inside the src-mesh - so that we dont duplicate them.
            //
            std::vector<traced_polygon_t>::const_iterator next_cs_border_he_poly_find_iter = std::find_if(
                m0_polygons.cbegin() + traced_sm_polygon_count, // offset to start of traced cut-mesh polygons
                m0_polygons.cend(),
                [&](const traced_polygon_t& cs_poly) {
                    // for each halfedge of cut-mesh polygon
                    for (traced_polygon_t::const_iterator cs_poly_he_iter = cs_poly.cbegin();
                         cs_poly_he_iter != cs_poly.cend();
                         ++cs_poly_he_iter) {

                        // check if the target is an intersection point
                        const vd_t tgt = m0.target(*cs_poly_he_iter);
                        const bool tgt_is_ivertex = m0_is_intersection_point(tgt, ps_vtx_cnt);

                        if (tgt_is_ivertex) { //..-->x
                            continue;
                        }

                        // is the halfedge on the border of the cut-mesh i.e. its opposite halfedge is not
                        // used to trace a polygon
                        const bool is_on_cs_border = m0_h_to_ply.find(m0.opposite(*cs_poly_he_iter)) == m0_h_to_ply.cend(); // opposite is used to traced a polygon

                        if (is_on_cs_border) {

                            // check if the src vertex of the current halfedge is a re-entrant vertex
                            // we search through the queue because it contains the tip re-entrant vertices
                            // that have not yet been visited (valid set).
                            // Note that this implies that src is also an intersection point
                            const vd_t src = m0.source(*cs_poly_he_iter);
                            const bool src_is_reentrant = std::find(reentrant_ivertex_queue.cbegin(), reentrant_ivertex_queue.cend(), src) != reentrant_ivertex_queue.cend();

                            if (src_is_reentrant) {

                                // we have found that first halfedge from which the remaining one(s)
                                // inside the src-mesh can be found
                                next_cs_border_he = *cs_poly_he_iter;
                                next_cs_border_he_idx = (int)std::distance(cs_poly.cbegin(), cs_poly_he_iter);
                                break;

                            } else {
                                continue;
                            }
                        } else {
                            continue;
                        }
                    }

                    return (next_cs_border_he != mesh_t::null_halfedge());
                });

            reentrant_ivertex_queue.pop_front(); // rm current_reentrant_ivertex

            // we could not find a halfedge whose src vertex is the current re-entrant vertex
            if (next_cs_border_he_poly_find_iter == m0_polygons.cend()) {
                // happens when a single cut-mesh partially cuts the src-mesh whereby
                // a single edge passes through 2 or more src-mesh faces e.g.
                // tet vs triangle partial cut
                continue;
            }

            // a reference to the polygon which is traced with the halfedge we found
            std::vector<traced_polygon_t>::const_iterator current_cs_border_he_poly_find_iter = m0_polygons.cend();

            //
            // we will now walk along the border of the cut-mesh saving all non
            // intersection points which must not be duplicated later
            //
            while (next_cs_border_he != mesh_t::null_halfedge()) {

                // current border halfedge
                current_cs_border_he = next_cs_border_he;
                // polygon of current border halfedge
                current_cs_border_he_poly_find_iter = next_cs_border_he_poly_find_iter;
                // index of current border halfedge
                current_cs_border_he_idx = next_cs_border_he_idx;

                // reset
                next_cs_border_he = mesh_t::null_halfedge();
                next_cs_border_he_idx = -1;
                next_cs_border_he_poly_find_iter = m0_polygons.cend();

                // save the non-intersection point on the border
                const vd_t current_cs_border_he_tgt = m0.target(current_cs_border_he);
                sm_interior_cs_border_vertices.push_back(current_cs_border_he_tgt);

                if (m0_is_intersection_point(current_cs_border_he_tgt, ps_vtx_cnt)) {
                    break; // done (finished walking along cut-mesh interior border)
                }

                //
                // find next halfedge along the border
                //

                // const int& current_cs_border_he_poly_idx = m0_h_to_ply.at(current_cs_border_he).front(); // NOTE: class-2 or class-1 ihalfedges are incident to only one polygon

                // the current polygon
                //const traced_polygon_t& current_cs_border_he_poly = *current_cs_border_he_poly_find_iter;
                // get reference to the current border halfedge in the polygon
                //const traced_polygon_t::const_iterator current_cs_border_he_find_iter = std::find(
                //    current_cs_border_he_poly.cbegin(), current_cs_border_he_poly.cend(), current_cs_border_he);

                // halfedge must exist in the polygon because it is used for tracing
                // MCUT_ASSERT(current_cs_border_he_find_iter != current_cs_border_he_poly.cend());

                //const int current_cs_border_he_idx = std::distance(current_cs_border_he_poly.cbegin(), current_cs_border_he_find_iter);

                // Here we now find the next border halfedge
                // -----------------------------------------

                // We do this by circulating around "current_cs_border_he_tgt" to find the next border halfedge
                // starting from the next after the current halfedge (around the vertex)
                std::vector<traced_polygon_t>::const_iterator next_he_poly_iter = current_cs_border_he_poly_find_iter;
                std::vector<traced_polygon_t>::const_iterator cur_he_poly_iter = m0_polygons.cend();
                int next_he_idx = wrap_integer(current_cs_border_he_idx + 1, 0, (int)next_he_poly_iter->size() - 1);
                int cur_he_idx = -1;
                hd_t next_he = next_he_poly_iter->at(next_he_idx);
                hd_t cur_he = mesh_t::null_halfedge();

                do {
                    cur_he = next_he;
                    MCUT_ASSERT(cur_he != mesh_t::null_halfedge());
                    next_he = mesh_t::null_halfedge();

                    cur_he_idx = next_he_idx;
                    MCUT_ASSERT(cur_he_idx != -1);
                    next_he_idx = -1;

                    cur_he_poly_iter = next_he_poly_iter;
                    MCUT_ASSERT(cur_he_poly_iter != m0_polygons.cend());
                    next_he_poly_iter = m0_polygons.cend();

                    // the next halfedge descriptor itself
                    //const hd_t& cur_he = current_cs_border_he_poly.at(cur_he_idx); // in the polygon of current_cs_border_he

                    // get the opposite of the next halfedge in order to enter the neighbouring
                    // polygon which has a border halfedge
                    const hd_t opp_of_cur_he = m0.opposite(cur_he);

                    bool opp_of_cur_he_is_border = m0_h_to_ply.find(opp_of_cur_he) == m0_h_to_ply.cend(); // opposite is used to traced a polygon

                    if (opp_of_cur_he_is_border) { // found!
                        next_cs_border_he = cur_he;
                        next_cs_border_he_idx = cur_he_idx;
                        next_cs_border_he_poly_find_iter = cur_he_poly_iter;
                    } else {

                        // get index of this neighouring polygon
                        MCUT_ASSERT(m0_h_to_ply.find(opp_of_cur_he) != m0_h_to_ply.cend());
                        const int& opp_of_cur_he_poly_idx = m0_h_to_ply.at(opp_of_cur_he).front(); // NOTE: class-2 or class-1 ihalfedges are incident to only one polygon
                        // reference to the neighbour/adjacent polygon
                        std::vector<traced_polygon_t>::const_iterator opp_of_cur_he_poly_iter = m0_polygons.cbegin() + (opp_of_cur_he_poly_idx);

                        MCUT_ASSERT(opp_of_cur_he_poly_iter != m0_polygons.cend());

                        // get the neighbouring polygon itself
                        const traced_polygon_t& opp_of_cur_he_poly = *opp_of_cur_he_poly_iter;
                        // get the reference to the next of opposite of current halfedge
                        const traced_polygon_t::const_iterator opp_of_cur_he_find_iter = std::find(
                            opp_of_cur_he_poly.cbegin(), opp_of_cur_he_poly.cend(), opp_of_cur_he);

                        MCUT_ASSERT(opp_of_cur_he_find_iter != opp_of_cur_he_poly.cend());

                        // index of "next of opposite of current halfedge" in the neighbour polygon
                        const int opp_of_cur_he_idx = (int)std::distance(opp_of_cur_he_poly.cbegin(), opp_of_cur_he_find_iter);
                        // get the next halfedge, which will be on the border
                        const int next_of_opp_of_cur_he_idx = wrap_integer(opp_of_cur_he_idx + 1, 0, (int)opp_of_cur_he_poly.size() - 1);

                        MCUT_ASSERT(next_of_opp_of_cur_he_idx < (int)opp_of_cur_he_poly.size());
                        const hd_t& next_of_opp_of_cur_he = opp_of_cur_he_poly.at(next_of_opp_of_cur_he_idx); // in the polygon of opp_of_next_he_poly
                        const hd_t opp_of_next_of_opp_of_cur_he = m0.opposite(next_of_opp_of_cur_he);
                        bool opp_of_next_of_opp_of_cur_he_is_border = m0_h_to_ply.find(opp_of_next_of_opp_of_cur_he) == m0_h_to_ply.cend(); // opposite is used to traced a polygon

                        //bool opp_of_next_of_opp_of_cur_he_is_border = m0_h_to_ply.find(opp_of_next_of_opp_of_cur_he) == m0_h_to_ply.cend(); // opposite is used to traced a polygon

                        if (opp_of_next_of_opp_of_cur_he_is_border) { // found!
                            next_cs_border_he = next_of_opp_of_cur_he;
                            next_cs_border_he_idx = next_of_opp_of_cur_he_idx;
                            next_cs_border_he_poly_find_iter = opp_of_cur_he_poly_iter;
                        } //else if (opp_of_next_of_opp_of_cur_he_is_border) {

                        // this is an edge-case:
                        // simple partial-cut intersection where an edge (of the cut-mesh) intersects two faces of the src-mesh

                        // get the target of the halfedge along the border of the adjacent polygon
                        //    const vd_t next_of_opp_of_cur_he_tgt = m0.target(next_of_opp_of_cur_he);

                        //     MCUT_ASSERT(m0_is_intersection_point(next_of_opp_of_cur_he_tgt, ps_vtx_cnt));

                        //    break;
                        //}
                        else {

                            const int& poly_idx = m0_h_to_ply.at(opp_of_next_of_opp_of_cur_he).front(); // NOTE: class-2 or class-1 ihalfedges are incident to only one polygon
                            // lg << "poly_idx=" << poly_idx << std::endl;
                            next_he_poly_iter = m0_polygons.cbegin() + (poly_idx);
                            MCUT_ASSERT(next_he_poly_iter != m0_polygons.cend());
                            const traced_polygon_t& poly = *next_he_poly_iter;
                            const traced_polygon_t::const_iterator he_find_iter = std::find(poly.cbegin(), poly.cend(), opp_of_next_of_opp_of_cur_he);
                            MCUT_ASSERT(he_find_iter != poly.cend());
                            const int idx = (int)std::distance(poly.cbegin(), he_find_iter);
                            // .. need to start from next-after, otherwise we end up in an infinite loop!
                            // see top of current do-while loop: i.e. this -> "opp_of_cur_he = m0.opposite(cur_he);" would
                            // bring us back into the current polygon
                            next_he_idx = wrap_integer(idx + 1, 0, (int)poly.size() - 1);
                            MCUT_ASSERT(next_he_idx < (int)poly.size());
                            // set next
                            // --------
                            next_he = poly[next_he_idx];
                        }
                    }

                    // while the next border halfedge has not been found OR
                    // if the next border he which is found is not actually equal to the current he
                } while (next_cs_border_he == mesh_t::null_halfedge());
            } // while (...)

        } while (!reentrant_ivertex_queue.empty());
    } // if (partial_cut_detected) {

    cs_tip_reentrant_ivtx_list.clear(); // free

    ///////////////////////////////////////////////////////////////////////////
    // Infer patch location based on graph coloring
    ///////////////////////////////////////////////////////////////////////////

    lg << "infer patch locations" << std::endl;

    //
    // Here we will now explicitly association location information to the graph
    // color. Thus for each color 'A', or 'B' we find out if it means "interior" or
    // "exterior" - in the geometric sense. Bare in mind that till this point the
    // notion graph node color was only used to classify patches into two sets.
    // The intuition behind this classification was that "there is two types" of
    // cut-mesh patches, those which are used to seal the interior of the src-mesh
    // and those used to seal the exterior". So here, we find out whether 'A' means
    // "interior" or "exterior", and vice versa for 'B'.
    //
    // DETAIL: Relying on topological alone to infer path location is insufficient to
    // determine whether a patch lies inside or outside the src-mesh (with the exception
    // of floating patches). There is ambiguity which prevents us from knowing exactly
    // what location each color 'A' or 'B' pertains to.
    //
    // NOTE: we will mark the exterior patches first to simplify the task of inferring
    // the color of floating patches if they exist (i.e. a situation when we have
    // multiple SCSs).
    // Consider this: If we mark interior patches first and the first
    // patch that is marked is a floating patch, then the task of inferring the color
    // of the remaining patches becomes complicated. First, of the two remaining patches,
    // we do not know which is interior or exterior because by definition the
    // floating patch which we have has no topological connectivity with them.
    // On the other hand, if we choose to mark exterior patches first, we then automatically
    // know that the remaining patches are interior irrespective of whether
    // they are floating patches or not. This is because exterior patches are never
    // floating-patches following the obervation that in order for the cut-mesh to be
    // recognised as having valid intersection the src-mesh, at least one src-mesh edge
    // must intersect a face from the cut-mesh.
    //

    std::map<char, cut_surface_patch_location_t> patch_color_label_to_location;

    // if no exterior cut-mesh polygons where found using re-entrant vertices
    if (known_exterior_cs_polygons.empty()) {
        //
        // entering this scope means that we have a floating patch.
        // Failing to find wny exterior patch(es) occurs only due the fact the
        // the cut-mesh intersectioned the src-mesh, but no edge in the cut-mesh
        // intersected a face of the src-mesh
        //
        //MCUT_ASSERT(patches.size() == 1);

        // What we will do then is assign cut-mesh  polygon a value of
        // cut_surface_patch_location_t::INSIDE since floating patches
        // are always interior

        const int patch_idx = patches.cbegin()->first;

        MCUT_ASSERT(patches.cbegin()->second.size() == 1); // should only be one polygon due to fact of having floating patch

        // find the colored entry containing the patch
        std::map<char, std::vector<int>>::const_iterator color_to_ccw_patches_find_iter = std::find_if(
            color_to_patch.cbegin(),
            color_to_patch.cend(),
            [&](const std::pair<char, std::vector<int>>& e) {
                return std::find(e.second.cbegin(), e.second.cend(), patch_idx) != e.second.cend();
            });

        // all patches must be associated with a patch by this point
        MCUT_ASSERT(color_to_ccw_patches_find_iter != color_to_patch.cend());

        //
        // We have successively inferred that the color label ('A' or 'B')
        // associated with the floating patch corresponds to "interior"
        //
        const char color_label = color_to_ccw_patches_find_iter->first;
        patch_color_label_to_location.insert(std::make_pair(color_label, cut_surface_patch_location_t::INSIDE));

        // So, given that we know what the other color label is, we can infer
        // the location associaed with the remaining color
        patch_color_label_to_location.insert(std::make_pair(color_label == 'A' ? 'B' : 'A', cut_surface_patch_location_t::OUTSIDE));
    } else {
        //
        // Here, we know the at least one polygon which lies on the exterior of the
        // src-mesh.
        // So lets find the patch which contains any such polygon and label this patch
        // as being "exterior".
        //

        // for each cut-mesh polygon
        // TODO: why don't we just loop over "known_exterior_cs_polygons" if we are just looking for one
        // exterior polygon???!!!
        for (std::map<int, int>::const_iterator cs_poly_to_patch_idx_iter = cs_poly_to_patch_idx.cbegin();
             cs_poly_to_patch_idx_iter != cs_poly_to_patch_idx.cend();
             ++cs_poly_to_patch_idx_iter) {

            // get index of polygon
            const int cs_poly_idx = cs_poly_to_patch_idx_iter->first;

            // check if polygon is an exterior cut-mesh polygon
            const std::vector<std::pair<int, int>>::const_iterator known_exterior_cs_polygons_find_iter = std::find_if(
                known_exterior_cs_polygons.cbegin(),
                known_exterior_cs_polygons.cend(),
                [&](const std::pair<int, int>& e) { return e.first == cs_poly_idx; });
            const bool poly_is_known_exterior_cs_polygon = (known_exterior_cs_polygons_find_iter != known_exterior_cs_polygons.cend());

            if (poly_is_known_exterior_cs_polygon) {
                // get the patch containing the polygon
                const int patch_idx = cs_poly_to_patch_idx_iter->second;

                // get the color of the patch
                std::map<char, std::vector<int>>::const_iterator color_to_ccw_patches_find_iter = std::find_if(
                    color_to_patch.cbegin(),
                    color_to_patch.cend(),
                    [&](const std::pair<char, std::vector<int>>& e) {
                        return std::find(e.second.cbegin(), e.second.cend(), patch_idx) != e.second.cend();
                    });

                MCUT_ASSERT(color_to_ccw_patches_find_iter != color_to_patch.cend());

                // thus, the color of the patch means it is an "exterior" patch because it contains an exterior
                // polygon
                const char color_label = color_to_ccw_patches_find_iter->first;
                patch_color_label_to_location.insert(std::make_pair(color_label, cut_surface_patch_location_t::OUTSIDE));

                // infer the opposite color label's meaning
                patch_color_label_to_location.insert(std::make_pair(color_label == 'A' ? 'B' : 'A', cut_surface_patch_location_t::INSIDE));

                break; // done (only need to find the first exterior polygon for use to know everything else)
            }
        }
    }

    MCUT_ASSERT(!patch_color_label_to_location.empty());

    known_exterior_cs_polygons.clear();
    cs_poly_to_patch_idx.clear();

    // dump
    lg << "color label values (dye)" << std::endl;
    lg.indent();
    for (std::map<char, std::vector<int>>::const_iterator color_to_ccw_patches_iter = color_to_patch.cbegin(); color_to_ccw_patches_iter != color_to_patch.cend(); ++color_to_ccw_patches_iter) {
        const char color_label = color_to_ccw_patches_iter->first;
        const cut_surface_patch_location_t color_label_dye = patch_color_label_to_location.at(color_label);

        lg << (char)color_label << "=" << (color_label_dye == cut_surface_patch_location_t::OUTSIDE ? "exterior" : "interior") << std::endl;
    }
    lg.unindent();

    ///////////////////////////////////////////////////////////////////////////
    // Create reverse patches
    ///////////////////////////////////////////////////////////////////////////

    lg << "create reversed patches" << std::endl;

    const int traced_polygon_count = (int)m0_polygons.size(); // does not include the reversed cs-polygons

    // note: reversed patches are called "cw" patches

    // MapKey=color
    // MapValue=reversed patch index
    std::map<char, std::vector<int>> color_to_cw_patch;
    // MapKey=patch index
    // MapValue=opposite patch index
    std::map<int, int> patch_to_opposite;

    lg << "colors = " << color_to_patch.size() << std::endl;

    // for each color
    for (std::map<char, std::vector<int>>::const_iterator color_to_ccw_patches_iter = color_to_patch.cbegin();
         color_to_ccw_patches_iter != color_to_patch.cend();
         ++color_to_ccw_patches_iter) {
        lg.indent();

        const char color_id = color_to_ccw_patches_iter->first;

        lg << "color = " << color_id << " (" << (patch_color_label_to_location.at(color_id) == cut_surface_patch_location_t::OUTSIDE ? "exterior" : "interior") << ")" << std::endl;

        // add entry
        std::pair<std::map<char, std::vector<int>>::iterator, bool> color_to_cw_patch_insertion = color_to_cw_patch.insert(std::make_pair(color_to_ccw_patches_iter->first, std::vector<int>()));

        MCUT_ASSERT(color_to_cw_patch_insertion.second == true);

        // list of reversed patches with current color
        std::vector<int>& cw_patch_color = color_to_cw_patch_insertion.first->second;

        lg << "patch count = " << color_to_ccw_patches_iter->second.size() << std::endl;

        // for each patch with current color
        for (std::vector<int>::const_iterator patch_iter = color_to_ccw_patches_iter->second.cbegin(); patch_iter != color_to_ccw_patches_iter->second.cend(); ++patch_iter) {
            lg.indent();

            const int patch_idx = *patch_iter;

            lg << "patch = " << patch_idx << " (normal)" << std::endl;

            const std::vector<int>& patch = patches.at(patch_idx);

            //
            // create reversed patch
            //

            const int cw_patch_idx = (int)patches.size();

            // relate patch to opposite
            patch_to_opposite[patch_idx] = cw_patch_idx;
            patch_to_opposite[cw_patch_idx] = patch_idx;

            lg << "patch = " << cw_patch_idx << " (reversed)" << std::endl;

            std::pair<std::map<int, std::vector<int>>::iterator, bool> patch_insertion = patches.insert(std::make_pair(cw_patch_idx, std::vector<int>()));
            MCUT_ASSERT(patch_insertion.second == true);

            std::vector<int>& cw_patch = patch_insertion.first->second;

            // add to list of patches with current color
            cw_patch_color.push_back(cw_patch_idx);

            /* 
                for each polygon in patch
                    if is floating-patch polygon
                        find the opposite polygon which already exists
                        add into opposite polygon into patch
                    else
                        create reversed version and update data structures
            */

            // number of polygon in the normal patch
            const int initial_patch_size = (int)patch.size();

            // for each polygon in the normal patch
            for (int ccw_patch_iter = 0; ccw_patch_iter < initial_patch_size; ++ccw_patch_iter) {

                lg.indent();

                // get the polygon index
                const int ccw_patch_poly_idx = patch.at(ccw_patch_iter);

                lg << "polygon = " << ccw_patch_poly_idx << " (normal)" << std::endl;

                // all polygon are stored in the same array so we can use that to infer
                // index of new reversed polygon
                int cw_poly_idx = (int)m0_polygons.size();
                // get the normal polygon
                const traced_polygon_t& patch_poly = m0_polygons.at(ccw_patch_poly_idx);
                const bool is_floating_patch = patch_to_floating_flag.at(patch_idx);

                if (is_floating_patch) {
                    //
                    // the reversed polygon of the current normal polygon already exists.
                    // So we can just use that. Note: this is because the polygon tracing
                    // that we did earlier (in "m0") always traces two versions of a polygon
                    // whose halfedges are all interior intersection-halfedges (x-->x)
                    //

                    // find opposite polygon
                    const hd_t& coincident_halfedge = patch_poly.front(); // can be any
                    const hd_t coincident_halfedge_opp = m0.opposite(coincident_halfedge);
                    const std::vector<int>& coincident_polys = m0_h_to_ply.at(coincident_halfedge_opp);
                    // find coincident cut-mesh polygon
                    const std::vector<int>::const_iterator coincident_polys_find_iter = std::find_if(
                        coincident_polys.cbegin(),
                        coincident_polys.cend(),
                        [&](const int& e) { return e >= traced_sm_polygon_count && e < traced_polygon_count; });

                    MCUT_ASSERT(coincident_polys_find_iter != coincident_polys.cend());

                    const int patch_poly_opp = *coincident_polys_find_iter;
                    cw_poly_idx = patch_poly_opp;
                } else {

                    //
                    // the current normal polygon does not form a floating patch,
                    // so we calculate the reversed polygon by to retracing the
                    // current normal polygon in reverse order
                    //

                    traced_polygon_t cw_poly;
                    traced_polygon_t tmp;

                    // for each halfedge of the normal polygon
                    for (traced_polygon_t::const_iterator patch_poly_he_iter = patch_poly.cbegin();
                         patch_poly_he_iter != patch_poly.cend();
                         ++patch_poly_he_iter) {

                        // get halfedge descriptor
                        const hd_t& patch_poly_he = *patch_poly_he_iter;
                        // get the opposite halfedge
                        const hd_t patch_poly_he_opp = m0.opposite(patch_poly_he);
                        // add into list defining reversed polygon
                        tmp.push_back(patch_poly_he_opp);
                        // check if another cut-mesh polygon is traced with this opposite halfedge.
                        std::map<hd_t, std::vector<int>>::iterator find_iter = m0_h_to_ply.find(patch_poly_he_opp);

                        if (find_iter == m0_h_to_ply.end()) { // "patch_poly_he_opp" not used to trace any polygon
                            //
                            // we only enter this scope of the halfedge "patch_poly_he_opp" is a
                            // border halfedge which is not used to trace a cut-mesh polygon.

                            // add entry for the halfedge
                            std::pair<std::map<hd_t, std::vector<int>>::iterator, bool> m0_he_to_poly_idx_insertion = m0_h_to_ply.insert(std::make_pair(patch_poly_he_opp, std::vector<int>()));

                            MCUT_ASSERT(m0_he_to_poly_idx_insertion.second == true);

                            find_iter = m0_he_to_poly_idx_insertion.first;
                        }

                        // associate "patch_poly_he_opp" with the new reversed polygon
                        find_iter->second.push_back(cw_poly_idx);
                    }

                    MCUT_ASSERT(tmp.size() == patch_poly.size());

                    // reverse the order to ensure correct winding, last for goes to beginning, and so on...
                    for (int h = 0; h < (int)tmp.size(); ++h) {
                        const int index = (int)tmp.size() - 1 - h;
                        cw_poly.push_back(tmp.at(index));
                    }

                    lg << "polygon = " << cw_poly_idx << " (reversed)" << std::endl;

                    lg.indent();
                    {
                        for (traced_polygon_t::const_iterator cw_poly_he_iter = cw_poly.cbegin(); cw_poly_he_iter != cw_poly.cend(); ++cw_poly_he_iter) {
                            lg << *cw_poly_he_iter << " <" << m0.source(*cw_poly_he_iter) << ", " << m0.target(*cw_poly_he_iter) << ">" << std::endl;
                        }
                    }
                    lg.unindent();

                    MCUT_ASSERT(m0.source(cw_poly.front()) == m0.target(cw_poly.back())); // must form loop

                    // save the new polygon
                    m0_polygons.push_back(cw_poly);
                }

                // the the new polygon's index as being part of the patch
                cw_patch.push_back(cw_poly_idx);

                lg.unindent();
            }
            lg.unindent();
        }
        lg.unindent();
    }

    // number of reversed cut-mesh polygons
    const int cw_cs_poly_count = ((int)m0_polygons.size() - traced_polygon_count);

    lg << "reversed cut-mesh polygons = " << cw_cs_poly_count << std::endl;

    // NOTE: at this stage, all patch polygons (normal) also have an opposite (reversed)

    lg << "merge normal and reversed patches" << std::endl;

    // merge the opposite color_to_patch data structure

    // for each color
    for (std::map<char, std::vector<int>>::const_iterator color_to_cw_patch_iter = color_to_cw_patch.cbegin();
         color_to_cw_patch_iter != color_to_cw_patch.cend();
         ++color_to_cw_patch_iter) {
        lg.indent();

        const char color_value = color_to_cw_patch_iter->first;

        lg << "color = " << color_value << " (" << (patch_color_label_to_location.at(color_value) == cut_surface_patch_location_t::OUTSIDE ? "exterior" : "interior") << ")" << std::endl;

        // get reversed patches
        const std::vector<int>& colored_cw_patches = color_to_cw_patch_iter->second;
        // get normal patches
        std::vector<int>& colored_patches = color_to_patch.at(color_value);
        // merge
        colored_patches.insert(colored_patches.end(), colored_cw_patches.cbegin(), colored_cw_patches.cend()); // merge

        // dump
        if (input.verbose) {
            lg << "total patches = " << colored_patches.size() << std::endl;

            for (std::vector<int>::const_iterator colored_patch_iter = colored_patches.cbegin();
                 colored_patch_iter != colored_patches.cend();
                 ++colored_patch_iter) {
                lg.indent();

                const int patch_idx = *colored_patch_iter;
                const std::vector<int>& patch = patches.at(patch_idx);
                const int is_ccw = (int)(std::distance(colored_patches.cbegin(), colored_patch_iter) < (int)(patch.size() / 2));

                lg << "patch = " << patch_idx << " (" << (is_ccw ? "normal" : "reversed") << ")" << std::endl;

                lg.indent();
                lg << "polygons=" << patch.size() << " :";
                for (std::vector<int>::const_iterator patch_poly_iter = patch.cbegin(); patch_poly_iter != patch.cend(); ++patch_poly_iter) {

                    lg << " " << *patch_poly_iter;
                }
                lg << std::endl;
                lg.unindent();

                lg.unindent();
            }
        }

        lg.unindent();
    }

    ///////////////////////////////////////////////////////////////////////////
    // save the patches into the output
    ///////////////////////////////////////////////////////////////////////////

    lg << "save patch meshes" << std::endl;

    // for each color
    for (std::map<char, std::vector<int>>::const_iterator color_to_patches_iter = color_to_patch.cbegin();
         color_to_patches_iter != color_to_patch.cend();
         ++color_to_patches_iter) {
        lg.indent();
        const char color_id = color_to_patches_iter->first;
        lg << "color=" << (char)color_id << std::endl;

        // for each patch with current color
        for (std::vector<int>::const_iterator patch_iter = color_to_patches_iter->second.cbegin();
             patch_iter != color_to_patches_iter->second.cend();
             ++patch_iter) {
            lg.indent();
            const int cur_patch_idx = *patch_iter;

            //
            // create mesh for patch
            //
            mesh_t patch_mesh;
            // MapKey=vertex descriptor in "m0"
            // MapValue=vertex descriptor in "patch_mesh"
            std::map<vd_t, vd_t> m0_to_patch_mesh_vertex;

            lg << "patch -  " << cur_patch_idx << std::endl;

            // NOTE: normal patches are created before their reversed counterparts (hence the modulo Operator trick)

            // is the a normal patch
            const bool is_ccw_patch = ((cur_patch_idx % total_ccw_patch_count) == cur_patch_idx);
            const cut_surface_patch_winding_order_t patch_descriptor = is_ccw_patch ? cut_surface_patch_winding_order_t::DEFAULT : cut_surface_patch_winding_order_t::REVERSE;
            const std::string cs_patch_descriptor_str = to_string(patch_descriptor);

            lg << "is " << cs_patch_descriptor_str << " patch" << std::endl;

            // get the patch's polygons
            const std::vector<int>& patch = patches.at(cur_patch_idx);

            //
            // add vertices into patch mesh
            //

            std::vector<vd_t> seam_vertices;

            // for each polygon in the patch
            for (std::vector<int>::const_iterator patch_poly_iter = patch.cbegin(); patch_poly_iter != patch.cend(); ++patch_poly_iter) {

                const int& patch_poly_idx = *patch_poly_iter;
                const traced_polygon_t& patch_poly = m0_polygons.at(patch_poly_idx);

                // for each halfedge of polygon
                for (traced_polygon_t::const_iterator patch_poly_he_iter = patch_poly.cbegin();
                     patch_poly_he_iter != patch_poly.cend();
                     ++patch_poly_he_iter) {

                    const vd_t m0_vertex = m0.target(*patch_poly_he_iter);
                    const bool vertex_already_mapped = m0_to_patch_mesh_vertex.find(m0_vertex) != m0_to_patch_mesh_vertex.cend();

                    if (!vertex_already_mapped) {
                        // map from "m0" to "patch_mesh" descriptor
                        const vd_t& patch_mesh_vertex = patch_mesh.add_vertex(m0.vertex(m0_vertex));

                        MCUT_ASSERT(patch_mesh_vertex != mesh_t::null_halfedge());

                        m0_to_patch_mesh_vertex.insert(std::make_pair(m0_vertex, patch_mesh_vertex));

                        // mark if is seam vertex
                        if (m0_is_intersection_point(m0_vertex, ps_vtx_cnt)) {
                            seam_vertices.push_back(patch_mesh_vertex); // seam vertices are intersection points
                        }
                    }
                }
            }

            MCUT_ASSERT(!seam_vertices.empty());

            //
            // add faces into patch mesh
            //

            // for each polygon
            for (std::vector<int>::const_iterator patch_poly_iter = patch.cbegin(); patch_poly_iter != patch.cend(); ++patch_poly_iter) {

                const int& patch_poly_idx = *patch_poly_iter;
                const traced_polygon_t& patch_poly = m0_polygons.at(patch_poly_idx);

                std::vector<vd_t> remapped_poly_vertices; // redefined face using "patch_mesh" descriptors

                // for each halfedhe
                for (traced_polygon_t::const_iterator patch_poly_he_iter = patch_poly.cbegin();
                     patch_poly_he_iter != patch_poly.cend();
                     ++patch_poly_he_iter) {
                    const vd_t m0_vertex = m0.target(*patch_poly_he_iter);
                    const vd_t patch_mesh_vertex = m0_to_patch_mesh_vertex.at(m0_vertex);
                    remapped_poly_vertices.push_back(patch_mesh_vertex);
                }

                const fd_t f = patch_mesh.add_face(remapped_poly_vertices);

                MCUT_ASSERT(f != mesh_t::null_face());
            }

            const cut_surface_patch_location_t& patch_location = patch_color_label_to_location.at(color_id);

            if (input.verbose) {
                dump_mesh(patch_mesh, ("patch" + std::to_string(cur_patch_idx) + "." + to_string(patch_location) + "." + cs_patch_descriptor_str).c_str());
            }

            output_mesh_info_t omi;
            omi.mesh = std::move(patch_mesh);
            omi.seam_vertices = std::move(seam_vertices);

            if (patch_location == cut_surface_patch_location_t::INSIDE) {
                output.inside_patches[patch_descriptor].emplace_back(std::move(omi));
            } else {
                output.outside_patches[patch_descriptor].emplace_back(std::move(omi));
            }
            lg.unindent();
        }
        lg.unindent();
    }

    ///////////////////////////////////////////////////////////////////////////
    // calculate the reversed patch seeds
    ///////////////////////////////////////////////////////////////////////////

    //
    // Here, we will infer the seed interior intersection-halfedges and polygons
    // for the newly create reversed polygons. We will also save information telling
    // whether each reversed patch is a floating patch or not. We will use this
    // information during stitching
    //
    lg << "calculate reversed-patch seed variables" << std::endl;

    // for each color
    for (std::map<char, std::vector<int>>::const_iterator color_to_cw_patch_iter = color_to_cw_patch.cbegin();
         color_to_cw_patch_iter != color_to_cw_patch.cend();
         ++color_to_cw_patch_iter) {
        lg.indent();

        const char color_value = color_to_cw_patch_iter->first;

        lg << "color = " << color_value << " (" << (patch_color_label_to_location.at(color_value) == cut_surface_patch_location_t::OUTSIDE ? "exterior" : "interior") << ")" << std::endl;

        // get the reversed patch of the current color
        const std::vector<int>& colored_cw_patches = color_to_cw_patch_iter->second;

        lg << "patches = " << colored_cw_patches.size() << std::endl;

        // for each patch
        for (std::vector<int>::const_iterator colored_cw_patch_iter = colored_cw_patches.cbegin();
             colored_cw_patch_iter != colored_cw_patches.cend();
             ++colored_cw_patch_iter) {
            lg.indent();

            const int cw_patch_idx = *colored_cw_patch_iter;

            lg << "patch = " << cw_patch_idx << " (reversed)" << std::endl;

            // get patch polygons
            const std::vector<int>& cw_patch = patches.at(cw_patch_idx);
            // get the opposite patch
            const int ccw_patch_idx = patch_to_opposite.at(cw_patch_idx); //opposite patch

            //
            // copy information from opposite (normal) patch
            //
            std::pair<std::map<int, bool>::const_iterator, bool> patch_to_floating_flag_insertion = patch_to_floating_flag.insert(
                std::make_pair(cw_patch_idx, patch_to_floating_flag.at(ccw_patch_idx)));

            MCUT_ASSERT(patch_to_floating_flag_insertion.second == true);

            // was the opposite patch determined to be a floating patch
            const bool is_floating_patch = patch_to_floating_flag_insertion.first->second;
            // get the index of seed interior intersection halfedge of the opposite normal patch
            const int ccw_patch_seed_interior_ihalfedge_idx = patch_to_seed_interior_ihalfedge_idx.at(ccw_patch_idx);
            // get the index of seed polygon of the opposite normal patch
            const int ccw_patch_seed_poly_idx = patch_to_seed_poly_idx.at(ccw_patch_idx);
            // get the seed polygon of the opposite normal patch
            const traced_polygon_t& ccw_patch_seed_poly = m0_polygons.at(ccw_patch_seed_poly_idx);
            // get the seed interior intersection halfedge of the opposite normal patch
            const hd_t& ccw_patch_seed_interior_ihalfedge = ccw_patch_seed_poly.at(ccw_patch_seed_interior_ihalfedge_idx);
            // opposite halfedge of the seed interior intersection halfedge of the opposite normal patch
            const hd_t ccw_patch_seed_interior_ihalfedge_opp = m0.opposite(ccw_patch_seed_interior_ihalfedge);

            lg << "seed interior intersection-halfedge = " << ccw_patch_seed_interior_ihalfedge_opp << std::endl;

            // find the reversed polygon which uses "ccw_patch_seed_interior_ihalfedge_opp"
            // this will be the seed polygon of the current reversed patch
            const std::vector<int>& coincident_polys = m0_h_to_ply.at(ccw_patch_seed_interior_ihalfedge_opp);

            std::vector<int>::const_iterator find_iter = std::find_if(coincident_polys.cbegin(), coincident_polys.cend(),
                [&](const int& e) {
                    if (is_floating_patch) {
                        return e >= traced_sm_polygon_count && e < traced_polygon_count; // interior ihalfedges of floating patches are already coincident to two polygons due to polygon tracing
                    } else {
                        return e >= traced_polygon_count;
                    }
                });

            MCUT_ASSERT(find_iter != coincident_polys.cend());

            // the index of the seed polygon of the current reversed patch
            const int cw_patch_seed_poly_idx = *find_iter;

            lg << "seed polygon = " << cw_patch_seed_poly_idx << std::endl;

            // the patch must contain the polygon
            MCUT_ASSERT(std::find(cw_patch.cbegin(), cw_patch.cend(), cw_patch_seed_poly_idx) != cw_patch.cend());

            const traced_polygon_t& cw_patch_seed_poly = m0_polygons.at(cw_patch_seed_poly_idx);
            traced_polygon_t::const_iterator he_find_iter = std::find(cw_patch_seed_poly.cbegin(), cw_patch_seed_poly.cend(), ccw_patch_seed_interior_ihalfedge_opp);

            MCUT_ASSERT(he_find_iter != cw_patch_seed_poly.cend());

            // the index of the interior intersection halfedge of the current reversed patch
            const int opposite_patch_seed_interior_ihalfedge_idx = (int)std::distance(cw_patch_seed_poly.cbegin(), he_find_iter);
            std::pair<std::map<int, int>::const_iterator, bool> seed_interior_ihalfedge_idx_insertion = patch_to_seed_interior_ihalfedge_idx.insert(std::make_pair(cw_patch_idx, opposite_patch_seed_interior_ihalfedge_idx));

            MCUT_ASSERT(seed_interior_ihalfedge_idx_insertion.second == true);

            std::pair<std::map<int, int>::const_iterator, bool> seed_poly_idx_insertion = patch_to_seed_poly_idx.insert(std::make_pair(cw_patch_idx, cw_patch_seed_poly_idx));

            MCUT_ASSERT(seed_poly_idx_insertion.second == true);

            lg.unindent();
        }
        lg.unindent();
    }

    patch_to_floating_flag.clear(); // free
    color_to_cw_patch.clear();
    patch_to_opposite.clear();

    ///////////////////////////////////////////////////////////////////////////
    // Stitch cut-mesh patches into connected components of the src-mesh
    ///////////////////////////////////////////////////////////////////////////

    //
    // We are now going to effectively "fill the holes"
    //
    // For each color, we have a halfdge data structure which is a copy of "m1".
    // We do this to make sure that the exterior patches will be stitched to
    // separate copies the connected components in "m1" from interior patches.
    // This helps us to distinguish between stitching the interior of the src-mesh
    // (hole-filling), and stitching the exterior (i.e. boolean merge operation
    // if the cut-mesh is water-tight)
    //

    lg << "stitch patches" << std::endl;

    // list of mesh copies containing the unseparated connected ccsponents each differing by one newly stitched polygon
    //std::map<char, std::vector<mesh_t> > color_to_unseparated_connected_ccsponents; // TODO: probably not useful, remove

    // NOTE: <value> has the same number of elements as "unseparated_stitching_CCs"

    // MapKey=color
    // MapValue=[
    //  MapKey=cc-id;
    //  MapValue=list of partially sealed connected components (first elem has 1
    //           stitched polygon and the last has all cut-mesh polygons stitched
    //           to fill holes).
    // ].
    std::map<char, std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>> color_to_separated_connected_ccsponents;

    // MapKey=color
    // MapValue=traced polygons in "m1"
    std::map<char, std::vector<traced_polygon_t>> color_to_m1_polygons;

    // a halfedge in "m0" that is used to trace a cut-mesh polygon will have
    // two "m1" versions - one for the normal patch and the other for the
    // reversed patch.
    //
    // MapKey=color
    // MapValue=[
    //  MapKey="m0" halfedge;
    //  MapValue=[
    //      MapKey=patch index;
    //      MapValue="m1" version of "m1" halfedge
    //  ]
    // ]
    std::map<char, std::map<hd_t /*m0*/, std::map<int /*patch idx*/, hd_t /*m1*/>>> color_to_m0_to_m1_he_instances;

    // for each color  ("interior" / "exterior")
    for (std::map<char, std::vector<int>>::const_iterator color_to_patches_iter = color_to_patch.cbegin();
         color_to_patches_iter != color_to_patch.cend();
         ++color_to_patches_iter) {
        lg.indent();

        const char color_id = color_to_patches_iter->first;

        lg << "color : " << color_id << " (" << (patch_color_label_to_location.at(color_id) == cut_surface_patch_location_t::OUTSIDE ? "exterior" : "interior") << ")" << std::endl;

        // get the copy of "m1" which is used to specifically stitch
        // patches of the current color
        mesh_t& m1_colored = color_to_m1.at(color_id);

        // used to keep track of already-calculated edges in "color_to_m1"
        // TODO: maybe its better to use a map for this look-up
        std::map<vd_t, std::vector<std::pair<vd_t, ed_t>>> m1_computed_edges; // note: local var and in same scope as m1_colored

        auto get_computed_edge = [&](/*const mesh_t& m,*/ const vd_t& src, const vd_t& tgt) -> ed_t {
            // for (std::vector<ed_t>::const_iterator i = m1_computed_edges.cbegin(); i != m1_computed_edges.cend(); ++i) {
            //    if ((m.vertex(*i, 0) == src && m.vertex(*i, 1) == tgt) || (m.vertex(*i, 0) == tgt && m.vertex(*i, 1) == src)) {
            //        return *i;
            //    }
            // }

            ed_t edge = mesh_t::null_edge();
            std::map<vd_t, std::vector<std::pair<vd_t, ed_t>>>::const_iterator i = m1_computed_edges.find(src);

            if (i != m1_computed_edges.cend()) {
                // for each vertex connected to "i"
                for (std::vector<std::pair<vd_t, ed_t>>::const_iterator j = i->second.cbegin(); j != i->second.cend(); ++j) {
                    MCUT_ASSERT(j->first != src); // src cannot be connected to itself
                    if (j->first == tgt) {
                        edge = j->second; // the edge which connnects them
                        break;
                    }
                }
            }

            return edge;
        };

        // create entry
        color_to_m0_to_m1_he_instances.insert(std::make_pair(color_id, std::map<hd_t, std::map<int, hd_t>>()));
        // ref
        std::map<hd_t, std::map<int, hd_t>>& m0_to_m1_he_instances = color_to_m0_to_m1_he_instances.at(color_id);
        // copy all of the "m1_polygons" that were created before we got to the stitching stage
        // Note: "m1_polygons" contains only src-mesh polygons, which have been partition to allow
        // separation of unsealed connected components
        std::pair<std::map<char, std::vector<traced_polygon_t>>::iterator, bool> color_to_m1_polygons_insertion = color_to_m1_polygons.insert(std::make_pair(color_id, m1_polygons)); // copy

        MCUT_ASSERT(color_to_m1_polygons_insertion.second == true);

        // ref to "m1_polygons" i.e. the src-mesh polygons with partitioning
        std::vector<traced_polygon_t>& m1_polygons_colored = color_to_m1_polygons_insertion.first->second;
        // TODO: this is not needed
        // std::vector<mesh_t>& unseparated_stitching_CCs = color_to_unseparated_connected_ccsponents[color_id];
        // reference to the list connected components (see declaration for details)
        std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>& separated_stitching_CCs = color_to_separated_connected_ccsponents[color_id]; // insert

        lg << "patches : " << color_to_patches_iter->second.size() << std::endl;

        // keeps track of the total number of cut-mesh polygons for the current
        // color
        int stitched_poly_counter = 0;

        // for each patch with current color
        for (std::vector<int>::const_iterator patch_iter = color_to_patches_iter->second.cbegin();
             patch_iter != color_to_patches_iter->second.cend();
             ++patch_iter) {
            lg.indent();

            // get patch index
            const int cur_patch_idx = *patch_iter;

            lg << "patch = " << cur_patch_idx << std::endl;

            // is it a normal patch i.e. not the reversed version
            // NOTE: normal patches are created before reversed counterparts (hence the modulo trick)
            const bool is_ccw_patch = ((cur_patch_idx % total_ccw_patch_count) == cur_patch_idx);

            // TODO: use the information about winding order to print patch info
            lg << "is " << (is_ccw_patch ? "ccw" : "cw") << " patch" << std::endl;

            MCUT_ASSERT(patches.find(cur_patch_idx) != patches.cend());

            // get list of patch polygons
            const std::vector<int>& patch = patches.at(cur_patch_idx);

            ///////////////////////////////////////////////////////////////////////////
            // stitch patch into a connected component stored in "m1_colored"
            ///////////////////////////////////////////////////////////////////////////

            //
            // We are basically going to search for the connected component to which
            // the current patch will be stitched/glued.
            //
            // PERSONAL NOTE REGARDING `NORMAL` PATCHES:
            // Interior patches are stitched to the connected components which "naturally"
            // match their winding order (i.e. they are stitched to the connected component
            // "below" the patch).
            // Exterior patches are stitched to connected components which DO NOT share
            // the "natural" winding order (i.e. they are stitched to the connected component
            // "above" the patch).
            //

            MCUT_ASSERT(patch_to_seed_poly_idx.find(cur_patch_idx) != patch_to_seed_poly_idx.cend());

            // get the seed polygon from which to begin the stitching
            // this polygon will be on the patch boundary/border
            const int m0_patch_seed_poly_idx = patch_to_seed_poly_idx.at(cur_patch_idx);

            // patch must contain the polygon
            MCUT_ASSERT(std::find(patch.cbegin(), patch.cend(), m0_patch_seed_poly_idx) != patch.cend());
            // the seed polygon must be from the ones that were traced in "m0" (see graph discovery stage above)
            MCUT_ASSERT(m0_patch_seed_poly_idx < (int)m0_polygons.size());

            // the seed polygon of the patch
            const traced_polygon_t& m0_patch_seed_poly = m0_polygons.at(m0_patch_seed_poly_idx);

            // patch must have a seed halfedge (the one used to traced the seed polygon)
            MCUT_ASSERT(patch_to_seed_interior_ihalfedge_idx.find(cur_patch_idx) != patch_to_seed_interior_ihalfedge_idx.cend());

            // get the index of the seed interior intersection halfedge of the patch
            // this is a halfedge defining the border of the patch and is used to trace
            // the seed polygon
            const int m0_patch_seed_poly_he_idx = patch_to_seed_interior_ihalfedge_idx.at(cur_patch_idx);

            // must be within the range of the number of halfedge defining the seed polygon
            MCUT_ASSERT(m0_patch_seed_poly_he_idx < (int)m0_patch_seed_poly.size());

            // get the seed halfedge descriptor
            const hd_t& m0_patch_seed_poly_he = m0_patch_seed_poly.at(m0_patch_seed_poly_he_idx);

            //
            // Here, we now infer connected component to which the current patch will be stitched.
            // to do this we can use he opposite halfedge of the seed halfedge. This opposite halfedge
            // is used to trace a src-mesh polygon next to the cut-path.
            //

            // get opposite halfedge of the seed halfedge of the current patch
            const hd_t m0_patch_seed_poly_he_opp = m0.opposite(m0_patch_seed_poly_he);

            // an "m1" version of this opposite halfedge must exist from the halfedge
            // partitioning problem we solved when duplicating intersection points to
            // partition the src-mesh
            MCUT_ASSERT(m0_to_m1_ihe.find(m0_patch_seed_poly_he_opp) != m0_to_m1_ihe.cend());

            // get the "m1" version of the opposite-halfedge of the seed-halfedge.
            // Note that this halfedge has already been used to trace a src-mesh polygon
            // in "m1"....
            const hd_t m1_seed_interior_ihe_opp = m0_to_m1_ihe.at(m0_patch_seed_poly_he_opp);
            // .... thus, we have to use its opposite, which will be the "m1" version of the
            // seed halfedge of the current patch.
            // NOTE: this probably requires a visual example to properly understand
            const hd_t m1_seed_interior_ihe_opp_opp = m1_colored.opposite(m1_seed_interior_ihe_opp); // i.e. m1 instance of m0_patch_seed_poly_he_opp

            lg << "stitch polygon-halfedges of patch" << std::endl;

            // this queue contains information identifying the patch polygons next-in-queue
            // to be stitched into the inferred connected component
            std::deque<std::tuple<hd_t /*m1*/, int /*m0 poly*/, int /*m0 he*/>> patch_poly_stitching_queue;
            // thus, the first element is the seed polygon and the seed halfedge
            patch_poly_stitching_queue.push_back(std::make_tuple(m1_seed_interior_ihe_opp_opp, m0_patch_seed_poly_idx, m0_patch_seed_poly_he_idx));

            //
            // In the following loop, we will stitch patch polygons iteratively as we
            // discover adjacent ones starting from the seed polygon. In each interation,
            // we process halfedges of the current polygon so that they reference the
            // correct vertex descriptors (src and tgt) in order to fill holes.
            //
            do {

                lg.indent();

                // the first processed/stitched of halfedge the current polygon (our starting point)
                hd_t m1_cur_patch_cur_poly_1st_he = mesh_t::null_halfedge();
                int m0_cur_patch_cur_poly_idx = -1; // index into m0_polygons
                int m0_cur_patch_cur_poly_1st_he_idx = -1; // index into m0_polygon

                // pop element from queue (the next polygon to stitch)
                std::tie(m1_cur_patch_cur_poly_1st_he, m0_cur_patch_cur_poly_idx, m0_cur_patch_cur_poly_1st_he_idx) = patch_poly_stitching_queue.front();

                lg << "polygon = " << m0_cur_patch_cur_poly_idx << std::endl;

                // must be within the range of the trace polygons (include the reversed ones)
                MCUT_ASSERT(m0_cur_patch_cur_poly_idx < (int)m0_polygons.size());

                // get the current polygon of the patch
                const traced_polygon_t& m0_cur_patch_cur_poly = m0_polygons.at(m0_cur_patch_cur_poly_idx);

                // the index of the starting halfedge must be within range of the polygon
                MCUT_ASSERT(m0_cur_patch_cur_poly_1st_he_idx < (int)m0_cur_patch_cur_poly.size());

                // get the descriptor of the starting halfedge
                const hd_t& m0_cur_patch_cur_poly_1st_he = m0_cur_patch_cur_poly.at(m0_cur_patch_cur_poly_1st_he_idx);

                // the processed/stitched version of the current polygon
                m1_polygons_colored.emplace_back(traced_polygon_t());
                traced_polygon_t& m1_poly = m1_polygons_colored.back(); // stitched version of polygon
                m1_poly.push_back(m1_cur_patch_cur_poly_1st_he);

                // the number of halfedges in the current polygon that have been processed
                // Note: we start from "1" because the initial halfedge (m0_cur_patch_cur_poly_1st_he) has already been processed.
                int transformed_he_counter = 1; //

                //
                // In the following loop, we will process polygon-halfedges iteratively as we
                // advance onto the next ones starting from the initial. In each interation,
                // we create an "m1" version of the of the current halfedge so that it references the
                // correct vertex descriptors (src and tgt). The next iteration moves onto the
                // next halfedge
                //

                do { // for each remaining halfedge of current polygon being stitched

                    lg.indent();

                    if (transformed_he_counter == 1) { // are we processing the second halfedge?
                        // log
                        // TODO: proper printing functions
                        lg << "transform: <" << m0.source(m0_cur_patch_cur_poly_1st_he) << ", " << m0.target(m0_cur_patch_cur_poly_1st_he) << "> - <"
                           << m1_colored.source(m1_cur_patch_cur_poly_1st_he) << " " << m1_colored.target(m1_cur_patch_cur_poly_1st_he) << ">" << std::endl;
                    }

                    // index of current halfedge index to be processed
                    const int m0_cur_patch_cur_poly_cur_he_idx = wrap_integer(m0_cur_patch_cur_poly_1st_he_idx + transformed_he_counter, 0, (int)m0_cur_patch_cur_poly.size() - 1);

                    // must be in range of polygon size
                    MCUT_ASSERT(m0_cur_patch_cur_poly_cur_he_idx < (int)m0_cur_patch_cur_poly.size());

                    // descriptor of current halfedge
                    const hd_t m0_cur_patch_cur_poly_cur_he = m0_cur_patch_cur_poly.at(m0_cur_patch_cur_poly_cur_he_idx); // current untransformed
                    // opposite of current halfedge
                    const hd_t m0_cur_patch_cur_poly_cur_he_opp = m0.opposite(m0_cur_patch_cur_poly_cur_he);
                    // target of the current halfedge
                    vd_t m0_cur_patch_cur_poly_cur_he_tgt = m0.target(m0_cur_patch_cur_poly_cur_he);
                    const bool src_is_ivertex = m0_is_intersection_point(m0.source(m0_cur_patch_cur_poly_cur_he), ps_vtx_cnt);
                    const bool tgt_is_ivertex = m0_is_intersection_point(m0.target(m0_cur_patch_cur_poly_cur_he), ps_vtx_cnt);
                    // is the current halfedge the last to be processed in the current polygon?
                    const bool cur_is_last_to_be_transformed = ((transformed_he_counter + 1) == (int)m0_cur_patch_cur_poly.size()); // i.e. current he is last one to be transform
                    // enumerator of previously processed halfedge
                    const int m1_cur_patch_cur_poly_prev_he_idx = transformed_he_counter - 1; // note: transformed_he_counter is init to 1

                    // must be in current polygon's range
                    MCUT_ASSERT(m1_cur_patch_cur_poly_prev_he_idx < (int)m1_poly.size());

                    // get descriptor of the processed copy of the preceeding halfedge in the current polygon
                    const hd_t m1_cur_patch_cur_poly_prev_he = m1_poly.at(m1_cur_patch_cur_poly_prev_he_idx); // previous transformed
                    // get target of transformed previous
                    const vd_t m1_cur_patch_cur_poly_prev_he_tgt = m1_colored.target(m1_cur_patch_cur_poly_prev_he); // transformed target of previous

                    ///////////////////////////////////////////////////////////////////////////
                    // create "m1" version of current halfedge
                    ///////////////////////////////////////////////////////////////////////////

                    // that source of the processed version of the current halfedge is the same as
                    // the target of the processed previous halfedge in the current polygon
                    vd_t m1_cs_cur_patch_polygon_he_src = m1_cur_patch_cur_poly_prev_he_tgt; // known from previous halfedge
                    // assume the target of the processed version of the current halfedge
                    // is the same descriptor as unprocessed version (this is generally true
                    // when processing non-boundary/border halfedges, and if the current patch
                    // is a normal patch).
                    vd_t m1_cs_cur_patch_polygon_he_tgt = m0_cur_patch_cur_poly_cur_he_tgt;

                    // flag whether to insert new edge into "m1_colored"
                    bool create_new_edge = false;

                    hd_t m1_cur_patch_cur_poly_cur_he = mesh_t::null_halfedge();

                    // is the current halfedge the last one to be process in the current polygon?
                    if (cur_is_last_to_be_transformed) {

                        lg << "last halfedge" << std::endl;

                        // we can infer the updated version of the target vertex from the halfedge
                        // which is already updated. Update tgt will be the source of the first
                        // updated halfedge of the current polygon.
                        m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_poly.front());

                        if (src_is_ivertex && tgt_is_ivertex) { // class 3 : // x-->x

                            //
                            // we now want to check if the current halfedge is interior or exterior
                            //

                            MCUT_ASSERT(m0_ivtx_to_ps_he.find(m0.source(m0_cur_patch_cur_poly_cur_he)) != m0_ivtx_to_ps_he.cend());

                            // get the ps-halfedge in the intersection-registry entry of src
                            const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0.source(m0_cur_patch_cur_poly_cur_he));

                            MCUT_ASSERT(m0_ivtx_to_ps_he.find(m0.target(m0_cur_patch_cur_poly_cur_he)) != m0_ivtx_to_ps_he.cend());

                            // get the ps-halfedge in the intersection-registry entry of src
                            const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0.target(m0_cur_patch_cur_poly_cur_he));
                            // get the ps-edges corresponding to the ps-halfedges
                            const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                            const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                            // is it an interior halfedge
                            bool is_valid_ambiguious_interior_edge = (src_ps_edge != tgt_ps_edge);

                            if (is_valid_ambiguious_interior_edge) { // check is interior ihalfedge (due ambiguity with exterior-interior halfedges x-->x)

                                MCUT_ASSERT(m0_to_m1_ihe.find(m0_cur_patch_cur_poly_cur_he_opp) != m0_to_m1_ihe.cend());

                                const hd_t m1_cur_patch_cur_poly_cur_he_opp = m0_to_m1_ihe.at(m0_cur_patch_cur_poly_cur_he_opp);
                                const hd_t m1_cur_patch_cur_poly_cur_he_opp_opp = m1_colored.opposite(m1_cur_patch_cur_poly_cur_he_opp);

                                // halfedge already exists. it was created during src-mesh partitioning
                                m1_cur_patch_cur_poly_cur_he = m1_cur_patch_cur_poly_cur_he_opp_opp;

                                // TODO: fix problem case2 (i.e. where cs-patch has one orig vertex at the middle and the rest are intersection points, to create a "scoop cut")
                                MCUT_ASSERT(m1_colored.target(m1_cur_patch_cur_poly_cur_he) == m1_cs_cur_patch_polygon_he_tgt);
                            }
                        }
                    } else if (!src_is_ivertex && tgt_is_ivertex) { // class 1 : o-->x : this type of halfedge can only be "coming in" i.e. pointing torward sm
                        // o-->x

                        /*
                            Steps:

                            transformed_src = transformed_prev_tgt // always available since cut-mesh polygon updating always starts from a halfedge whose opposite is already updated 
                            transformed_tgt = untransformed_tgt // assume descriptor will not be updated
                            create_new_edge = FALSE // to insert a new edge into halfedge data structure or not

                            IF "opp" has been transformed // note: if opp is transformed, then polygon coincident to that opp halfedge has been fully updated too since all halfedges of a cut-mesh polygon are transformed before moving onto others.
                                transformed_tgt = source of transformed "opp" 
                            ELSE 
                                transformed_tgt = source of transformed "next" // note: "next" will always be an interior intersection-halfedge since o-->x ihalfedges are always "incoming" i.e. torward the src-mesh 
                                create_new_edge = TRUE // because opposite does not exist
                            
                            IF create_new_edge
                                create new edge and use halfedge defined by transformed_src and transformed_tgt
                            ELSE
                                use halfedge defined by computed transformed_src and transformed_tgt
                        */

                        lg << "o-->x" << std::endl;

                        // check if opposite halfedge of current is updated. (NOTE: searching only through
                        // the polygons of the current patch)

                        // the updated opposite halfedge in the current patch
                        hd_t m1_cs_cur_patch_polygon_he_opp = mesh_t::null_halfedge();
                        // query instances of the "m1" version of the opposite halfedge
                        std::map<hd_t /*m0*/, std::map<int /*patch idx*/, hd_t /*m1*/>>::const_iterator m0_to_m1_he_instances_find_iter = m0_to_m1_he_instances.find(m0_cur_patch_cur_poly_cur_he_opp);

                        // do we have at least one updated copy of the opposite, irrespective of which patch it
                        // belongs to.
                        if (m0_to_m1_he_instances_find_iter != m0_to_m1_he_instances.cend()) {
                            // now check if there is an updated instance corresponding to the current patch
                            std::map<int /*initial patch polygon*/, hd_t /*m1*/>::const_iterator m1_he_instances_find_iter = m0_to_m1_he_instances_find_iter->second.find(cur_patch_idx);
                            if (m1_he_instances_find_iter != m0_to_m1_he_instances_find_iter->second.cend()) {
                                // we have found the already-updated instance of the opposite halfedge
                                m1_cs_cur_patch_polygon_he_opp = m1_he_instances_find_iter->second;
                            }
                        }

                        const bool opp_is_transformed = m1_cs_cur_patch_polygon_he_opp != mesh_t::null_halfedge();

                        if (opp_is_transformed) {
                            // infer tgt from opposite
                            m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_cs_cur_patch_polygon_he_opp);
                        } else {

                            //
                            // the opposite halfedge has not been transformed.
                            // We will infer the target from the updated "next" halfedge, and
                            // we have to create a new edge
                            //

                            // look up the updated "next" by looking forward and finding the coincident src-mesh polygon
                            // and then getting the updated instance of "next".
                            const int m0_next_cs_polygon_he_index = wrap_integer(m0_cur_patch_cur_poly_cur_he_idx + 1, 0, (int)m0_cur_patch_cur_poly.size() - 1);

                            MCUT_ASSERT(m0_next_cs_polygon_he_index < (int)m0_cur_patch_cur_poly.size());

                            const hd_t m0_cs_next_patch_polygon_he = m0_cur_patch_cur_poly.at(m0_next_cs_polygon_he_index); // next untransformed
                            const vd_t m0_cs_next_patch_polygon_he_src = m0.source(m0_cs_next_patch_polygon_he);
                            const vd_t m0_cs_next_patch_polygon_he_tgt = m0.target(m0_cs_next_patch_polygon_he);

                            MCUT_ASSERT(m0_is_intersection_point(m0_cs_next_patch_polygon_he_src, ps_vtx_cnt) && m0_is_intersection_point(m0_cs_next_patch_polygon_he_tgt, ps_vtx_cnt)); // .. because the current halfedge is "incoming"

                            // get the "m0" polygons which are traced with the "next" halfedge
                            const std::vector<int>& m0_poly_he_coincident_polys = m0_h_to_ply.at(m0_cs_next_patch_polygon_he);
                            // get reference to src-mesn polygon which is traced with "next" halfedge
                            //const std::vector<int>::const_iterator find_iter = std::find_if(
                            //    m0_poly_he_coincident_polys.cbegin(),
                            //    m0_poly_he_coincident_polys.cend(),
                            //    [&](const int& e) {
                            //        return (e < traced_sm_polygon_count); // match with src-mesn polygon
                            //    });

                            // "next" is always incident to a src-mesh polygon
                            MCUT_ASSERT(std::find_if(
                                m0_poly_he_coincident_polys.cbegin(),
                                m0_poly_he_coincident_polys.cend(),
                                [&](const int& e) {
                                    return (e < traced_sm_polygon_count); // match with src-mesn polygon
                                }) != m0_poly_he_coincident_polys.cend());

                            //
                            // At this point, we have found the adjacent connected component which is
                            // the one using m0_cs_next_patch_polygon_he. Therefore, we can directly
                            // determine the connected component by looking up the updated instance
                            // of m0_cs_next_patch_polygon_he_opp since m0_cs_next_patch_polygon_he_opp
                            // is guarranteed to have been updated because it is an interior intersection
                            // halfedge.
                            //
                            // REMEMBER: exterior patches are stitched to the "upper" src-mesh fragment
                            const hd_t m0_cs_next_patch_polygon_he_opp = m0.opposite(m0_cs_next_patch_polygon_he);
                            const hd_t m1_cs_next_patch_polygon_he_opp = m0_to_m1_ihe.at(m0_cs_next_patch_polygon_he_opp);
                            const hd_t m1_cs_next_patch_polygon_he_opp_opp = m1_colored.opposite(m1_cs_next_patch_polygon_he_opp);

                            m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_cs_next_patch_polygon_he_opp_opp);

                            // create_new_edge = true; // because opposite has not yet been created
                        }
                    } else if (src_is_ivertex && tgt_is_ivertex) { // class 3 : // x-->x

                        // the current halfedge will either be interior or exterior.

                        lg << "x-->x" << std::endl;

                        MCUT_ASSERT(m0_ivtx_to_ps_he.find(m0.source(m0_cur_patch_cur_poly_cur_he)) != m0_ivtx_to_ps_he.cend());

                        const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0.source(m0_cur_patch_cur_poly_cur_he));

                        MCUT_ASSERT(m0_ivtx_to_ps_he.find(m0.target(m0_cur_patch_cur_poly_cur_he)) != m0_ivtx_to_ps_he.cend());

                        const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0.target(m0_cur_patch_cur_poly_cur_he));
                        const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                        const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                        bool is_valid_ambiguious_interior_edge = (src_ps_edge != tgt_ps_edge);

                        // check if current halfedge is interior
                        if (is_valid_ambiguious_interior_edge) {

                            lg << "interior" << std::endl;

                            MCUT_ASSERT(m0_to_m1_ihe.find(m0_cur_patch_cur_poly_cur_he_opp) != m0_to_m1_ihe.cend());
                            const hd_t m1_cur_patch_cur_poly_cur_he_opp = m0_to_m1_ihe.at(m0_cur_patch_cur_poly_cur_he_opp);
                            const hd_t m1_cur_patch_cur_poly_cur_he_opp_opp = m1_colored.opposite(m1_cur_patch_cur_poly_cur_he_opp);

                            // halfedge already exists. it was created during src-mesh partitioning
                            m1_cur_patch_cur_poly_cur_he = m1_cur_patch_cur_poly_cur_he_opp_opp;
                            m1_cs_cur_patch_polygon_he_tgt = m1_colored.target(m1_cur_patch_cur_poly_cur_he_opp_opp);

                        } else { // its an exterior x-->x halfedge

                            lg << "exterior" << std::endl;

                            // look up the transformed "next" by looking finding the
                            // coincident src-mesh polygon and then getting the transformed instance of "next".
                            const int m0_next_cs_polygon_he_index = wrap_integer(m0_cur_patch_cur_poly_cur_he_idx + 1, 0, (int)m0_cur_patch_cur_poly.size() - 1);

                            MCUT_ASSERT(m0_next_cs_polygon_he_index < (int)m0_cur_patch_cur_poly.size());

                            const hd_t m0_cs_next_patch_polygon_he = m0_cur_patch_cur_poly.at(m0_next_cs_polygon_he_index); // next untransformed
                            const vd_t m0_cs_next_patch_polygon_he_src = m0.source(m0_cs_next_patch_polygon_he);
                            const vd_t m0_cs_next_patch_polygon_he_tgt = m0.target(m0_cs_next_patch_polygon_he);

                            MCUT_ASSERT(m0_is_intersection_point(m0_cs_next_patch_polygon_he_src, ps_vtx_cnt) && m0_is_intersection_point(m0_cs_next_patch_polygon_he_tgt, ps_vtx_cnt));
                            MCUT_ASSERT(m0_h_to_ply.find(m0_cs_next_patch_polygon_he) != m0_h_to_ply.cend());

                            const std::vector<int>& m0_poly_he_coincident_polys = m0_h_to_ply.at(m0_cs_next_patch_polygon_he);
                            const std::vector<int>::const_iterator find_iter = std::find_if( // points to src-mesh polygon
                                m0_poly_he_coincident_polys.cbegin(),
                                m0_poly_he_coincident_polys.cend(),
                                [&](const int& e) {
                                    return (e < traced_sm_polygon_count); // match with src-mesh polygon
                                });

                            // "next" is always incident to an src-mesh polygon
                            MCUT_ASSERT(find_iter != m0_poly_he_coincident_polys.cend());

                            const hd_t m0_cs_next_patch_polygon_he_opp = m0.opposite(m0_cs_next_patch_polygon_he);

                            // Note: this is always true, even in the case of scoop cuts. This is because
                            // halfedge along the cut-path are updated before stitching (srm-mesh parttioning)
                            // so we can infer the tgt easily
                            MCUT_ASSERT(m0_h_to_ply.find(m0_cs_next_patch_polygon_he_opp) != m0_h_to_ply.cend());

                            const hd_t m1_cs_next_patch_polygon_he_opp = m0_to_m1_ihe.at(m0_cs_next_patch_polygon_he_opp);
                            const hd_t m1_cs_next_patch_polygon_he_opp_opp = m1_colored.opposite(m1_cs_next_patch_polygon_he_opp);

                            m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_cs_next_patch_polygon_he_opp_opp);
                        }
                    } else { // class 0 or 2 i.e. o-->o or x-->o
                        lg << "o-->o or x-->o" << std::endl;
                        /*
                            In the following steps, our ability to infer the correct vertex instance 
                            by simply checking whether "opp" or "next" is updated before 
                            duplication is guarranteed to work. This is because we update polygons 
                            of a patch using BFS (following adjacency) which guarrantees that when 
                            the condition to create a duplicate vertex is reached, there will have 
                            been no other halfedge referencing the same vertex that had reached the 
                            same condition. 

                            transformed_src = transformed_prev_tgt // always available because cut-mesh polygon update always starts from a halfedge whose opposite is already updated 
                            transformed_tgt = untransformed_tgt
                            create_new_edge = FALSE

                            IF opposite patch is transformed
                                1. IF opposite halfedge is transformed
                                2.      infer from opposite halfedge
                                3. ELSE IF next halfedge is transformed
                                4.      infer from next
                                2. ELSE 
                                    IF updated halfedge point to tgt exists
                                        infer from that halfedge
                                    ELSE
                                        create duplicate of untransformed_tgt
                                        transformed_tgt = duplicate of untransformed_tgt
                                        create_new_edge = TRUE // because "opposite" AND "next" halfedge are not updated, so we have create a new connection between vertices
                            ELSE
                                    // Do nothing (keep transformed_tgt as it is) because there 
                                    // is no adjacent halfedge which updated, and the current 
                                    // patch gets precedence to use the first vertex instances
                        */
                        if (cur_is_last_to_be_transformed) {
                            // initial polygon halfedge which was transformed
                            m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_poly.front());
                        } else {

                            // check opposite patch of current is transformed
                            const bool opposite_patch_is_transformed = !is_ccw_patch; // ... since ccw patches are always transformed before their cw counterparts

                            if (opposite_patch_is_transformed) // if true, the current patch is the cw one
                            {
                                // check if opposite halfedge of current is transformed. (NOTE: searching
                                // only through the polygons of the current patch)

                                hd_t m1_cs_cur_patch_polygon_he_opp = mesh_t::null_halfedge(); // transformed instance of opposite
                                std::map<hd_t /*m0*/, std::map<int /*patch idx*/, hd_t /*m1*/>>::const_iterator m0_to_m1_he_instances_find_iter = m0_to_m1_he_instances.find(m0_cur_patch_cur_poly_cur_he_opp);

                                if (m0_to_m1_he_instances_find_iter != m0_to_m1_he_instances.cend()) { // must transformed at least once since opposite patch is transformed

                                    std::map<int /*initial patch polygon*/, hd_t /*m1*/>::const_iterator m1_he_instances_find_iter = m0_to_m1_he_instances_find_iter->second.find(cur_patch_idx);

                                    if (m1_he_instances_find_iter != m0_to_m1_he_instances_find_iter->second.cend()) {
                                        m1_cs_cur_patch_polygon_he_opp = m1_he_instances_find_iter->second;
                                    }
                                }

                                const bool opp_is_transformed = m1_cs_cur_patch_polygon_he_opp != mesh_t::null_halfedge();

                                if (opp_is_transformed) {
                                    m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_cs_cur_patch_polygon_he_opp);
                                } else {

                                    // check if next halfedge of current is transformed.
                                    const int m0_next_cs_polygon_he_index = wrap_integer(m0_cur_patch_cur_poly_cur_he_idx + 1, 0, (int)m0_cur_patch_cur_poly.size() - 1);
                                    const hd_t m0_cs_next_patch_polygon_he = m0_cur_patch_cur_poly.at(m0_next_cs_polygon_he_index); // next untransformed
                                    m0_to_m1_he_instances_find_iter = m0_to_m1_he_instances.find(m0_cs_next_patch_polygon_he);
                                    hd_t m1_cs_next_patch_polygon_he = mesh_t::null_halfedge();

                                    if (m0_to_m1_he_instances_find_iter != m0_to_m1_he_instances.cend()) { // must transformed at least once since opposite patch is transformed

                                        std::map<int, hd_t>::const_iterator m1_he_instances_find_iter = m0_to_m1_he_instances_find_iter->second.find(cur_patch_idx);

                                        if (m1_he_instances_find_iter != m0_to_m1_he_instances_find_iter->second.cend()) {
                                            m1_cs_next_patch_polygon_he = m1_he_instances_find_iter->second;
                                        }
                                    }

                                    const bool next_is_transformed = m1_cs_next_patch_polygon_he != mesh_t::null_halfedge();

                                    if (next_is_transformed) {
                                        m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_cs_next_patch_polygon_he);
                                    } else {

                                        //
                                        // find all updated tracing halfedges (in "m1") which connected to
                                        // m0_cur_patch_cur_poly_cur_he_tgt in the current patch
                                        //

                                        bool found_transformed_neigh_he = false; // any updated halfedge whose m0 instance references m0_cur_patch_cur_poly_cur_he_tgt

                                        //
                                        // TODO: replace the following loop over "patch", with a loop over the
                                        // halfedges around vertex, where vertex is "m0_cur_patch_cur_poly_cur_he_tgt"
                                        // finding the halfedges around a vertex can be found by use our traced polygons!
                                        //

                                        // for each polygon in current patch
                                        for (std::vector<int>::const_iterator patch_poly_idx_iter = patch.cbegin();
                                             patch_poly_idx_iter != patch.cend();
                                             ++patch_poly_idx_iter) {

                                            const int& patch_poly_idx = *patch_poly_idx_iter;
                                            const traced_polygon_t& patch_poly = m0_polygons.at(patch_poly_idx);

                                            // for each halfedge in polygon
                                            for (traced_polygon_t::const_iterator patch_poly_he_iter = patch_poly.cbegin();
                                                 patch_poly_he_iter != patch_poly.cend();
                                                 ++patch_poly_he_iter) {

                                                const hd_t& patch_poly_he = *patch_poly_he_iter;

                                                if (m0_cur_patch_cur_poly_cur_he == patch_poly_he) {
                                                    continue; // its the current halfedge we are trying to transform!
                                                }

                                                const vd_t patch_poly_he_src = m0.source(patch_poly_he);
                                                const vd_t patch_poly_he_tgt = m0.target(patch_poly_he);
                                                const bool connected_by_src = (patch_poly_he_src == m0_cur_patch_cur_poly_cur_he_tgt);
                                                const bool connected_by_tgt = (patch_poly_he_tgt == m0_cur_patch_cur_poly_cur_he_tgt);

                                                if (connected_by_src || connected_by_tgt) {

                                                    std::map<hd_t, std::map<int, hd_t>>::iterator m0_to_m1_he_instances_find_iter_ = m0_to_m1_he_instances.find(patch_poly_he);

                                                    if (m0_to_m1_he_instances_find_iter_ != m0_to_m1_he_instances.end()) {
                                                        std::map<int, hd_t>::const_iterator patch_to_m1_he_iter = m0_to_m1_he_instances_find_iter_->second.find(cur_patch_idx);

                                                        const bool is_transformed = (patch_to_m1_he_iter != m0_to_m1_he_instances_find_iter_->second.cend());

                                                        if (is_transformed) {
                                                            const hd_t& m1_patch_poly_he = patch_to_m1_he_iter->second;
                                                            if (connected_by_src) {
                                                                m1_cs_cur_patch_polygon_he_tgt = m1_colored.source(m1_patch_poly_he);
                                                            } else { //connected_by_tgt
                                                                m1_cs_cur_patch_polygon_he_tgt = m1_colored.target(m1_patch_poly_he);
                                                            }
                                                            found_transformed_neigh_he = true;
                                                            break; // done
                                                        }
                                                    }
                                                }
                                            }

                                            if (found_transformed_neigh_he) {
                                                break;
                                            }
                                        }

                                        if (!found_transformed_neigh_he) {
                                            //
                                            // none of the adjacent halfedges have been transformed, so we must duplicate m0_cur_patch_cur_poly_cur_he_tgt
                                            //

                                            // is "m1_cs_cur_patch_polygon_he_tgt" a vertex we can duplicate? (partial cut, interior sealing)
                                            const bool is_sm_interior_cs_boundary_vertex = std::find(sm_interior_cs_border_vertices.cbegin(), sm_interior_cs_border_vertices.cend(), m0_cur_patch_cur_poly_cur_he_tgt) != sm_interior_cs_border_vertices.cend();

                                            if (!is_sm_interior_cs_boundary_vertex) {

                                                const vd_t m0_poly_he_tgt_dupl = m1_colored.add_vertex(m0.vertex(m0_cur_patch_cur_poly_cur_he_tgt));

                                                lg << "add vertex = " << m0_poly_he_tgt_dupl << std::endl;

                                                MCUT_ASSERT(m0_poly_he_tgt_dupl != mesh_t::null_halfedge());

                                                m1_cs_cur_patch_polygon_he_tgt = m0_poly_he_tgt_dupl;
                                                create_new_edge = true;
                                            }
                                        }
                                    } // if (next_is_transformed) {
                                } // if (opp_is_transformed) {
                            } // if (opposite_patch_is_transformed)
                        } // if (cur_is_last_to_be_transformed) {
                    } // class 0 or 2 i.e. o-->o or x-->o

                    // if we could not infer from any pre-existing halfedge
                    if (m1_cur_patch_cur_poly_cur_he == mesh_t::null_halfedge()) {
                        // check if edge exists
                        // TODO: use mesh built "halfedge(...)" (may require minor update to function)
                        ed_t e = get_computed_edge(/*m1_colored, */ m1_cs_cur_patch_polygon_he_src, m1_cs_cur_patch_polygon_he_tgt);

                        lg << "edge = " << e << std::endl;

                        if (e != mesh_t::null_edge()) { // edge already exists

                            hd_t h0 = m1_colored.halfedge(e, 0);

                            if (m1_colored.source(h0) == m1_cs_cur_patch_polygon_he_src) {
                                m1_cur_patch_cur_poly_cur_he = h0;
                            } else {
                                hd_t h1 = m1_colored.halfedge(e, 1);
                                m1_cur_patch_cur_poly_cur_he = h1;
                            }
                        } else {

                            lg << "create new edge" << std::endl;

                            m1_cur_patch_cur_poly_cur_he = m1_colored.add_edge(m1_cs_cur_patch_polygon_he_src, m1_cs_cur_patch_polygon_he_tgt);
                            // TODO:replace with map (for O(Log N) searches)
                            //m1_computed_edges.push_back(m1_colored.edge(m1_cur_patch_cur_poly_cur_he));
                            //std::map<vd_t, std::vector<std::pair<vd_t, ed_t>>>

                            ed_t new_edge = m1_colored.edge(m1_cur_patch_cur_poly_cur_he);
                            m1_computed_edges[m1_cs_cur_patch_polygon_he_src].push_back(std::make_pair(m1_cs_cur_patch_polygon_he_tgt, new_edge));
                            m1_computed_edges[m1_cs_cur_patch_polygon_he_tgt].push_back(std::make_pair(m1_cs_cur_patch_polygon_he_src, new_edge));
                        }
                    } // if (m1_cur_patch_cur_poly_cur_he == mesh_t::null_halfedge()) {

                    lg << "transform: <" << m0.source(m0_cur_patch_cur_poly_cur_he) << ", " << m0.target(m0_cur_patch_cur_poly_cur_he) << "> - <"
                       << m1_colored.source(m1_cur_patch_cur_poly_cur_he) << " " << m1_colored.target(m1_cur_patch_cur_poly_cur_he) << ">" << std::endl;

                    // halfedge must have been found (created or inferred)
                    MCUT_ASSERT(m1_cur_patch_cur_poly_cur_he != mesh_t::null_halfedge());
                    MCUT_ASSERT(m1_colored.target(m1_poly.back()) == m1_colored.source(m1_cur_patch_cur_poly_cur_he));

                    // add transformed halfedge to currently transformed polygon
                    m1_poly.push_back(m1_cur_patch_cur_poly_cur_he);

                    //
                    // map halfedge to transformed instance of current patch
                    //

                    // NOTE: m0_cur_patch_cur_poly_cur_he will not exist if current patch has CCW orientation
                    // since such a patch will always be transformed first before its CW counterpart.
                    std::map<hd_t, std::map<int, hd_t>>::iterator m0_to_m1_he_instances_find_iter = m0_to_m1_he_instances.find(m0_cur_patch_cur_poly_cur_he);

                    if (m0_to_m1_he_instances_find_iter == m0_to_m1_he_instances.end()) { // not yet transformed at all (i.e. m0_cur_patch_cur_poly_cur_he belongs to CCW polygon )

                        std::pair<std::map<hd_t /*m0*/, std::map<int /*initial patch polygon*/, hd_t /*m1*/>>::iterator, bool> pair = m0_to_m1_he_instances.insert(std::make_pair(m0_cur_patch_cur_poly_cur_he, std::map<int, hd_t>()));

                        MCUT_ASSERT(pair.second == true);

                        m0_to_m1_he_instances_find_iter = pair.first;
                    }

                    // stores the an "m1" instance of the current halfedge, for each patch
                    std::map<int, hd_t /*m1*/>& patch_to_m1_he = m0_to_m1_he_instances_find_iter->second;
                    const std::map<int, hd_t /*m1*/>::const_iterator patch_idx_to_m1_he = patch_to_m1_he.find(cur_patch_idx);

                    // In general, a halfedge may only be transformed once for each patch it is be associated
                    // with (i.e it will have two copies with one for each opposing patch). Note however that
                    // in the case that the current halfedge is a border halfedge (partial cut), its transformed
                    // copy is the same as its untransformed copy for each patch
                    MCUT_ASSERT(patch_idx_to_m1_he == patch_to_m1_he.cend());

                    patch_to_m1_he.insert(std::make_pair(cur_patch_idx, m1_cur_patch_cur_poly_cur_he));
                    transformed_he_counter += 1; // next halfedge in m0_cur_patch_cur_poly

                    lg.unindent();

                } while (transformed_he_counter != (int)m0_cur_patch_cur_poly.size()); // while not all halfedges of the current polygon have been transformed.

                //
                // at this stage, all polygon halfedges have been transformed
                //

                // ... remove the stitching-initialiation data of current polygon.
                patch_poly_stitching_queue.pop_front();

                ///////////////////////////////////////////////////////////////////////////
                // find untransformed neighbouring polygons and queue them
                ///////////////////////////////////////////////////////////////////////////

                //
                // We are basically adding all unstitched neighbours of the current polygon
                // (we just sticthed) to the queue so they can be stitched as well. These
                // are polygons on the same patch as the current polygon and are adjacent to
                // it i.e. they share an edge.
                //

                MCUT_ASSERT(patches.find(cur_patch_idx) != patches.cend());

                // polygons of current patch
                const std::vector<int>& patch_polys = patches.at(cur_patch_idx);

                // for each halfedge of the polygon we just stitched
                for (traced_polygon_t::const_iterator m0_poly_he_iter = m0_cur_patch_cur_poly.cbegin();
                     m0_poly_he_iter != m0_cur_patch_cur_poly.cend();
                     ++m0_poly_he_iter) {

                    const hd_t m0_cur_patch_cur_poly_cur_he = *m0_poly_he_iter;

                    // Skip certain neighbours. The adjacent polygon has been processed (assuming
                    // it exists) if the following conditions are true. Theses conditions are
                    // evaluated on "m0_cur_patch_cur_poly_cur_he"
                    //
                    // 1. is same as initial halfedge
                    //      implies that opposite is already transformed (before the current polygon,
                    //      we transformed the polygon which is traced by the opposite halfedge of
                    //      m0_cur_patch_cur_poly_1st_he)
                    // 2. is interior intersection-halfedge
                    //      opposite is already transformed since interior intersection-halfedges
                    //      are on cut-path (the opposite halfedge is incident to one of the
                    //      src-mesh connected components)
                    // 3. is a halfedge whose opposite has been transformed
                    //      because that implies that its polygon has been transformed (so no need
                    //      to add to queue).
                    // 4. it is a border halfedge
                    //      (i.e. the only face incident to the opposite halfedge is another which
                    //      belong to the opposite patch)
                    //

                    //
                    // case 1
                    //
                    if (m0_cur_patch_cur_poly_cur_he == m0_cur_patch_cur_poly_1st_he) {
                        continue; // 1
                    }

                    //
                    // case 2
                    //
                    const vd_t m0_cur_patch_cur_poly_cur_he_src = m0.source(m0_cur_patch_cur_poly_cur_he);
                    const vd_t m0_cur_patch_cur_poly_cur_he_tgt = m0.target(m0_cur_patch_cur_poly_cur_he);
                    bool is_ambiguious_interior_edge_case = m0_is_intersection_point(m0_cur_patch_cur_poly_cur_he_src, ps_vtx_cnt) && m0_is_intersection_point(m0_cur_patch_cur_poly_cur_he_tgt, ps_vtx_cnt);

                    if (is_ambiguious_interior_edge_case) {

                        MCUT_ASSERT(m0_ivtx_to_ps_he.find(m0_cur_patch_cur_poly_cur_he_src) != m0_ivtx_to_ps_he.cend());

                        const hd_t src_coincident_ps_halfedge = m0_ivtx_to_ps_he.at(m0_cur_patch_cur_poly_cur_he_src);

                        MCUT_ASSERT(m0_ivtx_to_ps_he.find(m0_cur_patch_cur_poly_cur_he_tgt) != m0_ivtx_to_ps_he.cend());

                        const hd_t tgt_ps_h = m0_ivtx_to_ps_he.at(m0_cur_patch_cur_poly_cur_he_tgt);
                        const ed_t src_ps_edge = ps.edge(src_coincident_ps_halfedge);
                        const ed_t tgt_ps_edge = ps.edge(tgt_ps_h);
                        bool is_valid_ambiguious_interior_edge = (src_ps_edge != tgt_ps_edge);

                        if (is_valid_ambiguious_interior_edge) {
                            continue; // 2
                        }
                    }

                    //
                    // case 3
                    //
                    const hd_t m0_cur_patch_cur_poly_cur_he_opp = m0.opposite(m0_cur_patch_cur_poly_cur_he);
                    std::map<hd_t, std::map<int, hd_t>>::const_iterator m0_to_m1_he_instances_find_iter = m0_to_m1_he_instances.find(m0_cur_patch_cur_poly_cur_he_opp); // value will not exist if current patch positive

                    if (m0_to_m1_he_instances_find_iter != m0_to_m1_he_instances.cend()) { // check exists (i.e. m0_cur_patch_cur_poly_cur_he_opp has be transform but we dont know for which patch it has been transformed (CCW or CW)

                        const std::map<int, hd_t>& patch_to_m1_he = m0_to_m1_he_instances_find_iter->second;
                        std::map<int, hd_t>::const_iterator patch_idx_to_m1_he = patch_to_m1_he.find(cur_patch_idx);

                        if (patch_idx_to_m1_he != patch_to_m1_he.cend()) { // check is stitched
                            MCUT_ASSERT(patch_idx_to_m1_he->second != mesh_t::null_halfedge());
                            continue; // 3
                        }
                    }

                    //
                    // case 4
                    //

                    // must exist because m0_cur_patch_cur_poly_cur_he was just transformed
                    MCUT_ASSERT(m0_to_m1_he_instances.find(m0_cur_patch_cur_poly_cur_he) != m0_to_m1_he_instances.cend());

                    // find m1_cur_polygon_he which is the transformed instance of m0_cur_patch_cur_poly_cur_he
                    const std::map<int, hd_t>& patch_to_m1_he = m0_to_m1_he_instances.at(m0_cur_patch_cur_poly_cur_he);

                    MCUT_ASSERT(patch_to_m1_he.find(cur_patch_idx) != patch_to_m1_he.cend());

                    const hd_t m1_cur_polygon_he = patch_to_m1_he.at(cur_patch_idx);
                    // transformed halfedge used by adjacent polygon
                    const hd_t m1_next_poly_seed_he = m1_colored.opposite(m1_cur_polygon_he);

                    // infer the index of the next stitched polygon which is traced with m0_cur_patch_cur_poly_cur_he_opp
                    MCUT_ASSERT(m0_h_to_ply.find(m0_cur_patch_cur_poly_cur_he_opp) != m0_h_to_ply.cend());

                    //
                    // find the adjacent polygon in the current patch using the opposite of the
                    // current halfedge
                    //

                    // get the polygons traced with the opposite halfedge
                    const std::vector<int> m0_poly_he_opp_coincident_polys = m0_h_to_ply.at(m0_cur_patch_cur_poly_cur_he_opp);
                    const std::vector<int>::const_iterator find_iter = std::find_if( // find the cs polygon of current patch
                        m0_poly_he_opp_coincident_polys.cbegin(),
                        m0_poly_he_opp_coincident_polys.cend(),
                        [&](const int poly_idx) {
                            bool has_patch_winding_orientation = false;

                            // check if polygon has the same winding order as the current patch

                            if (is_ccw_patch) { // is the current patch a "normal" patch?
                                has_patch_winding_orientation = (poly_idx < traced_polygon_count);
                            } else {
                                has_patch_winding_orientation = (poly_idx >= traced_polygon_count);
                            }

                            // polygon has same winding-order as current patch, and polygon is part of the current patch
                            // TODO: use the cs_poly_to_patch map for O(Log N) improvement
                            return has_patch_winding_orientation && std::find(patch_polys.cbegin(), patch_polys.cend(), poly_idx) != patch_polys.cend(); // NOTE: only one polygon in the current patch will match
                        });

                    // note: if the current halfedge is on the border of the cut-mesh, then its opposite
                    // halfedge can only be traced one polygon, which is the opposite polygon to the
                    // current. Hence, if find_iter is null then it means "m0_cur_patch_cur_poly_cur_he"
                    // is on the border of the cut-mesh.
                    const bool opp_is_border_halfedge = (find_iter == m0_poly_he_opp_coincident_polys.cend()); // current patch is reversed-patch and

                    if (opp_is_border_halfedge) {
                        // 4 there is no neighbouring polygon which is coincident to
                        // "m0_cur_patch_cur_poly_cur_he_opp"
                        continue;
                    }

                    // the adjacent polygon
                    const int m0_next_poly_idx = *find_iter;

                    //
                    // TODO: the following conditions below could also be speeded up if we
                    // create a tmp vector/map which stores all of the adjacent polygons we have
                    // already queued. Searching over this vector could be that bit faster.
                    // We could do the right here now that "m0_next_poly_idx" is known.
                    //

                    // infer the index of the next polygon's seed m0 halfedge
                    MCUT_ASSERT(m0_next_poly_idx < (int)m0_polygons.size());

                    // adjacent polygon
                    const traced_polygon_t& next_poly = m0_polygons.at(m0_next_poly_idx);
                    // pointer to the first halfedge in the polygon from which its
                    // stitching will begin
                    const traced_polygon_t::const_iterator he_find_iter = std::find(next_poly.cbegin(), next_poly.cend(), m0_cur_patch_cur_poly_cur_he_opp);

                    // "m0_cur_patch_cur_poly_cur_he_opp" must exist in next_poly since we have
                    // already established that "m0_cur_patch_cur_poly_cur_he" is not a border
                    // halfedge. This is further supported by the fact that "next_poly" is in
                    // current patch and coincident to "m0_cur_patch_cur_poly_cur_he_opp"
                    MCUT_ASSERT(he_find_iter != next_poly.cend());

                    // index of halfedge from which stitching of the adjacent polygon will begin
                    const int m0_next_poly_he_idx = (int)std::distance(next_poly.cbegin(), he_find_iter);

                    // NOTE: there is no need to check if the next polygon is transformed here
                    // because our 4 conditions above implicitely take care of this.
                    // However, we do have to take care not to add the polygon to the queue more
                    // than once (due to BFS nature of stitching), hence the following.

                    //
                    // NOTE: the following is redundant!
                    // we don't need to calculate "poly_is_already_stitched_wrt_cur_patch" because
                    // this is precisely waht case 3 above is checking for. Remove this ASAP, its also a major performance hit.
                    //
                    bool poly_is_already_stitched_wrt_cur_patch = std::find_if(
                                                                      m0_to_m1_he_instances.cbegin(),
                                                                      m0_to_m1_he_instances.cend(),
                                                                      // for each transformed halfedge, check it it has been transformed w.r.t the
                                                                      // current patch. Further, check if the transformed value is the one we are about
                                                                      // to seed the next polygon's stitching with (m1_next_poly_seed_he)
                                                                      [&](const std::pair<hd_t, std::map<int, hd_t>>& elem) {
                                                                          bool is_transformed_in_current_patch = false;

                                                                          for (std::map<int, hd_t>::const_iterator iter = elem.second.cbegin(); iter != elem.second.cend(); ++iter) {
                                                                              if (iter->first == cur_patch_idx && iter->second == m1_next_poly_seed_he) {
                                                                                  is_transformed_in_current_patch = true;
                                                                                  break;
                                                                              }
                                                                          }
                                                                          return is_transformed_in_current_patch;
                                                                      })
                        != m0_to_m1_he_instances.cend();

                    if (!poly_is_already_stitched_wrt_cur_patch) { // TODO: the [if check] will have to go once "poly_is_already_stitched_wrt_cur_patch" is removed
                        const bool poly_is_already_in_queue = std::find_if(
                                                                  patch_poly_stitching_queue.cbegin(),
                                                                  patch_poly_stitching_queue.cend(),
                                                                  [&](const std::tuple<hd_t, int, int>& elem) {
                                                                      return std::get<1>(elem) == m0_next_poly_idx; // there is an element in the queue with the polygon's ID
                                                                  })
                            != patch_poly_stitching_queue.cend();

                        if (!poly_is_already_in_queue) {
                            patch_poly_stitching_queue.push_back(std::make_tuple(m1_next_poly_seed_he, m0_next_poly_idx, m0_next_poly_he_idx));
                        }
                    }
                } // for each m0 halfedge of current patch-polygon

                //
                // NOTE: At this stage, we have finished transforming all the halfedges of the current polygon
                // and we have also added all its neighbouring polygons to the queue for stitching.
                //

                MCUT_ASSERT(patch_color_label_to_location.find(color_id) != patch_color_label_to_location.cend());

                ///////////////////////////////////////////////////////////////////////////
                // Update output (with the current polygon stitched into a cc)
                ///////////////////////////////////////////////////////////////////////////

                // TODO: replace with to_string call
                const std::string color_dye_string_id = (patch_color_label_to_location.at(color_id) == cut_surface_patch_location_t::OUTSIDE ? "e" : "i");

//dump_mesh_summary(m1_colored, color_dye_string_id);
#if !MCUT_KEEP_TEMP_CCs_DURING_PATCH_STITCHING
                unseparated_stitching_CCs.clear();
//unseparated_stitching_CCs.erase(unseparated_stitching_CCs.begin(), unseparated_stitching_CCs.begin() + unseparated_stitching_CCs.size() - 1);
#endif // #if !MCUT_KEEP_TEMP_CCs_DURING_PATCH_STITCHING

                // save meshes and dump

                // std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_location_t> > >
                //int numInstancesPrev = 0; // Fragment history: an instance is a fragment containing at least one more/less polygon than all other fragments (history)
                //for (auto it = separated_stitching_CCs.cbegin(); it != separated_stitching_CCs.cend(); ++it) {
                //    numInstancesPrev += it->second.size();
                // }
                //const int ccsp_count_pre = numInstancesPrev; //separated_stitching_CCs.unseparated_stitching_CCs.size();

                if (input.keep_partially_sealed_connected_components == true) {
                    ///////////////////////////////////////////////////////////////////////////
                    // create the sealed meshes defined by the [current] set of traced polygons
                    ///////////////////////////////////////////////////////////////////////////

                    extract_connected_components(separated_stitching_CCs, m1_colored, m1_polygons_colored, sm_polygons_below_cs, sm_polygons_above_cs, m1_vertex_to_seam_flag);
                }
                //int numInstancesCur = 0;
                // for (auto it = separated_stitching_CCs.cbegin(); it != separated_stitching_CCs.cend(); ++it) {
                //     numInstancesCur += it->second.size();
                // }
                //const int ccsp_count_post = numInstancesCur; //unseparated_stitching_CCs.size();

                // TODO: enable this assert later once we start using "connected_components". This is
                // because at the moment, all fragments (upper/lower) are added into "separated_stitching_CCs"
                // at the moment. Thus we get duplicate copies of meshes!!
                // If you forget what you meant, re-enable the assert line below and run with the example "src-mesh000.off" "cut-mesh000.off"
                // MCUT_ASSERT(numInstancesPrev == 0 || (ccsp_count_post - ccsp_count_pre == 1));

#if !MCUT_KEEP_TEMP_CCs_DURING_PATCH_STITCHING
                //
                // TODO: the following code is messy and could do with some cleaning up
                // It has one possible use, which is to allows for the user to choose if we
                // we different instances of the fragments or not.
                //

                //for (int index = ccsp_count_pre; index < ccsp_count_post; ++index) { // TODO: this loop may be redundant because unseparated_stitching_CCs.size() always increases by one.

                //const mesh_t& new_unseparated_connected_ccsponent = unseparated_stitching_CCs.back();
                //dump_mesh(new_unseparated_connected_ccsponent, ("c." + color_dye_string_id + "." + "p" + std::to_string(stitched_poly_counter)).c_str());

                for (std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>::iterator cc_iter = separated_stitching_CCs.begin();
                     cc_iter != separated_stitching_CCs.end();
                     ++cc_iter) {

                    //int cc_id = static_cast<int>(cc_iter->first);
                    std::vector<std::pair<mesh_t, output_mesh_info_t>>& incremental_instances = cc_iter->second;

                    //const mesh_t& mesh_inst = incremental_instances.back().first;
                    //const connected_component_location_t location = incremental_instances.back().second;

                    //dump_mesh(mesh_inst, ("cc" + std::to_string(cc_id) + "." + color_dye_string_id + "." + "p" + std::to_string(stitched_poly_counter) + "." + to_string(location)).c_str());

                    // keep only the last (most up-to-date) copy
                    incremental_instances.erase(incremental_instances.cbegin(), incremental_instances.cbegin() + incremental_instances.size() - 1);
                }
#endif // #if !MCUT_KEEP_TEMP_CCs_DURING_PATCH_STITCHING 

                ++global_cs_poly_stitch_counter;
                stitched_poly_counter++;

                lg.unindent();
            } while (!patch_poly_stitching_queue.empty()); // for each polygon of patch

            //
            // NOTE: At this stage we have finished stitching all polygons of the current patch.
            // So, the current patch has been stitch to a src-mesh fragment
            //

            lg.unindent();
        } // for each patch
        lg.unindent();
    } // for each color

    m0_ivtx_to_ps_he.clear(); // free
    m0_polygons.clear();
    m0_h_to_ply.clear();
    m0_to_m1_ihe.clear();
    m1_polygons.clear();
    patches.clear();
    patch_to_seed_interior_ihalfedge_idx.clear();
    patch_to_seed_interior_ihalfedge_idx.clear();
    patch_to_seed_poly_idx.clear();
    color_to_patch.clear();
    sm_interior_cs_border_vertices.clear();
    color_to_m0_to_m1_he_instances.clear();

    //
    // NOTE: At this stage, all patches of the current have been stitched
    //

    lg << "total cut-mesh polygons stitched = " << global_cs_poly_stitch_counter << std::endl;

    if (input.keep_partially_sealed_connected_components == false) {
        ///////////////////////////////////////////////////////////////////////////
        // create the sealed meshes defined by the final set of traced polygons
        ///////////////////////////////////////////////////////////////////////////

        lg << "create final sealed connected components" << std::endl;

        for (std::map<char, std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>>::iterator color_to_separated_CCs_iter = color_to_separated_connected_ccsponents.begin();
             color_to_separated_CCs_iter != color_to_separated_connected_ccsponents.end();
             ++color_to_separated_CCs_iter) {

            const char color_label = color_to_separated_CCs_iter->first;
            //const cut_surface_patch_location_t location = patch_color_label_to_location.at(color_label);
            std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>& separated_sealed_CCs = color_to_separated_CCs_iter->second;

            mesh_t& m1_colored = color_to_m1.at(color_label);
            std::vector<traced_polygon_t>& m1_polygons_colored = color_to_m1_polygons.at(color_label);

            // extract the seam vertices
            extract_connected_components(separated_sealed_CCs, m1_colored, m1_polygons_colored, sm_polygons_below_cs, sm_polygons_above_cs, m1_vertex_to_seam_flag);
        }
    }

    sm_polygons_below_cs.clear(); // free
    sm_polygons_above_cs.clear();
    m1_vertex_to_seam_flag.clear();
    color_to_m1.clear();
    color_to_m1_polygons.clear();

    ///////////////////////////////////////////////////////////////////////////
    // save output and finish
    ///////////////////////////////////////////////////////////////////////////
    int ccc = 0;
    for (std::map<char, std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>>::const_iterator color_to_separated_CCs_iter = color_to_separated_connected_ccsponents.cbegin();
         color_to_separated_CCs_iter != color_to_separated_connected_ccsponents.cend();
         ++color_to_separated_CCs_iter) {
        const char color_label = color_to_separated_CCs_iter->first;
        const cut_surface_patch_location_t location = patch_color_label_to_location.at(color_label);
        const std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>& separated_sealed_CCs = color_to_separated_CCs_iter->second;

        for (std::map<std::size_t, std::vector<std::pair<mesh_t, connected_component_info_t>>>::const_iterator cc_iter = separated_sealed_CCs.cbegin();
             cc_iter != separated_sealed_CCs.cend();
             ++cc_iter) {

            //const int cc_id = (int)cc_iter->first;
            const std::vector<std::pair<mesh_t, connected_component_info_t>>& cc_instances = cc_iter->second;
            const std::pair<mesh_t, connected_component_info_t>& cc_mesh_data = cc_instances.back(); // last element is the final version which is copy with all polygons stitched

            std::map<connected_component_location_t, std::map<cut_surface_patch_location_t, std::vector<output_mesh_info_t>>>& out_sep_CCs = output.connected_components;
            if (input.verbose) {
                dump_mesh(cc_mesh_data.first, (std::string("cc") + std::to_string(ccc++)).c_str());
            }

            output_mesh_info_t omi;
            omi.mesh = std::move(cc_mesh_data.first);
            omi.seam_vertices = std::move(cc_mesh_data.second.seam_vertices);
            out_sep_CCs[cc_mesh_data.second.location][location].emplace_back(std::move(omi));
        }
    }

    patch_color_label_to_location.clear(); // free
    color_to_separated_connected_ccsponents.clear();

    lg << "end" << std::endl;

    return;
} // dispatch

} // namespace mcut
