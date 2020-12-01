#ifndef MCUT_KERNEL_H
#define MCUT_KERNEL_H
#include <mcut/internal/halfedge_mesh.h>

#include <map>
#include <vector>

namespace mcut {

//
// final execution states (i.e. did anything go wrong..?)
//
enum class status_t {
    SUCCESS = 0,
    // mesh is malformed
    // * vertices less than 3
    // * no faces
    // * non-manifold
    // * contains more than one connected component
    INVALID_SRC_MESH = -1,
    INVALID_CUT_MESH = -2,
    // there exists no edge in the input mesh which intersects cut-surface polygon
    INVALID_MESH_INTERSECTION = -3,
    // The bounding volume heirarchies of the input mesh and cut surface do not overlap
    INVALID_BVH_INTERSECTION = -4
};

//
// Position of a cut surface patch with respect to the input mesh
//
enum class cut_surface_patch_location_t : unsigned char {
    INSIDE, // + : The patch is located inside the input mesh volume (i.e. it is used to seal holes)
    OUTSIDE, // - : The patch is located outside the input mesh volume (boolean union).
    UNDEFINED // ~ : The notion of INSIDE/OUTSIDE is not applicable because input mesh is non-watertight
};

//
// Position of a connected component (CC) relative to cut-surface
//
enum class connected_component_location_t : unsigned char {
    ABOVE, // + : The CC is on positive side of the cut-surface (normal direction)
    BELOW, // - :  The CC is on negative side of the cut-surface (normal direction)
    UNDEFINED // ~ : The notion of ABOVE/BELOW is not applicable because the CC has [partially] cut
};

//
// The winding order of the polygons of a cut surface patch
//
enum class cut_surface_patch_winding_order_t : unsigned char {
    DEFAULT, // + : The polygons of the patch have the [same] winding order as the cut-surface (e.g. CCW)
    REVERSE, // - : The polygons of the patch have the [opposite] winding order as the cut-surface (e.g. CW)
};

//
// settings for how to execute the function "mcut::dispatch(...)"
//
struct input_t {
    const mesh_t* src_mesh;
    const mesh_t* cut_mesh;
    bool verbose = true;
    bool keep_partially_sealed_connected_components = false;
    bool require_looped_cutpaths = false; // ... i.e. bail on partial cuts (any!)
};

struct output_mesh_info_t {
    mesh_t mesh;
    std::vector<vd_t> seam_vertices;
};

//
// the output returned from the function "mcut::dispatch"
//
struct output_t {
    status_t status = status_t::SUCCESS;
    logger_t logger;
    std::map<connected_component_location_t, std::map<cut_surface_patch_location_t, std::vector<output_mesh_info_t>>> connected_components;
    std::map<connected_component_location_t, std::vector<output_mesh_info_t>> unsealed_cc; // connected components before hole-filling
    std::map<cut_surface_patch_winding_order_t, std::vector<output_mesh_info_t>> inside_patches; // .. between neigbouring connected ccsponents (cs-sealing patches)
    std::map<cut_surface_patch_winding_order_t, std::vector<output_mesh_info_t>> outside_patches;
    // the input meshes which also include the edges that define the cut path
    // NOTE: not always define (depending on the arising cutpath configurations)
    output_mesh_info_t seamed_src_mesh;
    output_mesh_info_t seamed_cut_mesh;
};

//
// returns string equivalent value (e.g. for printing)
//
std::string to_string(const connected_component_location_t&);
std::string to_string(const cut_surface_patch_location_t&);
std::string to_string(const status_t&);
std::string to_string(const cut_surface_patch_winding_order_t&);

void dispatch(output_t& out, const input_t& in);

} // namespace mcut

#endif // #ifndef MCUT_KERNEL_H
