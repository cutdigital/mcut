#include "mcut/internal/frontend.h"

#include "mcut/internal/halfedge_mesh.h"
#include "mcut/internal/geom.h"
#include "mcut/internal/kernel.h"
#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"
#include "mcut/internal/bvh.h"

#include <numeric> // std::partial_sum
#include <queue>
#include <random> // for numerical perturbation


namespace frontend {

    // If the inputs are found to not be in general position, then we perturb the
// cut-mesh by this constant (scaled by bbox diag times a random variable [0.1-1.0]).
const double GENERAL_POSITION_ENFORCMENT_CONSTANT = 1e-4;
const int MAX_PERTUBATION_ATTEMPTS = 1 << 3;

// this function converts an index array mesh (e.g. as recieved by the dispatch
// function) into a halfedge mesh representation for the kernel backend.
bool indexArrayMeshToHalfedgeMesh(
    std::unique_ptr<McDispatchContextInternal>& context_uptr,
    mcut::mesh_t& halfedgeMesh,
    double& bboxDiagonal,
    const void* pVertices,
    const uint32_t* pFaceIndices,
    const uint32_t* pFaceSizes,
    const uint32_t numVertices,
    const uint32_t numFaces,
    const mcut::math::vec3* perturbation = NULL)
{
    TIMESTACK_PUSH(__FUNCTION__);

    context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "construct halfedge mesh");

    // minor optimization
    halfedgeMesh.reserve_for_additional_elements(numVertices);

    TIMESTACK_PUSH("add vertices");

    // did the user provide vertex arrays of 32-bit floats...?
    if (context_uptr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_FLOAT) {
        const float* vptr = reinterpret_cast<const float*>(pVertices);

        // for each input mesh-vertex
        for (uint32_t i = 0; i < numVertices; ++i) {
            const float& x = vptr[(i * 3) + 0];
            const float& y = vptr[(i * 3) + 1];
            const float& z = vptr[(i * 3) + 2];

            // insert our vertex into halfedge mesh
            mcut::vd_t vd = halfedgeMesh.add_vertex(
                double(x) + (perturbation != NULL ? (*perturbation).x() : double(0.)),
                double(y) + (perturbation != NULL ? (*perturbation).y() : double(0.)),
                double(z) + (perturbation != NULL ? (*perturbation).z() : double(0.)));

            MCUT_ASSERT(vd != mcut::mesh_t::null_vertex() && (uint32_t)vd < numVertices);
        }
    }
    // did the user provide vertex arrays of 64-bit double...?
    else if (context_uptr->dispatchFlags & MC_DISPATCH_VERTEX_ARRAY_DOUBLE) {
        const double* vptr = reinterpret_cast<const double*>(pVertices);

        // for each input mesh-vertex
        for (uint32_t i = 0; i < numVertices; ++i) {
            const double& x = vptr[(i * 3) + 0];
            const double& y = vptr[(i * 3) + 1];
            const double& z = vptr[(i * 3) + 2];

            // insert our vertex into halfedge mesh
            mcut::vd_t vd = halfedgeMesh.add_vertex(
                double(x) + (perturbation != NULL ? (*perturbation).x() : double(0.)),
                double(y) + (perturbation != NULL ? (*perturbation).y() : double(0.)),
                double(z) + (perturbation != NULL ? (*perturbation).z() : double(0.)));

            MCUT_ASSERT(vd != mcut::mesh_t::null_vertex() && (uint32_t)vd < numVertices);
        }
    }

    TIMESTACK_POP();

    // compute the mesh bounding box while we are at it (for numerical perturbation)
    mcut::math::vec3 bboxMin(1e10);
    mcut::math::vec3 bboxMax(-1e10);

    TIMESTACK_PUSH("create bbox");
    for (mcut::vertex_array_iterator_t i = halfedgeMesh.vertices_begin(); i != halfedgeMesh.vertices_end(); ++i) {
        const mcut::math::vec3& coords = halfedgeMesh.vertex(*i);
        bboxMin = mcut::math::compwise_min(bboxMin, coords);
        bboxMax = mcut::math::compwise_max(bboxMax, coords);
    }
    bboxDiagonal = mcut::math::length(bboxMax - bboxMin);
    TIMESTACK_POP();

    TIMESTACK_PUSH("create faces");

#if defined(MCUT_MULTI_THREADED)
    std::vector<uint32_t> partial_sums(numFaces, 0); // prefix sum result
    std::partial_sum(pFaceSizes, pFaceSizes + numFaces, partial_sums.data());

    {
        typedef std::vector<uint32_t>::const_iterator InputStorageIteratorType;
        typedef std::pair<InputStorageIteratorType, InputStorageIteratorType> OutputStorageType; // range of faces
        std::atomic_int atm_result;
        atm_result.store((int)McResult::MC_NO_ERROR); // 0 = ok;/ 1 = invalid face size; 2 invalid vertex index

        std::vector<std::vector<mcut::vd_t>> faces(numFaces);

        auto fn_create_faces = [&](
                                   InputStorageIteratorType block_start_,
                                   InputStorageIteratorType block_end_) -> OutputStorageType {
            for (InputStorageIteratorType i = block_start_; i != block_end_; ++i) {
                uint32_t faceID = (uint32_t)std::distance(partial_sums.cbegin(), i);
                std::vector<mcut::vd_t>& faceVertices = faces[faceID];
                int numFaceVertices = ((uint32_t*)pFaceSizes)[faceID];

                if (numFaceVertices < 3) {
                    int zero = (int)McResult::MC_NO_ERROR;
                    bool exchanged = atm_result.compare_exchange_strong(zero, 1);
                    if (exchanged) // first thread to detect error
                    {
                        context_uptr->log( //
                            MC_DEBUG_SOURCE_API, //
                            MC_DEBUG_TYPE_ERROR, //
                            0, //
                            MC_DEBUG_SEVERITY_HIGH, //
                            "invalid face-size for face - " + std::to_string(faceID) + " (size = " + std::to_string(numFaceVertices) + ")");
                    }
                    break;
                }

                faceVertices.resize(numFaceVertices);
                int faceBaseOffset = (*i) - numFaceVertices;

                for (int j = 0; j < numFaceVertices; ++j) {
                    uint32_t idx = ((uint32_t*)pFaceIndices)[faceBaseOffset + j];

                    MCUT_ASSERT(idx < numVertices);

                    const mcut::vertex_descriptor_t descr(idx);
                    const bool isDuplicate = std::find(faceVertices.cbegin(), faceVertices.cend(), descr) != faceVertices.cend();

                    if (isDuplicate) {
                        int zero = (int)McResult::MC_NO_ERROR;
                        bool exchanged = atm_result.compare_exchange_strong(zero, 2);

                        if (exchanged) // first thread to detect error
                        {
                            context_uptr->log(
                                MC_DEBUG_SOURCE_API,
                                MC_DEBUG_TYPE_ERROR,
                                0,
                                MC_DEBUG_SEVERITY_HIGH,
                                "found duplicate vertex in face - " + std::to_string(faceID));
                        }
                        break;
                    }

                    faceVertices[j] = (descr);
                }
            }
            return std::make_pair(block_start_, block_end_);
        };

        std::vector<std::future<OutputStorageType>> futures;
        OutputStorageType partial_res;

        parallel_fork_and_join(
            context_uptr->scheduler,
            partial_sums.cbegin(),
            partial_sums.cend(),
            (1 << 8),
            fn_create_faces,
            partial_res, // output computed by master thread
            futures);

        auto add_faces = [&](InputStorageIteratorType block_start_,
                             InputStorageIteratorType block_end_) -> McResult {
            for (InputStorageIteratorType face_iter = block_start_;
                 face_iter != block_end_; ++face_iter) {
                uint32_t faceID = (uint32_t)std::distance(partial_sums.cbegin(), face_iter);
                const std::vector<mcut::vd_t>& faceVertices = faces.at(faceID);
                mcut::fd_t fd = halfedgeMesh.add_face(faceVertices);

                if (fd == mcut::mesh_t::null_face()) {
                    result = McResult::MC_INVALID_VALUE;
                    if (result != McResult::MC_NO_ERROR) {
                        context_uptr->log( //
                            MC_DEBUG_SOURCE_API, //
                            MC_DEBUG_TYPE_ERROR, //
                            0, //
                            MC_DEBUG_SEVERITY_HIGH, //
                            "invalid vertices on face - " + std::to_string(faceID));
                        return result;
                    }
                }
            }
            return McResult::MC_NO_ERROR;
        };

        bool okay = true;
        for (int i = 0; i < (int)futures.size(); ++i) {
            std::future<OutputStorageType>& f = futures[i];
            MCUT_ASSERT(f.valid()); // The behavior is undefined if valid()== false before the call to wait_for
            OutputStorageType future_res = f.get();

            const int val = atm_result.load();
            okay = okay && val == 0;
            if (!okay) {
                continue; // just go on (to next iteration) in order to at-least wait for all tasks to finish before we return to user
            }

            result = add_faces(future_res.first, future_res.second);
            okay = okay && result == McResult::MC_NO_ERROR;
        }

        if (!okay) {
            return McResult::MC_INVALID_VALUE;
        }

        // add lastly in order to maintain order
        result = add_faces(partial_res.first, partial_res.second);
        if (result != McResult::MC_NO_ERROR) {
            return result;
        }
    }
#else // #if defined(MCUT_MULTI_THREADED)
    int faceSizeOffset = 0;
    for (uint32_t i = 0; i < numFaces; ++i) {
        std::vector<mcut::vd_t> faceVertices;
        int numFaceVertices = 3; // triangle

        numFaceVertices = ((uint32_t*)pFaceSizes)[i];

        if (numFaceVertices < 3) {

            context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_ERROR, 0, MC_DEBUG_SEVERITY_HIGH, "invalid face-size for face - " + std::to_string(i) + " (size = " + std::to_string(numFaceVertices) + ")");

            return false;
        }

        faceVertices.reserve(numFaceVertices);

        for (int j = 0; j < numFaceVertices; ++j) {

            uint32_t idx = ((uint32_t*)pFaceIndices)[faceSizeOffset + j];
            const mcut::vertex_descriptor_t descr(idx); // = fIter->second; //vmap[*fIter.first];
            const bool isDuplicate = std::find(faceVertices.cbegin(), faceVertices.cend(), descr) != faceVertices.cend();

            if (isDuplicate) {

                context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_ERROR, 0, MC_DEBUG_SEVERITY_HIGH, "found duplicate vertex in face - " + std::to_string(i));

                return false;
            }

            faceVertices.push_back(descr);
        }

        mcut::fd_t fd = halfedgeMesh.add_face(faceVertices);

        if (fd == mcut::mesh_t::null_face()) {
            // Hint: this can happen when the mesh does not have a consistent
            // winding order i.e. some faces are CCW and others are CW
            context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_ERROR, 0, MC_DEBUG_SEVERITY_HIGH, "non-manifold edge on face " + std::to_string(i));

            return false;
        }

        faceSizeOffset += numFaceVertices;
    }
#endif
    TIMESTACK_POP();

    TIMESTACK_POP();

    return true;
}

// this function converts a halfedge mesh representation (from the kernel
// backend) to an index array mesh (for the user).
void halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
    const std::unique_ptr<McDispatchContextInternal>& context_uptr,
#endif
    IndexArrayMesh& indexArrayMesh,
    const mcut::output_mesh_info_t& halfedgeMeshInfo,
    const std::unordered_map<mcut::vd_t, mcut::math::vec3>& addedFpPartitioningVerticesOnCorrespondingInputSrcMesh,
    const std::unordered_map<mcut::fd_t, mcut::fd_t>& fpPartitionChildFaceToCorrespondingInputSrcMeshFace,
    const std::unordered_map<mcut::vd_t, mcut::math::vec3>& addedFpPartitioningVerticesOnCorrespondingInputCutMesh,
    const std::unordered_map<mcut::fd_t, mcut::fd_t>& fpPartitionChildFaceToCorrespondingInputCutMeshFace,
    const int userSrcMeshVertexCount,
    const int userSrcMeshFaceCount,
    const int internalSrcMeshVertexCount,
    const int internalSrcMeshFaceCount)
{
    SCOPED_TIMER(__FUNCTION__);

    //
    // vertices
    //
    TIMESTACK_PUSH("Add vertices");
    // create the vertices

    // number of vertices is the same irrespective of whether we are dealing with a
    // triangulated mesh instance or not. Thus, only one set of vertices is stored

    indexArrayMesh.numVertices = halfedgeMeshInfo.mesh.number_of_vertices();

    MCUT_ASSERT(indexArrayMesh.numVertices >= 3);

    indexArrayMesh.pVertices = std::unique_ptr<double[]>(new double[(std::size_t)indexArrayMesh.numVertices * 3u]);

    if (!halfedgeMeshInfo.data_maps.vertex_map.empty()) {
        indexArrayMesh.pVertexMapIndices = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numVertices]);
    }

#if defined(MCUT_MULTI_THREADED)
    {
        typedef mcut::vertex_array_iterator_t InputStorageIteratorType;
        typedef int OutputStorageType;

        auto fn_copy_vertices = [&](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) -> OutputStorageType {
            for (InputStorageIteratorType viter = block_start_; viter != block_end_; ++viter) {
                const mcut::math::vec3& point = halfedgeMeshInfo.mesh.vertex(*viter);
                const uint32_t i = (uint32_t)std::distance(halfedgeMeshInfo.mesh.vertices_begin(), viter);

                indexArrayMesh.pVertices[((size_t)i * 3u) + 0u] = point.x();
                indexArrayMesh.pVertices[((size_t)i * 3u) + 1u] = point.y();
                indexArrayMesh.pVertices[((size_t)i * 3u) + 2u] = point.z();

                if (!halfedgeMeshInfo.data_maps.vertex_map.empty()) {
                    MCUT_ASSERT((size_t)*viter < halfedgeMeshInfo.data_maps.vertex_map.size() /*halfedgeMeshInfo.data_maps.vertex_map.count(*vIter) == 1*/);

                    uint32_t internalInputMeshVertexDescr = halfedgeMeshInfo.data_maps.vertex_map.at(*viter);
                    uint32_t userInputMeshVertexDescr = UINT32_MAX;
                    bool internalInputMeshVertexDescrIsForIntersectionPoint = (internalInputMeshVertexDescr == UINT32_MAX);

                    if (!internalInputMeshVertexDescrIsForIntersectionPoint) { // user-mesh vertex or vertex that is added due to face-partitioning
                        bool vertexExistsDueToFacePartition = false;
                        const bool internalInputMeshVertexDescrIsForSrcMesh = ((int)internalInputMeshVertexDescr < internalSrcMeshVertexCount);

                        if (internalInputMeshVertexDescrIsForSrcMesh) {
                            std::unordered_map<mcut::vd_t, mcut::math::vec3>::const_iterator fiter = addedFpPartitioningVerticesOnCorrespondingInputSrcMesh.find(mcut::vd_t(internalInputMeshVertexDescr));
                            vertexExistsDueToFacePartition = (fiter != addedFpPartitioningVerticesOnCorrespondingInputSrcMesh.cend());
                        } else // internalInputMeshVertexDescrIsForCutMesh
                        {
                            std::unordered_map<mcut::vd_t, mcut::math::vec3>::const_iterator fiter = addedFpPartitioningVerticesOnCorrespondingInputCutMesh.find(mcut::vd_t(internalInputMeshVertexDescr));
                            vertexExistsDueToFacePartition = (fiter != addedFpPartitioningVerticesOnCorrespondingInputCutMesh.cend());
                        }

                        if (!vertexExistsDueToFacePartition) { // user-mesh vertex

                            MCUT_ASSERT(internalSrcMeshVertexCount > 0);

                            if (!internalInputMeshVertexDescrIsForSrcMesh) // is it a cut-mesh vertex discriptor ..?
                            {
                                const uint32_t internalInputMeshVertexDescrNoOffset = (internalInputMeshVertexDescr - internalSrcMeshVertexCount);
                                userInputMeshVertexDescr = (internalInputMeshVertexDescrNoOffset + userSrcMeshVertexCount); // ensure that we offset using number of [user-provided mesh] vertices
                            } else {
                                userInputMeshVertexDescr = internalInputMeshVertexDescr; // src-mesh vertices have no offset unlike cut-mesh vertices
                            }
                        }
                    }

                    indexArrayMesh.pVertexMapIndices[i] = userInputMeshVertexDescr;
                }
            }
            return 0;
        };

        std::vector<std::future<int>> futures;
        int _1;

        parallel_fork_and_join(
            context_uptr->scheduler,
            halfedgeMeshInfo.mesh.vertices_begin(),
            halfedgeMeshInfo.mesh.vertices_end(),
            (1 << 8),
            fn_copy_vertices,
            _1, // out
            futures);

        for (int i = 0; i < (int)futures.size(); ++i) {
            std::future<int>& f = futures[i];
            MCUT_ASSERT(f.valid());
            f.wait(); // simply wait for result to be done
        }
    }
#else // #if defined(MCUT_MULTI_THREADED)

    for (uint32_t i = 0; i < indexArrayMesh.numVertices; ++i) {

        // mcut::vertex_array_iterator_t vIter = halfedgeMeshInfo.mesh.vertices_begin();
        // std::advance(vIter, i);
        mcut::vd_t vdescr(i);
        const mcut::math::vec3& point = halfedgeMeshInfo.mesh.vertex(vdescr /**vIter*/);

        indexArrayMesh.pVertices[((size_t)i * 3u) + 0u] = point.x();
        indexArrayMesh.pVertices[((size_t)i * 3u) + 1u] = point.y();
        indexArrayMesh.pVertices[((size_t)i * 3u) + 2u] = point.z();

        // std::cout << indexArrayMesh.pVertices[(i * 3u) + 0u] << " " << indexArrayMesh.pVertices[(i * 3u) + 1u] << " " << indexArrayMesh.pVertices[(i * 3u) + 2u] << std::endl;

        // vmap[*vIter] = i;

        if (!halfedgeMeshInfo.data_maps.vertex_map.empty()) {
            MCUT_ASSERT((size_t)i < halfedgeMeshInfo.data_maps.vertex_map.size() /*halfedgeMeshInfo.data_maps.vertex_map.count(*vIter) == 1*/);

            // Here we use whatever value was assigned to the current vertex by the kernel.
            // Vertices that are polygon intersection points have a value of uint_max i.e. null_vertex().
            uint32_t internalInputMeshVertexDescr = halfedgeMeshInfo.data_maps.vertex_map.at(vdescr /**vIter*/);
            // We use the same default value as that used by the kernel for intersection
            // points (intersection points at mapped to uint_max i.e. null_vertex())
            uint32_t userInputMeshVertexDescr = UINT32_MAX;
            // This is true only for polygon intersection points computed by the kernel
            bool internalInputMeshVertexDescrIsForIntersectionPoint = (internalInputMeshVertexDescr == UINT32_MAX);

            if (!internalInputMeshVertexDescrIsForIntersectionPoint) { // user-mesh vertex or vertex that is added due to face-partitioning
                // NOTE: The kernel will assign/map a 'proper' index value to vertices that exist due to face partitioning.
                // 'proper' here means that the kernel treats these vertices as 'original vertices' from a user-provided input
                // mesh. In reality, we added such vertices in order to partition a face. i.e. the kernel is not aware
                // that a given input mesh it is working with is modified.
                // So, here we have to fix that mapping information to correctly state that "any vertex added due to face
                // partitioning was not in the user provided input mesh" and should therefore be treated/labelled as an intersection
                // point i.e. it should map to UINT32_MAX because it does not map to any vertex in the user provided input mesh.
                bool vertexExistsDueToFacePartition = false;
                const bool internalInputMeshVertexDescrIsForSrcMesh = ((int)internalInputMeshVertexDescr < internalSrcMeshVertexCount);

                if (internalInputMeshVertexDescrIsForSrcMesh) {
                    std::unordered_map<mcut::vd_t, mcut::math::vec3>::const_iterator fiter = addedFpPartitioningVerticesOnCorrespondingInputSrcMesh.find(mcut::vd_t(internalInputMeshVertexDescr));
                    vertexExistsDueToFacePartition = (fiter != addedFpPartitioningVerticesOnCorrespondingInputSrcMesh.cend());
                } else // internalInputMeshVertexDescrIsForCutMesh
                {
                    std::unordered_map<mcut::vd_t, mcut::math::vec3>::const_iterator fiter = addedFpPartitioningVerticesOnCorrespondingInputCutMesh.find(mcut::vd_t(internalInputMeshVertexDescr));
                    vertexExistsDueToFacePartition = (fiter != addedFpPartitioningVerticesOnCorrespondingInputCutMesh.cend());
                }

                if (!vertexExistsDueToFacePartition) { // user-mesh vertex

                    MCUT_ASSERT(internalSrcMeshVertexCount > 0);

                    if (!internalInputMeshVertexDescrIsForSrcMesh) // is it a cut-mesh vertex discriptor ..?
                    {

                        // vertices added due to face-partitioning will have an unoffsetted index/descr that is >= userSrcMeshVertexCount
                        const uint32_t internalInputMeshVertexDescrNoOffset = (internalInputMeshVertexDescr - internalSrcMeshVertexCount);

                        // if (internalInputMeshVertexDescrNoOffset < userCutMeshVertexCount) {
                        // const int offset_descrepancy = (internalSrcMeshVertexCount - userSrcMeshVertexCount);
                        userInputMeshVertexDescr = (internalInputMeshVertexDescrNoOffset + userSrcMeshVertexCount); // ensure that we offset using number of [user-provided mesh] vertices
                        //}
                    } else {
                        // if (internalInputMeshVertexDescr < userSrcMeshVertexCount) {
                        // const int offset_descrepancy = (internalSrcMeshVertexCount - userSrcMeshVertexCount);
                        userInputMeshVertexDescr = internalInputMeshVertexDescr; // src-mesh vertices have no offset unlike cut-mesh vertices
                        //}
                    }
                }
            }

            indexArrayMesh.pVertexMapIndices[i] = userInputMeshVertexDescr;
        }
    }
#endif
    // MCUT_ASSERT(!vmap.empty());

    TIMESTACK_POP();

    // create array of seam vertices

    TIMESTACK_PUSH("Create seam vertices");
    uint32_t numSeamVertexIndices = (uint32_t)halfedgeMeshInfo.seam_vertices.size();
    indexArrayMesh.numSeamVertexIndices = numSeamVertexIndices;
    if (indexArrayMesh.numSeamVertexIndices > 0u) {
        indexArrayMesh.pSeamVertexIndices = std::unique_ptr<uint32_t[]>(new uint32_t[numSeamVertexIndices]);
        for (uint32_t i = 0; i < numSeamVertexIndices; ++i) {
            indexArrayMesh.pSeamVertexIndices[i] = halfedgeMeshInfo.seam_vertices[i];
        }
    }
    TIMESTACK_POP();

    //
    // TODO: add functionality to add seam edges
    //

    //
    // faces
    //

    TIMESTACK_PUSH("Create faces");

    indexArrayMesh.numFaces = halfedgeMeshInfo.mesh.number_of_faces();

    MCUT_ASSERT(indexArrayMesh.numFaces > 0);

    indexArrayMesh.pFaceSizes = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaces]);

    if (!halfedgeMeshInfo.data_maps.face_map.empty()) {
        indexArrayMesh.pFaceMapIndices = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaces]);
    }

    indexArrayMesh.pFaceAdjFacesSizes = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaces]);

    //
    // Here, we collect size information about faces
    //
    std::vector<std::vector<mcut::fd_t>> gatheredFacesAdjFaces(indexArrayMesh.numFaces);
    std::vector<std::vector<mcut::vd_t>> gatheredFaces(indexArrayMesh.numFaces);

#if defined(MCUT_MULTI_THREADED)
    {
        typedef mcut::face_array_iterator_t InputStorageIteratorType;
        typedef int OutputStorageType;

        auto fn_copy_face_info0 = [&](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) -> OutputStorageType {
            for (InputStorageIteratorType i = block_start_; i != block_end_; ++i) {
                const uint32_t faceID = (uint32_t)std::distance(halfedgeMeshInfo.mesh.faces_begin(), i);

                {
                    std::vector<mcut::vd_t> vertices_around_face = halfedgeMeshInfo.mesh.get_vertices_around_face(*i);
                    indexArrayMesh.pFaceSizes[faceID] = (uint32_t)vertices_around_face.size();
                    gatheredFaces[faceID] = std::move(vertices_around_face);
                }

                {
                    std::vector<mcut::fd_t> adjFaces = halfedgeMeshInfo.mesh.get_faces_around_face(*i);
                    indexArrayMesh.pFaceAdjFacesSizes[faceID] = (uint32_t)adjFaces.size();
                    gatheredFacesAdjFaces[*i] = std::move(adjFaces);
                }

                if (!halfedgeMeshInfo.data_maps.face_map.empty()) {
                    MCUT_ASSERT((size_t)*i < halfedgeMeshInfo.data_maps.face_map.size() /*halfedgeMeshInfo.data_maps.face_map.count(*i) == 1*/);

                    uint32_t internalInputMeshFaceDescr = (uint32_t)halfedgeMeshInfo.data_maps.face_map.at(*i);
                    uint32_t userInputMeshFaceDescr = INT32_MAX;
                    const bool internalInputMeshFaceDescrIsForSrcMesh = ((int)internalInputMeshFaceDescr < internalSrcMeshFaceCount);

                    if (internalInputMeshFaceDescrIsForSrcMesh) {
                        std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator fiter = fpPartitionChildFaceToCorrespondingInputSrcMeshFace.find(mcut::fd_t(internalInputMeshFaceDescr));
                        if (fiter != fpPartitionChildFaceToCorrespondingInputSrcMeshFace.cend()) {
                            userInputMeshFaceDescr = fiter->second;
                        } else {
                            userInputMeshFaceDescr = internalInputMeshFaceDescr;
                        }
                        MCUT_ASSERT((int)userInputMeshFaceDescr < (int)userSrcMeshFaceCount);
                    } else // internalInputMeshVertexDescrIsForCutMesh
                    {
                        std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator fiter = fpPartitionChildFaceToCorrespondingInputCutMeshFace.find(mcut::fd_t(internalInputMeshFaceDescr));
                        if (fiter != fpPartitionChildFaceToCorrespondingInputCutMeshFace.cend()) {
                            uint32_t unoffsettedDescr = (fiter->second - internalSrcMeshFaceCount);
                            userInputMeshFaceDescr = unoffsettedDescr + userSrcMeshFaceCount;
                        } else {
                            uint32_t unoffsettedDescr = (internalInputMeshFaceDescr - internalSrcMeshFaceCount);
                            userInputMeshFaceDescr = unoffsettedDescr + userSrcMeshFaceCount;
                        }
                    }

                    MCUT_ASSERT(userInputMeshFaceDescr != INT32_MAX);

                    indexArrayMesh.pFaceMapIndices[(uint32_t)(*i)] = userInputMeshFaceDescr;
                } // if (!halfedgeMeshInfo.data_maps.face_map.empty()) {
            }
            return 0;
        };
        std::vector<std::future<int>> futures;
        int _1;

        auto vvv = halfedgeMeshInfo.mesh.vertices_begin();
        ;
        std::advance(vvv, 1);
        auto hhh = halfedgeMeshInfo.mesh.halfedges_begin();
        ;
        hhh += 1;
        // std::advance(hhh, 1);
        mcut::face_array_iterator_t fff = halfedgeMeshInfo.mesh.faces_begin();
        fff += 1;
        // std::advance(fff, (std::size_t)1);

        parallel_fork_and_join(
            context_uptr->scheduler,
            halfedgeMeshInfo.mesh.faces_begin(),
            halfedgeMeshInfo.mesh.faces_end(),
            (1 << 7),
            fn_copy_face_info0,
            _1, // out
            futures);

        for (int i = 0; i < (int)futures.size(); ++i) {
            std::future<int>& f = futures[i];
            MCUT_ASSERT(f.valid());
            f.wait(); // simply wait for result to be done
        }
    }
#else // #if defined(MCUT_MULTI_THREADED)

    int faceID = 0; // std::distance(halfedgeMeshInfo.mesh.faces_begin(), i);
    for (mcut::face_array_iterator_t i = halfedgeMeshInfo.mesh.faces_begin(); i != halfedgeMeshInfo.mesh.faces_end(); ++i) {
        // const int faceID = std::distance(halfedgeMeshInfo.mesh.faces_begin(), i);

        {
            std::vector<mcut::vd_t> vertices_around_face = halfedgeMeshInfo.mesh.get_vertices_around_face(*i);
            indexArrayMesh.pFaceSizes[faceID] = (uint32_t)vertices_around_face.size();
            gatheredFaces[faceID] = std::move(vertices_around_face);
        }

        {
            std::vector<mcut::fd_t> adjFaces = halfedgeMeshInfo.mesh.get_faces_around_face(*i);
            indexArrayMesh.pFaceAdjFacesSizes[faceID] = (uint32_t)adjFaces.size();
            gatheredFacesAdjFaces[*i] = std::move(adjFaces);
        }

        if (!halfedgeMeshInfo.data_maps.face_map.empty()) {
            MCUT_ASSERT((size_t)*i < halfedgeMeshInfo.data_maps.face_map.size() /*halfedgeMeshInfo.data_maps.face_map.count(*i) == 1*/);

            uint32_t internalInputMeshFaceDescr = (uint32_t)halfedgeMeshInfo.data_maps.face_map.at(*i);
            uint32_t userInputMeshFaceDescr = INT32_MAX;
            const bool internalInputMeshFaceDescrIsForSrcMesh = ((int)internalInputMeshFaceDescr < internalSrcMeshFaceCount);

            if (internalInputMeshFaceDescrIsForSrcMesh) {
                std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator fiter = fpPartitionChildFaceToCorrespondingInputSrcMeshFace.find(mcut::fd_t(internalInputMeshFaceDescr));
                if (fiter != fpPartitionChildFaceToCorrespondingInputSrcMeshFace.cend()) {
                    userInputMeshFaceDescr = fiter->second;
                } else {
                    userInputMeshFaceDescr = internalInputMeshFaceDescr;
                }
                MCUT_ASSERT((int)userInputMeshFaceDescr < (int)userSrcMeshFaceCount);
            } else // internalInputMeshVertexDescrIsForCutMesh
            {
                std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator fiter = fpPartitionChildFaceToCorrespondingInputCutMeshFace.find(mcut::fd_t(internalInputMeshFaceDescr));
                if (fiter != fpPartitionChildFaceToCorrespondingInputCutMeshFace.cend()) {
                    uint32_t unoffsettedDescr = (fiter->second - internalSrcMeshFaceCount);
                    userInputMeshFaceDescr = unoffsettedDescr + userSrcMeshFaceCount;
                } else {
                    uint32_t unoffsettedDescr = (internalInputMeshFaceDescr - internalSrcMeshFaceCount);
                    userInputMeshFaceDescr = unoffsettedDescr + userSrcMeshFaceCount;
                }
            }

            MCUT_ASSERT(userInputMeshFaceDescr != INT32_MAX);

            indexArrayMesh.pFaceMapIndices[(uint32_t)(*i)] = userInputMeshFaceDescr;
        } // if (!halfedgeMeshInfo.data_maps.face_map.empty()) {

        faceID++;
    }
#endif //#if defined(MCUT_MULTI_THREADED)
    MCUT_ASSERT(gatheredFacesAdjFaces.size() == indexArrayMesh.numFaces); // sanity check

    //
    // Here, we store information about faces (vertex indices, adjacent faces etc.)
    //

    std::vector<uint32_t> adjFaceArrayPartialSums(indexArrayMesh.numFaces, 0);
    std::partial_sum( //
        indexArrayMesh.pFaceAdjFacesSizes.get(), //
        indexArrayMesh.pFaceAdjFacesSizes.get() + indexArrayMesh.numFaces, //
        adjFaceArrayPartialSums.data());

    indexArrayMesh.numFaceAdjFaceIndices = adjFaceArrayPartialSums.back();
    indexArrayMesh.pFaceAdjFaces = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaceAdjFaceIndices]);

    std::vector<uint32_t> faceIndicesArrayPartialSums(indexArrayMesh.numFaces, 0);
    std::partial_sum( //
        indexArrayMesh.pFaceSizes.get(), //
        indexArrayMesh.pFaceSizes.get() + indexArrayMesh.numFaces, //
        faceIndicesArrayPartialSums.data());

    indexArrayMesh.numFaceIndices = faceIndicesArrayPartialSums.back();
    indexArrayMesh.pFaceIndices = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numFaceIndices]);

#if defined(MCUT_MULTI_THREADED)
    {
        typedef mcut::face_array_iterator_t InputStorageIteratorType;
        typedef int OutputStorageType;

        auto fn_copy_face_info1 = [&](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) -> OutputStorageType {
            for (InputStorageIteratorType i = block_start_; i != block_end_; ++i) {
                const uint32_t faceID = (uint32_t)std::distance(halfedgeMeshInfo.mesh.faces_begin(), i);
                { // store face-vertex indices
                    const std::vector<mcut::vd_t>& faceVertices = gatheredFaces[faceID];
                    const uint32_t faceSize = (uint32_t)faceVertices.size();
                    const int faceVertexIndexOffset = faceIndicesArrayPartialSums[faceID] - faceSize;

                    for (uint32_t j = 0; j < faceSize; ++j) {
                        const mcut::vd_t vd = faceVertices[j];
                        indexArrayMesh.pFaceIndices[(size_t)faceVertexIndexOffset + j] = (uint32_t)vd; // vmap[vd];
                    }
                }

                { // store adjacent-face indices
                    const std::vector<mcut::fd_t>& faceAdjFaces = gatheredFacesAdjFaces[faceID];
                    const uint32_t adjFacesSize = (uint32_t)faceAdjFaces.size();
                    const int faceAdjFaceIndexOffset = adjFaceArrayPartialSums[faceID] - adjFacesSize;

                    for (uint32_t j = 0; j < adjFacesSize; ++j) {
                        const mcut::fd_t adjFace = faceAdjFaces[j];
                        indexArrayMesh.pFaceAdjFaces[(size_t)faceAdjFaceIndexOffset + j] = (uint32_t)adjFace;
                    }
                }
            }
            return 0;
        };

        std::vector<std::future<int>> futures;
        int _1;

        parallel_fork_and_join(
            context_uptr->scheduler,
            halfedgeMeshInfo.mesh.faces_begin(),
            halfedgeMeshInfo.mesh.faces_end(),
            (1 << 8),
            fn_copy_face_info1,
            _1, // out
            futures);

        for (int i = 0; i < (int)futures.size(); ++i) {
            std::future<int>& f = futures[i];
            MCUT_ASSERT(f.valid());
            f.wait(); // simply wait for result to be done
        }
    }
#else // #if defined(MCUT_MULTI_THREADED)
    faceID = 0; // std::distance(halfedgeMeshInfo.mesh.faces_begin(), i);
    // for each face
    for (mcut::face_array_iterator_t i = halfedgeMeshInfo.mesh.faces_begin(); i != halfedgeMeshInfo.mesh.faces_end(); ++i) {

        { // store face-vertex indices
            const std::vector<mcut::vd_t>& faceVertices = gatheredFaces[faceID];
            const uint32_t faceSize = (uint32_t)faceVertices.size();
            const int faceVertexIndexOffset = faceIndicesArrayPartialSums[faceID] - faceSize;

            for (uint32_t j = 0; j < faceSize; ++j) {
                const mcut::vd_t vd = faceVertices[j];
                indexArrayMesh.pFaceIndices[(size_t)faceVertexIndexOffset + j] = (uint32_t)vd; // vmap[vd];
            }
        }

        { // store adjacent-face indices
            const std::vector<mcut::fd_t>& faceAdjFaces = gatheredFacesAdjFaces[faceID];
            const uint32_t adjFacesSize = (uint32_t)faceAdjFaces.size();
            const int faceAdjFaceIndexOffset = adjFaceArrayPartialSums[faceID] - adjFacesSize;

            for (uint32_t j = 0; j < adjFacesSize; ++j) {
                const mcut::fd_t adjFace = faceAdjFaces[j];
                indexArrayMesh.pFaceAdjFaces[(size_t)faceAdjFaceIndexOffset + j] = (uint32_t)adjFace;
            }
        }

        faceID++;
    }
#endif
    TIMESTACK_POP();

    //
    // edges
    //

    TIMESTACK_PUSH("Create edges");
    indexArrayMesh.numEdgeIndices = halfedgeMeshInfo.mesh.number_of_edges() * 2;

    MCUT_ASSERT(indexArrayMesh.numEdgeIndices > 0);
    indexArrayMesh.pEdges = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numEdgeIndices]);

    // std::vector<std::pair<mcut::vd_t, mcut::vd_t>> gatheredEdges;
#if defined(MCUT_MULTI_THREADED)
    {
        typedef mcut::edge_array_iterator_t InputStorageIteratorType;
        typedef int OutputStorageType;

        auto fn_copy_edges = [&](InputStorageIteratorType block_start_, InputStorageIteratorType block_end_) -> OutputStorageType {
            // uint32_t bs =*block_start_;
            // uint32_t be =*block_end_;

            for (InputStorageIteratorType eiter = block_start_; eiter != block_end_; ++eiter) {
                // printf("block_start_=%u; block_end_=%u eiter=%u\n", (uint32_t)*block_start_,  (uint32_t)*block_end_, (uint32_t)*eiter);
                // bool is_end = eiter == block_end_;
                // uint32_t edge_id = std::distance(halfedgeMeshInfo.mesh.edges_begin(), eiter);
                mcut::vd_t v0 = halfedgeMeshInfo.mesh.vertex(*eiter, 0);
                mcut::vd_t v1 = halfedgeMeshInfo.mesh.vertex(*eiter, 1);

                // uint32_t r = halfedgeMeshInfo.mesh.count_removed_elements_in_range(halfedgeMeshInfo.mesh.edges_begin(), eiter);
                //  NOTE: our override of std::distance accounts for removed elements
                uint32_t edge_idx = (uint32_t)std::distance(halfedgeMeshInfo.mesh.edges_begin(), eiter); // - r;
                // printf("edge_idx = %d (%u)\n",edge_idx, (uint32_t)*eiter );
                // MCUT_ASSERT((size_t)v0 < vmap.size());
                MCUT_ASSERT(((size_t)edge_idx * 2u) + 0u < indexArrayMesh.numEdgeIndices);
                indexArrayMesh.pEdges[((size_t)edge_idx * 2u) + 0u] = (uint32_t)v0; // vmap[v0];
                // MCUT_ASSERT((size_t)v1 < vmap.size());
                MCUT_ASSERT(((size_t)edge_idx * 2u) + 1u < indexArrayMesh.numEdgeIndices);
                indexArrayMesh.pEdges[((size_t)edge_idx * 2u) + 1u] = (uint32_t)v1; //  vmap[v1];
            }

            return 0;
        };

        std::vector<std::future<int>> futures;
        int _1;

        parallel_fork_and_join(
            context_uptr->scheduler,
            halfedgeMeshInfo.mesh.edges_begin(),
            halfedgeMeshInfo.mesh.edges_end(),
            (1 << 8),
            fn_copy_edges,
            _1, // out
            futures);

        for (int i = 0; i < (int)futures.size(); ++i) {
            std::future<int>& f = futures[i];
            MCUT_ASSERT(f.valid());
            f.wait(); // simply wait for result to be done
        }
    }
#else // #if defined(MCUT_MULTI_THREADED)
    // note: cannot use std::distance with halfedge mesh iterators
    // not implemented because it'd be too slow
    uint32_t edge_idx = 0; // std::distance(halfedgeMeshInfo.mesh.edges_begin(), i);

    for (mcut::edge_array_iterator_t i = halfedgeMeshInfo.mesh.edges_begin(); i != halfedgeMeshInfo.mesh.edges_end(); ++i) {

        mcut::vd_t v0 = halfedgeMeshInfo.mesh.vertex(*i, 0);
        mcut::vd_t v1 = halfedgeMeshInfo.mesh.vertex(*i, 1);

        // gatheredEdges.emplace_back(v0, v1);
        MCUT_ASSERT(((size_t)edge_idx * 2u) + 0u < indexArrayMesh.numEdgeIndices);
        // MCUT_ASSERT((size_t)v0 < vmap.size());
        indexArrayMesh.pEdges[((size_t)edge_idx * 2u) + 0u] = (uint32_t)v0; // vmap[v0];
        MCUT_ASSERT(((size_t)edge_idx * 2u) + 1u < indexArrayMesh.numEdgeIndices);
        // MCUT_ASSERT((size_t)v1 < vmap.size());
        indexArrayMesh.pEdges[((size_t)edge_idx * 2u) + 1u] = (uint32_t)v1; // vmap[v1];

        edge_idx++;
    }
#endif
#if 0
    // sanity check

    MCUT_ASSERT(gatheredEdges.size() == indexArrayMesh.numEdgeIndices / 2);

    indexArrayMesh.pEdges = std::unique_ptr<uint32_t[]>(new uint32_t[indexArrayMesh.numEdgeIndices]);

    for (uint32_t i = 0; i < (uint32_t)gatheredEdges.size(); ++i)
    {
        const std::pair<mcut::vd_t, mcut::vd_t> &edge = gatheredEdges[i];
        mcut::vd_t v0 = edge.first;
        mcut::vd_t v1 = edge.second;

        MCUT_ASSERT((size_t)v0 < vmap.size());
        indexArrayMesh.pEdges[((size_t)i * 2u) + 0u] = vmap[v0];
        MCUT_ASSERT((size_t)v1 < vmap.size());
        indexArrayMesh.pEdges[((size_t)i * 2u) + 1u] = vmap[v1];
    }
#endif
    TIMESTACK_POP();
}

bool is_coplanar(const mcut::mesh_t& m, const mcut::fd_t& f, int& fv_count)
{
    const std::vector<mcut::vd_t> vertices = m.get_vertices_around_face(f);
    fv_count = (int)vertices.size();
    if (fv_count > 3) // non-triangle
    {
        for (int i = 0; i < (fv_count - 3); ++i) {
            const int j = (i + 1) % fv_count;
            const int k = (i + 2) % fv_count;
            const int l = (i + 3) % fv_count;

            const mcut::vd_t& vi = vertices[i];
            const mcut::vd_t& vj = vertices[j];
            const mcut::vd_t& vk = vertices[k];
            const mcut::vd_t& vl = vertices[l];

            const mcut::math::vec3& vi_coords = m.vertex(vi);
            const mcut::math::vec3& vj_coords = m.vertex(vj);
            const mcut::math::vec3& vk_coords = m.vertex(vk);
            const mcut::math::vec3& vl_coords = m.vertex(vl);

            const bool are_coplaner = mcut::geom::coplaner(vi_coords, vj_coords, vk_coords, vl_coords);

            if (!are_coplaner) {
                return false;
            }
        }
    }
    return true;
}

// check that the halfedge-mesh version of a user-provided mesh is valid (i.e.
// it is a non-manifold mesh containing a single connected component etc.)
bool check_input_mesh(std::unique_ptr<McDispatchContextInternal>& context_uptr, const mcut::mesh_t& m)
{
    if (m.number_of_vertices() < 3) {
        context_uptr->log(
            MC_DEBUG_SOURCE_API,
            MC_DEBUG_TYPE_ERROR,
            0,
            MC_DEBUG_SEVERITY_HIGH,
            "Invalid vertex count (V=" + std::to_string(m.number_of_vertices()) + ")");
        return false;
    }

    if (m.number_of_faces() < 1) {
        context_uptr->log(
            MC_DEBUG_SOURCE_API,
            MC_DEBUG_TYPE_ERROR,
            0,
            MC_DEBUG_SEVERITY_HIGH,
            "Invalid face count (F=" + std::to_string(m.number_of_faces()) + ")");
        return false;
    }

    std::vector<int> fccmap;
    std::vector<int> cc_to_vertex_count;
    std::vector<int> cc_to_face_count;
    int n = mcut::find_connected_components(fccmap, m, cc_to_vertex_count, cc_to_face_count);

    if (n != 1) {
        context_uptr->log(
            MC_DEBUG_SOURCE_API,
            MC_DEBUG_TYPE_ERROR,
            0,
            MC_DEBUG_SEVERITY_HIGH,
            "Detected multiple connected components in mesh (N=" + std::to_string(n) + ")");
        return false;
    }

    // check that the vertices of each face are co-planar
    for (mcut::face_array_iterator_t f = m.faces_begin(); f != m.faces_end(); ++f) {
        int fv_count = 0;
        const bool face_is_coplanar = is_coplanar(m, *f, fv_count);
        if (!face_is_coplanar) {
            context_uptr->log(
                MC_DEBUG_SOURCE_API,
                MC_DEBUG_TYPE_OTHER,
                0,
                MC_DEBUG_SEVERITY_NOTIFICATION,
                "Vertices (" + std::to_string(fv_count) + ") on face " + std::to_string(*f) + " not coplanar");
            // No need to return false, simply warn. It is difficult to
            // know whether the non-coplanarity is severe enough to cause
            // confusion when computing intersection points between two
            // polygons (min=2 but sometimes can get 1 due to non-coplanarity
            // of face vertices).
            // In general, the more vertices on a face, the less likely
            // they are to be co-planar. Faces with a low number of polygons
            // are ideal (3 vertices being the best)
            // result = false;
            // break;
        }
    }

    return true;
}

McResult convert(const mcut::status_t& v)
{
    McResult result = McResult::MC_RESULT_MAX_ENUM;
    switch (v) {
    case mcut::status_t::SUCCESS:
        result = McResult::MC_NO_ERROR;
        break;
    case mcut::status_t::GENERAL_POSITION_VIOLATION:
    case mcut::status_t::INVALID_MESH_INTERSECTION:
        result = McResult::MC_INVALID_OPERATION;
        break;
    default:
        std::fprintf(stderr, "[MCUT]: warning - conversion error (McResult=%d)\n", (int)v);
    }
    return result;
}

McPatchLocation convert(const mcut::cut_surface_patch_location_t& v)
{
    McPatchLocation result = McPatchLocation::MC_PATCH_LOCATION_ALL;
    switch (v) {
    case mcut::cut_surface_patch_location_t::INSIDE:
        result = McPatchLocation::MC_PATCH_LOCATION_INSIDE;
        break;
    case mcut::cut_surface_patch_location_t::OUTSIDE:
        result = McPatchLocation::MC_PATCH_LOCATION_OUTSIDE;
        break;
    case mcut::cut_surface_patch_location_t::UNDEFINED:
        result = McPatchLocation::MC_PATCH_LOCATION_UNDEFINED;
        break;
    default:
        std::fprintf(stderr, "[MCUT]: warning - conversion error (McPatchLocation)\n");
    }
    return result;
}

McFragmentLocation convert(const mcut::connected_component_location_t& v)
{
    McFragmentLocation result = McFragmentLocation::MC_FRAGMENT_LOCATION_ALL;
    switch (v) {
    case mcut::connected_component_location_t::ABOVE:
        result = McFragmentLocation::MC_FRAGMENT_LOCATION_ABOVE;
        break;
    case mcut::connected_component_location_t::BELOW:
        result = McFragmentLocation::MC_FRAGMENT_LOCATION_BELOW;
        break;
    case mcut::connected_component_location_t::UNDEFINED:
        result = McFragmentLocation::MC_FRAGMENT_LOCATION_UNDEFINED;
        break;
    default:
        std::fprintf(stderr, "[MCUT]: warning - conversion error (McFragmentLocation)\n");
    }
    return result;
}

void intersect(
    std::unique_ptr<McDispatchContextInternal>& context_uptr,
    const void* pSrcMeshVertices,
    const uint32_t* pSrcMeshFaceIndices,
    const uint32_t* pSrcMeshFaceSizes,
    uint32_t numSrcMeshVertices,
    uint32_t numSrcMeshFaces,
    const void* pCutMeshVertices,
    const uint32_t* pCutMeshFaceIndices,
    const uint32_t* pCutMeshFaceSizes,
    uint32_t numCutMeshVertices,
    uint32_t numCutMeshFaces)
{
    mcut::mesh_t srcMeshInternal;
    double srcMeshBboxDiagonal(0.0);

    if (false == indexArrayMeshToHalfedgeMesh(context_uptr, srcMeshInternal, srcMeshBboxDiagonal, pSrcMeshVertices, pSrcMeshFaceIndices, pSrcMeshFaceSizes, numSrcMeshVertices, numSrcMeshFaces)) {
        throw std::invalid_argument("invalid source-mesh arrays");
    }

    if (false == check_input_mesh(context_uptr, srcMeshInternal)) {
        throw std::invalid_argument("invalid source-mesh connectivity");
    }

    mcut::input_t backendInput; // kernel/backend inpout

#if defined(MCUT_MULTI_THREADED)
    backendInput.scheduler = &context_uptr->scheduler;
#endif

    backendInput.src_mesh = &srcMeshInternal;

    backendInput.verbose = false;
    backendInput.require_looped_cutpaths = false;

    backendInput.verbose = static_cast<bool>((context_uptr->flags & MC_DEBUG) && (context_uptr->debugType & MC_DEBUG_SOURCE_KERNEL));
    backendInput.require_looped_cutpaths = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_REQUIRE_THROUGH_CUTS);
    backendInput.populate_vertex_maps = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_INCLUDE_VERTEX_MAP);
    backendInput.populate_face_maps = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_INCLUDE_FACE_MAP);

    uint32_t filterFlagsAll = ( //
        MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE | //
        MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW | //
        MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | //
        MC_DISPATCH_FILTER_FRAGMENT_SEALING_NONE | //
        MC_DISPATCH_FILTER_PATCH_INSIDE | //
        MC_DISPATCH_FILTER_PATCH_OUTSIDE | //
        MC_DISPATCH_FILTER_SEAM_SRCMESH | //
        MC_DISPATCH_FILTER_SEAM_CUTMESH);

    const bool dispatchFilteringEnabled = static_cast<bool>(context_uptr->dispatchFlags & filterFlagsAll); // any

    if (dispatchFilteringEnabled) { // user only wants [some] output connected components
        backendInput.keep_fragments_below_cutmesh = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW);
        backendInput.keep_fragments_above_cutmesh = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE);
        backendInput.keep_fragments_sealed_outside = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE);
        backendInput.keep_fragments_sealed_inside = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE);
        backendInput.keep_unsealed_fragments = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_SEALING_NONE);
        backendInput.keep_fragments_partially_cut = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_FRAGMENT_LOCATION_UNDEFINED);
        backendInput.keep_inside_patches = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_PATCH_INSIDE);
        backendInput.keep_outside_patches = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_PATCH_OUTSIDE);
        backendInput.keep_srcmesh_seam = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_SEAM_SRCMESH);
        backendInput.keep_cutmesh_seam = static_cast<bool>(context_uptr->dispatchFlags & MC_DISPATCH_FILTER_SEAM_CUTMESH);
    } else { // compute all possible types of connected components
        backendInput.keep_fragments_below_cutmesh = true;
        backendInput.keep_fragments_above_cutmesh = true;
        backendInput.keep_fragments_partially_cut = true;
        backendInput.keep_unsealed_fragments = true;
        backendInput.keep_fragments_sealed_outside = true; // mutually exclusive with exhaustive case
        backendInput.keep_fragments_sealed_inside = true;
        backendInput.keep_fragments_sealed_outside_exhaustive = false;
        backendInput.keep_fragments_sealed_inside_exhaustive = false;
        backendInput.keep_inside_patches = true;
        backendInput.keep_outside_patches = true;
        backendInput.keep_srcmesh_seam = true;
        backendInput.keep_cutmesh_seam = true;
    }

    backendInput.enforce_general_position = 0 != (context_uptr->dispatchFlags & MC_DISPATCH_ENFORCE_GENERAL_POSITION);

    // Construct BVHs
    // ::::::::::::::

    context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "Build source-mesh BVH");

#if defined(USE_OIBVH)
    std::vector<mcut::geom::bounding_box_t<mcut::math::vec3>> srcMeshBvhAABBs;
    std::vector<mcut::fd_t> srcMeshBvhLeafNodeFaces;
    std::vector<mcut::geom::bounding_box_t<mcut::math::vec3>> srcMeshFaceBboxes;
    mcut::bvh::constructOIBVH(srcMeshInternal, srcMeshBvhAABBs, srcMeshBvhLeafNodeFaces, srcMeshFaceBboxes);
#else
    mcut::bvh::BoundingVolumeHierarchy srcMeshBVH;
    srcMeshBVH.buildTree(srcMeshInternal);
#endif
    context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "Build cut-mesh BVH");

    std::unordered_map<mcut::fd_t, mcut::fd_t> fpPartitionChildFaceToInputSrcMeshFace;
    std::unordered_map<mcut::fd_t, mcut::fd_t> fpPartitionChildFaceToInputCutMeshFace;
    // descriptors and coordinates of vertices that are added into an input mesh
    // in order to carry out partitioning
    std::unordered_map<mcut::vd_t, mcut::math::vec3> addedFpPartitioningVerticesOnSrcMesh;
    std::unordered_map<mcut::vd_t, mcut::math::vec3> addedFpPartitioningVerticesOnCutMesh;

    int numSourceMeshFacesInLastDispatchCall = numSrcMeshFaces;

    mcut::output_t backendOutput;

    mcut::mesh_t cutMeshInternal;
    double cutMeshBboxDiagonal(0.0);

#if defined(USE_OIBVH)
    std::vector<mcut::geom::bounding_box_t<mcut::math::vec3>> cutMeshBvhAABBs;
    std::vector<mcut::fd_t> cutMeshBvhLeafNodeFaces;
    std::vector<mcut::geom::bounding_box_t<mcut::math::vec3>> cutMeshFaceBboxes;
#else
    mcut::bvh::BoundingVolumeHierarchy cutMeshBVH; // built later (see below)
#endif
    bool anyBvhWasRebuilt = true; // used to determine whether we should retraverse BVHs

    std::map<mcut::fd_t, std::vector<mcut::fd_t>> ps_face_to_potentially_intersecting_others; // result of BVH traversal

#if defined(MCUT_MULTI_THREADED)
    backendOutput.status.store(mcut::status_t::SUCCESS);
#else
    backendOutput.status = mcut::status_t::SUCCESS;
#endif

    int perturbationIters = 0;
    int kernelDispatchCallCounter = -1;
    double perturbation_const = 0.0; // = cutMeshBboxDiagonal * GENERAL_POSITION_ENFORCMENT_CONSTANT;

    do {
        kernelDispatchCallCounter++;

#if defined(MCUT_MULTI_THREADED)
        bool general_position_assumption_was_violated = ((backendOutput.status.load() == mcut::status_t::GENERAL_POSITION_VIOLATION));
        bool floating_polygon_was_detected = backendOutput.status.load() == mcut::status_t::DETECTED_FLOATING_POLYGON;
#else
        bool general_position_assumption_was_violated = (/*perturbationIters != -1 &&*/ (backendOutput.status == mcut::status_t::GENERAL_POSITION_VIOLATION));
        bool floating_polygon_was_detected = backendOutput.status == mcut::status_t::DETECTED_FLOATING_POLYGON;
#endif

        // ::::::::::::::::::::::::::::::::::::::::::::::::::::
#if defined(MCUT_MULTI_THREADED)
        backendOutput.status.store(mcut::status_t::SUCCESS);
#else
        backendOutput.status = mcut::status_t::SUCCESS;
#endif

        mcut::math::vec3 perturbation;

        if (general_position_assumption_was_violated) {
            MCUT_ASSERT(floating_polygon_was_detected == false); // cannot occur at same time!
            perturbationIters++;

            if (perturbationIters > 0) {

                if (perturbationIters >= MAX_PERTUBATION_ATTEMPTS) {
                    context_uptr->log(MC_DEBUG_SOURCE_KERNEL, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_MEDIUM, backendOutput.logger.get_reason_for_failure());
                   
                    throw std::runtime_error("max perturbation iteratons reached");
                }
                // use by the kernel track if the most-recent perturbation causes the cut-mesh and src-mesh to
                // not intersect at all, which means we need to perturb again.
                backendInput.general_position_enforcement_count = perturbationIters;

                MCUT_ASSERT(perturbation_const != double(0.0));

                std::default_random_engine rd(perturbationIters);
                std::mt19937 mt(rd());
                std::uniform_real_distribution<double> dist(-1.0, 1.0);
                perturbation = mcut::math::vec3(
                    double(static_cast<double>(dist(mt))) * perturbation_const,
                    double(static_cast<double>(dist(mt))) * perturbation_const,
                    double(static_cast<double>(dist(mt))) * perturbation_const);
            }
        } // if (general_position_assumption_was_violated) {

        if ((perturbationIters == 0 /*no perturbs required*/ || general_position_assumption_was_violated) && floating_polygon_was_detected == false) {

            // TODO: assume that re-adding elements (vertices and faces) is going to change the order
            // from the user-provided order. So we still need to fix the mapping, which may no longer
            // be one-to-one as in the case when things do not change.
            cutMeshInternal.reset();

            // TODO: the number of cut-mesh faces and vertices may increase due to polygon partitioning
            // Therefore: we need to perturb [the updated cut-mesh] i.e. the one containing partitioned polygons
            // "pCutMeshFaces" are simply the user provided faces
            // We must also use the newly added vertices (coords) due to polygon partitioning as "unperturbed" values
            // This will require some intricate mapping
            if (false == indexArrayMeshToHalfedgeMesh(context_uptr, cutMeshInternal, cutMeshBboxDiagonal, pCutMeshVertices, pCutMeshFaceIndices, pCutMeshFaceSizes, numCutMeshVertices, numCutMeshFaces, ((perturbationIters == 0) ? NULL : &perturbation))) {
                throw std::invalid_argument("invalid cut-mesh arrays");
            }

            perturbation_const = cutMeshBboxDiagonal * GENERAL_POSITION_ENFORCMENT_CONSTANT;

            backendInput.cut_mesh = &cutMeshInternal;

            if (perturbationIters == 0) {
#if defined(USE_OIBVH)
                cutMeshBvhAABBs.clear();
                cutMeshBvhLeafNodeFaces.clear();
                mcut::bvh::constructOIBVH(cutMeshInternal, cutMeshBvhAABBs, cutMeshBvhLeafNodeFaces, cutMeshFaceBboxes, perturbation_const);
#else
                cutMeshBVH.buildTree(cutMeshInternal, perturbation_const);
#endif
                anyBvhWasRebuilt = true;
            }
        }

        TIMESTACK_PUSH("partition floating polygons");
        if (floating_polygon_was_detected) {
            MCUT_ASSERT(general_position_assumption_was_violated == false); // cannot occur at same time (GP violation is detected before FPs)!

            bool srcMeshIsUpdated = false;
            bool cutMeshIsUpdated = false;

            for (std::map<mcut::fd_t, std::vector<mcut::floating_polygon_info_t>>::const_iterator detected_floating_polygons_iter = backendOutput.detected_floating_polygons.cbegin();
                 detected_floating_polygons_iter != backendOutput.detected_floating_polygons.cend();
                 ++detected_floating_polygons_iter) {

                // get the [origin] input-mesh face index (Note: this index may be offsetted
                // to distinguish between source-mesh and cut-mesh faces).
                const mcut::fd_t fpOffsettedOriginFaceDescriptor = detected_floating_polygons_iter->first;

                // NOTE: this boolean needs to be evaluated with "numSourceMeshFacesInLastDispatchCall" since the number of
                // src-mesh faces might change as we add more polygons due to partitioning.
                bool fpIsOnSrcMesh = ((uint32_t)fpOffsettedOriginFaceDescriptor < (uint32_t)numSourceMeshFacesInLastDispatchCall);

                // pointer to input mesh with face containing floating polygon
                // Note: this mesh will be modified as we add new faces.
                mcut::mesh_t* fpOriginInputMesh = (fpIsOnSrcMesh ? &srcMeshInternal : &cutMeshInternal);

                srcMeshIsUpdated = srcMeshIsUpdated || fpIsOnSrcMesh;
                cutMeshIsUpdated = cutMeshIsUpdated || !fpIsOnSrcMesh;

                // This data structure maps the new faces in the modified input mesh, to the original partitioned face in the [user-provided] input mesh.
                std::unordered_map<mcut::fd_t, mcut::fd_t>& fpOriginFaceChildFaceToUserInputMeshFace = (fpIsOnSrcMesh ? fpPartitionChildFaceToInputSrcMeshFace : fpPartitionChildFaceToInputCutMeshFace);
                // This data structure stores the vertices added into the input mesh partition one or more face .
                // We store the coordinates here too because they are sometimes needed to performed perturbation.
                // This perturbation can happen when an input mesh face is partitioned with e.g. edge where that
                // is sufficient to resolve all floating polygons detected of that input mesh face.
                std::unordered_map<mcut::vd_t, mcut::math::vec3>& fpOriginMeshAddedPartitioningVertices = (fpIsOnSrcMesh ? addedFpPartitioningVerticesOnSrcMesh : addedFpPartitioningVerticesOnCutMesh);

                // Now compute the actual input mesh face index (accounting for offset)
                const mcut::fd_t fpOriginFace = fpIsOnSrcMesh ? fpOffsettedOriginFaceDescriptor : mcut::fd_t((uint32_t)fpOffsettedOriginFaceDescriptor - (uint32_t)numSourceMeshFacesInLastDispatchCall); // accounting for offset (NOTE: must updated "srcMeshInternal" state)

                MCUT_ASSERT(static_cast<uint32_t>(fpOriginFace) < (uint32_t)fpOriginInputMesh->number_of_faces());

                // for each floating polygon detected on current ps-face
                for (std::vector<mcut::floating_polygon_info_t>::const_iterator psFaceFloatingPolyIter = detected_floating_polygons_iter->second.cbegin();
                     psFaceFloatingPolyIter != detected_floating_polygons_iter->second.cend();
                     ++psFaceFloatingPolyIter) {

                    const mcut::floating_polygon_info_t& fpi = *psFaceFloatingPolyIter;

                    // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // Here we now need to partition "origin_face" in "fpOriginInputMesh"
                    // by adding a new edge which is guarranteed to pass through the area
                    // spanned by the floating polygon.

                    // gather vertices of floating polygon (just a list of 3d coords provided by the kernel)

                    const size_t fpVertexCount = fpi.polygon_vertices.size();
                    MCUT_ASSERT(fpVertexCount >= 3);
                    const size_t fpEdgeCount = fpVertexCount; // num edges is same as num verts

                    // project the floating polygon " to 2D

                    std::vector<mcut::math::vec2> fpVertexCoords2D;

                    mcut::geom::project2D(fpVertexCoords2D, fpi.polygon_vertices, fpi.polygon_normal, fpi.polygon_normal_largest_component);

                    // face to be (potentially) partitioned
                    mcut::fd_t origin_face = fpOriginFace;

                    std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator originFaceToBirthFaceIter = fpOriginFaceChildFaceToUserInputMeshFace.find(origin_face);
                    // This is true if "fpOffsettedOriginFaceDescriptor" had more than one floating polygon.
                    // We need this to handle the case where another partition of "fpOffsettedOriginFaceDescriptor" (from one of the other floating polys)
                    // produced a new edge (i.e. the one partitioning "fpOffsettedOriginFaceDescriptor") that passes through the current floating poly. If this
                    // variable is true, we will need to
                    // 1) find all faces in "fpOriginFaceChildFaceToUserInputMeshFace" that are mapped to same birth-face as that of "fpOffsettedOriginFaceDescriptor" (i.e. search by value)
                    // 2) for each such face check to see if any one of its edges intersect the current floating polygon
                    // This is necessary to ensure a minimal set of partitions. See below for details.
                    bool birth_face_partitioned_atleast_once = (originFaceToBirthFaceIter != fpOriginFaceChildFaceToUserInputMeshFace.cend());
                    mcut::fd_t birth_face = mcut::mesh_t::null_face();

                    bool mustPartitionCurrentFace = true;
                    // check if we still need to partition origin_face.
                    // If a partitions has already been made that added an edge into "fpOriginInputMesh" which passes through the current
                    // floating poly, then we will not need to partition "fpOffsettedOriginFaceDescriptor".
                    // NOTE: there is no guarrantee that the previously added edge that partitions "fpOffsettedOriginFaceDescriptor" will not violate general-position w.r.t the current floating poly.
                    // Thus, general position might potentially be violated such that we would have to resort to numerical perturbation in the next mcut::dispatch(...) call.
                    if (birth_face_partitioned_atleast_once) {
                        birth_face = originFaceToBirthFaceIter->second;
                        MCUT_ASSERT(origin_face == originFaceToBirthFaceIter->first);

                        // the child face that we create by partitioning "birth_face" (possibly over multiple dispatch calls
                        // in the case that GP is violated by an added edge)
                        std::vector<mcut::fd_t> faces_from_partitioned_birth_face;

                        // for all other faces that share "birth_face"
                        for (std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator it = fpOriginFaceChildFaceToUserInputMeshFace.cbegin();
                             it != fpOriginFaceChildFaceToUserInputMeshFace.cend();
                             ++it) {
                            if (it->second == birth_face) { // matching birth face ?
                                faces_from_partitioned_birth_face.push_back(it->first);
                            }
                        }

                        bool haveFaceIntersectingFP = false;
                        // Should it be the case that we must proceed to make [another] partition of the
                        // birth-face, then "faceContainingFP" represent the existing face (a child of the birth face)
                        // in which the current floating polygon lies.
                        mcut::fd_t faceContainingFP = mcut::mesh_t::null_face();

                        // for each face sharing a birth face with origin_face
                        for (std::vector<mcut::fd_t>::const_iterator it = faces_from_partitioned_birth_face.cbegin();
                             it != faces_from_partitioned_birth_face.cend();
                             ++it) {

                            mcut::fd_t face = *it;

                            // ::::::::::::::::::::::
                            // get face vertex coords
                            const std::vector<mcut::vd_t> faceVertexDescriptors = fpOriginInputMesh->get_vertices_around_face(face);
                            std::vector<mcut::math::vec3> faceVertexCoords3D(faceVertexDescriptors.size());

                            for (std::vector<mcut::vd_t>::const_iterator i = faceVertexDescriptors.cbegin(); i != faceVertexDescriptors.cend(); ++i) {
                                const size_t idx = std::distance(faceVertexDescriptors.cbegin(), i);
                                const mcut::math::vec3& coords = fpOriginInputMesh->vertex(*i);
                                faceVertexCoords3D[idx] = coords;
                            }

                            // :::::::::::::::::::::::::
                            // project face coords to 2D
                            std::vector<mcut::math::vec2> faceVertexCoords2D;

                            mcut::geom::project2D(faceVertexCoords2D, faceVertexCoords3D, fpi.polygon_normal, fpi.polygon_normal_largest_component);

                            const int numFaceEdges = (int)faceVertexDescriptors.size(); // num edges == num verts
                            const int numFaceVertices = numFaceEdges;

                            // for each edge of face
                            for (int edgeIter = 0; edgeIter < numFaceEdges; ++edgeIter) {

                                const mcut::math::vec2& faceEdgeV0 = faceVertexCoords2D.at(((size_t)edgeIter) + 0);
                                const mcut::math::vec2& faceEdgeV1 = faceVertexCoords2D.at((((size_t)edgeIter) + 1) % numFaceVertices);

                                // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                                // Does the current edge of "face" intersect/pass through the area of
                                // the current floating polygon?

                                bool haveEdgeIntersectingFP = false;

                                // for each edge of current floating poly
                                for (int fpEdgeIter = 0; fpEdgeIter < (int)fpEdgeCount; ++fpEdgeIter) {

                                    const mcut::math::vec2& fpEdgeV0 = fpVertexCoords2D.at(((size_t)fpEdgeIter) + 0);
                                    const mcut::math::vec2& fpEdgeV1 = fpVertexCoords2D.at((((size_t)fpEdgeIter) + 1) % fpVertexCount);

                                    // placeholders
                                    double _1; // unused
                                    double _2; // unused
                                    mcut::math::vec2 _3; // unused

                                    const char res = mcut::geom::compute_segment_intersection(faceEdgeV0, faceEdgeV1, fpEdgeV0, fpEdgeV1, _3, _1, _2);

                                    if (res == '1') { // implies a propery segment-segment intersection
                                        haveEdgeIntersectingFP = true;
                                        break;
                                    }
                                }

                                if (haveEdgeIntersectingFP == false && faceContainingFP == mcut::mesh_t::null_face()) {
                                    // here we also do a test to find if the current face actually contains
                                    // the floating polygon in its area. We will need this information in order to
                                    // know the correct birth-face child-face that will be further partitioned
                                    // so as to prevent the current floating polygon from coming up again in the
                                    // next dispatch call.

                                    // for each floating polygon vertex ...
                                    for (int fpVertIter = 0; fpVertIter < (int)fpVertexCoords2D.size(); ++fpVertIter) {
                                        const char ret = mcut::geom::compute_point_in_polygon_test(fpVertexCoords2D.at(fpVertIter), faceVertexCoords2D);
                                        if (ret == 'i') { // check if strictly interior
                                            faceContainingFP = *it;
                                            break;
                                        }
                                    }
                                }

                                if (haveEdgeIntersectingFP) {
                                    haveFaceIntersectingFP = true;
                                    break;
                                }
                            } // for (std::vector<mcut::hd_t>::const_iterator hIt = halfedges.cbegin(); ...

                            if (haveFaceIntersectingFP) {
                                break; // done
                            }

                        } // for (std::vector<mcut::fd_t>::const_iterator it = faces_from_partitioned_birth_face.cbegin(); ...

                        // i.e. there exists no partitioning-edge which passes through the current floating polygon
                        mustPartitionCurrentFace = (haveFaceIntersectingFP == false);

                        if (mustPartitionCurrentFace) {
                            // update which face we treat as "origin_face" i.e. the one that we will partition
                            MCUT_ASSERT(faceContainingFP != mcut::mesh_t::null_face());
                            origin_face = faceContainingFP;
                        }

                    } // if (birth_face_partitioned_atleast_once) {
                    else {
                        birth_face = origin_face;
                    }

                    if (!mustPartitionCurrentFace) {
                        // skip current floating polygon no need to partition "origin_face" this time
                        // because an already-added edge into "fpOriginInputMesh" will prevent the current
                        // floating polygon from arising
                        continue; // got to next floating polygon
                    }

                    // gather vertices of "origin_face" (descriptors and 3d coords)

                    // std::vector<mcut::vd_t> originFaceVertexDescriptors = fpOriginInputMesh->get_vertices_around_face(origin_face);
                    std::vector<mcut::math::vec3> originFaceVertices3d;
                    // get information about each edge (used by "origin_face") that needs to be split along the respective intersection point
                    const std::vector<mcut::hd_t>& origFaceHalfedges = fpOriginInputMesh->get_halfedges_around_face(origin_face);

                    for (std::vector<mcut::hd_t>::const_iterator i = origFaceHalfedges.cbegin(); i != origFaceHalfedges.cend(); ++i) {
                        const mcut::vd_t src = fpOriginInputMesh->source(*i); // NOTE: we use source so that edge iterators/indices match with internal mesh storage
                        originFaceVertices3d.push_back(fpOriginInputMesh->vertex(src));
                    }

                    // MCUT_ASSERT(fpi.projection_component != -1); // should be defined when we identify the floating polygon in the kernel

                    // project the "origin_face" to 2D
                    // Since the geometry operations we are concerned about are inherently in 2d, here we project
                    // our coords from 3D to 2D. We project by eliminating the component corresponding
                    // to the "origin_face"'s normal vector's largest component. ("origin_face" and our
                    // floating polygon have the same normal!)
                    //

                    std::vector<mcut::math::vec2> originFaceVertexCoords2D;
                    mcut::geom::project2D(originFaceVertexCoords2D, originFaceVertices3d, fpi.polygon_normal, fpi.polygon_normal_largest_component);

                    // ROUGH STEPS TO COMPUTE THE LINE THAT WILL BE USED TO PARTITION origin_face
                    // 1. pick two edges in the floating polygon
                    // 2. compute their mid-points
                    // 3. construct a [segment] with these two mid-points
                    // 4. if any vertex of the floating-poly is on the [line] defined by the segment OR
                    //  ... if any vertex of the origin_face on the [line] defined by the segment:
                    //  --> GOTO step 1 and select another pair of edges in the floating poly
                    // 5. construct a ray with the segment whose origin lies outside origin_face
                    // 6. intersect the ray with all edges of origin_face, and keep the intersection points [on the boundary] of origin_face
                    // 7. compute mid-point of our segment (from the two mid-points in step 3)
                    // 8. Get the two closest intersection points to this mid-point of our segment
                    // 9. Partition origin_face using the two closest intersection points this mid-point
                    // 10. Likewise update the connectivity of neighbouring faces of origin_face
                    // --> Neighbours to update are inferred from the halfedges that are partitioned at the two intersection points
                    // 11. remove "origin_face" from "fpOriginInputMesh"
                    // 12. remove neighbours of "origin_face" from "fpOriginInputMesh" that shared the edge on which the two intersection points lie.
                    // 13. add the child_polygons of "origin_face" and the re-traced neighbours into "fpOriginInputMesh"
                    // 14.  store a mapping from newly traced polygons to the original (user provided) input mesh elements
                    // --> This will also be used client vertex- and face-data mapping.

                    auto fpGetEdgeVertexCoords = [&](const int fpEdgeIdx, mcut::math::vec2& fpEdgeV0, mcut::math::vec2& fpEdgeV1) {
                        const int fpFirstEdgeV0Idx = (((size_t)fpEdgeIdx) + 0);
                        fpEdgeV0 = fpVertexCoords2D.at(fpFirstEdgeV0Idx);
                        const int fpFirstEdgeV1Idx = (((size_t)fpEdgeIdx) + 1) % fpVertexCount;
                        fpEdgeV1 = fpVertexCoords2D.at(fpFirstEdgeV1Idx);
                    };

                    auto fpGetEdgeMidpoint = [&](int edgeIdx) {
                        mcut::math::vec2 edgeV0;
                        mcut::math::vec2 edgeV1;
                        fpGetEdgeVertexCoords(edgeIdx, edgeV0, edgeV1);

                        const mcut::math::vec2 midPoint(
                            (edgeV0.x() + edgeV1.x()) / double(2.0), //
                            (edgeV0.y() + edgeV1.y()) / double(2.0));

                        return midPoint;
                    };

                    auto fpGetMidpointDistance = [&](std::pair<int, int> edgePair) {
                        const mcut::math::vec2 edge0MidPoint = fpGetEdgeMidpoint(edgePair.first);
                        const mcut::math::vec2 edge1MidPoint = fpGetEdgeMidpoint(edgePair.second);
                        const double dist = mcut::math::squared_length(edge1MidPoint - edge0MidPoint);
                        return dist;
                    };

                    // NOTE: using max (i.e. < operator) lead to floating point precision issues on
                    // test 40. The only solution to which is exact arithmetic. However, since we still
                    // want MCUT to work even if the user only has fixed precision numbers.
                    // We pick edges based on this which are closest. No worries about colinear edges
                    // because they will be detected later and skipped!
                    auto fpMaxDistancePredicate = [&](std::pair<int, int> edgePairA, std::pair<int, int> edgePairB) -> bool {
                        const double aDist = fpGetMidpointDistance(edgePairA);
                        const double bDist = fpGetMidpointDistance(edgePairB);
                        return aDist < bDist;
                    };

                    std::priority_queue<
                        std::pair<int, int>, //
                        std::vector<std::pair<int, int>>, //
                        decltype(fpMaxDistancePredicate)>
                        fpEdgePairQueue(fpMaxDistancePredicate);

                    // populate queue with [unique] pairs of edges from the floating polygon
                    // priority is given to those pairs with the farthest distance between then
                    for (int i = 0; i < (int)fpEdgeCount; ++i) {
                        for (int j = i + 1; j < (int)fpEdgeCount; ++j) {
                            fpEdgePairQueue.push(std::make_pair(i, j));
                        }
                    }

                    MCUT_ASSERT(fpEdgePairQueue.size() >= 3); // we can have at least 3 pairs for the simplest polygon (triangle) i.e. assuming it is not generate

                    // In the next while loop, each iteration will attempt to contruct a line [passing through
                    // our floating polygon] that will be used partition "origin_face" .
                    // NOTE: the reason we have a while loop is because it allows us to test several possible lines
                    // with-which "origin_face" can be partitioned. Some lines may not usable because they pass through
                    // a vertex of the floating polygon or a vertex the "origin_face" - in which case GP will be
                    // violated.
                    //

                    bool haveSegmentOnFP = false; // the current pair of floating polygon edges worked!

                    // the line segment constructed from midpoints of two edges of the
                    // floating polygon
                    std::pair<mcut::math::vec2, mcut::math::vec2> fpSegment;

                    while (!fpEdgePairQueue.empty() && !haveSegmentOnFP) {

                        const std::pair<int, int> fpEdgePairCur = fpEdgePairQueue.top();
                        fpEdgePairQueue.pop();

                        const mcut::math::vec2 fpEdge0Midpoint = fpGetEdgeMidpoint(fpEdgePairCur.first);
                        const mcut::math::vec2 fpEdge1Midpoint = fpGetEdgeMidpoint(fpEdgePairCur.second);

                        // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                        // if the line intersects/passes through a vertex in "origin_face" or a vertex in
                        // the floating polygon then try another edge pair.

                        auto anyPointIsOnLine = [&](
                                                    const mcut::math::vec2& segStart,
                                                    const mcut::math::vec2& segEnd,
                                                    const std::vector<mcut::math::vec2>& polyVerts) -> bool {
                            double predResult(0xdeadbeef);
                            for (std::vector<mcut::math::vec2>::const_iterator it = polyVerts.cbegin(); it != polyVerts.cend(); ++it) {

                                bool are_collinear = mcut::geom::collinear(segStart, segEnd, (*it), predResult);
                                // last ditch attempt to prevent the possibility of creating a partitioning
                                // edge that more-or-less passes through a vertex (of origin-face or the floatig poly itself)
                                // see: test41
                                const double epsilon = 1e-6;
                                if (are_collinear || (!are_collinear && epsilon > std::fabs(predResult))) {
                                    return true;
                                }
                            }
                            return false;
                        }; // end lambda

                        // do we have general position? i.e. line segment does not pass through a vertex of the
                        // floating polygon and "origin_face"
                        bool haveGPOnFP = !anyPointIsOnLine(fpEdge0Midpoint, fpEdge1Midpoint, fpVertexCoords2D);
                        bool haveGPOnOriginFace = !anyPointIsOnLine(fpEdge0Midpoint, fpEdge1Midpoint, originFaceVertexCoords2D);
                        bool haveGP = haveGPOnFP && haveGPOnOriginFace;

                        if (haveGP /*|| true*/) {
                            haveSegmentOnFP = true;
                            fpSegment.first = fpEdge1Midpoint;
                            fpSegment.second = fpEdge0Midpoint;
                        }

                    } // while (fpEdgePairQueue.size() > 0 && successivelyPartitionedOriginFaceWithCurrentEdgePair == false) {

                    if (!haveSegmentOnFP) {
                        // OH OH!
                        // You have encountered an extremely rare problem case.
                        // Email the developers (there is a solution but it requires numerical perturbation on "fpSegment").
                        throw std::logic_error("Floating-polygon partitioning step could not find a usable fpSegment");
                    }

                    // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // At this point we have a valid line segment with which we can proceed to
                    // partition the "origin_mesh".

                    // Now we compute intersection points between every edge in "origin_face" and
                    // our segment (treating "fpSegment" as an infinitely-long line)

                    const int originFaceVertexCount = (int)originFaceVertices3d.size();
                    MCUT_ASSERT(originFaceVertexCount >= 3);
                    const int originFaceEdgeCount = originFaceVertexCount;

                    // this maps stores the intersection points between our line segment and the
                    // edges of "origin_face"
                    std::vector<
                        // information about "origin_face" edge that is to be split
                        std::pair<
                            // index of an edge (i.e. index in the halfedge-list, where this list is defined w.r.t the
                            // order of halfedges_around_face(origin_face)
                            int,
                            std::pair<
                                mcut::math::vec2, // intersection point coords
                                double //  parameter value (t) of intersection point along our edge (used to recover 3D coords)
                                >>>
                        originFaceIntersectedEdgeInfo;

                    // *************************************************************************************************************
                    // NOTE: "origFaceEdgeIter==0" corresponds to the second halfedge in the list returned by
                    // "get_halfedges_around_face(origin_face)".
                    // This is because "get_halfedges_around_face" builds the list of vertices by storing the target (not source) of
                    // each halfedge of a given face.
                    // *************************************************************************************************************

                    // for each edge in "origin_face"
                    for (int origFaceEdgeIter = 0; origFaceEdgeIter < originFaceEdgeCount; ++origFaceEdgeIter) {

                        const mcut::math::vec2& origFaceEdgeV0 = originFaceVertexCoords2D.at(((size_t)origFaceEdgeIter) + 0);
                        const mcut::math::vec2& origFaceEdgeV1 = originFaceVertexCoords2D.at(((origFaceEdgeIter) + 1) % originFaceVertexCount);

                        const double garbageVal(0xdeadbeef);
                        mcut::math::vec2 intersectionPoint(garbageVal);

                        double origFaceEdgeParam;
                        double fpEdgeParam;

                        char intersectionResult = mcut::geom::compute_segment_intersection(
                            origFaceEdgeV0, origFaceEdgeV1, fpSegment.first, fpSegment.second, intersectionPoint, origFaceEdgeParam, fpEdgeParam);

                        // These assertion must hold since, by construction, "fpSegment" (computed from two edges
                        // of the floating polygon) partitions the floating polygon which lies inside the area
                        // of "origin_face".
                        // Thus "fpSegment" can never intersect any half|edge/segment of "origin_face". It is the
                        // infinite-line represented by the "fpSegment" that can intersect edges of "origin_face".
                        MCUT_ASSERT(intersectionResult != '1'); // implies segment-segment intersection
                        MCUT_ASSERT(intersectionResult != 'v'); // implies that at-least one vertex of one segment touches the other
                        MCUT_ASSERT(intersectionResult != 'e'); // implies that segments collinearly overlap

                        if (
                            // intersection point was successively computed i.e. the infinite-line of "fpSegment" intersected the edge of "origin_face" (somewhere including outside of "origin_face")
                            (intersectionPoint.x() != garbageVal && intersectionPoint.y() != garbageVal) &&
                            // no actual segment-segment intersection exists, which is what we want
                            intersectionResult == '0') {
                            originFaceIntersectedEdgeInfo.push_back(std::make_pair(origFaceEdgeIter, std::make_pair(intersectionPoint, origFaceEdgeParam)));
                        }
                    } // for (int origFaceEdgeIter = 0; origFaceEdgeIter < originFaceEdgeCount; ++origFaceEdgeIter) {

                    // compute mid-point of "fpSegment", which we will used to find closest intersection points

                    const mcut::math::vec2 fpSegmentMidPoint(
                        (fpSegment.first.x() + fpSegment.second.x()) * double(0.5), //
                        (fpSegment.first.y() + fpSegment.second.y()) * double(0.5));

                    // Get the two closest [valid] intersection points to "fpSegmentMidPoint".
                    // We do this by sorting elements of "originFaceIntersectedEdgeInfo" by the distance
                    // of their respective intersection point from "fpSegmentMidPoint". We skip intersection
                    // points that do not lie on an edge of "origin_face" because they introduce ambiguities
                    // and that they are technically not usable (i.e. they are outside "origin_face").

                    std::sort(originFaceIntersectedEdgeInfo.begin(), originFaceIntersectedEdgeInfo.end(),
                        [&](const std::pair<int, std::pair<mcut::math::vec2, double>>& a, //
                            const std::pair<int, std::pair<mcut::math::vec2, double>>& b) {
                            double aDist(std::numeric_limits<double>::max()); // bias toward points inside polygon
                            // char aOnEdge = mcut::geom::compute_point_in_polygon_test(
                            //     a.second.first,
                            //     originFaceVertexCoords2D.data(),
                            //     (int)originFaceVertexCoords2D.size());

                            bool aOnEdge = (double(.0) <= a.second.second && double(1.) >= a.second.second);
                            // for (int i = 0; i < (int)originFaceVertexCoords2D.size(); ++i) {
                            //     int i0 = i;
                            //     int i1 = (i0 + 1) % (int)originFaceVertexCoords2D.size();
                            //     if (mcut::geom::collinear(originFaceVertexCoords2D[i0], originFaceVertexCoords2D[i1], a.second.first)) {
                            //          aOnEdge = true;
                            //          break;
                            //     }
                            // }

                            if (aOnEdge) {
                                const mcut::math::vec2 aVec = a.second.first - fpSegmentMidPoint;
                                aDist = mcut::math::squared_length(aVec);
                            }

                            double bDist(std::numeric_limits<double>::max());
                            // char bOnEdge = mcut::geom::compute_point_in_polygon_test(
                            //     b.second.first,
                            //     originFaceVertexCoords2D.data(),
                            //     (int)originFaceVertexCoords2D.size());
                            bool bOnEdge = (double(.0) <= b.second.second && double(1.) >= b.second.second);

                            // for (int i = 0; i < (int)originFaceVertexCoords2D.size(); ++i) {
                            //     int i0 = i;
                            //     int i1 = (i0 + 1) % (int)originFaceVertexCoords2D.size();
                            //     if (mcut::geom::collinear(originFaceVertexCoords2D[i0], originFaceVertexCoords2D[i1], b.second.first)) {
                            //         bOnEdge = true;
                            //         break;
                            //     }
                            // }

                            if (bOnEdge) {
                                const mcut::math::vec2 bVec = b.second.first - fpSegmentMidPoint;
                                bDist = mcut::math::squared_length(bVec);
                            }

                            return aDist < bDist;
                        });

                    // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // At this point we have all information necessary to partition "origin_face" using
                    // the two closest intersection points to "fpSegmentMidPoint".
                    //

                    // this std::vector stores the faces that use an edge that will be partitioned
                    std::vector<mcut::fd_t> replaced_input_mesh_faces = { origin_face };

                    MCUT_ASSERT(originFaceIntersectedEdgeInfo.size() >= 2); // we partition atleast two edges of origin_face [always!]

                    // origFaceEdge0: This is the first edge in the list after sorting.
                    // ---------------------------------------------------------------

                    const std::pair<int, std::pair<mcut::math::vec2, double>>& originFaceIntersectedEdge0Info = originFaceIntersectedEdgeInfo[0]; // first elem
                    const int origFaceEdge0Idx = originFaceIntersectedEdge0Info.first;
                    const double& origFaceEdge0IntPointEqnParam = originFaceIntersectedEdge0Info.second.second;

                    // NOTE: minus-1 since "get_vertices_around_face(origin_face)" builds a list using halfedge target vertices
                    // See the starred note above
                    int halfedgeIdx = origFaceEdge0Idx; // mcut::wrap_integer(origFaceEdge0Idx - 1, 0, (int)originFaceEdgeCount - 1); //(origFaceEdge0Idx + 1) % originFaceEdgeCount;
                    const mcut::hd_t origFaceEdge0Halfedge = origFaceHalfedges.at(halfedgeIdx);
                    MCUT_ASSERT(origin_face == fpOriginInputMesh->face(origFaceEdge0Halfedge));
                    const mcut::ed_t origFaceEdge0Descr = fpOriginInputMesh->edge(origFaceEdge0Halfedge);
                    const mcut::vd_t origFaceEdge0HalfedgeSrcDescr = fpOriginInputMesh->source(origFaceEdge0Halfedge);
                    const mcut::vd_t origFaceEdge0HalfedgeTgtDescr = fpOriginInputMesh->target(origFaceEdge0Halfedge);

                    // query src and tgt coords and build edge vector (i.e. "tgt - src"), which is in 3d
                    const mcut::math::vec3& origFaceEdge0HalfedgeSrc = fpOriginInputMesh->vertex(origFaceEdge0HalfedgeSrcDescr);
                    const mcut::math::vec3& origFaceEdge0HalfedgeTgt = fpOriginInputMesh->vertex(origFaceEdge0HalfedgeTgtDescr);

                    // infer 3D intersection point along edge using "origFaceEdge0IntPointEqnParam"
                    const mcut::math::vec3 origFaceEdge0Vec = (origFaceEdge0HalfedgeTgt - origFaceEdge0HalfedgeSrc);
                    const mcut::math::vec3 origFaceEdge0IntPoint3d = origFaceEdge0HalfedgeSrc + (origFaceEdge0Vec * origFaceEdge0IntPointEqnParam);
                    // TODO: ensure that "origFaceEdge0IntPoint3d" lies on the plane of "origFace", this is a source of many problems"""
                    const mcut::hd_t origFaceEdge0HalfedgeOpp = fpOriginInputMesh->opposite(origFaceEdge0Halfedge);
                    const mcut::fd_t origFaceEdge0HalfedgeOppFace = fpOriginInputMesh->face(origFaceEdge0HalfedgeOpp);

                    if (origFaceEdge0HalfedgeOppFace != mcut::mesh_t::null_face()) { // exists
                        // this check is needed in the case that both partitioned edges in "origin_face"
                        // are incident to the same two faces
                        const bool contained = std::find(replaced_input_mesh_faces.cbegin(), replaced_input_mesh_faces.cend(), origFaceEdge0HalfedgeOppFace) != replaced_input_mesh_faces.cend();
                        if (!contained) {
                            replaced_input_mesh_faces.push_back(origFaceEdge0HalfedgeOppFace);
                        }
                    }

                    // origFaceEdge1: This is the second edge in the list after sorting.
                    // ---------------------------------------------------------------

                    const std::pair<int, std::pair<mcut::math::vec2, double>>& originFaceIntersectedEdge1Info = originFaceIntersectedEdgeInfo[1]; // second elem
                    const int origFaceEdge1Idx = originFaceIntersectedEdge1Info.first;
                    const double& origFaceEdge1IntPointEqnParam = originFaceIntersectedEdge1Info.second.second;

                    halfedgeIdx = origFaceEdge1Idx; /// mcut::wrap_integer(origFaceEdge1Idx - 1, 0, (int)originFaceEdgeCount - 1); // (origFaceEdge1Idx + 1) % originFaceEdgeCount;
                    const mcut::hd_t origFaceEdge1Halfedge = origFaceHalfedges.at(halfedgeIdx);
                    MCUT_ASSERT(origin_face == fpOriginInputMesh->face(origFaceEdge1Halfedge));
                    const mcut::ed_t origFaceEdge1Descr = fpOriginInputMesh->edge(origFaceEdge1Halfedge);
                    const mcut::vd_t origFaceEdge1HalfedgeSrcDescr = fpOriginInputMesh->source(origFaceEdge1Halfedge);
                    const mcut::vd_t origFaceEdge1HalfedgeTgtDescr = fpOriginInputMesh->target(origFaceEdge1Halfedge);

                    // query src and tgt positions and build vector tgt - src
                    const mcut::math::vec3& origFaceEdge1HalfedgeSrc = fpOriginInputMesh->vertex(origFaceEdge1HalfedgeSrcDescr);
                    const mcut::math::vec3& origFaceEdge1HalfedgeTgt = fpOriginInputMesh->vertex(origFaceEdge1HalfedgeTgtDescr);

                    // infer intersection point in 3d using "origFaceEdge0IntPointEqnParam"
                    const mcut::math::vec3 origFaceEdge1Vec = (origFaceEdge1HalfedgeTgt - origFaceEdge1HalfedgeSrc);
                    const mcut::math::vec3 origFaceEdge1IntPoint3d = origFaceEdge1HalfedgeSrc + (origFaceEdge1Vec * origFaceEdge1IntPointEqnParam);

                    const mcut::hd_t origFaceEdge1HalfedgeOpp = fpOriginInputMesh->opposite(origFaceEdge1Halfedge);
                    const mcut::fd_t origFaceEdge1HalfedgeOppFace = fpOriginInputMesh->face(origFaceEdge1HalfedgeOpp);

                    if (origFaceEdge1HalfedgeOppFace != mcut::mesh_t::null_face()) { // exists
                        const bool contained = std::find(replaced_input_mesh_faces.cbegin(), replaced_input_mesh_faces.cend(), origFaceEdge1HalfedgeOppFace) != replaced_input_mesh_faces.cend();
                        if (!contained) {
                            replaced_input_mesh_faces.push_back(origFaceEdge1HalfedgeOppFace);
                        }
                    }

                    // gather halfedges of each neighbouring face of "origin_face" that is to be replaced
                    std::unordered_map<mcut::fd_t, std::vector<mcut::hd_t>> replacedOrigFaceNeighbourToOldHalfedges;

                    for (std::vector<mcut::fd_t>::const_iterator it = replaced_input_mesh_faces.cbegin(); it != replaced_input_mesh_faces.cend(); ++it) {
                        if (*it == origin_face) {
                            continue;
                        }
                        replacedOrigFaceNeighbourToOldHalfedges[*it] = fpOriginInputMesh->get_halfedges_around_face(*it);
                    }

                    // :::::::::::::::::::::::::::::::::::::::::::::::::::::
                    //** add new intersection points into fpOriginInputMesh

                    const mcut::vd_t origFaceEdge0IntPoint3dDescr = fpOriginInputMesh->add_vertex(origFaceEdge0IntPoint3d);
                    MCUT_ASSERT(fpOriginMeshAddedPartitioningVertices.count(origFaceEdge0IntPoint3dDescr) == 0);
                    fpOriginMeshAddedPartitioningVertices[origFaceEdge0IntPoint3dDescr] = origFaceEdge0IntPoint3d;

                    const mcut::vd_t origFaceEdge1IntPoint3dDescr = fpOriginInputMesh->add_vertex(origFaceEdge1IntPoint3d);
                    MCUT_ASSERT(fpOriginMeshAddedPartitioningVertices.count(origFaceEdge1IntPoint3dDescr) == 0);
                    fpOriginMeshAddedPartitioningVertices[origFaceEdge1IntPoint3dDescr] = origFaceEdge1IntPoint3d;

                    // :::::::::::
                    //** add edges

                    // halfedge between the intersection points
                    const mcut::hd_t intPointHalfedgeDescr = fpOriginInputMesh->add_edge(origFaceEdge0IntPoint3dDescr, origFaceEdge1IntPoint3dDescr);

                    // partitioning edges for origFaceEdge0
                    const mcut::hd_t origFaceEdge0FirstNewHalfedgeDescr = fpOriginInputMesh->add_edge(origFaceEdge0HalfedgeSrcDescr, origFaceEdge0IntPoint3dDescr); // o --> x
                    const mcut::hd_t origFaceEdge0SecondNewHalfedgeDescr = fpOriginInputMesh->add_edge(origFaceEdge0IntPoint3dDescr, origFaceEdge0HalfedgeTgtDescr); // x --> o

                    // partitioning edges for origFaceEdge1
                    const mcut::hd_t origFaceEdge1FirstNewHalfedgeDescr = fpOriginInputMesh->add_edge(origFaceEdge1HalfedgeSrcDescr, origFaceEdge1IntPoint3dDescr); // o--> x
                    const mcut::hd_t origFaceEdge1SecondNewHalfedgeDescr = fpOriginInputMesh->add_edge(origFaceEdge1IntPoint3dDescr, origFaceEdge1HalfedgeTgtDescr); // x --> o

                    // We will now re-trace the face that are incident to the partitioned edges to create
                    // new faces.
                    std::unordered_map<mcut::fd_t, std::vector<mcut::hd_t>> replacedOrigFaceNeighbourToNewHalfedges;

                    // NOTE: first we retrace the neighbouring polygons that shared a partitioned edge with "origin_face".
                    // These are somewhat easier to deal with first because a fixed set of steps can be followed with a simple for-loop.

                    // for each neighbouring face (w.r.t. "origin_face") to be replaced
                    for (std::unordered_map<mcut::fd_t, std::vector<mcut::hd_t>>::const_iterator i = replacedOrigFaceNeighbourToOldHalfedges.cbegin();
                         i != replacedOrigFaceNeighbourToOldHalfedges.cend();
                         ++i) {

                        mcut::fd_t face = i->first;
                        MCUT_ASSERT(face != origin_face); // avoid complex case here, where we need to partition the polygon in two. We'll handle that later.

                        const std::vector<mcut::hd_t>& oldHalfedges = i->second;

                        // for each halfedge of face
                        for (std::vector<mcut::hd_t>::const_iterator j = oldHalfedges.cbegin(); j != oldHalfedges.cend(); ++j) {

                            const mcut::hd_t oldHalfedge = *j;
                            mcut::hd_t newHalfedge = mcut::mesh_t::null_halfedge();
                            const mcut::ed_t oldHalfedgeEdge = fpOriginInputMesh->edge(oldHalfedge);

                            // is the halfedge part of an edge that is to be partitioned...?

                            if (oldHalfedgeEdge == origFaceEdge0Descr) {
                                mcut::hd_t firstNewHalfedge = fpOriginInputMesh->opposite(origFaceEdge0SecondNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(firstNewHalfedge);
                                mcut::hd_t secondNewHalfedge = fpOriginInputMesh->opposite(origFaceEdge0FirstNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(secondNewHalfedge);
                            } else if (oldHalfedgeEdge == origFaceEdge1Descr) {
                                mcut::hd_t firstNewHalfedge = fpOriginInputMesh->opposite(origFaceEdge1SecondNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(firstNewHalfedge);
                                mcut::hd_t secondNewHalfedge = fpOriginInputMesh->opposite(origFaceEdge1FirstNewHalfedgeDescr);
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(secondNewHalfedge);
                            } else {
                                replacedOrigFaceNeighbourToNewHalfedges[face].push_back(oldHalfedge); // maintain unpartitioned halfedge
                            }
                        } // for (std::vector<mcut::hd_t>::const_iterator j = oldHalfedges.cbegin(); j != oldHalfedges.cend(); ++j) {

                        // remove neighbour face
                        fpOriginInputMesh->remove_face(i->first);

                        // immediately add the updated tracing of the neighbour face so that it maintains the same desciptor!
                        std::vector<mcut::vd_t> faceVertices;
                        for (std::vector<mcut::hd_t>::const_iterator it = replacedOrigFaceNeighbourToNewHalfedges[face].cbegin();
                             it != replacedOrigFaceNeighbourToNewHalfedges[face].cend(); ++it) {
                            const mcut::vd_t tgt = fpOriginInputMesh->target(*it);
                            faceVertices.push_back(tgt);
                        }

                        const mcut::fd_t fdescr = fpOriginInputMesh->add_face(faceVertices);
                        MCUT_ASSERT(fdescr == i->first);

#if 0
                        std::unordered_map<mcut::fd_t, mcut::fd_t>::const_iterator fiter = fpOriginFaceChildFaceToUserInputMeshFace.find(fdescr);

                        bool descrIsMapped = (fiter != fpOriginFaceChildFaceToUserInputMeshFace.cend());

                        if (!descrIsMapped) {
                            fpOriginFaceChildFaceToUserInputMeshFace[fdescr] = birth_face;
                        }
#endif
                    } // for (std::unordered_map<mcut::fd_t, std::vector<mcut::hd_t>>::const_iterator i = replacedOrigFaceNeighbourToOldHalfedges.cbegin(); i != replacedOrigFaceNeighbourToOldHalfedges.cend(); ++i) {

                    // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                    // Here we now handle the complex case where we need to partition
                    // "origin_face" in two new faces.

                    fpOriginInputMesh->remove_face(origin_face); // one free slot

                    // This queue contains the halfegdes that we'll start to trace our new faces from
                    // (those connected to our new intersection points)
                    std::queue<mcut::hd_t> origFaceiHalfedges;
                    origFaceiHalfedges.push(intPointHalfedgeDescr);
                    origFaceiHalfedges.push(fpOriginInputMesh->opposite(intPointHalfedgeDescr));

                    // this list containing all halfedges along the boundary of "origin_face"
                    std::vector<mcut::hd_t> origFaceBoundaryHalfdges = { // first add the new boundary-edge partitioning halfedges, since we already know them
                        origFaceEdge0FirstNewHalfedgeDescr,
                        origFaceEdge0SecondNewHalfedgeDescr,
                        origFaceEdge1FirstNewHalfedgeDescr,
                        origFaceEdge1SecondNewHalfedgeDescr
                    };

                    // .... now we add the remaining boundary halfedges of "origin_face" i.e. those not partitioneds
                    for (std::vector<mcut::hd_t>::const_iterator it = origFaceHalfedges.cbegin(); it != origFaceHalfedges.cend(); ++it) {
                        if (*it != origFaceEdge0Halfedge && *it != origFaceEdge1Halfedge) { // if its not one of the replaced/partitioned halfedges
                            origFaceBoundaryHalfdges.push_back(*it);
                        }
                    }

                    // here we will store the tracing of the two child polygons that result from partitioning "origin_face"
                    std::vector<std::vector<mcut::hd_t>> origFaceChildPolygons;

                    do { // each iteration will trace a child polygon
                        mcut::hd_t childPolyHE_cur = mcut::mesh_t::null_halfedge();
                        mcut::hd_t childPolyHE_next = origFaceiHalfedges.front(); // start
                        origFaceiHalfedges.pop();

                        origFaceChildPolygons.push_back(std::vector<mcut::hd_t>());
                        std::vector<mcut::hd_t>& origFaceChildPoly = origFaceChildPolygons.back();

                        const mcut::hd_t firstHalfedge = childPolyHE_next;
                        const mcut::vd_t firstHalfedgeSrc = fpOriginInputMesh->source(firstHalfedge);

                        do {
                            childPolyHE_cur = childPolyHE_next;
                            origFaceChildPoly.push_back(childPolyHE_cur);
                            const mcut::vd_t childPolyHE_curTgt = fpOriginInputMesh->target(childPolyHE_cur);
                            childPolyHE_cur = mcut::mesh_t::null_halfedge();
                            childPolyHE_next = mcut::mesh_t::null_halfedge();

                            if (childPolyHE_curTgt != firstHalfedgeSrc) {
                                // find next halfedge to continue building the current child polygon
                                std::vector<mcut::hd_t>::const_iterator fiter = std::find_if(origFaceBoundaryHalfdges.cbegin(), origFaceBoundaryHalfdges.cend(),
                                    [&](const mcut::hd_t h) { // find a boundary halfedge that can be connected to the current halfedge
                                        const mcut::vd_t src = fpOriginInputMesh->source(h);
                                        return src == childPolyHE_curTgt;
                                    });

                                MCUT_ASSERT(fiter != origFaceBoundaryHalfdges.cend());

                                childPolyHE_next = *fiter;
                            }

                        } while (childPolyHE_next != mcut::mesh_t::null_halfedge());

                        MCUT_ASSERT(origFaceChildPoly.size() >= 3); // minimum size of valid polygon (triangle)

                        // Add child face into mesh
                        std::vector<mcut::vd_t> origFaceChildPolyVertices;

                        for (std::vector<mcut::hd_t>::const_iterator hIt = origFaceChildPoly.cbegin(); hIt != origFaceChildPoly.cend(); ++hIt) {
                            const mcut::vd_t tgt = fpOriginInputMesh->target(*hIt);
                            origFaceChildPolyVertices.push_back(tgt);
                        }

                        const mcut::fd_t fdescr = fpOriginInputMesh->add_face(origFaceChildPolyVertices);
                        MCUT_ASSERT(fdescr != mcut::mesh_t::null_face());

                        if (origFaceChildPolygons.size() == 1) {
                            // the first child face will re-use the descriptor of "origin_face".
                            MCUT_ASSERT(fdescr == origin_face);
                        }

                        fpOriginFaceChildFaceToUserInputMeshFace[fdescr] = birth_face;

                    } while (origFaceiHalfedges.empty() == false);

                    MCUT_ASSERT(origFaceChildPolygons.size() == 2); // "origin_face" shall only ever be partition into two child polygons

                    // remove the partitioned/'splitted' edges
                    fpOriginInputMesh->remove_edge(origFaceEdge0Descr);
                    fpOriginInputMesh->remove_edge(origFaceEdge1Descr);

                } // for (std::vector<mcut::floating_polygon_info_t>::const_iterator psFaceFloatingPolyIter = detected_floating_polygons_iter->second.cbegin(); ...
            } // for (std::vector<mcut::floating_polygon_info_t>::const_iterator detected_floating_polygons_iter = backendOutput.detected_floating_polygons.cbegin(); ...

            // ::::::::::::::::::::::::::::::::::::::::::::
            // rebuild the BVH of "fpOriginInputMesh" again

            if (srcMeshIsUpdated) {
#if defined(USE_OIBVH)
                srcMeshBvhAABBs.clear();
                srcMeshBvhLeafNodeFaces.clear();
                mcut::bvh::constructOIBVH(srcMeshInternal, srcMeshBvhAABBs, srcMeshBvhLeafNodeFaces, srcMeshFaceBboxes);
#else
                srcMeshBVH.buildTree(srcMeshInternal);
#endif
            }
            if (cutMeshIsUpdated) {
#if defined(USE_OIBVH)
                cutMeshBvhAABBs.clear();
                cutMeshBvhLeafNodeFaces.clear();
                mcut::bvh::constructOIBVH(cutMeshInternal, cutMeshBvhAABBs, cutMeshBvhLeafNodeFaces, cutMeshFaceBboxes, perturbation_const);
#else
                cutMeshBVH.buildTree(cutMeshInternal, perturbation_const);
#endif
            }

            anyBvhWasRebuilt = srcMeshIsUpdated || cutMeshIsUpdated;
            MCUT_ASSERT(anyBvhWasRebuilt == true);

            backendOutput.detected_floating_polygons.clear();
        } // if (floating_polygon_was_detected) {
        TIMESTACK_POP();

        // Check for mesh defects
        // ::::::::::::::::::::::

        // NOTE: we check for defects here since both input meshes may be modified by the polygon partitioning process above.
        // Partitiining is involked after atleast one dispatch call.
        context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "Check source-mesh for defects");

        if (false == check_input_mesh(context_uptr, srcMeshInternal)) {
            throw std::invalid_argument("invalid source-mesh connectivity");
        }

        context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "Check cut-mesh for defects");

        if (false == check_input_mesh(context_uptr, cutMeshInternal)) {
            throw std::invalid_argument("invalid cut-mesh connectivity");
        }

        if (anyBvhWasRebuilt) {
            // Evaluate BVHs to find polygon pairs that will be tested for intersection
            // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            anyBvhWasRebuilt = false;
            context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "Find potentially-intersecting polygons");

            ps_face_to_potentially_intersecting_others.clear();
#if defined(USE_OIBVH)
            mcut::bvh::intersectOIBVHs(ps_face_to_potentially_intersecting_others, srcMeshBvhAABBs, srcMeshBvhLeafNodeFaces, cutMeshBvhAABBs, cutMeshBvhLeafNodeFaces);
#else
            mcut::bvh::BoundingVolumeHierarchy::intersectBVHTrees(
#if defined(MCUT_MULTI_THREADED)
                context_uptr->scheduler,
#endif
                ps_face_to_potentially_intersecting_others,
                srcMeshBVH,
                cutMeshBVH,
                0,
                srcMeshInternal.number_of_faces());

#endif

            context_uptr->log(
                MC_DEBUG_SOURCE_API,
                MC_DEBUG_TYPE_OTHER,
                0,
                MC_DEBUG_SEVERITY_NOTIFICATION,
                "Polygon-pairs found = " + std::to_string(ps_face_to_potentially_intersecting_others.size()));

            if (ps_face_to_potentially_intersecting_others.empty()) {
                if (general_position_assumption_was_violated && perturbationIters > 0) {
                    // perturbation lead to an intersection-free state at the BVH level (and of-course the polygon level).
                    // We need to perturb again. (The whole cut mesh)
#if defined(MCUT_MULTI_THREADED)
                    backendOutput.status.store(mcut::status_t::GENERAL_POSITION_VIOLATION);
#else
                    backendOutput.status = mcut::status_t::GENERAL_POSITION_VIOLATION;
#endif
                    continue;
                } else {
                    context_uptr->log(MC_DEBUG_SOURCE_API, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "Mesh BVHs do not overlap.");
                    return; // we are done
                }
            }
        }

        backendInput.ps_face_to_potentially_intersecting_others = &ps_face_to_potentially_intersecting_others;
#if defined(USE_OIBVH)
        backendInput.srcMeshFaceBboxes = &srcMeshFaceBboxes;
        backendInput.cutMeshFaceBboxes = &cutMeshFaceBboxes;
#else
        backendInput.srcMeshBVH = &srcMeshBVH;
        backendInput.cutMeshBVH = &cutMeshBVH;
#endif
        // Invokee the kernel by calling the mcut::internal dispatch function
        // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

        numSourceMeshFacesInLastDispatchCall = srcMeshInternal.number_of_faces();

        try {
            context_uptr->log(MC_DEBUG_SOURCE_KERNEL, MC_DEBUG_TYPE_OTHER, 0, MC_DEBUG_SEVERITY_NOTIFICATION, "dispatch kernel");
            mcut::dispatch(backendOutput, backendInput);
        } catch (const std::exception &e) {
            fprintf(stderr, "fatal kernel exception caught : %s\n", e.what());
            throw e;
        }
    } while (
#if defined(MCUT_MULTI_THREADED)
        (backendOutput.status.load() == mcut::status_t::GENERAL_POSITION_VIOLATION && backendInput.enforce_general_position) || //
        backendOutput.status.load() == mcut::status_t::DETECTED_FLOATING_POLYGON
#else
        // general position voliation
        (backendOutput.status == mcut::status_t::GENERAL_POSITION_VIOLATION && backendInput.enforce_general_position) || //
        // kernel detected a floating polygon and we now need to re-partition the origin polygon (in src mesh or cut-mesh) to then recall mcut::dispatch
        backendOutput.status == mcut::status_t::DETECTED_FLOATING_POLYGON
#endif
    );

    if (convert(backendOutput.status) != McResult::MC_NO_ERROR) {

        context_uptr->log(
            MC_DEBUG_SOURCE_KERNEL,
            MC_DEBUG_TYPE_ERROR,
            0,
            MC_DEBUG_SEVERITY_HIGH,
            mcut::to_string(backendOutput.status) + " : " + backendOutput.logger.get_reason_for_failure());

        throw std::runtime_error("incomplete kernel execution");
    }

    TIMESTACK_PUSH("create face partition maps");
    // NOTE: face descriptors in "fpPartitionChildFaceToInputCutMeshFace", need to be offsetted
    // by the number of internal source-mesh faces/vertices. This is to ensure consistency with the kernel's data-mapping and make
    // it easier for us to map vertex and face descriptors in connected components to the correct instance in the user-provided
    // input meshes.
    // This offsetting follows a design choice used in the kernel that ("ps-faces" belonging to cut-mesh start [after] the
    // source-mesh faces).
    // Refer to the function "halfedgeMeshToIndexArrayMesh()" on how we use this information.
    std::unordered_map<mcut::fd_t, mcut::fd_t> fpPartitionChildFaceToInputCutMeshFaceOFFSETTED = fpPartitionChildFaceToInputCutMeshFace;
    for (std::unordered_map<mcut::fd_t, mcut::fd_t>::iterator i = fpPartitionChildFaceToInputCutMeshFace.begin();
         i != fpPartitionChildFaceToInputCutMeshFace.end(); ++i) {
        mcut::fd_t offsettedDescr = mcut::fd_t(i->first + srcMeshInternal.number_of_faces());
        fpPartitionChildFaceToInputCutMeshFaceOFFSETTED[offsettedDescr] = mcut::fd_t(i->second + numSourceMeshFacesInLastDispatchCall); // apply offset
                                                                                                                                        // i->second = mcut::fd_t(i->second + numSourceMeshFacesInLastDispatchCall); // apply offset
    }

    std::unordered_map<mcut::vd_t, mcut::math::vec3> addedFpPartitioningVerticesOnCutMeshOFFSETTED;
    for (std::unordered_map<mcut::vd_t, mcut::math::vec3>::const_iterator i = addedFpPartitioningVerticesOnCutMesh.begin();
         i != addedFpPartitioningVerticesOnCutMesh.end(); ++i) {
        mcut::vd_t offsettedDescr = mcut::vd_t(i->first + srcMeshInternal.number_of_vertices());
        addedFpPartitioningVerticesOnCutMeshOFFSETTED[offsettedDescr] = i->second; // apply offset
    }
    TIMESTACK_POP();

    //
    // sealed-fragment connected components
    //
    TIMESTACK_PUSH("store sealed-fragment connected components");
    for (std::map<mcut::connected_component_location_t, std::map<mcut::cut_surface_patch_location_t, std::vector<mcut::output_mesh_info_t>>>::const_iterator i = backendOutput.connected_components.cbegin();
         i != backendOutput.connected_components.cend();
         ++i) {

        for (std::map<mcut::cut_surface_patch_location_t, std::vector<mcut::output_mesh_info_t>>::const_iterator j = i->second.cbegin();
             j != i->second.cend();
             ++j) {

            // const std::string cs_patch_loc_str = mcut::to_string(j->first);

            for (std::vector<mcut::output_mesh_info_t>::const_iterator k = j->second.cbegin(); k != j->second.cend(); ++k) {

                std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> frag = std::unique_ptr<McFragmentConnComp, void (*)(McConnCompBase*)>(new McFragmentConnComp, ccDeletorFunc<McFragmentConnComp>);
                McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(frag.get());
                context_uptr->connComps.emplace(clientHandle, std::move(frag));
                McFragmentConnComp* asFragPtr = dynamic_cast<McFragmentConnComp*>(context_uptr->connComps.at(clientHandle).get());
                asFragPtr->type = MC_CONNECTED_COMPONENT_TYPE_FRAGMENT;
                asFragPtr->fragmentLocation = convert(i->first);
                asFragPtr->patchLocation = convert(j->first);

                MCUT_ASSERT(asFragPtr->patchLocation != MC_PATCH_LOCATION_UNDEFINED);
                asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_COMPLETE;

                halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
                    context_uptr,
#endif
                    asFragPtr->indexArrayMesh, *k,
                    addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
                    numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
            }
        }
    }
    TIMESTACK_POP();

    //
    // unsealed connected components (fragements)
    //
    TIMESTACK_PUSH("store unsealed connected components");
    for (std::map<mcut::connected_component_location_t, std::vector<mcut::output_mesh_info_t>>::const_iterator i = backendOutput.unsealed_cc.cbegin();
         i != backendOutput.unsealed_cc.cend();
         ++i) { // for each cc location flag (above/below/undefined)

        for (std::vector<mcut::output_mesh_info_t>::const_iterator j = i->second.cbegin(); j != i->second.cend(); ++j) { // for each mesh

            std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> unsealedFrag = std::unique_ptr<McFragmentConnComp, void (*)(McConnCompBase*)>(new McFragmentConnComp, ccDeletorFunc<McFragmentConnComp>);
            McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(unsealedFrag.get());
            context_uptr->connComps.emplace(clientHandle, std::move(unsealedFrag));
            McFragmentConnComp* asFragPtr = dynamic_cast<McFragmentConnComp*>(context_uptr->connComps.at(clientHandle).get());
            asFragPtr->type = MC_CONNECTED_COMPONENT_TYPE_FRAGMENT;
            asFragPtr->fragmentLocation = convert(i->first);
            asFragPtr->patchLocation = McPatchLocation::MC_PATCH_LOCATION_UNDEFINED;
            asFragPtr->srcMeshSealType = McFragmentSealType::MC_FRAGMENT_SEAL_TYPE_NONE;

            halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
                context_uptr,
#endif
                asFragPtr->indexArrayMesh, *j,
                addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
                numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
        }
    }
    TIMESTACK_POP();

    // inside patches
    TIMESTACK_PUSH("store interior patches");
    const std::vector<mcut::output_mesh_info_t>& insidePatches = backendOutput.inside_patches[mcut::cut_surface_patch_winding_order_t::DEFAULT];

    for (std::vector<mcut::output_mesh_info_t>::const_iterator it = insidePatches.cbegin();
         it != insidePatches.cend();
         ++it) {

        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> patchConnComp = std::unique_ptr<McPatchConnComp, void (*)(McConnCompBase*)>(new McPatchConnComp, ccDeletorFunc<McPatchConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(patchConnComp.get());
        context_uptr->connComps.emplace(clientHandle, std::move(patchConnComp));
        McPatchConnComp* asPatchPtr = dynamic_cast<McPatchConnComp*>(context_uptr->connComps.at(clientHandle).get());
        asPatchPtr->type = MC_CONNECTED_COMPONENT_TYPE_PATCH;
        asPatchPtr->patchLocation = MC_PATCH_LOCATION_INSIDE;

        halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
            context_uptr,
#endif
            asPatchPtr->indexArrayMesh, *it,
            addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
            numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
    }
    TIMESTACK_POP();

    // outside patches
    TIMESTACK_PUSH("store exterior patches");
    const std::vector<mcut::output_mesh_info_t>& outsidePatches = backendOutput.outside_patches[mcut::cut_surface_patch_winding_order_t::DEFAULT];

    for (std::vector<mcut::output_mesh_info_t>::const_iterator it = outsidePatches.cbegin(); it != outsidePatches.cend(); ++it) {

        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> patchConnComp = std::unique_ptr<McPatchConnComp, void (*)(McConnCompBase*)>(new McPatchConnComp, ccDeletorFunc<McPatchConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(patchConnComp.get());
        context_uptr->connComps.emplace(clientHandle, std::move(patchConnComp));
        McPatchConnComp* asPatchPtr = dynamic_cast<McPatchConnComp*>(context_uptr->connComps.at(clientHandle).get());
        asPatchPtr->type = MC_CONNECTED_COMPONENT_TYPE_PATCH;
        asPatchPtr->patchLocation = MC_PATCH_LOCATION_OUTSIDE;

        halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
            context_uptr,
#endif
            asPatchPtr->indexArrayMesh, *it,
            addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
            numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
    }
    TIMESTACK_POP();

    // seam connected components
    // -------------------------

    // NOTE: seam meshes are not available if there was a partial cut intersection (due to constraints imposed by halfedge construction rules).

    //  src mesh

    if (backendOutput.seamed_src_mesh.mesh.number_of_faces() > 0) {
        TIMESTACK_PUSH("store source-mesh seam");
        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> srcMeshSeam = std::unique_ptr<McSeamConnComp, void (*)(McConnCompBase*)>(new McSeamConnComp, ccDeletorFunc<McSeamConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(srcMeshSeam.get());
        context_uptr->connComps.emplace(clientHandle, std::move(srcMeshSeam));
        McSeamConnComp* asSrcMeshSeamPtr = dynamic_cast<McSeamConnComp*>(context_uptr->connComps.at(clientHandle).get());
        asSrcMeshSeamPtr->type = MC_CONNECTED_COMPONENT_TYPE_SEAM;
        asSrcMeshSeamPtr->origin = MC_SEAM_ORIGIN_SRCMESH;
        halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
            context_uptr,
#endif
            asSrcMeshSeamPtr->indexArrayMesh, backendOutput.seamed_src_mesh,
            addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
            numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
        TIMESTACK_POP();
    }

    //  cut mesh

    if (backendOutput.seamed_cut_mesh.mesh.number_of_faces() > 0) {
        TIMESTACK_PUSH("store cut-mesh seam");
        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> cutMeshSeam = std::unique_ptr<McSeamConnComp, void (*)(McConnCompBase*)>(new McSeamConnComp, ccDeletorFunc<McSeamConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(cutMeshSeam.get());
        context_uptr->connComps.emplace(clientHandle, std::move(cutMeshSeam));
        McSeamConnComp* asCutMeshSeamPtr = dynamic_cast<McSeamConnComp*>(context_uptr->connComps.at(clientHandle).get());
        asCutMeshSeamPtr->type = MC_CONNECTED_COMPONENT_TYPE_SEAM;
        asCutMeshSeamPtr->origin = MC_SEAM_ORIGIN_CUTMESH;

        halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
            context_uptr,
#endif
            asCutMeshSeamPtr->indexArrayMesh, backendOutput.seamed_cut_mesh,
            addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
            numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
        TIMESTACK_POP();
    }

    // input connected components
    // --------------------------

    // internal cut-mesh (possibly with new faces and vertices)
    {
        TIMESTACK_PUSH("store original cut-mesh");
        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> internalCutMesh = std::unique_ptr<McInputConnComp, void (*)(McConnCompBase*)>(new McInputConnComp, ccDeletorFunc<McInputConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(internalCutMesh.get());
        context_uptr->connComps.emplace(clientHandle, std::move(internalCutMesh));
        McInputConnComp* asCutMeshInputPtr = dynamic_cast<McInputConnComp*>(context_uptr->connComps.at(clientHandle).get());
        asCutMeshInputPtr->type = MC_CONNECTED_COMPONENT_TYPE_INPUT;
        asCutMeshInputPtr->origin = MC_INPUT_ORIGIN_CUTMESH;

        mcut::output_mesh_info_t omi;
        omi.mesh = cutMeshInternal; // naive copy (could use std::move)

        // TODO: assume that re-adding elements (vertices and faces) e.g. prior to perturbation or partitioning is going to change the order
        // from the user-provided order. So we still need to fix the mapping, which may no longer
        // be one-to-one (even if with an sm offset ) as in the case when things do not change.

        if (backendInput.populate_vertex_maps) {
            omi.data_maps.vertex_map.resize(cutMeshInternal.number_of_vertices());
            for (mcut::vertex_array_iterator_t i = cutMeshInternal.vertices_begin(); i != cutMeshInternal.vertices_end(); ++i) {
                omi.data_maps.vertex_map[*i] = mcut::vd_t((*i) + srcMeshInternal.number_of_vertices()); // apply offset like kernel does
            }
        }

        if (backendInput.populate_face_maps) {
            omi.data_maps.face_map.resize(cutMeshInternal.number_of_faces());
            for (mcut::face_array_iterator_t i = cutMeshInternal.faces_begin(); i != cutMeshInternal.faces_end(); ++i) {
                omi.data_maps.face_map[*i] = mcut::fd_t((*i) + srcMeshInternal.number_of_faces()); // apply offset like kernel does
            }
        }

        omi.seam_vertices = {}; // empty. an input connected component has no polygon intersection points

        halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
            context_uptr,
#endif
            asCutMeshInputPtr->indexArrayMesh, omi,
            addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
            numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
        TIMESTACK_POP();
    }

    // internal source-mesh (possibly with new faces and vertices)
    {
        TIMESTACK_PUSH("store original src-mesh");
        std::unique_ptr<McConnCompBase, void (*)(McConnCompBase*)> internalSrcMesh = std::unique_ptr<McInputConnComp, void (*)(McConnCompBase*)>(new McInputConnComp, ccDeletorFunc<McInputConnComp>);
        McConnectedComponent clientHandle = reinterpret_cast<McConnectedComponent>(internalSrcMesh.get());
        context_uptr->connComps.emplace(clientHandle, std::move(internalSrcMesh));
        McInputConnComp* asSrcMeshInputPtr = dynamic_cast<McInputConnComp*>(context_uptr->connComps.at(clientHandle).get());
        asSrcMeshInputPtr->type = MC_CONNECTED_COMPONENT_TYPE_INPUT;
        asSrcMeshInputPtr->origin = MC_INPUT_ORIGIN_SRCMESH;

        mcut::output_mesh_info_t omi;
        omi.mesh = srcMeshInternal; // naive copy
        if (backendInput.populate_vertex_maps) {
            omi.data_maps.vertex_map.resize(srcMeshInternal.number_of_vertices());
            for (mcut::vertex_array_iterator_t i = srcMeshInternal.vertices_begin(); i != srcMeshInternal.vertices_end(); ++i) {
                omi.data_maps.vertex_map[*i] = *i; // one to one mapping
            }
        }

        if (backendInput.populate_face_maps) {
            omi.data_maps.face_map.resize(srcMeshInternal.number_of_faces());
            for (mcut::face_array_iterator_t i = srcMeshInternal.faces_begin(); i != srcMeshInternal.faces_end(); ++i) {
                omi.data_maps.face_map[*i] = *i; // one to one mapping
            }
        }

        omi.seam_vertices = {}; // empty. an input connected component has no polygon intersection points

        halfedgeMeshToIndexArrayMesh(
#if defined(MCUT_MULTI_THREADED)
            context_uptr,
#endif
            asSrcMeshInputPtr->indexArrayMesh, omi,
            addedFpPartitioningVerticesOnSrcMesh, fpPartitionChildFaceToInputSrcMeshFace, addedFpPartitioningVerticesOnCutMeshOFFSETTED, fpPartitionChildFaceToInputCutMeshFaceOFFSETTED,
            numSrcMeshVertices, numSrcMeshFaces, srcMeshInternal.number_of_vertices(), srcMeshInternal.number_of_faces());
        TIMESTACK_POP();
    }
}
}