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

#ifndef MCUT_HALFEDGE_MESH_H_
#define MCUT_HALFEDGE_MESH_H_

#include "mcut/internal/math.h"
#include "mcut/internal/utils.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory> // shared_ptr
#include <vector>

namespace mcut {

template <typename T>
class descriptor_t_ {
public:
    typedef unsigned int index_type;
    descriptor_t_() { }
    virtual ~descriptor_t_() { }
    explicit descriptor_t_(index_type i = (std::numeric_limits<index_type>::max)())
        : m_value(i)
    {
    }

    operator index_type() const { return m_value; }

    void reset() { m_value = (std::numeric_limits<index_type>::max)(); }

    bool is_valid() const
    {
        index_type inf = (std::numeric_limits<index_type>::max)();
        return m_value != inf;
    }

    descriptor_t_& operator=(const index_type& _rhs)
    {
        m_value = _rhs;
        return *this;
    }

    descriptor_t_& operator=(const T& _rhs) const
    {
        m_value = _rhs.m_value;
        return *this;
    }

    bool operator==(const T& _rhs) const
    {
        return m_value == _rhs.m_value;
    }

    bool operator!=(const T& _rhs) const
    {
        return m_value != _rhs.m_value;
    }

    bool operator<(const T& _rhs) const
    {
        return m_value < _rhs.m_value;
    }

    descriptor_t_& operator++()
    {
        ++m_value;
        return *this;
    }

    descriptor_t_& operator--()
    {
        --m_value;
        return *this;
    }

    descriptor_t_ operator++(int)
    {
        descriptor_t_ tmp(*this);
        ++m_value;
        return tmp;
    }

    descriptor_t_ operator--(int)
    {
        descriptor_t_ tmp(*this);
        --m_value;
        return tmp;
    }

    descriptor_t_& operator+=(std::ptrdiff_t n)
    {
        m_value = m_value + n;
        return *this;
    }

protected:
    unsigned int m_value;
};

class halfedge_descriptor_t : public descriptor_t_<halfedge_descriptor_t> {
public:
    halfedge_descriptor_t()
        : descriptor_t_<halfedge_descriptor_t>(std::numeric_limits<index_type>::max())
    {
    }

    explicit halfedge_descriptor_t(descriptor_t_<halfedge_descriptor_t>::index_type idx)
        : descriptor_t_<halfedge_descriptor_t>(idx)
    {
    }

    virtual ~halfedge_descriptor_t()
    {
    }
};

class edge_descriptor_t : public descriptor_t_<edge_descriptor_t> {
public:
    edge_descriptor_t()
        : descriptor_t_<edge_descriptor_t>((std::numeric_limits<index_type>::max)())
    {
    }

    explicit edge_descriptor_t(descriptor_t_<edge_descriptor_t>::index_type idx)
        : descriptor_t_<edge_descriptor_t>(idx)
    {
    }

    virtual ~edge_descriptor_t()
    {
    }
};

class face_descriptor_t : public descriptor_t_<face_descriptor_t> {
public:
    face_descriptor_t()
        : descriptor_t_<face_descriptor_t>((std::numeric_limits<index_type>::max)())
    {
    }

    explicit face_descriptor_t(descriptor_t_<face_descriptor_t>::index_type idx)
        : descriptor_t_<face_descriptor_t>(idx)
    {
    }

    virtual ~face_descriptor_t()
    {
    }
};

class vertex_descriptor_t : public descriptor_t_<vertex_descriptor_t> {
public:
    vertex_descriptor_t()
        : descriptor_t_<vertex_descriptor_t>((std::numeric_limits<index_type>::max)())
    {
    }

    explicit vertex_descriptor_t(descriptor_t_<vertex_descriptor_t>::index_type idx)
        : descriptor_t_<vertex_descriptor_t>(idx)
    {
    }

    virtual ~vertex_descriptor_t()
    {
    }
};

/*
  Internal mesh data structure used for cutting meshes

  Memory Management
  
  Memory management is semi-automatic. Memory grows as more elements are added to the structure but does not shrink when elements are removed. 
  When you add elements and the capacity of the underlying vector is exhausted, the vector reallocates memory. 
  As descriptors are basically indices, they refer to the same element after a reallocation. 
  When you remove an element it is only marked as removed. 
  Internally it is put in a free list, and when you add elements to the surface mesh, they are taken from the free list in case it is not empty.
  
  For all elements there is a function to obtain the number of used elements, as well as the number of used [and] removed elements. 
  For vertices the functions are mesh_t::number_of_vertices() and mesh_t::number_of_internal_vertices(), respectively. 
  The first function is slightly different from the free function num_vertices(const G&) of the BGL package. 
 
  Iterators such as mesh_t::vertex_iterator_t only enumerate elements that are not marked as deleted.
*/
class mesh_t {

public:
    template<typename T>
    struct mesh_data_t
    {
        typedef T descriptor_type;
    };

    struct halfedge_data_t : mesh_data_t<halfedge_descriptor_t>{
        halfedge_descriptor_t o; // opposite halfedge
        halfedge_descriptor_t n; // next halfedge
        halfedge_descriptor_t p; // previous halfedge
        vertex_descriptor_t t; // target vertex
        edge_descriptor_t e; // edge
        face_descriptor_t f; // face

        halfedge_data_t()
            : o(null_halfedge())
            , n(null_halfedge())
            , p(null_halfedge())
            , t(null_vertex())
            , e(null_edge())
            , f(null_face())
        {
        }
    };

    struct edge_data_t : mesh_data_t<edge_descriptor_t>  {
        halfedge_descriptor_t h; // primary halfedge (even idx)
    };

    struct face_data_t  : mesh_data_t<face_descriptor_t>{
        std::vector<halfedge_descriptor_t> m_halfedges;
    };

    struct vertex_data_t : mesh_data_t<vertex_descriptor_t> {
        mcut::math::vec3 p; // geometry coordinates
        std::vector<face_descriptor_t> m_faces; // ... incident to vertex
        std::vector<halfedge_descriptor_t> m_halfedges; // ... which point to vertex (note: can be used to infer edges too)
    };

    typedef std::vector<vertex_data_t> vertex_array_t;
    typedef std::vector<edge_data_t> edge_array_t;
    typedef std::vector<halfedge_data_t> halfedge_array_t;
    typedef std::vector<face_data_t> face_array_t;
#if 0
    // iterator over the keys of a map (base class)
    template <typename M>
    class key_iterator_t : public M::const_iterator {
        const mesh_t* const mesh_ptr;

    public:
        key_iterator_t()
            : M::const_iterator()
            , mesh_ptr(nullptr) {};
        explicit key_iterator_t(typename M::const_iterator it_, const mesh_t* const mesh)
            : M::const_iterator(it_)
            , mesh_ptr(mesh)
        {
        }

        typename M::key_type* operator->()
        {
            return (typename M::key_type* const)&(M::const_iterator::operator->()->first);
        }

        typename M::key_type& operator*()
        {
            return *this->operator->();
        }

        // postfix increment (i++)
        // increment pointer to the next valid element (i.e. we skip removed elements).
        typename M::key_type operator++(int)
        {
            bool old_elem_is_removed = false;

            do {
                typename M::const_iterator old_elem = M::const_iterator::operator++(0); // (i++)
                old_elem_is_removed = mesh_ptr->is_removed(old_elem->first);
                if (!old_elem_is_removed) {
                    return old_elem->first;
                }

                // keep iterating until the value returned by the (i++) operator returns a valid element
                // i.e. one that is not marked removed!
            } while ((*this) != cend<std::remove_reference<decltype(*this)>::type>() && old_elem_is_removed);

            return M::key_type();
        }

        // prefix increment (++i)
        // increment pointer to the next valid element (i.e. we skip removed elements).
        typename M::key_type& operator++()
        {
            bool cur_elem_is_removed = false;
            bool reached_end = false;
            do {
                M::const_iterator::operator++(); // (++i)
                reached_end = (*this) == cend<typename std::remove_reference<decltype(*this)>::type>();
                cur_elem_is_removed = false;

                if (!reached_end) {
                    cur_elem_is_removed = mesh_ptr->is_removed(this->operator*());
                    if (!cur_elem_is_removed) {
                        break;
                    }
                }

                // keep iterating until the value pointed to after the (++i) operator is a valid element
                // i.e. one that is not marked removed!

            } while (cur_elem_is_removed);
            static typename M::key_type garbage;

            return !reached_end ? this->operator*() : garbage;
        }

        // The following are helper functions which are specialised (via type-deduction)
        // for the type of mesh elements that *this* iterator walks over in "mesh_ptr"
        // e.g. faces. These functions are used to determine when *this* iterator has
        // reached the end of the respective std::map data structure over which we are
        // iterating.

        template <typename T>
        struct identity {
            typedef T type;
        };

        template <typename I>
        I cend()
        {
            return cend(identity<I>()); // https://stackoverflow.com/questions/3052579/explicit-specialization-in-non-namespace-scope
        }

        static std::ptrdiff_t distance(const key_iterator_t<M>& beg, const key_iterator_t<M>& end)
        {
            key_iterator_t<M> it = beg;
            typename std::ptrdiff_t dist = 0;
            while (it != end) {
                dist++;
                ++it;
            }
            return dist;
        }

    private:
        template <typename I>
        I cend(identity<I>)
        {
            return I(); // unused
        }

        key_iterator_t<face_map_t> cend(identity<key_iterator_t<face_map_t>>)
        {
            return mesh_ptr->faces_end();
        }

        
#if 0
        key_iterator_t<halfedge_map_t> cend(identity<key_iterator_t<halfedge_map_t>>)
        {
            return mesh_ptr->halfedges_end();
        }

        key_iterator_t<edge_map_t> cend(identity<key_iterator_t<edge_map_t>>)
        {
            return mesh_ptr->edges_end();
        }

        key_iterator_t<vertex_array_t> cend(identity<key_iterator_t<vertex_array_t>>)
        {
            return mesh_ptr->vertices_end();
        }
#endif
    };

    
#endif

    template <typename V>
    class array_iterator_t : public V::const_iterator {
        const mesh_t* const mesh_ptr;
        typedef typename V::value_type::descriptor_type element_descriptor_type;
        typename V::value_type* operator->() = delete;
    public:
        array_iterator_t()
            : V::const_iterator()
            , mesh_ptr(nullptr) {};
        array_iterator_t(typename V::const_iterator it_, const mesh_t* const mesh)
            : V::const_iterator(it_)
            , mesh_ptr(mesh)
        {
        }
#if 0
        array_iterator_t<V>(array_iterator_t<V>&& v) : V::const_iterator{std::move(v)} 
        {
        }

        array_iterator_t<V>& operator=(array_iterator_t<V>&& v)
        {
           V::const_iterator::operator=(std::move(v)); // Selects base class function
            return *this;
        }

        // copy assignment
        array_iterator_t<V>& operator=(const array_iterator_t<V>& other)
        {
            // Guard self assignment
            if (this == &other)
                return *this;
        
            *this = other;
            return *this;
        }
#endif
        typename V::value_type::descriptor_type operator*() // TODO: test this, I think its broken
        {
            size_t raw_index = std::distance(cbegin<>(false), (*this)); // account for remove elements too
            element_descriptor_type d = static_cast<element_descriptor_type>(raw_index);
            return d;
        }

        // postfix increment (i++)
        // increment pointer to the next valid element (i.e. we skip removed elements).
        array_iterator_t<V> operator++(int)
        {
            bool old_elem_is_removed = false;

            do {
                array_iterator_t<V> old_elem = this->operator++(0); // (i++)
                element_descriptor_type raw_descriptor = (element_descriptor_type)std::distance(cbegin<>(false), old_elem);
                old_elem_is_removed = mesh_ptr->is_removed(raw_descriptor);
                if (!old_elem_is_removed) {
                    return old_elem;
                }

                // keep iterating until the value returned by the (i++) operator returns a valid element
                // i.e. one that is not marked removed!
            } while ((*this) != cend<>() && old_elem_is_removed);

            return cend<>();
        }
#if 0
// move assignment
        array_iterator_t<V>& operator=(array_iterator_t<V>&& other)
        {
            // Guard self assignment
            if (this == &other)
                return *this; // delete[]/size=0 would also be ok
        
            *this = other; // loop
            return *this;
        }

        array_iterator_t<V>(array_iterator_t<V>&& o) noexcept :
           s(std::move(o.s)),       // explicit move of a member of class type
           k(std::exchange(o.k, 0)) // explicit move of a member of non-class type
    { }

        

        


        array_iterator_t<V>& operator=(array_iterator_t<V> other) noexcept // call copy or move constructor to construct other
        {
            *this = other;
            return *this;
        } 
#endif

    //array_iterator_t<V>(const array_iterator_t<V>&) = default;
    //array_iterator_t<V>(array_iterator_t<V>&&) = default;
    //array_iterator_t<V>& operator=(const array_iterator_t<V>&) = default;
   // array_iterator_t<V>& operator=(array_iterator_t<V>&&) = default;

        // prefix increment (++i)
        // increment pointer to the next valid element (i.e. we skip removed elements).
        array_iterator_t<V>& operator++()
        {
            bool cur_elem_is_removed = false;
            bool reached_end = false;
            do {
                V::const_iterator::operator++(); 
                //(*this) = array_iterator_t<V>(V::const_iterator::operator++(), mesh_ptr); // (++i)
                reached_end = (*this) == cend<>();
                cur_elem_is_removed = false;

                if (!reached_end) {
                    size_t raw_descriptor = std::distance(cbegin<array_iterator_t<V>>(false), (*this)); // O(1) ??
                    cur_elem_is_removed = mesh_ptr->is_removed((element_descriptor_type)raw_descriptor );
                    if (!cur_elem_is_removed) {
                        break;
                    }
                }

                // keep iterating until the value pointed to after the (++i) operator is a valid element
                // i.e. one that is not marked removed!

            } while (cur_elem_is_removed && !reached_end);

            return (*this);
        }

        // The following are helper functions which are specialised (via type-deduction)
        // for the type of mesh elements that *this* iterator walks over in "mesh_ptr"
        // e.g. faces. These functions are used to determine when *this* iterator has
        // reached the end of the respective std::map data structure over which we are
        // iterating.

        template <typename T>
        struct identity {
            typedef T type;
        };

        template <typename I = array_iterator_t<V>>
        I cend()
        {
            return cend(identity<I>()); // https://stackoverflow.com/questions/3052579/explicit-specialization-in-non-namespace-scope
        }

        template <typename I = array_iterator_t<V>>
        I cbegin(bool account_for_removed_elems)
        {
            return cbegin(account_for_removed_elems, identity<I>()); 
        }

#if 0
        static std::ptrdiff_t distance(const array_iterator_t<V>& beg, const array_iterator_t<V>& end, bool account_for_removed_elems)
        {
            array_iterator_t<V> it = beg;
            //typename std::ptrdiff_t dist = (&end) - (&beg);
            //while (it != end) {
            //    dist++;
            //    ++it;
            }
            return dist;
        }
#endif
    private:
        template <typename I = array_iterator_t<V>>
        I cend(identity<I>)
        {
            return I(); // unused
        }

        template <typename I = array_iterator_t<V>>
        I cbegin(bool account_for_removed_elems, identity<I>)
        {
            return I(); // unused
        }

#if 0
        array_iterator_t<face_map_t> cend(identity<array_iterator_t<face_map_t>>)
        {
            return mesh_ptr->faces_end();
        }

        array_iterator_t<edge_map_t> cend(identity<array_iterator_t<edge_map_t>>)
        {
            return mesh_ptr->edges_end();
        }

        array_iterator_t<halfedge_map_t> cend(identity<array_iterator_t<halfedge_map_t>>)
        {
            return mesh_ptr->halfedges_end();
        }
#endif

        array_iterator_t<vertex_array_t> cbegin(bool account_for_removed_elems, identity<array_iterator_t<vertex_array_t>> = {})
        {
            return mesh_ptr->vertices_begin(account_for_removed_elems);
        }

        array_iterator_t<vertex_array_t> cend(identity<array_iterator_t<vertex_array_t>>)
        {
            return mesh_ptr->vertices_end();
        }

        array_iterator_t<edge_array_t> cbegin(bool account_for_removed_elems, identity<array_iterator_t<edge_array_t>> = {})
        {
            return mesh_ptr->edges_begin(account_for_removed_elems);
        }

        array_iterator_t<edge_array_t> cend(identity<array_iterator_t<edge_array_t>>)
        {
            return mesh_ptr->edges_end();
        }

        array_iterator_t<halfedge_array_t> cbegin(bool account_for_removed_elems, identity<array_iterator_t<halfedge_array_t>> = {})
        {
            return mesh_ptr->halfedges_begin(account_for_removed_elems);
        }

        array_iterator_t<halfedge_array_t> cend(identity<array_iterator_t<halfedge_array_t>>)
        {
            return mesh_ptr->halfedges_end();
        }

        array_iterator_t<face_array_t> cbegin(bool account_for_removed_elems, identity<array_iterator_t<face_array_t>> = {})
        {
            return mesh_ptr->faces_begin(account_for_removed_elems);
        }

        array_iterator_t<face_array_t> cend(identity<array_iterator_t<face_array_t>>)
        {
            return mesh_ptr->faces_end();
        }
    };

    typedef array_iterator_t<vertex_array_t> vertex_iterator_t;
    typedef array_iterator_t<edge_array_t> edge_iterator_t;
    typedef array_iterator_t<halfedge_array_t> halfedge_iterator_t;
    typedef array_iterator_t<face_array_t> face_iterator_t;

    mesh_t();
    ~mesh_t();

    // static member functions
    // -----------------------

    static vertex_descriptor_t null_vertex();

    static halfedge_descriptor_t null_halfedge();

    static edge_descriptor_t null_edge();

    static face_descriptor_t null_face();

    // regular member functions
    // ------------------------

    int number_of_vertices() const;

    int number_of_edges() const;

    int number_of_halfedges() const;

    int number_of_faces() const;

    vertex_descriptor_t source(const halfedge_descriptor_t& h) const;

    vertex_descriptor_t target(const halfedge_descriptor_t& h) const;

    halfedge_descriptor_t opposite(const halfedge_descriptor_t& h) const;

    halfedge_descriptor_t prev(const halfedge_descriptor_t& h) const;

    halfedge_descriptor_t next(const halfedge_descriptor_t& h) const;

    void set_next(const halfedge_descriptor_t& h, const halfedge_descriptor_t& nxt);

    void set_previous(const halfedge_descriptor_t& h, const halfedge_descriptor_t& prev);

    edge_descriptor_t edge(const halfedge_descriptor_t& h) const;

    face_descriptor_t face(const halfedge_descriptor_t& h) const;

    vertex_descriptor_t vertex(const edge_descriptor_t e, const int v) const;

    bool is_border(const halfedge_descriptor_t h);

    bool is_border(const edge_descriptor_t e);

    halfedge_descriptor_t halfedge(const edge_descriptor_t e, const int i) const;

    // finds a halfedge between two vertices. Returns a default constructed halfedge descriptor, if source and target are not connected.
    halfedge_descriptor_t halfedge(const vertex_descriptor_t s, const vertex_descriptor_t t, bool strict_check = false) const;
    // finds an edge between two vertices. Returns a default constructed halfedge descriptor, if source and target are not connected.
    edge_descriptor_t edge(const vertex_descriptor_t s, const vertex_descriptor_t t, bool strict_check = false) const;

    vertex_descriptor_t add_vertex(const math::vec3& point);

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    vertex_descriptor_t add_vertex(const math::fast_vec3& point);
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

    vertex_descriptor_t add_vertex(const math::real_number_t& x, const math::real_number_t& y, const math::real_number_t& z);

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

#else
    vertex_descriptor_t add_vertex(const char* x, const char* y, const char* z);
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

    // adds an edges into the mesh data structure, creating incident halfedges, and returns the
    // halfedge whole target is "v1"
    halfedge_descriptor_t add_edge(const vertex_descriptor_t v0, const vertex_descriptor_t v1);

    face_descriptor_t add_face(const std::vector<vertex_descriptor_t>& vi);

    // also disassociates (not remove) any halfedges(s) and vertices incident to face
    void remove_face(const face_descriptor_t f)
    {
        MCUT_ASSERT(f != null_face());
        MCUT_ASSERT(std::find(m_faces_removed.cbegin(), m_faces_removed.cend(), f) == m_faces_removed.cend());

        face_data_t& fd = m_faces.at(f);

        std::vector<vertex_descriptor_t> face_vertices; // ... that are used by face

        // disassociate halfedges

        for (std::vector<halfedge_descriptor_t>::const_iterator it = fd.m_halfedges.cbegin(); it != fd.m_halfedges.cend(); ++it) {
            halfedge_data_t& hd = m_halfedges.at(*it);
            MCUT_ASSERT(hd.f != null_face());
            hd.f = null_face();

            // NOTE: "next" and "previous" are only meaningful when the halfedge is used by a
            // face. So we reset that information here since the halfedge is not longer used
            // by a face
            if (hd.n != null_halfedge()) { // disassociate "next"
                const halfedge_descriptor_t hn = hd.n;
                halfedge_data_t& hnd = m_halfedges.at(hn);
                MCUT_ASSERT(hnd.p == *it);
                hnd.p = null_halfedge();
                //
                hd.n = null_halfedge();
            }

            if (hd.p != null_halfedge()) { // disassociate "previous"
                const halfedge_descriptor_t hp = hd.p;
                halfedge_data_t& hpd = m_halfedges.at(hp);
                MCUT_ASSERT(hpd.n == *it);
                hpd.n = null_halfedge();
                //
                hd.p = null_halfedge();
            }

            face_vertices.push_back(hd.t);
        }

        // disassociate vertices

        // for each vertex used by face
        for (std::vector<vertex_descriptor_t>::const_iterator it = face_vertices.cbegin(); it != face_vertices.cend(); ++it) {
            vertex_descriptor_t face_vertex = *it;
            vertex_data_t& vd = m_vertices.at(face_vertex);

            std::vector<face_descriptor_t>::const_iterator fIter = std::find(vd.m_faces.cbegin(), vd.m_faces.cend(), f);

            if (fIter != vd.m_faces.cend()) {
                vd.m_faces.erase(fIter); // remove association
            }
        }

        m_faces_removed.push_back(f);
    }

    // also disassociates (not remove) the halfedges(s) and vertex incident to this halfedge
    void remove_halfedge(halfedge_descriptor_t h)
    {
        MCUT_ASSERT(h != null_halfedge());
        MCUT_ASSERT(std::find(m_halfedges_removed.cbegin(), m_halfedges_removed.cend(), h) == m_halfedges_removed.cend());

        halfedge_data_t& hd = m_halfedges.at(h);

        MCUT_ASSERT(hd.e == null_edge()); // there must not be an edge dependent on h if we are to remove h
        MCUT_ASSERT(hd.f == null_face()); // there must not be a face dependent on h if we are to remove h

        if (hd.n != null_halfedge()) { // disassociate next
            const halfedge_descriptor_t hn = hd.n;
            halfedge_data_t& hnd = m_halfedges.at(hn);
            MCUT_ASSERT(hnd.p == h);
            hnd.p = null_halfedge();
            //
            hd.n = null_halfedge();
        }

        if (hd.o != null_halfedge()) { // disassociate opposite
            const halfedge_descriptor_t ho = hd.o;
            halfedge_data_t& hod = m_halfedges.at(ho);
            MCUT_ASSERT(hod.o == h);
            hod.o = null_halfedge();
            //
            hd.o = null_halfedge();
        }

        if (hd.p != null_halfedge()) { // disassociate previous
            const halfedge_descriptor_t hp = hd.p;
            halfedge_data_t& hpd = m_halfedges.at(hp);
            MCUT_ASSERT(hpd.n == h);
            hpd.n = null_halfedge();
            //
            hd.p = null_halfedge();
        }

        MCUT_ASSERT(hd.t != null_vertex()); // every h has a target vertex which is effectively dependent on h

        // disassociate target vertex
        vertex_data_t& htd = m_vertices.at(hd.t);
        std::vector<halfedge_descriptor_t>::const_iterator hIter = std::find(htd.m_halfedges.cbegin(), htd.m_halfedges.cend(), h);

        MCUT_ASSERT(hIter != htd.m_halfedges.cend()); // because not yet removed h

        htd.m_halfedges.erase(hIter); // remove association

        m_halfedges_removed.push_back(h);
    }

    // also disassociates (not remove) any face(s) incident to edge via its halfedges, and also disassociates the halfedges
    void remove_edge(const edge_descriptor_t e, bool remove_halfedges = true)
    {
        MCUT_ASSERT(e != null_edge());
        MCUT_ASSERT(std::find(m_edges_removed.cbegin(), m_edges_removed.cend(), e) == m_edges_removed.cend());

        edge_data_t& ed = m_edges.at(e);
        std::vector<halfedge_descriptor_t> halfedges = { ed.h, opposite(ed.h) }; // both halfedges incident to edge must be disassociated

        for (std::vector<halfedge_descriptor_t>::const_iterator it = halfedges.cbegin(); it != halfedges.cend(); ++it) {
            const halfedge_descriptor_t h = *it;
            MCUT_ASSERT(h != null_halfedge());

            // disassociate halfedge
            halfedge_data_t& hd = m_halfedges.at(h);
            MCUT_ASSERT(hd.e == e);
            hd.e = null_edge();
            if (remove_halfedges) {
                remove_halfedge(h);
            }
        }

        ed.h = null_halfedge(); // we are removing the edge so every associated data element must be nullified

        m_edges_removed.push_back(e);
    }

    void remove_vertex(const vertex_descriptor_t v)
    {
        MCUT_ASSERT(v != null_vertex());
        MCUT_ASSERT((size_t)v < m_vertices.size());
        MCUT_ASSERT(std::find(m_vertices_removed.cbegin(), m_vertices_removed.cend(), v) == m_vertices_removed.cend());
        MCUT_ASSERT(m_vertices.at(v).m_faces.empty());
        MCUT_ASSERT(m_vertices.at(v).m_halfedges.empty());

        m_vertices_removed.push_back(v);
    }

    void remove_elements()
    {
        for (face_iterator_t i = faces_begin(); i != faces_end(); ++i) {
            remove_face(*i);
        } 

        for (edge_iterator_t i = edges_begin(); i != edges_end(); ++i) {
            remove_edge(*i);
        }

        for (halfedge_iterator_t i = halfedges_begin(); i != halfedges_end(); ++i) {
            remove_halfedge(*i);
        }

        for (vertex_iterator_t i = vertices_begin(); i != vertices_end(); ++i) {
            remove_vertex(*i);
        }
    }

    int number_of_internal_faces() const
    {
        return static_cast<int>(m_faces.size());
    }

    int number_of_internal_edges() const
    {
        return static_cast<int>(m_edges.size());
    }

    int number_of_internal_halfedges() const
    {
        return static_cast<int>(m_halfedges.size());
    }

    int number_of_internal_vertices() const
    {
        return static_cast<int>(m_vertices.size());
    }

    //
    int number_of_vertices_removed() const
    {
        return (int)this->m_vertices_removed.size();
    }

    int number_of_edges_removed() const
    {
        return (int)this->m_edges_removed.size();
    }

    int number_of_halfedges_removed() const
    {
        return (int)this->m_halfedges_removed.size();
    }

    int number_of_faces_removed() const
    {
        return (int)this->m_faces_removed.size();
    }

    bool is_removed(face_descriptor_t f) const
    {
        return std::find(m_faces_removed.cbegin(), m_faces_removed.cend(), f) != m_faces_removed.cend();
    }

    bool is_removed(edge_descriptor_t e) const
    {
        return std::find(m_edges_removed.cbegin(), m_edges_removed.cend(), e) != m_edges_removed.cend();
    }

    bool is_removed(halfedge_descriptor_t h) const
    {
        return std::find(m_halfedges_removed.cbegin(), m_halfedges_removed.cend(), h) != m_halfedges_removed.cend();
    }

    bool is_removed(vertex_descriptor_t v) const
    {
        return std::find(m_vertices_removed.cbegin(), m_vertices_removed.cend(), v) != m_vertices_removed.cend();
    }

    const math::vec3& vertex(const vertex_descriptor_t& vd) const;

    // returns vector of halfedges which point to vertex (i.e. "v" is their target)
    const std::vector<halfedge_descriptor_t>& get_halfedges_around_vertex(const vertex_descriptor_t v) const;

    std::vector<vertex_descriptor_t> get_vertices_around_face(const face_descriptor_t f) const;
    std::vector<vertex_descriptor_t> get_vertices_around_vertex(const vertex_descriptor_t v) const;

    const std::vector<halfedge_descriptor_t>& get_halfedges_around_face(const face_descriptor_t f) const;

    const std::vector<face_descriptor_t> get_faces_around_face(const face_descriptor_t f) const;

    // iterators
    // ---------

    vertex_iterator_t vertices_begin(bool account_for_removed_elems = true) const;

    vertex_iterator_t vertices_end() const;

    edge_iterator_t edges_begin(bool account_for_removed_elems = true) const;

    edge_iterator_t edges_end() const;

    halfedge_iterator_t halfedges_begin(bool account_for_removed_elems = true) const;

    halfedge_iterator_t halfedges_end() const;

    face_iterator_t faces_begin(bool account_for_removed_elems = true) const;

    face_iterator_t faces_end() const;

private:
    // member variables
    // ----------------

    std::vector<vertex_data_t> m_vertices;
    std::vector<edge_data_t> m_edges;
    std::vector<halfedge_data_t> m_halfedges;
    std::vector<face_data_t> m_faces;

    // NOTE: I use std::vector because we'll have very few (typically zero)
    // elements removed at a given time. In fact removal only happens during
    // input-mesh face-partitioning to resolve floating polygons, which is
    // rare. Maybe in the future thing change...
    std::vector<face_descriptor_t> m_faces_removed;
    std::vector<edge_descriptor_t> m_edges_removed;
    std::vector<halfedge_descriptor_t> m_halfedges_removed;
    std::vector<vertex_descriptor_t> m_vertices_removed;

}; // class mesh_t {

typedef vertex_descriptor_t vd_t;
typedef halfedge_descriptor_t hd_t;
typedef edge_descriptor_t ed_t;
typedef face_descriptor_t fd_t;

void write_off(const char* fpath, const mcut::mesh_t& mesh);
void read_off(mcut::mesh_t& mesh, const char* fpath);

} // namespace mcut

#endif // #ifndef MCUT_HALFEDGE_MESH_H_
