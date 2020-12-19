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

#include <limits>
#include <map>
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

class mesh_t {

public:
    struct halfedge_data_t {
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

    struct edge_data_t {
        halfedge_descriptor_t h; // primary halfedge (even idx)
        edge_data_t()
            : h(null_halfedge())
        {
        }
    };

    struct face_data_t {
        std::vector<halfedge_descriptor_t> m_halfedges;
    };

    struct vertex_data_t {
        mcut::math::vec3 p; // geometry coordinates
        std::vector<face_descriptor_t> m_faces; // ... incident to vertex
        std::vector<halfedge_descriptor_t> m_halfedges; // ... which point to vertex (note: can be used to infer edges too)
    };

    typedef std::map<vertex_descriptor_t, vertex_data_t> vertex_map_t;
    typedef std::map<edge_descriptor_t, edge_data_t> edge_map_t;
    typedef std::map<halfedge_descriptor_t, halfedge_data_t> halfedge_map_t;
    typedef std::map<face_descriptor_t, face_data_t> face_map_t;

    // iterator over the keys of a map (base class)
    template <typename M>
    class key_iterator_t : public M::const_iterator {

    public:
        key_iterator_t()
            : M::const_iterator() {};
        key_iterator_t(typename M::const_iterator it_)
            : M::const_iterator(it_) {};

        typename M::key_type* operator->()
        {
            return (typename M::key_type* const) & (M::const_iterator::operator->()->first);
        }

        typename M::key_type operator*()
        {
            return M::const_iterator::operator*().first;
        }
    };

    typedef key_iterator_t<vertex_map_t> vertex_iterator_t;
    typedef key_iterator_t<edge_map_t> edge_iterator_t;
    typedef key_iterator_t<halfedge_map_t> halfedge_iterator_t;
    typedef key_iterator_t<face_map_t> face_iterator_t;

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

    vertex_descriptor_t add_vertex(const math::vec3& point);

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    vertex_descriptor_t add_vertex(const math::fast_vec3& point);
#endif // #if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

    vertex_descriptor_t add_vertex(const math::real_number_t& x, const math::real_number_t& y, const math::real_number_t& z);

#if defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)

#else
    vertex_descriptor_t add_vertex(const char* x, const char* y, const char* z);
#endif // #if !defined(MCUT_WITH_ARBITRARY_PRECISION_NUMBERS)
    halfedge_descriptor_t add_edge(const vertex_descriptor_t v0, const vertex_descriptor_t v1);

    face_descriptor_t add_face(const std::vector<vertex_descriptor_t>& vi);

    const math::vec3& vertex(const vertex_descriptor_t& vd) const;

    std::vector<vertex_descriptor_t> get_vertices_around_face(const face_descriptor_t f) const;

    const std::vector<halfedge_descriptor_t>& get_halfedges_around_face(const face_descriptor_t f) const;

    const std::vector<face_descriptor_t> get_faces_around_face(const face_descriptor_t f) const;

    // iterators
    // ---------

    vertex_iterator_t vertices_begin() const;

    vertex_iterator_t vertices_end() const;

    edge_iterator_t edges_begin() const;

    edge_iterator_t edges_end() const;

    halfedge_iterator_t halfedges_begin() const;

    halfedge_iterator_t halfedges_end() const;

    face_iterator_t faces_begin() const;

    face_iterator_t faces_end() const;

private:
    // member variables
    // ----------------

    std::map<vertex_descriptor_t, vertex_data_t> m_vertices;
    std::map<edge_descriptor_t, edge_data_t> m_edges;
    std::map<halfedge_descriptor_t, halfedge_data_t> m_halfedges;
    std::map<face_descriptor_t, face_data_t> m_faces;
}; // class mesh_t {

typedef vertex_descriptor_t vd_t;
typedef halfedge_descriptor_t hd_t;
typedef edge_descriptor_t ed_t;
typedef face_descriptor_t fd_t;

void write_off(const char* fpath, const mcut::mesh_t& mesh);
void read_off(mcut::mesh_t& mesh, const char* fpath);

} // namespace mcut

#endif // #ifndef MCUT_HALFEDGE_MESH_H_
