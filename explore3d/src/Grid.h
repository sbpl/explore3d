////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014, Andrew Dornbush All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#ifndef Grid_h
#define Grid_h

#include <assert.h>
#include <stdlib.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <type_traits>
#include <vector>

namespace au
{

template <int N, typename T>
class Grid
{
public:

    class GridIterator;
    class GridIndex;

    typedef std::size_t size_type;
    typedef T value_type;
    typedef T& reference;
    typedef const T& const_reference;
    typedef GridIterator iterator;
    typedef const GridIterator const_iterator;

    Grid();

    template <typename... sizes>
    Grid(sizes... counts);

    Grid(const Grid& other);
    Grid& operator=(const Grid& rhs);

    ~Grid();

    /// Element access
    /// @{
    template <typename... CoordTypes> reference operator()(CoordTypes... coords);
    template <typename... CoordTypes> const_reference operator()(CoordTypes... coords) const;
    template <typename... CoordTypes> reference at(CoordTypes... coords);
    template <typename... CoordTypes> const_reference at(CoordTypes... coords) const;

    template <typename... CoordTypes> reference operator()(const GridIndex& index);
    template <typename... CoordTypes> const_reference operator()(const GridIndex& index) const;
    template <typename... CoordTypes> reference at(const GridIndex& index);
    template <typename... CoordTypes> const_reference at(const GridIndex& index) const;

    T* data() { return data_; }
    /// @}

    /// Iterators
    /// @{
    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    iterator grid_begin(const GridIndex& start, const GridIndex& end);
    const_iterator grid_begin(const GridIndex& start, const GridIndex& end) const;
    iterator grid_end(const GridIndex& start, const GridIndex& end);
    const_iterator grid_end(const GridIndex& start, const GridIndex& end) const;
    /// @}

    /// Capacity
    /// @{
    size_type size(size_type dim) const;
    size_type total_size() const;
    /// @}

    /// Modifiers
    /// @{
    void clear();
    template <typename... SizeTypes> void resize(SizeTypes... sizes);
    void assign(const T& value);
    /// @}

    class GridIndex
    {
    public:

        GridIndex();

        template <typename... CoordTypes>
        GridIndex(CoordTypes... coords);

        size_type& operator()(size_type dim) { return coords_[dim]; }
        const size_type& operator()(size_type dim) const { return coords_[dim]; }

    private:

        size_type coords_[N];

        template <size_type DIM, typename SizeType, typename... SizeTypes>
        void set_coords(SizeType size, SizeTypes... sizes)
        {
            static_assert(DIM < N-1, "Invalid dimension passed to set_coords");
            coords_[DIM] = size;
            this->set_coords<DIM+1>(sizes...);
        }

        template <size_type DIM, typename SizeType>
        void set_coords(SizeType size)
        {
            static_assert(DIM == N-1, "Invalid number of sizes passed to set_coords");
            coords_[N-1] = size;
        }

        void assign_all(size_type coord);
    };

    class GridIterator
    {
        friend class Grid;

    public:

        typedef typename Grid<N, T>::size_type size_type;
        typedef typename Grid<N, T>::value_type value_type;

        GridIterator();

        GridIterator(const GridIterator& other);
        GridIterator& operator=(const GridIterator& rhs);

        /// Iterator API
        /// @{
        GridIterator& operator++();
        GridIterator operator++(int);

        bool operator==(const GridIterator& other) const;
        bool operator!=(const GridIterator& other) const;

        value_type& operator*();
        value_type* operator->();

        GridIterator& operator--();
        GridIterator operator--(int);
        /// @}

        size_type coord(size_type dim) const { return curr_(dim); }

    private:

        Grid<N, T>* grid_;
        GridIndex begin_;
        GridIndex end_;
        GridIndex curr_;

        size_type size(size_type dim) const { return end_(dim) - begin_(dim) + 1; }
    };

private:

    value_type* data_;
    size_type   dims_[N];

    void copy(const Grid& other);

    template <size_type DIM, typename SizeType, typename... SizeTypes> // TODO: how to declare DIM in a separate implementation?
    void set_sizes(SizeType size, SizeTypes... sizes)
    {
        static_assert(DIM < N-1, "Invalid dimension passed to set_sizes");
        dims_[DIM] = size;
        this->set_sizes<DIM+1>(sizes...);
    }

    template <size_type DIM, typename SizeType>
    void set_sizes(SizeType size)
    {
        static_assert(DIM == N-1, "Invalid number of sizes passed to set_sizes");
        dims_[N-1] = size;
    }

    size_type coord_to_index(const GridIndex& index);

    template <typename... CoordTypes>
    size_type coord_to_index(CoordTypes... coords);

    template <int DIM, typename... CoordTypes, typename Coord>
    size_type coord_to_index_rec(size_type& agg, Coord coord, CoordTypes... coords);

    template <int DIM, typename Coord>
    size_type coord_to_index_rec(size_type& agg, Coord coord);

    template <int DIM>
    void assign_sizes(GridIndex& index);

    GridIndex create_last_index() const;
};

} // namespace au

#endif

#include "Grid-inl.h"
