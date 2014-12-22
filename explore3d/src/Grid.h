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

    typedef std::size_t size_type;
    typedef T value_type;
    typedef T& reference;
    typedef const T& const_reference;

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
    T* data() { return data_; }
    /// @}

    /// Iterators
    /// @{
    typedef int iterator;
    typedef int const_iterator;
    iterator grid_begin();
    const_iterator grid_begin() const;
    iterator grid_end();
    const_iterator grid_end() const;
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

    template <typename... CoordTypes>
    size_type coord_to_index(CoordTypes... coords);

    template <int DIM, typename... CoordTypes, typename Coord>
    size_type coord_to_index_rec(size_type& agg, Coord coord, CoordTypes... coords);

    template <int DIM, typename Coord>
    size_type coord_to_index_rec(size_type& agg, Coord coord);
};

} // namespace au

#endif

#include "Grid-inl.h"
