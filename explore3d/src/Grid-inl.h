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

#ifndef Grid_inl_h
#define Grid_inl_h

#include "Grid.h"

#include <sstream>

template<class T> typename std::add_rvalue_reference<T>::type val();
template <class T> struct id { typedef T type; };

template <class T, class... P> struct mul_type;

// single-type base case
template <class T> struct mul_type<T> : id<T> {};

template<class T, class U, class... P>
struct mul_type<T,U,P...> : id<decltype(val<T>() * val<typename mul_type<U,P...>::type>())> {};

template <typename A, typename B>
typename mul_type<A, B>::type mul(A a, B b)
{
    return a * b;
}

template <typename A0, typename A1, typename... sizes>
typename mul_type<A0, A1, sizes...>::type mul(A0 a0, A1 a1, sizes... dims)
{
   return a0 * mul(a1, dims...);
}

namespace au
{

template <int N, typename T>
Grid<N, T>::Grid() :
    data_(nullptr)
{
    this->clear();
}

template <int N, typename T>
template <typename... sizes>
Grid<N, T>::Grid(sizes... counts) :
    data_(nullptr)
{
    static_assert(sizeof...(counts) == N, "Invalid dimensions passed to Grid constructor");
    this->resize(counts...);
}

template <int N, typename T>
Grid<N, T>::Grid(const Grid& other) :
    data_(nullptr)
{
    this->copy(other);
}

template <int N, typename T>
Grid<N, T>& Grid<N, T>::operator=(const Grid& rhs)
{
    if (this != &rhs) {
        this->copy(rhs);
    }
    return *this;
}

template <int N, typename T>
Grid<N, T>::~Grid()
{
    this->clear();
}

template <int N, typename T>
template <typename... CoordTypes>
typename Grid<N, T>::reference Grid<N, T>::operator()(CoordTypes... coords)
{
    static_assert(sizeof...(coords) == N, "Invalid number of coordinates passed to operator()");
    size_type ind = this->coord_to_index(coords...);
    return data_[ind];
}

template <int N, typename T>
template <typename... CoordTypes>
typename Grid<N, T>::const_reference Grid<N, T>::operator()(CoordTypes... coords) const
{
    return const_cast<const T&>(const_cast<Grid<N, T>*>(this)->operator()(coords...));
}

template <int N, typename T>
template <typename... CoordTypes>
typename Grid<N, T>::reference Grid<N, T>::at(CoordTypes... coords)
{
    size_type ind = this->coord_to_index(coords...);
    if (!(ind < this->total_size())) {
        std::stringstream ss;
        ss << "invalid index " << ind << " into array of size " << this->total_size();
        throw std::out_of_range(ss.str());
    }
    return data_[ind];
}

template <int N, typename T>
template <typename... CoordTypes>
typename Grid<N, T>::const_reference Grid<N, T>::at(CoordTypes... coords) const
{
    return const_cast<const T&>(const_cast<Grid<N, T>*>(this)->at(coords...));
}

template <int N, typename T>
typename Grid<N, T>::iterator Grid<N, T>::grid_begin()
{
    return 0;
}

template <int N, typename T>
typename Grid<N, T>::const_iterator Grid<N, T>::grid_begin() const
{
    return 0;
}

template <int N, typename T>
typename Grid<N, T>::iterator Grid<N, T>::grid_end()
{
    return 0;
}

template <int N, typename T>
typename Grid<N, T>::const_iterator Grid<N, T>::grid_end() const
{
    return 0;
}

template <int N, typename T>
typename Grid<N, T>::size_type Grid<N, T>::size(size_type dim) const
{
    return dims_[dim];
}

template <int N, typename T>
typename Grid<N, T>::size_type Grid<N, T>::total_size() const
{
    size_type s = 1;
    for (int i = 0; i < N; ++i) { s *= dims_[i]; }
    return s;
}

template <int N, typename T>
void Grid<N, T>::clear()
{
    if (data_) {
        delete[] data_;
        data_ = nullptr;
    }

    for (int i = 0; i < N; ++i) {
        dims_[i] = 0;
    }
}

template <int N, typename T>
template <typename... SizeTypes>
void Grid<N, T>::resize(SizeTypes... sizes)
{
    static_assert(sizeof...(sizes) == N, "resize requires same number of arguments as dimension");

    this->clear();
    size_type total_size = mul(sizes...);
    data_ = new T[total_size];
    this->set_sizes<0>(sizes...);
}

template <int N, typename T>
void Grid<N, T>::assign(const T& value)
{
    for (size_type i = 0; i < this->total_size(); ++i) {
        data_[i] = value;
    }
}

template <int N, typename T>
void Grid<N, T>::copy(const Grid& other)
{
    this->clear();

    if (other.data_) {
        size_type other_size = other.total_size();
        data_ = new T[other_size];
        for (size_type i = 0; i < other_size; ++i) {
            data_[i] = other.data_[i];
        }
        for (int i = 0; i < N; ++i) {
            dims_[i] = other.dims_[i];
        }
    }
}

template <int N, typename T>
template <typename... CoordTypes>
typename Grid<N, T>::size_type Grid<N, T>::coord_to_index(CoordTypes... coords)
{
    size_type s = 1;
    return this->coord_to_index_rec<0>(s, coords...);
}

template <int N, typename T>
template <int DIM, typename... CoordTypes, typename Coord>
typename Grid<N, T>::size_type Grid<N, T>::coord_to_index_rec(size_type& agg, Coord coord, CoordTypes... coords)
{
    size_type s = this->coord_to_index_rec<DIM+1>(agg, coords...);
    agg *= dims_[DIM + 1];
    return agg * coord + s;
}

template <int N, typename T>
template <int DIM, typename Coord>
typename Grid<N, T>::size_type Grid<N, T>::coord_to_index_rec(size_type& agg, Coord coord)
{
    static_assert(DIM == N - 1, "Something is wrong");
    return agg * coord;
}

} // namespace au

#endif
