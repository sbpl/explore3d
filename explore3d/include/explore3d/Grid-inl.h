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

#include <iostream>
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

////////////////////////////////////////////////////////////////////////////////
// Grid Implementation
////////////////////////////////////////////////////////////////////////////////

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
template <typename... CoordTypes>
auto Grid<N, T>::operator()(const GridIndex& index) -> reference
{
    size_type ind = this->coord_to_index(index);
    return data_[ind];
}

template <int N, typename T>
template <typename... CoordTypes>
auto Grid<N, T>::operator()(const GridIndex& index) const -> const_reference
{
    return const_cast<const T&>(const_cast<Grid<N, T>*>(this)->operator()(index));
}

template <int N, typename T>
template <typename... CoordTypes>
auto Grid<N, T>::at(const GridIndex& index) -> reference
{
    size_type ind = this->coord_to_index(index);
    if (!(ind < this->total_size())) {
        std::stringstream ss;
        ss << "invalid index " << ind << " into array of size " << this->total_size();
        throw std::out_of_range(ss.str());
    }
    return data_[ind];
}

template <int N, typename T>
template <typename... CoordTypes>
auto Grid<N, T>::at(const GridIndex& index) const -> const_reference
{
    return const_cast<const T&>(const_cast<Grid<N, T>*>(this)->at(index));
}

template <int N, typename T>
typename Grid<N, T>::iterator Grid<N, T>::begin()
{
    GridIterator it;
    it.grid_ = this;
    it.end_ = this->create_last_index();
    return it;
}

template <int N, typename T>
typename Grid<N, T>::iterator Grid<N, T>::end()
{
    GridIterator it;
    it.grid_ = this;
    it.end_ = this->create_last_index();
    it.curr_ = this->create_last_index();
    ++it.curr_(N-1); // increment the last coordinate by 1 to push it past the end
    return it;
}

template <int N, typename T>
typename Grid<N, T>::const_iterator Grid<N, T>::begin() const
{
    return const_cast<typename Grid<N, T>::const_iterator>(const_cast<Grid<N, T>*>(this)->begin());
}

template <int N, typename T>
typename Grid<N, T>::const_iterator Grid<N, T>::end() const
{
    return const_cast<typename Grid<N, T>::const_iterator>(const_cast<Grid<N, T>*>(this)->end());
}

template <int N, typename T>
typename Grid<N, T>::iterator Grid<N, T>::grid_begin(const GridIndex& begin, const GridIndex& end)
{
    GridIterator it;
    it.grid_ = this;
    it.begin_ = begin;
    it.end_ = end;
    it.curr_ = begin;
    return it;
}

template <int N, typename T>
typename Grid<N, T>::const_iterator Grid<N, T>::grid_begin(const GridIndex& begin, const GridIndex& end) const
{
    return const_cast<typename Grid<N, T>::const_iterator>(const_cast<Grid<N, T>*>(this)->grid_begin(begin, end));
}

template <int N, typename T>
typename Grid<N, T>::iterator Grid<N, T>::grid_end(const GridIndex& begin, const GridIndex& end)
{
    GridIterator it;
    it.grid_ = this;
    it.begin_ = begin;
    it.end_ = end;
    it.curr_ = end;
    ++it.curr_(N-1);
    return it;
}

template <int N, typename T>
typename Grid<N, T>::const_iterator Grid<N, T>::grid_end(const GridIndex& begin, const GridIndex& end) const
{
    return const_cast<typename Grid<N, T>::const_iterator>(const_cast<Grid<N, T>*>(this)->grid_end(begin, end));
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
    static_assert(sizeof...(sizes) == N, "resize requires same number of arguments as dimensions");

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
struct IndexCoordinatorBase
{
    typedef Grid<N, T> grid_type;
    typedef typename grid_type::size_type size_type;
    typedef typename grid_type::GridIndex index_type;

    IndexCoordinatorBase(size_type& agg) : agg_(agg) { }
    size_type& agg_;
};

template <int N, typename T, int DIM>
struct IndexCoordinator;

template <int N, typename T, int DIM>
struct IndexCoordinator : public IndexCoordinatorBase<N, T>
{
    typedef IndexCoordinatorBase<N, T> base;
    typedef typename base::grid_type grid_type;
    typedef typename base::size_type size_type;
    typedef typename base::index_type index_type;

    IndexCoordinator(size_type& agg) : base(agg) { }

    size_type operator()(const grid_type& grid, const index_type& index)
    {
        size_type s = IndexCoordinator<N, T, DIM+1>(base::agg_)(grid, index);
        if (DIM + 1 != N) {
            base::agg_ *= grid.size(DIM + 1);
        }
        return base::agg_ * index(DIM) + s;
    }
};

template <int N, typename T>
struct IndexCoordinator<N, T, N> : public IndexCoordinatorBase<N, T>
{
    typedef IndexCoordinatorBase<N, T> base;
    typedef typename base::grid_type grid_type;
    typedef typename base::size_type size_type;
    typedef typename base::index_type index_type;

    IndexCoordinator(size_type& agg) : base(agg) { }

    size_type operator()(const grid_type& grid, const index_type& index)
    {
        return 0;
    }
};

template <int N, typename T>
typename Grid<N, T>::size_type Grid<N, T>::coord_to_index(const GridIndex& index)
{
    std::size_t agg = 1;
    return IndexCoordinator<N, T, 0>(agg)(*this, index);
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

template <int N, typename T, int DIM>
struct IndexSizeFiller
{
    void operator()(const Grid<N, T>& grid, typename Grid<N, T>::GridIndex& index)
    {
        index(DIM) = grid.size(DIM) - 1;
        IndexSizeFiller<N, T, DIM+1>()(grid, index);
    }
};

template <int N, typename T>
struct IndexSizeFiller<N, T, N>
{
    void operator()(const Grid<N, T>& grid, typename Grid<N, T>::GridIndex& index)
    {
    }
};

template <int N, typename T>
auto Grid<N, T>::create_last_index() const -> typename Grid<N, T>::GridIndex
{
    GridIndex ind;
    IndexSizeFiller<N, T, 0>()(*this, ind);
    return ind;
}

////////////////////////////////////////////////////////////////////////////////
// GridIndex Implementation
////////////////////////////////////////////////////////////////////////////////

template <int N, typename T, int DIM>
struct IndexAssigner
{
    void operator()(typename Grid<N, T>::GridIndex& ind, typename Grid<N, T>::size_type coord)
    {
        ind(DIM) = coord;
        IndexAssigner<N, T, DIM+1>()(ind, coord);
    }
};

template <int N, typename T>
struct IndexAssigner<N, T, N>
{
    void operator()(typename Grid<N, T>::GridIndex& ind, typename Grid<N, T>::size_type coord)
    {
    }
};

template <int N, typename T>
Grid<N, T>::GridIndex::GridIndex()
{
    this->assign_all(0);
}

template <int N, typename T>
template <typename... Coords>
Grid<N, T>::GridIndex::GridIndex(Coords... coords)
{
    static_assert(sizeof...(coords) == N, "GridIndex constructor requires same number of arguments as dimensions");
    this->set_coords<0>(coords...);
}

template <int N, typename T>
void Grid<N, T>::GridIndex::assign_all(size_type coord)
{
    IndexAssigner<N, T, 0>()(*this, coord);
}

////////////////////////////////////////////////////////////////////////////////
// GridIterator Implementation
////////////////////////////////////////////////////////////////////////////////

template <int N, typename T>
Grid<N, T>::GridIterator::GridIterator() :
    grid_(nullptr),
    begin_(),
    end_(),
    curr_()
{
}

template <int N, typename T>
Grid<N, T>::GridIterator::GridIterator(const GridIterator& other) :
    grid_(other.grid_),
    begin_(other.begin_),
    end_(other.end_),
    curr_(other.curr_)
{
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator=(const GridIterator& rhs) -> GridIterator&
{
    if (this != &rhs) {
        grid_ = rhs.grid_;
        for (int i = 0; i < N; ++i) {
            begin_(i) = rhs.begin_(i);
            end_(i) = rhs.end_(i);
            curr_(i) = rhs.curr_(i);
        }
    }
    return *this;
}

template <int N, typename T>
std::string to_string(const typename Grid<N, T>::GridIndex& index)
{
    std::stringstream ss;
    ss << "( ";
    for (int i = 0; i < N; ++i) {
        ss << index(i) << ' ';
    }
    ss << ')';
    return ss.str();
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator++() -> GridIterator&
{
    for (int i = N - 1; i >= 0; --i) {
        if (curr_(i) < this->size(i) - 1) {
            ++curr_(i);
            return *this;
        }
        else if (i == 0) {
            // special case for the first coordinate; push "one past the end"
            ++curr_(N-1);
        }
        else if (curr_(i - 1) < this->size(i - 1) - 1) { // increment the above counter
            ++curr_(i - 1);
            for (int j = i; j < N; ++j) {
                curr_(j) = 0;
            }
            return *this;
        }
    }
    return *this;
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator++(int) -> GridIterator
{
    GridIterator git(*this);
    this->operator++();
    return git;
}

template <int N, typename T>
bool Grid<N, T>::GridIterator::operator==(const GridIterator& other) const
{
    auto index_equals = [](const GridIndex& gi, const GridIndex& gj) -> bool
    {
        for (int i = 0; i < N; ++i) {
            if (gi(i) != gj(i)) {
                return false;
            }
        }
        return true;
    };

    return index_equals(curr_, other.curr_) &&
           grid_ == other.grid_ &&
           index_equals(begin_, other.begin_) &&
           index_equals(end_, other.end_);
}

template <int N, typename T>
bool Grid<N, T>::GridIterator::operator!=(const GridIterator& other) const
{
    return !(this->operator==(other));
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator*() -> value_type&
{
    return grid_->at(curr_);
//    return grid_->operator()(curr_);
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator->() -> value_type*
{
    return grid_->at(curr_);
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator--() -> GridIterator&
{
    // TODO: implement
    return *this;
}

template <int N, typename T>
auto Grid<N, T>::GridIterator::operator--(int) -> GridIterator
{
    GridIterator git(*this);
    this->operator--();
    return git;
}

} // namespace au

#endif
