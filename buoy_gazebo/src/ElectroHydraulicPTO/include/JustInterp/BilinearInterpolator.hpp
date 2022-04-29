// Copyright 2021 alexander-valov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Skip uncrustify linting
/* *INDENT-OFF* */

#pragma once

#include <vector>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <cassert>

#include "JustInterp/Utils.hpp"
#include "JustInterp/LinearInterpolator.hpp"

namespace JustInterp {

template<class Real, int StorageOrder_ = YMajor>
class BilinearInterpolator {

public:

    /********************************************************************
     * Constructors
     *********************************************************************/
    BilinearInterpolator() = default;
    template<class Index>
    BilinearInterpolator(Index nx, Index ny, const Real* x_1d, const Real* y_1d, Real* z_all) { SetData(nx, ny, x_1d, y_1d, z_all); }
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    BilinearInterpolator(const Container& x_1d, const Container& y_1d, const Container& z_all) { SetData(x_1d, y_1d, z_all); }

    /********************************************************************
     * @brief Set known grid points and values.
     * @param[in] nx Size of grid points along x-axis
     * @param[in] ny Size of grid points along y-axis
     * @param[in] x_1d Grid points along x-axis
     * @param[in] y_1d Grid points along y-axis
     * @param[in] z_all All node values stored as 1d array
     *********************************************************************/
    template<class Index>
    void SetData(Index nx, Index ny, const Real* x_1d, const Real* y_1d, const Real* z_all) {
        assert(((nx > 0 && ny > 0) && "Empty data for interpolation: nx or ny equals zero"));
        x_1d_.clear();
        y_1d_.clear();
        z_all_.clear();
        std::copy(x_1d, x_1d + nx, std::back_inserter(x_1d_));
        std::copy(y_1d, y_1d + ny, std::back_inserter(y_1d_));
        std::copy(z_all, z_all + nx * ny, std::back_inserter(z_all_));
    }

    /********************************************************************
     * @brief Set known grid points and values.
     * 
     * Container must have real-type values and the following methods:
     * data(), size(), begin(), end()
     * 
     * @param[in] x_1d Grid points along x-axis
     * @param[in] y_1d Grid points along y-axis
     * @param[in] z_all All node values stored as 1d array
     *********************************************************************/
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    void SetData(const Container& x_1d, const Container& y_1d, const Container& z_all) {
        assert((x_1d.size() * y_1d.size() == z_all.size() && "Sizes of whole grid points and node values are mismatch"));
        SetData(x_1d.size(), y_1d.size(), x_1d.data(), y_1d.data(), z_all.data());
    }

    /********************************************************************
     * Access
     *********************************************************************/
    const std::vector<Real>& GetX1D() const { return x_1d_; }
    const std::vector<Real>& GetY1D() const { return y_1d_; }
    const std::vector<Real>& GetZAll() const { return z_all_; }

    /********************************************************************
     * Calculate interpolation function at the given point.
     * If point is outside the data extrapolation is performed.
     * @param[in] x X-coordinate
     * @param[in] y Y-coordinate
     * @return Interpolated value
     *********************************************************************/
    Real operator()(const Real& x, const Real& y) const {
        Real result;
        if (x <= x_1d_.front() || x >= x_1d_.back() || y <= y_1d_.front() || y >= y_1d_.back()) {
            result = Extrapolate(x, y);
        } else {
            /* Find nearest indices to the x and y that is less than x and y */
            auto lower_x = std::lower_bound(x_1d_.begin(), x_1d_.end(), x);
            std::size_t ix = std::distance(x_1d_.begin(), lower_x) - 1;
            auto lower_y = std::lower_bound(y_1d_.begin(), y_1d_.end(), y);
            std::size_t iy = std::distance(y_1d_.begin(), lower_y) - 1;

            result = InterpolateRect(x, y, ix, iy);
        }
        return result;
    }

    /********************************************************************
     * Calculate interpolation function at each given points.
     * If some points are outside the data extrapolation is performed.
     * 
     * Container must have real-type values and the following methods:
     * data(), size(), begin(), end()
     * 
     * @param[in] x Array of x-coordinate. x.size() must be equals y.size()
     * @param[in] y Array of y-coordinate. x.size() must be equals y.size()
     * @return std::vector<Real> of interpolated values
     *********************************************************************/
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    std::vector<Real> operator()(const Container& x, const Container& y) const {
        assert((x.size() == y.size() && "Sizes of x and y are mismatch"));
        std::vector<Real> result;
        result.reserve(x.size());
        for (std::size_t i = 0; i < x.size(); i++) {
            Real x_i = *(x.begin() + i);
            Real y_i = *(y.begin() + i);
            result.push_back(this->operator()(x_i, y_i));
        }
        return result;
    }

    /********************************************************************
     * Calculate interpolation function for 2D grid. 2D grid is defined 
     * by two 1D arrays of coordinates along the x-axis and y-axis respectively. 
     * 1D result array is stored according to StorageOrder_.
     * 
     * Container must have real-type values and the following methods:
     * data(), size(), begin(), end()
     * 
     * @param[in] x_1d Grid coordinates along x-axis
     * @param[in] y_1d Grid coordinates along y-axis
     * @return std::vector<Real> of interpolated values stored according to StorageOrder_
     *********************************************************************/
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    std::vector<Real> GridInterpolation(const Container& x_1d, const Container& y_1d) const {
        std::vector<Real> result;
        result.reserve(x_1d.size() * y_1d.size());

        if constexpr (StorageOrder_ == YMajor) {
            for (std::size_t ix = 0; ix < x_1d.size(); ix++) {
                Real x = *(x_1d.begin() + ix);
                for (std::size_t iy = 0; iy < y_1d.size(); iy++) {
                    Real y = *(y_1d.begin() + iy);
                    result.push_back(this->operator()(x, y));
                }
            }
        } else if constexpr (StorageOrder_ == XMajor) {
            for (std::size_t iy = 0; iy < y_1d.size(); iy++) {
                Real y = *(y_1d.begin() + iy);
                for (std::size_t ix = 0; ix < x_1d.size(); ix++) {
                    Real x = *(x_1d.begin() + ix);
                    result.push_back(this->operator()(x, y));
                }
            }
        } else {
            static_assert(utils::always_false<Real>::value, "Unknown StorageOrder. Avaliable options are YMajor, XMajor");
        }
        return result;
    }

private:

    /********************************************************************
     * Calculate interpolation function at the given point (x, y)
     * for given rectangle (ix, iy) - left-bottom corner index
     * 
     * @see https://en.wikipedia.org/wiki/Bilinear_interpolation
     * 
     * @param[in] x X-coordinate
     * @param[in] y Y-coordinate
     * @return Interpolated value
     *********************************************************************/
    template<class Index>
    Real InterpolateRect(const Real& x, const Real& y, Index ix, Index iy) const {
        /* Calculate coordinate vectors and value matrix */
        std::array<Real, 2> vx{(x_1d_[ix + 1] - x), (x - x_1d_[ix])};
        std::array<Real, 2> vy{(y_1d_[iy + 1] - y), (y - y_1d_[iy])};
        std::array<std::array<Real, 2>, 2> fQ{{
            {z_all_[Idx(ix    , iy)], z_all_[Idx(ix    , iy + 1)]},
            {z_all_[Idx(ix + 1, iy)], z_all_[Idx(ix + 1, iy + 1)]}
        }};

        /* Matrix-vector multiplication: f(Q) * vy */
        std::array<Real, 2> fQ_vy{
            std::inner_product(fQ[0].begin(), fQ[0].end(), vy.begin(), Real(0)),
            std::inner_product(fQ[1].begin(), fQ[1].end(), vy.begin(), Real(0))
        };

        /* Return: (1 / norm) * vx * f(Q) * vy */
        Real norm = (x_1d_[ix + 1] - x_1d_[ix]) * (y_1d_[iy + 1] - y_1d_[iy]);
        return std::inner_product(vx.begin(), vx.end(), fQ_vy.begin(), Real(0)) / norm;
    }

    /********************************************************************
     * Extrapolate function at the given point. (x, y) should be outside
     * domain interior, boundary points are allowed.
     * @param[in] x X-coordinate
     * @param[in] y Y-coordinate
     * @return Extrapolated value
     *********************************************************************/
    Real Extrapolate(const Real& x, const Real& y) const {
        auto nx = x_1d_.size();
        auto ny = y_1d_.size();

        if (nx > 1 && ny > 1) {
            /* bilinear extrapolation */
            std::size_t ix, iy;
            if (x <= x_1d_.front()) {
                ix = 0;
                iy = LessIndex(y, y_1d_);
            } else if (x >= x_1d_.back()) {
                ix = x_1d_.size() - 2;
                iy = LessIndex(y, y_1d_);
            } else if (y <= y_1d_.front()) {
                ix = LessIndex(x, x_1d_);
                iy = 0;
            } else {
                ix = LessIndex(x, x_1d_);
                iy = y_1d_.size() - 2;
            }
            return InterpolateRect(x, y, ix, iy);
        } else {
            if (nx > 1) {
                /* constant along y-axis, linear along x-axis */
                auto interp_x = LinearInterpolator<Real, LinearExtrapolation>(x_1d_, z_all_);
                return interp_x(x);
            } else if (ny > 1) {
                /* constant along x-axis, linear along y-axis */
                auto interp_y = LinearInterpolator<Real, LinearExtrapolation>(y_1d_, z_all_);
                return interp_y(y);
            } else {
                /* constant extrapolation */
                return z_all_.front();
            }
        }
    }

    /********************************************************************
     * Find nearest index to the val that is less or equal than val.
     * For val < vec[1] index equals 0, for val >= vec[n-2] index equals n-2.
     * @param[in] val Value to compare the elements to
     * @param[in] vec Array to examine. Size must be >= 2
     * @return Found index
     *********************************************************************/
    std::size_t LessIndex(const Real& val, const std::vector<Real>& vec) const {
        auto n = vec.size();
        if (val < vec[1]) { return 0; }
        if (val >= vec[n - 2]) { return n - 2; }
        auto iter = std::upper_bound(vec.begin() + 1, vec.end() - 2, val);
        return std::distance(vec.begin(), iter) - 1;
    }

    /********************************************************************
     * Plain index of 2D array according to StorageOrder_.
     * @param[in] ix Index along x-axis
     * @param[in] iy Index along y-axis
     * @return Plain index
     *********************************************************************/
    template<class Index>
    Index Idx(Index ix, Index iy) const {
        if constexpr (StorageOrder_ == YMajor) {
            return ix * y_1d_.size() + iy;
        } else if constexpr (StorageOrder_ == XMajor) {
            return iy * x_1d_.size() + ix;
        } else {
            static_assert(utils::always_false<Real>::value, "Unknown StorageOrder. Avaliable options are YMajor, XMajor");
        }
    }

private:

    std::vector<Real> x_1d_, y_1d_, z_all_;
};
}
