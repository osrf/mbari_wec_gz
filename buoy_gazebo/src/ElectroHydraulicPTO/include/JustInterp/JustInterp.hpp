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

// JustInterp repo:
// https://github.com/alexander-valov/JustInterp.git

#pragma once

// #include "JustInterp/Utils.hpp"


#include <type_traits>

namespace JustInterp {

namespace utils {

    /********************************************************************
     * C++17 compatible implementation of std::experimental::is_detected
     * @see https://en.cppreference.com/w/cpp/experimental/is_detected
     * @see https://people.eecs.berkeley.edu/~brock/blog/detection_idiom.php
     * @see https://blog.tartanllama.xyz/detection-idiom/
     *********************************************************************/
    namespace detail {
        template <template <class...> class Trait, class Enabler, class... Args>
        struct is_detected : std::false_type{};

        template <template <class...> class Trait, class... Args>
        struct is_detected<Trait, std::void_t<Trait<Args...>>, Args...> : std::true_type{};
    }
    template <template <class...> class Trait, class... Args>
    using is_detected = typename detail::is_detected<Trait, void, Args...>::type;


    /********************************************************************
     * Aliases to check support of specific methods or types
     *********************************************************************/
    /* check data() method support */
    template <class T>
    using method_data_t = decltype(std::declval<T>().data());
    template <class T>
    using supports_data = is_detected<method_data_t, T>;

    /* check size() method support */
    template <class T>
    using method_size_t = decltype(std::declval<T>().size());
    template <class T>
    using supports_size = is_detected<method_size_t, T>;

    /* check begin() method support */
    template <class T>
    using method_begin_t = decltype(std::declval<T>().begin());
    template <class T>
    using supports_begin = is_detected<method_begin_t, T>;

    /* check end() method support */
    template <class T>
    using method_end_t = decltype(std::declval<T>().end());
    template <class T>
    using supports_end = is_detected<method_end_t, T>;

    /* check data() is Real* */
    template<class T, class Real>
    using is_real_type_data = std::is_same<Real*, decltype(std::declval<T>().data())>;


    /********************************************************************
     * Alias to detect real-type iterable container
     *********************************************************************/
    template<typename T, typename Real>
    using IsRealContainer = std::enable_if_t<
        std::conjunction_v<
            is_real_type_data<T, Real>,
            supports_begin<T>,
            supports_end<T>,
            supports_data<T>,
            supports_size<T>
        >,
        bool
    >;

    /********************************************************************
     * Alias to detect iterable container of real-type iterable containers
     *********************************************************************/
    template<template<typename> class Container, typename RealContainer, typename Real>
    using IsContainerOfRealContainers = std::enable_if_t<
        std::conjunction_v<
            supports_begin<Container<RealContainer>>,
            supports_end<Container<RealContainer>>,
            supports_data<Container<RealContainer>>,
            supports_size<Container<RealContainer>>,
            is_real_type_data<RealContainer, Real>,
            supports_begin<RealContainer>,
            supports_end<RealContainer>,
            supports_data<RealContainer>,
            supports_size<RealContainer>
        >,
        bool
    >;

    /********************************************************************
     * False type at instantiation time
     * @see https://stackoverflow.com/questions/58694521/what-is-stdfalse-type-or-stdtrue-type
     * @see https://stackoverflow.com/questions/14637356/static-assert-fails-compilation-even-though-template-function-is-called-nowhere
     * @see https://artificial-mind.net/blog/2020/10/03/always-false
     *********************************************************************/
    template <typename T>
    struct always_false : std::false_type {};
}

enum StorageOrder {
    YMajor = 0,
    XMajor = 1
};

enum ExtrapolationType {
    ConstantExtrapolation = 0,
    LinearExtrapolation = 1
};

}
// #include "JustInterp/LinearInterpolator.hpp"


#include <vector>
#include <algorithm>
#include <iterator>
#include <cassert>

// #include "JustInterp/Utils.hpp"


namespace JustInterp {

template<class Real, int ExtrapolationType_ = ConstantExtrapolation>
class LinearInterpolator {

public:

    /********************************************************************
     * Constructors
     *********************************************************************/
    LinearInterpolator() = default;
    template<class Index>
    LinearInterpolator(Index size, const Real* x, const Real* y) { SetData(size, x, y); }
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    LinearInterpolator(const Container& x, const Container& y) { SetData(x, y); }

    /********************************************************************
     * @brief Set known data points and values.
     * @param[in] size Size of given arrays
     * @param[in] x Known points array
     * @param[in] y Known values array
     *********************************************************************/
    template<class Index>
    void SetData(Index size, const Real* x, const Real* y) {
        assert((size > 0 && "Empty data for interpolation"));
        xData_.clear();
        yData_.clear();
        std::copy(x, x + size, std::back_inserter(xData_));
        std::copy(y, y + size, std::back_inserter(yData_));
    }

    /********************************************************************
     * @brief Set known data points and values.
     *
     * Container must have real-type values and the following methods:
     * data(), size(), begin(), end()
     *
     * @param[in] x Known points array
     * @param[in] y Known values array
     *********************************************************************/
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    void SetData(const Container& x, const Container& y) {
        assert((x.size() == y.size() && "Sizes of points and values are mismatch"));
        SetData(x.size(), x.data(), y.data());
    }

    /********************************************************************
     * Access
     *********************************************************************/
    const std::vector<Real>& GetX() const { return xData_; }
    const std::vector<Real>& GetY() const { return yData_; }

    /********************************************************************
     * @brief Calculate interpolation function at the given point.
     * @param[in] x Point to interpolate
     * @return Interpolated value
     *********************************************************************/
    Real operator()(const Real& x) const {
        if (x <= xData_.front() || x >= xData_.back()) {
            return Extrapolate(x);
        } else {
            auto lower = std::lower_bound(xData_.begin(), xData_.end(), x);
            std::size_t i = std::distance(xData_.begin(), lower) - 1;
            return yData_[i] + (x - xData_[i]) * (yData_[i + 1] - yData_[i]) / (xData_[i + 1] - xData_[i]);
        }
    }

    /********************************************************************
     * @brief Calculate interpolation function at each point of given array.
     *
     * Container must have real-type values and the following methods:
     * data(), size(), begin(), end()
     *
     * @param[in] x Array of points to interpolate
     * @return std::vector<Real> of interpolated values
     *********************************************************************/
    template<class Container, utils::IsRealContainer<Container, Real> = true>
    std::vector<Real> operator()(const Container& x) const {
        std::vector<Real> result;
        result.reserve(x.size());
        for (const auto& item : x) {
            result.push_back(this->operator()(item));
        }
        return result;
    }

private:

    Real Extrapolate(const Real& x) const {
        if constexpr (ExtrapolationType_ == ConstantExtrapolation) {
            return ExtrapolateConstant(x);
        } else if constexpr (ExtrapolationType_ == LinearExtrapolation) {
            return ExtrapolateLinear(x);
        } else {
            static_assert(utils::always_false<Real>::value, "Unknown ExtrapolationType. Avaliable options are ConstantExtrapolation, LinearExtrapolation");
        }
    }

    Real ExtrapolateConstant(const Real& x) const {
        if (x <= xData_.front()) {
            return yData_.front();
        } else {
            return yData_.back();
        }
    }

    Real ExtrapolateLinear(const Real& x) const {
        auto n = xData_.size();
        if (n > 1) {
            /* linear extrapolation */
            if (x <= xData_.front()) {
                return yData_[0] + (x - xData_[0]) * (yData_[1] - yData_[0]) / (xData_[1] - xData_[0]);
            } else {
                return yData_[n - 2] + (x - xData_[n - 2]) * (yData_[n - 1] - yData_[n - 2]) / (xData_[n - 1] - xData_[n - 2]);
            }
        } else {
            /* constant extrapolation */
            return yData_.front();
        }
    }

private:

    std::vector<Real> xData_, yData_;

};

}
// #include "JustInterp/BilinearInterpolator.hpp"


#include <vector>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <cassert>

// #include "JustInterp/Utils.hpp"

// #include "JustInterp/LinearInterpolator.hpp"


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
// #include "JustInterp/TableInterpolator.hpp"


#include <vector>

// #include "JustInterp/Utils.hpp"

// #include "JustInterp/LinearInterpolator.hpp"


namespace JustInterp {

template<class Real, int ExtrapolationType_ = ConstantExtrapolation>
class TableInterpolator {

public:

    /********************************************************************
     * Constructors
     *********************************************************************/
    TableInterpolator() = default;
    template<class RealContainer, utils::IsRealContainer<RealContainer, Real> = true>
    TableInterpolator(
        const RealContainer& x_1d,
        const std::vector<RealContainer>& y_1d_arrays,
        const std::vector<RealContainer>& z_1d_arrays
    ) {
        SetData(x_1d, y_1d_arrays, z_1d_arrays);
    }

    /********************************************************************
     * @brief Set known data points and values.
     * @param[in] x_1d 1D array of points along x-axis
     * @param[in] y_1d_arrays Array of 1D arrays of points along y-axis
     *                        for corresponding point from x_1d
     * @param[in] z_1d_arrays Array of 1D arrays of values for corresponding
     *                        points from x_1d and y_1d_arrays
     *********************************************************************/
    template<class RealContainer, utils::IsRealContainer<RealContainer, Real> = true>
    void SetData(
        const RealContainer& x_1d,
        const std::vector<RealContainer>& y_1d_arrays,
        const std::vector<RealContainer>& z_1d_arrays
    ) {
        /* Check dimensions */
        assert((x_1d.size() > 0 && "Empty x_1d data for interpolation"));
        assert((y_1d_arrays.size() == x_1d.size() && "Sizes of y_1d_arrays and x_1d are mismatch"));
        assert((z_1d_arrays.size() == x_1d.size() && "Sizes of z_1d_arrays and x_1d are mismatch"));
        /* Copy 1D array of points along x-axis */
        xData_.clear();
        std::copy(x_1d.data(), x_1d.data() + x_1d.size(), std::back_inserter(xData_));
        /* Initialize 1D interpolators for each line of data along y-axis */
        for (std::size_t ix = 0; ix < xData_.size(); ix++) {
            yInterpData_.emplace_back(y_1d_arrays[ix], z_1d_arrays[ix]);
        }
    }

    /********************************************************************
     * Access
     *********************************************************************/
    const std::vector<Real>& GetX() const { return xData_; }
    const std::vector<Real>& GetY(const std::size_t& index) const {
        return yInterpData_.at(index).GetX();
    }

    /********************************************************************
     * Calculate interpolation function at the given point.
     * If point is outside the data extrapolation is performed.
     * @param[in] x X-coordinate
     * @param[in] y Y-coordinate
     * @return Interpolated value
     *********************************************************************/
    Real operator()(const Real& x, const Real& y) const {
        std::vector<Real> x_arr, value_arr;
        if (x <= xData_.front()) {
            /* left extrapolation along x-axis */
            x_arr.push_back(xData_[0]);
            value_arr.push_back(yInterpData_[0](y));
            if (xData_.size() > 1) {
                x_arr.push_back(xData_[1]);
                value_arr.push_back(yInterpData_[1](y));
            }
        } else if (x >= xData_.back()) {
            /* right extrapolation along x-axis */
            auto n = xData_.size();
            if (xData_.size() > 1) {
                x_arr.push_back(xData_[n - 2]);
                value_arr.push_back(yInterpData_[n - 2](y));
            }
            x_arr.push_back(xData_[n - 1]);
            value_arr.push_back(yInterpData_[n - 1](y));
        } else {
            /* linear interpolation between neighboring xData_ points */
            auto lower = std::lower_bound(xData_.begin(), xData_.end(), x);
            std::size_t i = std::distance(xData_.begin(), lower) - 1;
            x_arr.push_back(xData_[i]);
            value_arr.push_back(yInterpData_[i](y));
            x_arr.push_back(xData_[i + 1]);
            value_arr.push_back(yInterpData_[i + 1](y));
        }
        LinearInterpolator<Real, ExtrapolationType_> x_interpolator(x_arr, value_arr);
        return x_interpolator(x);
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

private:

    std::vector<Real> xData_;
    std::vector<LinearInterpolator<Real, ExtrapolationType_>> yInterpData_;

};

}

/* *INDENT-ON* */
