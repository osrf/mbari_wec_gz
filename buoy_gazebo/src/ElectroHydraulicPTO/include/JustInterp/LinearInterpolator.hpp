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
#include <cassert>

#include "JustInterp/Utils.hpp"

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