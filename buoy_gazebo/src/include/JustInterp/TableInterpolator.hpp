#pragma once

#include <vector>

#include "JustInterp/Utils.hpp"
#include "JustInterp/LinearInterpolator.hpp"

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