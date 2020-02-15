#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include <vector>

namespace interpolate {
    //////////////////////////////////////////////////////////////////////
    // @brief linearly interpolates y value from given x value and points
    //      along curve (x, y)
    // @param xData - list of x values along curve.  must have at least two
    //      elements, sorted, monotonically increasing.
    // @param yData - list of y values along curve
    // @param x - x value of interpolated point
    // @param extrapolate - determines if extrapolation is performed for x
    //      values that are beyond the range of xData
    //////////////////////////////////////////////////////////////////////
    double interp(
        const std::vector<double> &xData,
        const std::vector<double> &yData,
        double x,
        bool extrapolate
    );

    //////////////////////////////////////////////////////////////////////
    // @brief linearly interpolates y value from given x value and points
    //      along curve (x, y) and normalize difference to range by 
    //      accounting for wraparound
    // @param xData - list of x values along curve.  must have at least two
    //      elements, sorted, monotonically increasing.
    // @param yData  - list of y values along curve
    // @param x - x value of interpolated point
    // @param extrapolate - determines if extrapolation is performed for x
    //      values that are beyond the range of xData
    // @param rangeMin - range min
    // @param rangeMax - range max
    //////////////////////////////////////////////////////////////////////
    double rangedInterp(
        const std::vector<double> &xData,
        const std::vector<double> &yData,
        double x,
        bool extrapolate,
        double rangeMin,
        double rangeMax
    );
}

#endif // INTERPOLATE_H
