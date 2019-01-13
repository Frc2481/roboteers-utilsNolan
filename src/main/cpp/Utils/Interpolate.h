#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include <vector>

namespace interpolate {
    //////////////////////////////////////////////////////////////////////
    // linearly interpolates y value from given x value and points along
    // curve (x, y)
    // xData        list of x values along curve.  must have at least two
    //              elements, sorted, monotonically increasing.
    // yData        list of y values along curve
    // x            x value of interpolated point
    // extrapolate  determines if extrapolation is performed for x values
    //              that are beyond the range of xData
    // return       y value of interpolated point
    //////////////////////////////////////////////////////////////////////
    double interp(
        const std::vector<double> &xData,
        const std::vector<double> &yData,
        double x,
        bool extrapolate
    );
}

#endif // INTERPOLATE_H
