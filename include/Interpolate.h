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
        const double &x,
        const bool &extrapolate
    );

    /*
    //////////////////////////////////////////////////////////////////////
    // distance based linear interpolation along a general curve
    // (xData, yData) in space
    // distData     list of fractional distance values along curve with
    //              respect to total distance of curve.  for example, 0.5
    //              is the midpoint of the curve.  must have at least two
    //              elements, sorted, monotonically increasing, within
    //              range of [0, 1].
    // xData        list of x values along curve
    // yData        list of y values along curve
    // extrapolate  determines if extrapolation is performed for dist values
    //              that are beyond the range of distData
    // dist         fractional distance value of interpolated point.  must
    //              be within range of [0, 1].
    // x            x value of interpolated point
    // y            y value of interpolated point
    // return       success or failure
    //////////////////////////////////////////////////////////////////////
    void interparc(
        const std::vector<double> &distData,
        const std::vector<double> &xData,
        const std::vector<double> &yData,
        const bool &extrapolate,
        const double &dist,
        double &x,
        double &y
    );
    */
}
