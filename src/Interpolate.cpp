#include "Interpolate.h"

double interpolate::interp(
    const std::vector<double> &xData,
    const std::vector<double> &yData,
    const double &x,
    const bool &extrapolate) {

    // find index of interval for interpolation
    int i = 0;
    if(x >= xData[xData.size() - 2]) { // check if beyond right limit
        i = xData.size() - 2;
    }
    else {
        while(x > xData[i + 1]) {
            i++;
        }
    }

    // get interval for interpolation
    double xL = xData[i];
    double yL = yData[i];
    double xR = xData[i + 1];
    double yR = yData[i + 1];

    // do not extrapolate if set by setting y values to be equal
    if(!extrapolate) {
        // check if left limit
        if(x < xL) {
            yR = yL;
        }
        // check if right limit
        if(x > xR) {
            yL = yR;
        }
    }

    // calculate gradient
    double dydx = (yR - yL) / (xR - xL);

    // return linearly interpolated value
    return yL + dydx * (x - xL);
}

/*
void interpolate::interparc(
    const std::vector<double> &distData,
    const std::vector<double> &xData,
    const std::vector<double> &yData,
    const bool &extrapolate,
    const double &dist,
    double &x,
    double &y
) {
    // find index of interval for interpolation
    int i = 0;
    if(dist >= distData[distData.size() - 2]) { // check if beyond right limit
        i = distData.size() - 2;
    }
    else {
        while(dist > distData[i + 1]) {
            i++;
        }
    }

    // get interval for interpolation
    double dL = distData[i];
    double xL = xData[i];
    double yL = yData[i];
    double dR = distData[i];
    double xR = xData[i + 1];
    double yR = yData[i + 1];

    // do not extrapolate if set by setting y values to be equal
    if(!extrapolate) {
        // check if left limit
        if(dist < dL) {
            xR = xL;
            yR = yL;
        }
        // check if right limit
        if(dist > dR) {
            xL = xR;
            yL = yR;
        }
    }

    // calculate gradient
    double dxdd = (xR - xL) / (dR - dL);
    double dydd = (yR - yL) / (dR - dL);

    // linearly interpolate point
    x = xL + dxdd * (dist - dL);
    y = yL + dydd * (dist - dL);
}
*/
