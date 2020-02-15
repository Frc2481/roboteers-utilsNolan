#include "Utils/IsDoubleEqual.h"
#include <math.h>

bool isDoubleEqual::isDoubleEqual(const double value1, const double value2, const double epsilon) {
    return fabs(value1 - value2) < epsilon;
}
