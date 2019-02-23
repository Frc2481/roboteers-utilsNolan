#include "Utils/NormalizeToRange.h"
#include <algorithm>
#include <cmath>
#include <stdio.h>

namespace normalizeToRange {
    double normalizeToRange(double value, double rangeMin, double rangeMax, bool includeMax) {
        if(rangeMin > rangeMax) {
            std::swap(rangeMin, rangeMax);
        }

        value -= rangeMin;
        rangeMax -= rangeMin;

        value = std::fmod(value, rangeMax);

        if(value < 0) {
            value += rangeMax;
        }
        else if(includeMax && (value == 0)) {
            value = rangeMax;
        }

        return value + rangeMin;
    }

    double rangedDifference(double difference, double rangeMin, double rangeMax) {
        if(std::fabs(difference) > (rangeMax - rangeMin) / 2.0) {
            if(difference > 0) {
                difference -= rangeMax - rangeMin;
            }
            else {
                difference += rangeMax - rangeMin;
            }
        }

        return difference;
    }
}
