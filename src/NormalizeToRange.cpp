#include "NormalizeToRange.h"
#include <algorithm>
#include <math.h>

namespace normalizeToRange {
    double normalizeToRange(double value, double rangeMin, double rangeMax, bool includeMax) {
        if(rangeMin > rangeMax) {
            std::swap(rangeMin, rangeMax);
        }

        value = std::fmod(value + rangeMax, rangeMax - rangeMin);
        if(value > 0) {
            value += rangeMin;
        }
        else if(value < 0) {
            value += rangeMax;
        }
        else if(includeMax) {
            value = rangeMax;
        }
        else {
            value = rangeMin;
        }

        return value;
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