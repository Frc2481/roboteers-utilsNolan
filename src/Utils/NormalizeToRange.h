#ifndef NORMALIZE_TO_RANGE_H
#define NORMALIZE_TO_RANGE_H

namespace normalizeToRange {
    //////////////////////////////////////////////////////////////////////
    // @brief normalize value to range
    // @param value - value to normalize to range
    // @param rangeMin - range min
    // @param rangeMax - range max
    // @param includeMax - true = include max in range
    //                   - false = include min in range
    //////////////////////////////////////////////////////////////////////
    double normalizeToRange(double value, double rangeMin, double rangeMax, bool includeMax);

    //////////////////////////////////////////////////////////////////////
    // @brief normalize difference to range by accounting for wraparound
    // @param difference - difference to normalize to range
    // @param rangeMin - range min
    // @param rangeMax - range max
    //////////////////////////////////////////////////////////////////////
    double rangedDifference(double difference, double rangeMin, double rangeMax);
}

#endif // NORMALIZE_TO_RANGE_H
