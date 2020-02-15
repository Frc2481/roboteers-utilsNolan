#ifndef ISDOUBLEEQUAL_H
#define ISDOUBLEEQUAL_H

namespace isDoubleEqual {
    //////////////////////////////////////////////////////////////////////
    // @brief compares two doubles and determines if they are equal to each
    //      other within some epsilon
    // @param value1 - first value to compare
    // @param value2 - second value to compare
    //////////////////////////////////////////////////////////////////////
    bool isDoubleEqual(
        const double value1,
        const double value2,
        const double epsilon
    );
}

#endif // ISDOUBLEEQUAL_H
