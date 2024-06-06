#pragma once 

#include "constants.h"

namespace phisics::damp {
inline Double constexpr Coef(Double S_gap, Double S)
{
    Double x = std::max(0.001, S_gap / S);
    return 0.0001 * (A + B * std::exp(C * x));
}
} // namespace phisics::damp
