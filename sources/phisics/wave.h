#pragma once

#include "constants.h"

namespace phisics::wave {
inline double constexpr Height(double x, double t)
{
    return A * std::sin(k * x + omega * t);
}
} // namespace phisics::wave
