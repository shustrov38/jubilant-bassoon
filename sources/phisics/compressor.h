#pragma once

#include <algorithm>

#include "constants.h"

namespace phisics::compressor {
inline constexpr double Clamp(double p)
{
    return std::clamp(p, p_min, p_max);
}

inline constexpr double Q_in(double p)
{
    // A * Qin * Qin + B * Qin + (C - p) = 0 
    //      D = B * B - 4 * A * (C - p)
    return (-B - std::sqrt(B * B - 4 * A * (C - p))) / 2 / A;
}

inline constexpr double Q_out(double p, double S_gap)
{
    return globals::chi * std::sqrt(2 * p / globals::rho_a) * S_gap;
}
} // namespace phisics::compressor
