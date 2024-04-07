#pragma once

#include <cmath>
#include <limits>

#include "mytypes.h"

namespace utils {
namespace detail {
inline Double constexpr NewtonRaphsonEps = 1e-7;

Double constexpr SqrtNewtonRaphson(Double x, Double curr, Double prev)
{
    return std::abs(curr - prev) < NewtonRaphsonEps 
        ? curr 
        : SqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}
} // namespace detail

// constexpr версия квадратного корня через метод ньютона 
Double constexpr Sqrt(Double x)
{
	return x >= 0 && x < std::numeric_limits<Double>::infinity()
		? detail::SqrtNewtonRaphson(x, x, 0)
		: std::numeric_limits<Double>::quiet_NaN();
}

// повернуть точку p относительно точки p0 на уголь phi
template <typename VecT>
VecT constexpr RotatePointXY(VecT const& p0, VecT const& p, Double phi)
{
    VecT pp = p;
    pp.x = p0.x + (p.x - p0.x) * std::cos(phi) + (p.y - p0.y) * std::sin(phi);
    pp.y = p0.y - (p.x - p0.x) * std::sin(phi) + (p.y - p0.y) * std::cos(phi);
    return pp;
}

struct Vector {
    Double x;
    Double y;
    Double z;

    friend Vector& operator +=(Vector &lhs, Vector const& rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        return lhs;
    }

    friend Vector operator +(Vector lhs, Vector const& rhs)
    {
        return lhs += rhs;
    }

    friend Vector operator *(Vector lhs, double rhs)
    {
        lhs.x *= rhs;
        lhs.y *= rhs;
        lhs.z *= rhs;
        return lhs;
    }
};
} // namespace utils
