#pragma once

#include <cmath>
#include <tuple>
#include <array>
#include <cassert>
#include <algorithm>

#include "envvar.h"
#include "csv_utils.h"

namespace phisics {
struct Vector {
    double x;
    double y;
    double z;

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

namespace detail {
template <typename Vec>
constexpr Vec RotatePointXY(Vec const& p0, Vec const& p, double phi)
{
    Vec pp = p;
    pp.x = p0.x + (p.x - p0.x) * std::cos(phi) + (p.y - p0.y) * std::sin(phi);
    pp.y = p0.y - (p.x - p0.x) * std::sin(phi) + (p.y - p0.y) * std::cos(phi);
    return pp;
}
} // namespace detail


namespace globals {
inline double constexpr g = 9.8; // ускорение свободного падения 
inline double constexpr n = 1.4; // показатель политропы
inline double constexpr chi = 1; // коэффициент истечения
inline double constexpr p_a = 1e5; // атмосферное давление
inline double constexpr rho_a = 1.2690; // плотность воздуха
inline double constexpr rho_w = 1000.0; // плотность воды
} // namespace globals

namespace wave {
inline constexpr double y(double x) {
    // return std::sin(x) / 3;
    return 0.0;
}
}

struct ACV {
    static double constexpr m = 16000; // масса
    static double constexpr l_AC = 0; // расстояние от центра давления до центра тяжести, вдоль OX
    static double constexpr L = 15; // длина воздушной подушки
    static double constexpr b = 6; // ширина воздушной подушки
    static double constexpr d_max = 0.7; // максимальная высота воздушной подушки
    static double constexpr S = L * b; // площадь воздушной подушки
    static double constexpr I_z = 250000; // момент инерции судна

    static double constexpr theta = 0.1; // параметр влияния волны на судно

    static double constexpr V_x = 5; // постоянная скорость буксира

    static int    constexpr N = 300; // количество сечений ВП
    static double constexpr delta_L = L / N; // ширина сечения ВП

    struct Segment {
        bool isFirstOrLast;

        double x; // знаковое смещение сегмента относительно центра тяжести судна

        Vector c; // координата верхней части сегмента ВП
        double W; // объем сечения

        Vector V; // вектор скорости
        Vector w; // вектор угловой скорости

        double d; // высота столба воздуха в сегменте

        double S_wash; // пятно контакта
        double V_scalar; // скорость сегмента
        double F_contact; // сила действующая на пятно контакта 
        double M_contact; // момент силы действующей на пятно контакта

        double S_gap; // ширина зазора в сегменте

        Segment() = default;
        explicit Segment(int index, double W);

        constexpr void UpdateParams(Vector const& new_c, Vector const& new_V, Vector const& new_w, double phi);
        constexpr void UpdateState();

        constexpr bool HasContact() const;
        constexpr double GapHeight() const;

        constexpr bool IsFirstOrLast() const;
    };

    struct Compressor {
        static double constexpr A = -2.756;
        static double constexpr B = 48.46;
        static double constexpr C = 2771;

        static double constexpr Q_min = 0.0;
        static double constexpr Q_max = -B / 2 / A; // вершина квадратичной функции
        static double constexpr p_min = 0.0; // минимальное избыточное давление ВП
        static double constexpr p_max = A * Q_max * Q_max + B * Q_max + C; // максимальное избыточное давление ВП

        static constexpr double Clamp(double p)
        {
            return std::clamp(p, p_min, p_max);
        }

        static constexpr double Q_in(double p)
        {
            // A * Qin * Qin + B * Qin + (C - p) = 0 
            //      D = B * B - 4 * A * (C - p)
            return (-B - std::sqrt(B * B - 4 * A * (C - p))) / 2 / A;
        }

        static constexpr double Q_out(double p, double S_gap)
        {
            return globals::chi * std::sqrt(2 * p / globals::rho_a) * S_gap;
        }
    };

    std::array<Segment, N> segments; // все N сечений судна

    Vector c; // координата судна
    double W; // объем ВП

    double p; // избыточное давление ВП
    double p_qs;
    double p_damp;
    
    double phi; // тангаж

    Vector V; // вектор скорости
    Vector w; // вектор угловой скорости

    double Q_in; // расход возуха в ВП
    double Q_out; // расход возуха из ВП

    ACV();

    void Update(double dt);

    std::tuple<double, double, double, double> CalcSegmentsCharacteristics(
        Vector const& new_c, 
        Vector const& new_V, 
        Vector const& new_w,
        double phi
    );
};
} // namespace phisics