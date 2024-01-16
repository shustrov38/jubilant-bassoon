#include <cmath>
#include <array>
#include <tuple>
#include <vector>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <iostream>

#define SQ(x) (x) * (x)

struct Vector {
    double x;
    double y;
    double z;
};

Vector& operator +=(Vector &lhs, Vector const& rhs)
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

Vector operator +(Vector lhs, Vector const& rhs)
{
    return lhs += rhs;
}

namespace globals {
inline double constexpr g = 9.8; // ускорение свободного падения 
inline double constexpr n = 1.4; // показатель политропы
inline double constexpr chi = 1; // коэффициент истечения
inline double constexpr p_a = 1e5; // атмосферное давление
inline double constexpr rho_a = 1.2690; // плотность воздуха
inline double constexpr rho_w = 1000.0; // плотность воды
} // namespace globals

namespace wave {
constexpr double y(double x) {
    return 0.0;
}
};

struct ACV {
    static double constexpr m = 16000; // масса
    static double constexpr l_AC = 0; // расстояние от центра давления до центра тяжести, вдоль OX
    static double constexpr L = 15; // длина воздушной подушки
    static double constexpr b = 6; // ширина воздушной подушки
    static double constexpr d_max = 0.7; // максимальная высота воздушной подушки
    static double constexpr S = L * b; // площадь воздушной подушки
    static double constexpr I_z = 250000; // момент инерции судна

    static double constexpr V_x = 2; // постоянная скорость буксира

    static int    constexpr N = 10; // количество сечений ВП
    static double constexpr delta_L = L / N; // ширина сечения ВП

    struct Segment {
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

        explicit Segment(int index, double W)
            : W(W)
            , x(L / 2 - delta_L * (index - 0.5))
        {
        }

        constexpr void UpdateParams(Vector const& new_c, Vector const& new_V, Vector const& new_w)
        {
            c = new_c;
            c.x += x; // применить смещение
            V = new_V;
            w = new_w;
            UpdateState();
        }

        constexpr void UpdateState()
        {
            d = std::max(0.0, c.y - wave::y(c.x));
            W = d * b * delta_L;
            S_wash = (HasContact() ? delta_L * b : 0);
            V_scalar = std::sqrt(SQ(V.x) + (SQ(V.y) + w.z * x));
            F_contact = x * globals::rho_w * SQ(V_scalar) / 2 * S_wash;
            M_contact = F_contact * x;
            S_gap = 2 * GapHeight() * delta_L;
        }

        constexpr bool HasContact() const
        {
            return c.y <= wave::y(c.x);
        }

        constexpr double GapHeight() const
        {
            return std::max(0.0, c.y - d - wave::y(c.x));
        }
    };

    struct Compressor {
        static double constexpr A = -2.756;
        static double constexpr B = 48.46;
        static double constexpr C = 2771;

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

    Vector V; // вектор скорости
    Vector w; // вектор угловой скорости

    ACV()
    {
        W = S * d_max / 2; // половина ВП наполнена
        c = {0, d_max / 2, 0}; // центр тяжести на половине клиренса ВП

        p = 0; // без давления в подушке только атмосфера, избыточного давления нет
        
        V = {V_x, 0, 0}; // присутствует только скорость буксира
        w = {0, 0, 0}; // угловой скорости нет вообще

        for (int i = 0; i < N; ++i) {
            segments[i] = Segment(i + 1, W / N);
        }
    }

    void Update(double dt)
    {
        UpdateSegmentsParams(c, V, w);
        auto [_W, F_wave, M_contact, S_gap] = CalcSegmentsCharacteristics();

        double dWdt = W - _W;
        W = _W;

        double Q_in = Compressor::Q_in(p);
        double Q_out = Compressor::Q_out(p, S_gap);

        double dV_ydt = (p * S - m * globals::g + F_wave) / m;
        double dpdt = globals::n * globals::p_a * (Q_in - Q_out - dWdt) / W;
        double dw_zdt = (p * S * l_AC + M_contact) / I_z;

        V.y += dV_ydt * dt;
        p = Compressor::Clamp(p + dpdt * dt);

        w.z += dw_zdt * dt;

        // TODO: create operator
        c.x += V.x * dt;
        c.y += V.y * dt;
        c.z += V.z * dt;

        std::cout << p << ' ' << W << ' ' << S_gap << '\n';
    }

    void UpdateSegmentsParams(Vector const& new_c, Vector const& new_V, Vector const& new_w)
    {
        for (auto &seg: segments) {
            seg.UpdateParams(new_c, new_V, new_w);
        }
    }

    std::tuple<double, double, double, double> CalcSegmentsCharacteristics() const
    {
        double _W = 0;
        double _F_wave = 0;
        double _M_contact = 0;
        double _S_gap = 0;
        for (size_t i = 0; i < N; ++i) {
            _W += segments[i].W;
            _F_wave += segments[i].F_contact;
            _M_contact += segments[i].M_contact;
            _S_gap += segments[i].S_gap;
        }
        return {_W, _F_wave, _M_contact, _S_gap};
    }
};

int main(int argc, char** argv)
{
    double const dt = 0.05;
    ACV acv;
    for (double t = 0.0; t < 5.0; t += dt) {
        acv.Update(dt);
    }
    return EXIT_SUCCESS;
}