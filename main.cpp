#include <cmath>
#include <vector>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace globals {
inline double constexpr g = 9.8; // ускорение свободного падения 
inline double constexpr n = 1.4; // показатель политропы
inline double constexpr chi = 1; // коэффициент истечения
inline double constexpr pa = 1e5; // атмосферное давление
inline double constexpr rho = 1.2690; // плотность воздуха
inline double constexpr eps = 1e-6; // маленькое число
} // namespace Globals

namespace ACV {
inline double constexpr m = 16000; // масса
inline double constexpr l = 1; // расстояние от центра давления до центра тяжести, вдоль OX
inline double constexpr a = 15; // длина воздушной подушки
inline double constexpr b = 6; // ширина воздушной подушки
inline double constexpr d = 0.7; // высота воздушной подушки
inline double constexpr S = a * b; // площадь воздушной подушки
inline double constexpr Iz = 250000; // момент инерции судна
} // namespace ACV

class Compressor 
{
public:
    static constexpr double Clamp(double p)
    {
        return std::clamp(p, lo, hi);
    }

    static constexpr double Qin(double p)
    {
        // A * Qin * Qin + B * Qin + (C - p) = 0 
        double const D = B * B - 4 * A * (C - p);
        return (-B - std::sqrt(D)) / 2 / A;
    }

    static constexpr double Qout(double p)
    {
        return globals::chi * std::sqrt(2 * p / globals::rho) * Sgap;
    }

private:
    static double constexpr A = -2.756;
    static double constexpr B = 48.46;
    static double constexpr C = 2771;

    static double constexpr Qtop = -B / 2 / A; // вершина квадратичной функции
    static double constexpr lo = 0.0;
    static double constexpr hi = A * Qtop * Qtop + B * Qtop + C;

    static double constexpr Sgap = 0.012;
};

namespace RK4 {
struct State
{
    double V = 0.0; // скорость судна
    double H = 0.0; // вертикальная координата центра тяжести судна
    double W = 0.0; // объем воздушной подушки
    double p = 0.0; // избыточное давление
    double Vphi = 0.0; // скорость изменения дифферента
    double phi = 0.0; // диффирент
};

struct Derivative
{
    double dV = 0.0; // скорость судна
    double dH = 0.0; // вертикальная координата центра тяжести судна
    double dW = 0.0; // объем воздушной подушки
    double dp = 0.0; // избыточное давление
    double dVphi = 0.0; // скорость изменения дифферента
    double dphi = 0.0; // диффирент

    Derivative() = default;

    Derivative(Derivative d[4])
        : dV(1.0 / 6.0 * (d[0].dV + 2.0 * (d[1].dV + d[2].dV) + d[3].dV))
        , dH(1.0 / 6.0 * (d[0].dH + 2.0 * (d[1].dH + d[2].dH) + d[3].dH))
        , dW(1.0 / 6.0 * (d[0].dW + 2.0 * (d[1].dW + d[2].dW) + d[3].dW))
        , dp(1.0 / 6.0 * (d[0].dp + 2.0 * (d[1].dp + d[2].dp) + d[3].dp))
        , dVphi(1.0 / 6.0 * (d[0].dVphi + 2.0 * (d[1].dVphi + d[2].dVphi) + d[3].dVphi))
        , dphi(1.0 / 6.0 * (d[0].dphi + 2.0 * (d[1].dphi + d[2].dphi) + d[3].dphi))
    {
    }
};

std::ostream& operator <<(std::ostream& out, State const& state)
{
    static size_t constexpr numberWidth = 10;
#define SETW std::setw(numberWidth)
    out << std::fixed << std::setprecision(3);
    out << "State[";
    out << "V=" << SETW << state.V << ", ";
    out << "H=" << SETW << state.H << ", ";
    out << "W=" << SETW << state.W << ", ";
    out << "p=" << SETW << state.p << ", ";
    out << "Vphi=" << SETW << state.Vphi << ", ";
    out << "phi=" << SETW << state.phi << ", ";
    out << "Qin=" << SETW << Compressor::Qin(state.p) << ", ";
    out << "Qout=" << SETW << Compressor::Qout(state.p);
    out << "]";
#undef SETW
    return out;
}

std::ostream& operator <<(std::ostream& out, Derivative const& deriv)
{
    static size_t constexpr numberWidth = 10;
#define SETW std::setw(numberWidth)
    out << std::fixed << std::setprecision(3);
    out << "Deriv[";
    out << "V=" << SETW << deriv.dV << ", ";
    out << "H=" << SETW << deriv.dH << ", ";
    out << "W=" << SETW << deriv.dW << ", ";
    out << "p=" << SETW << deriv.dp << ", ";
    out << "Vphi=" << SETW << deriv.dVphi << ", ";
    out << "phi=" << SETW << deriv.dphi;
    out << "]";
#undef SETW
    return out;
}

class Integrator
{
public:
    static void Integrate(State &state, double dt) {
        Derivative d[4];
        d[0] = Eval(state, 0.0, {});
        d[1] = Eval(state, 0.5 * dt, d[0]);
        d[2] = Eval(state, 0.5 * dt, d[1]);
        d[3] = Eval(state, dt, d[2]);
        Derivative comb(d);

        state.V = state.V + comb.dV * dt;
        state.H = state.H + comb.dH * dt;
        state.W = state.W + comb.dW * dt;
        state.p = state.p + comb.dp * dt;
        state.Vphi = state.Vphi + comb.dVphi * dt;
        state.phi = state.phi + comb.dphi * dt;
    
        std::cout << state << '\n';
        std::cout << comb << '\n';
        std::cout << '\n';
    }

private:
    static Derivative Eval(State const& initial, double dt, Derivative const& d)
    {
        State state;
        state.V = initial.V + d.dV * dt;
        state.H = initial.H + d.dH * dt;
        state.W = initial.W + d.dW * dt;
        state.p = initial.p + d.dp * dt;
        state.Vphi = initial.Vphi + d.dVphi * dt;
        state.phi = initial.phi + d.dphi * dt;

        Derivative output;
        output.dV = (state.p * ACV::S - ACV::m * globals::g) / ACV::m;
        output.dH = state.V;
        output.dVphi = state.p * ACV::S * ACV::l / ACV::Iz;
        output.dphi = state.Vphi;
        output.dW = ACV::S * output.dH + ACV::S * ACV::l * output.dphi;
        output.dp = globals::n * globals::pa * (Compressor::Qin(state.p) - Compressor::Qout(state.p) - output.dW) / state.W;
        return output;
    }
};
} // namespace RK4

namespace csv {
class csv_proxy
{
public:
    explicit csv_proxy(std::ostream& out)
        : mOut(out)
    {
    }

    template<typename T>
    friend std::ostream& operator <<(csv_proxy const& proxy, T const& o)
    {
        return proxy.mOut << o;
    }

    friend std::ostream& operator <<(csv_proxy const& proxy, RK4::State const& state)
    {
        proxy.mOut << state.V << ",";
        proxy.mOut << state.H << ",";
        proxy.mOut << state.W << ",";
        proxy.mOut << state.p << ",";
        proxy.mOut << state.Vphi << ",";
        proxy.mOut << state.phi << ",";
        proxy.mOut << Compressor::Qin(state.p) << ",";
        proxy.mOut << Compressor::Qout(state.p);
        return proxy.mOut << '\n';
    }

    friend std::ostream& operator <<(csv_proxy const& proxy, std::vector<std::string> const& line)
    {
        if (!line.empty()) {
            for (size_t i = 0; i < line.size() - 1; ++i) {
                proxy.mOut << line[i] << ',';
            }
            proxy.mOut << line.back();
        }
        return proxy.mOut << '\n';
    }

private:
    std::ostream& mOut;
};

struct csv_creator {} csv;

csv_proxy operator <<(std::ostream& out, csv_creator)
{
    return csv_proxy(out);
}
} // namespace csv

int main(int argc, char** argv)
{
    std::ofstream fout("./states.csv", std::ofstream::out);
    std::vector<std::string> const stateHeader = {"V", "H", "W", "p", "Vphi", "phi", "Qin", "Qout"};
    fout << csv::csv << stateHeader;

    RK4::State state;
    state.W = ACV::S * ACV::d / 2;

    double const dt = 0.05;
    RK4::Derivative deriv;
    for (double t = 0.0; t < 5; t += dt) {
        deriv.dV = (state.p * ACV::S - ACV::m * globals::g) / ACV::m;
        deriv.dH = state.V;
        deriv.dVphi = state.p * ACV::S * ACV::l / ACV::Iz;
        deriv.dphi = state.Vphi;
        deriv.dW = ACV::S * deriv.dH + ACV::S * ACV::l * deriv.dphi;
        deriv.dp = globals::n * globals::pa * (Compressor::Qin(state.p) - Compressor::Qout(state.p) - deriv.dW) / state.W;
        
        state.V += deriv.dV * dt;
        state.H += deriv.dH * dt;
        state.W += deriv.dW * dt;
        state.p += deriv.dp * dt;
        state.Vphi += deriv.dVphi * dt;
        state.phi += deriv.dphi * dt;
        state.p = Compressor::Clamp(state.p);
        fout << csv::csv << state;
    }

    fout.close();

    return EXIT_SUCCESS;
}