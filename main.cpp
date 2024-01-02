#include <cmath>
#include <iostream>
#include <iomanip>

namespace globals {
inline double constexpr g = 9.8; // ускорение свободного падения 
inline double constexpr n = 1.4; // показатель политропы
inline double constexpr chi = 1; // коэффициент истечения
inline double constexpr pa = 1e5; // атмосферное давление
inline double constexpr rho = 1.2690; // плотность воздуха
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

namespace compressor {
double Qin(double p) {
    // -2.756 * Qin * Qin + 48.46 * Qin + 2771 - p = 0 
    double const A = -2.756;
    double const B = 48.46;
    double const C = 2771 - p;
    double const D = B * B - 4 * A * C;
    return (-B - std::sqrt(D)) / 2 / A;
}

double Qout(double p) {
    static double constexpr Sgap = 0.012;
    return globals::chi * std::sqrt(2 * p / globals::rho) * Sgap;
}
} // namespace compressor

namespace RK4 {
struct State
{
    double V; // скорость судна
    double H; // вертикальная координата центра тяжести судна
    double W; // объем воздушной подушки
    double p; // избыточное давление
    double Vphi; // скорость изменения дифферента
    double phi; // диффирент
};

struct Derivative
{
    double dV; // скорость судна
    double dH; // вертикальная координата центра тяжести судна
    double dW; // объем воздушной подушки
    double dp; // избыточное давление
    double dVphi; // скорость изменения дифферента
    double dphi; // диффирент
};

std::ostream& operator <<(std::ostream& out, State const& state)
{
    static size_t constexpr numberWidth = 10;
    out << std::fixed << std::setprecision(3);
    out << "State[";
    out << "V=" << std::setw(numberWidth) << state.V << ", ";
    out << "H=" << std::setw(numberWidth) << state.H << ", ";
    out << "W=" << std::setw(numberWidth) << state.W << ", ";
    out << "p=" << std::setw(numberWidth) << state.p << ", ";
    out << "Vphi=" << std::setw(numberWidth) << state.Vphi << ", ";
    out << "phi=" << std::setw(numberWidth) << state.phi;
    out << "]";
    return out;
}

std::ostream& operator <<(std::ostream& out, Derivative const& deriv)
{
    static size_t constexpr numberWidth = 10;
    out << std::fixed << std::setprecision(3);
    out << "Derivative[";
    out << "dV=" << std::setw(numberWidth) << deriv.dV << ", ";
    out << "dH=" << std::setw(numberWidth) << deriv.dH << ", ";
    out << "dW=" << std::setw(numberWidth) << deriv.dW << ", ";
    out << "dp=" << std::setw(numberWidth) << deriv.dp << ", ";
    out << "dVphi=" << std::setw(numberWidth) << deriv.dVphi << ", ";
    out << "dphi=" << std::setw(numberWidth) << deriv.dphi;
    out << "]";
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

        double dVdt = 1.0 / 6.0 * (d[0].dV + 2.0 * (d[1].dV + d[2].dV) + d[3].dV);
        double dHdt = 1.0 / 6.0 * (d[0].dH + 2.0 * (d[1].dH + d[2].dH) + d[3].dH);
        double dWdt = 1.0 / 6.0 * (d[0].dW + 2.0 * (d[1].dW + d[2].dW) + d[3].dW);
        double dpdt = 1.0 / 6.0 * (d[0].dp + 2.0 * (d[1].dp + d[2].dp) + d[3].dp);
        double dVphidt = 1.0 / 6.0 * (d[0].dVphi + 2.0 * (d[1].dVphi + d[2].dVphi) + d[3].dVphi);
        double dphidt = 1.0 / 6.0 * (d[0].dphi + 2.0 * (d[1].dphi + d[2].dphi) + d[3].dphi);

        state.V = state.V + dVdt * dt;
        state.H = state.H + dHdt * dt;
        state.W = state.W + dWdt * dt;
        state.p = state.p + dpdt * dt;
        state.Vphi = state.Vphi + dVphidt * dt;
        state.phi = state.phi + dphidt * dt;
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
        output.dp = globals::n * globals::pa * (compressor::Qin(state.p) - compressor::Qout(state.p) - output.dW) / state.W;
        std::cout << output << '\n';
        return output;
    }
};
} // namespace RK4


int main(int argc, char** argv)
{
    RK4::State state;
    state.W = ACV::S * ACV::d;    
    
    double const dt = 0.05;
    for (double t = 0.0; t < 0.5; t += dt) {
        RK4::Integrator::Integrate(state, dt);
    }

    return EXIT_SUCCESS;
}