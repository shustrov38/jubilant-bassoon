#include "acv_phisics.h"

namespace options {
DEFINE_OPTION(CSV_LOG)
} // namespace options

static csv::Writer gWriter(options::GET_OPTION(CSV_LOG).GetValue());

namespace phisics {
#define SQ(x) (x) * (x)

ACV::Segment::Segment(int index, double W)
    : W(W)
    , x(L / 2 - delta_L * (index - 0.5))
    , isFirstOrLast(index == 1 || index == N) 
{
}

constexpr void ACV::Segment::UpdateParams(Vector const& new_c, Vector const& new_V, Vector const& new_w, double phi)
{
    c = new_c;
    c.x += x; // применить смещение
    c = detail::RotatePointXY(new_c, c, phi);
    V = new_V;
    w = new_w;
    UpdateState();
}

constexpr void ACV::Segment::UpdateState()
{
    d = std::max(0.0, c.y - wave::y(c.x));
    W = d * b * delta_L;
    S_wash = (HasContact() ? delta_L * b : 0);
    V_scalar = std::sqrt(SQ(V.x) + SQ(V.y + w.z * (x + l_AC)));
    F_contact = theta * globals::rho_w * SQ(V_scalar) / 2 * S_wash;
    M_contact = F_contact * (x + l_AC);
    S_gap = GapHeight() * (2 * delta_L + IsFirstOrLast() * b);
}

constexpr bool ACV::Segment::HasContact() const
{
    return c.y <= wave::y(c.x);
}
constexpr double ACV::Segment::GapHeight() const
{
    return std::max(0.0, c.y - d_max - wave::y(c.x));
}
constexpr bool ACV::Segment::IsFirstOrLast() const
{
    return isFirstOrLast;
}

ACV::ACV()
{
    double const d_gap = 0.01; // высота зазора
    W = S * (d_max +  d_gap);
    c = {0, d_max +  d_gap, 0};

    p_qs = m * globals::g / S;
    p_damp = 0;
    p = p_qs + p_damp;

    phi = 0; // судно стоит горизонтально

    V = {V_x, 0, 0}; // присутствует только скорость буксира
    w = {0, 0, 0}; // угловой скорости нет вообще

    for (int i = 0; i < N; ++i) {
        segments[i] = Segment(i + 1, W / N);
    }

    gWriter.WriteRow("H", "W", "phi", "p", "Q_in", "Q_out", "dH/dt", "p_qs", "p_damp", "S_gap", "S");
}

constexpr double D(double S_gap, double S)
{
    double constexpr A = 31253.553;
    double constexpr B = 6930673.352;
    double constexpr C = -1025.178;
    double const x = std::max(0.001, S_gap / S);
    return A + B * std::exp(C * x);
}

void ACV::Update(double dt)
{
    auto [_W, F_wave, M_contact, S_gap] = CalcSegmentsCharacteristics(c, V, w, phi);

    double dWdt = _W - W;
    W = _W;

    Q_in = Compressor::Q_in(p);
    Q_out = Compressor::Q_out(p, S_gap);

    double const F_AC = p * S;
    double const F_damp = p_damp * S;
    double const F_attr = m * globals::g;
    double dV_y__dt = (F_AC - F_attr + F_damp + F_wave) / m;

    double dp_qs__dt = globals::n * globals::p_a * (Q_in - Q_out - dWdt) / W;
    p_qs = Compressor::Clamp(p_qs + dp_qs__dt * dt);
    p_damp = D(S_gap, S) * globals::rho_a * (-Q_in / S * V.y + 0.5 * SQ(V.y)) * S;

    double dw_z__dt = (p * S * l_AC - M_contact) / I_z;

    V.y += dV_y__dt * dt;
    w.z += dw_z__dt * dt;

    p = p_qs; // p_damp is not included

    c += V * dt;
    phi += w.z * dt;

    gWriter.WriteRow(c.y, W, phi, p, Q_in, Q_out, dV_y__dt, p_qs, p_damp * dt, S_gap, S);
}

std::tuple<double, double, double, double> ACV::CalcSegmentsCharacteristics(
    Vector const& new_c, 
    Vector const& new_V, 
    Vector const& new_w,
    double phi
)
{
    double _W = 0;
    double _F_wave = 0;
    double _M_contact = 0;
    double _S_gap = 0;
    for (size_t i = 0; i < N; ++i) {
        segments[i].UpdateParams(new_c, new_V, new_w, phi);
        _W += segments[i].W;
        _F_wave += segments[i].F_contact;
        _M_contact += segments[i].M_contact;
        _S_gap += segments[i].S_gap;
    }
    return {_W, _F_wave, _M_contact, _S_gap};
}
} // namespace phisics