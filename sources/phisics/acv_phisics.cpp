#include "acv_phisics.h"

namespace options {
DEFINE_OPTION(CSV_LOG)
} // namespace options

static csv::Writer<1000ul> gWriter(options::GET_OPTION(CSV_LOG).GetValue());

namespace phisics {
#define SQ(x) (x) * (x)

ACV::Segment::Segment(int index, double W)
    : W(W)
    , x(L / 2 - delta_L * (index - 0.5))
    , isFirstOrLast(index == 1 || index == N) 
{
}

constexpr void ACV::Segment::UpdateParams(
    Vector const& new_c,
    Vector const& new_V,
    Vector const& new_w,
    double new_phi,
    double new_t
)
{
    phi = new_phi;
    c = new_c;
    c.x += x; // применить смещение
    c = detail::RotatePointXY(new_c, c, phi);
    V = new_V;
    w = new_w;
    t_prev = t;
    t = new_t;
    UpdateState();
}

constexpr void ACV::Segment::UpdateState()
{
    d = std::max(0.0, c.y - Wave::Y(c.x, t));
    W = d * b * delta_L;
    S_wash = (HasContact() ? delta_L * b : 0);
    V_scalar = std::sqrt(SQ(V.x) + SQ(V.y + w.z * (x + l_AC)));
    F_contact = theta * globals::rho_w * SQ(V_scalar) / 2 * S_wash;
    M_contact = F_contact * (x + l_AC);
    S_gap = GapHeight() * (2 * delta_L + IsFirstOrLast() * b);
    V_y = Wave::Y(c.x, t_prev) - Wave::Y(c.x, t);
}

constexpr bool ACV::Segment::HasContact() const
{
    return c.y <= Wave::Y(c.x, t);
}
constexpr double ACV::Segment::GapHeight() const
{
    Vector lowestPoint(c.x, c.y - d_max, c.z);
    lowestPoint = detail::RotatePointXY(c, lowestPoint, phi);
    return std::max(0.0, lowestPoint.y - Wave::Y(c.x, t));
}
constexpr bool ACV::Segment::IsFirstOrLast() const
{
    return isFirstOrLast;
}

ACV::ACV()
{
    double const d_gap = 0.01; // высота зазора
    W = S * (d_max + d_gap);
    c = {0, d_max + d_gap, 0};
    // c = {0, 0.4, 0};

    p_qs = m * globals::g / S;
    p_damp = 0;
    p = p_qs;

    phi = 0; // судно стоит горизонтально

    V = {V_x, 0, 0}; // присутствует только скорость буксира
    w = {0, 0, 0}; // угловой скорости нет вообще

    t = 0;

    for (int i = 0; i < N; ++i) {
        segments[i] = Segment(i + 1, W / N);
    }

    gWriter.WriteRow("H", "W", "phi", "p", "Q_in", "Q_out");
}

double D(double S_gap, double S)
{
    double const A = 31253.553;
    double const B = 6930673.352;
    double const C = -1025.178;
    double const x = std::max(0.001, S_gap / S);
    return A + B * std::exp(C * x);
}

void ACV::Update(double dt)
{
    auto const [_W, F_wave, M_contact, S_gap, V_y_wave] = CalcSegmentsCharacteristics(c, V, w, phi, t);

    double const dW__dt = _W - W;
    W = _W;

    Q_in = Compressor::Q_in(p);
    Q_out = Compressor::Q_out(p, S_gap);

    double const F_AC = p * S;
    double const F_damp = p_damp * S;
    double const F_attr = m * globals::g;
    double const dV_y__dt = (F_AC - F_attr + F_damp + F_wave) / m;
    double const V_damp = V.y - V_y_wave / dt;

    double const dp_qs__dt = globals::n * globals::p_a * (Q_in - Q_out - dW__dt) / W;
    p_qs = Compressor::Clamp(p_qs + dp_qs__dt * dt);
    p_damp = D(S_gap, S) * globals::rho_a * (-Q_in / S * V_damp + 0.5 * SQ(V_damp)) * S;

    double const dw_z__dt = (p * S * l_AC - M_contact) / I_z;

    V.y += dV_y__dt * dt;
    w.z += dw_z__dt * dt;

    p = p_qs; // p_damp не включено

    c += V * dt;
    phi += w.z * dt;

    t += dt;

    gWriter.WriteRow(c.y, W, phi, p, Q_in, Q_out);
}

ACV::ACVSummary ACV::CalcSegmentsCharacteristics(
    Vector const& new_c, 
    Vector const& new_V, 
    Vector const& new_w,
    double new_phi,
    double new_t
)
{
    ACVSummary result;
    for (auto &segment : segments) {
        segment.UpdateParams(new_c, new_V, new_w, new_phi, new_t);
        result.W += segment.W;
        result.F_wave += segment.F_contact;
        result.M_contact += segment.M_contact;
        result.S_gap += segment.S_gap;
        result.V_y_wave += segment.V_y;
    }
    result.V_y_wave /= N; // хотим вычислять среднее значение скорости
    return result;
}
} // namespace phisics