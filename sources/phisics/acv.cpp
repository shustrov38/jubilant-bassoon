#include "acv.h"
#include "wave.h"
#include "damp.h"
#include "constants.h"
#include "compressor.h"

#include "../common/envvar.h"
#include "../common/csv_utils.h"

namespace options {
DEFINE_OPTION(CSV_LOG)
} // namespace options

static csv::Writer gWriter(options::GET_OPTION(CSV_LOG).GetValue());

namespace phisics::acv {
#define SQ(x) (x) * (x)

ACV::Segment::Segment(Ptr acv, Int index)
    : index(index)
    , x(L / 2 - delta_L * (index - 0.5))
    , W(acv->W)
    , acv(acv)
{
}

void ACV::Segment::Update()
{
    // координата сегмента имеет смещение относительно координаты судна,
    // также нужно наклонить сегмент относитльно координаты центря тяжести
    c = Vector(acv->c.x + x, acv->c.y, acv->c.z);
    c = utils::RotatePointXY(acv->c, c, acv->phi); 

    d = std::max(0.0, c.y - wave::Height(c.x, acv->t));
    W = d * b * delta_L;
    S_wash = HasContactWithWave() ? b * delta_L : 0.0;
    V_scalar = std::sqrt(SQ(acv->V.x) + SQ(acv->V.y + acv->w.z * (x * l_AC)));
    F_contact = theta * globals::rho_w * SQ(V_scalar) / 2 * S_wash;
    M_contact = F_contact * (x + l_AC);
    S_gap = GapHeight() * (2 * delta_L + IsFirstOrLastSegment() * b);
    dy_wave = wave::Height(c.x, acv->t_prev) - wave::Height(c.x, acv->t);
}

bool ACV::Segment::IsFirstOrLastSegment() const
{
    return index == 1 || index == N;
}

bool ACV::Segment::HasContactWithWave() const
{
    return c.y <= wave::Height(c.x, acv->t);
}

Double ACV::Segment::GapHeight() const
{
    Vector lowestPoint(c.x, c.y - d_max, c.z);
    // lowestPoint = utils::RotatePointXY(c, lowestPoint, acv->phi);
    return std::max(0.0, lowestPoint.y - wave::Height(c.x, acv->t));
}

void ACV::Init()
{
    // стартовая высота зазора в 5 мм
    Double const d_gap = 0.005;

    Double fraction = 0.5;

    c = {0, fraction * d_max + d_gap, 0};
    W = S * c.y;

    p_qs = m * globals::g / S;
    p_damp = 0;
    p = p_qs;

    // судно стоит горизонтально
    phi = 0;

    // присутствует только скорость буксира
    V = {V_x, 0, 0};
    // угловой скорости нет вообще
    w = {0, 0, 0};

    t_prev = 0;
    t = 0;

    for (Int i = 0; i < N; ++i) {
        mSegments[i] = Segment(this, i + 1);
    }

    std::vector<std::string> header = {"H", "dV_y/dt", "phi", "p", "Q_in", "Q_out"};
    gWriter.Delimiter('\t').Header(header);
}

void ACV::Update(Double dt)
{
    t_prev = t;
    t += dt;

    auto const [_W, F_wave, M_contact, S_gap, dy_wave] = ProcessSegments();

    Double const V_damp = V.y - dy_wave / dt;

    Double const dW__dt = _W - W;
    W = _W;

    Q_in = compressor::Q_in(p);
    Q_out = compressor::Q_out(p, S_gap);

    Double const F_AC = p * S;
    Double const F_damp = p_damp * S;
    Double const F_attr = m * globals::g;
    dV_y__dt = (F_AC - F_attr + F_damp + F_wave) / m;

    Double const dp_qs__dt = globals::n * globals::p_a * (Q_in - Q_out - dW__dt) / W;
    p_qs = compressor::Clamp(p_qs + dp_qs__dt * dt);
    p_damp = damp::Coef(S_gap, S) * globals::rho_a * (-Q_in / S * V_damp + SQ(V_damp) / 2) * S;

    Double const dw_z__dt = (p * S * l_AC - M_contact) / I_z;

    V.y += dV_y__dt * dt;
    w.z += dw_z__dt * dt;

    // p_damp не включаем в давление ВП
    p = p_qs;

    c += V * dt;
    phi += w.z * dt;

    phi = std::clamp(phi, -0.174533, 0.174533);

    if (iteration == options::period) [[unlikely]] {
        gWriter.WriteRow(c.y, dV_y__dt, phi, p, Q_in, Q_out);
        iteration = 0;
    }

    ++iteration;
}

ACV::SegmentSummary ACV::ProcessSegments()
{
    SegmentSummary result;
    for (Int i = 0; i < N; ++i) {
        mSegments[i].Update();
        result += static_cast<SegmentSummary>(mSegments[i]);
    }
    result.dy_wave /= N; // хотим знать усредненное изменение высоты волны
    return result;
}
} // namespace phisics::acv