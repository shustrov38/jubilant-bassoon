#pragma once

#include <array>
#include <memory>

#include "constants.h"

namespace phisics::acv {
using utils::Vector;

class ACV {
    struct Segment;
    friend struct Segment;

    using Ptr = ACV *;

    struct SegmentSummary {
        Double W = 0;
        Double F_wave = 0;
        Double M_contact = 0;
        Double S_gap = 0;
        Double dy_wave = 0;

        friend SegmentSummary &operator +=(SegmentSummary &lhs, SegmentSummary const& rhs)
        {
            lhs.W += rhs.W;
            lhs.F_wave += rhs.F_wave;
            lhs.M_contact += rhs.M_contact;
            lhs.S_gap += rhs.S_gap;
            lhs.dy_wave += rhs.dy_wave;
            return lhs;
        }

        friend SegmentSummary operator +(SegmentSummary lhs, SegmentSummary const& rhs)
        {
            return lhs += rhs;
        }
    };

    struct Segment {
        Segment() = default;
        explicit Segment(Ptr acv, Int index);

        explicit operator SegmentSummary() const
        {
            return { W, F_contact, M_contact, S_gap, dy_wave };
        }

        void Update();

        inline bool IsFirstOrLastSegment() const;
        inline bool HasContactWithWave() const;
        inline Double GapHeight() const;

        // индекс сегмента ВП
        Int index;

        // знаковое смещение сегмента относительно центра тяжести судна
        Double x;

        // координата верхней части сегмента ВП
        Vector c;
        // объем сечения
        Double W;

        // высота столба воздуха в сегменте
        Double d;

        // пятно контакта
        Double S_wash;
        // скорость сегмента
        Double V_scalar;
        // сила действующая на пятно контакта
        Double F_contact; 
        // момент силы действующей на пятно контакта
        Double M_contact;

        // ширина зазора в сегменте
        Double S_gap;

        // изменение высоты волны под сегментом
        Double dy_wave;

        Ptr acv;
    };

public:
    void Init();
    void Update(double dt);

private:
    SegmentSummary ProcessSegments();

    // все N сечений судна
    std::array<Segment, N> mSegments;

    // координата судна
    Vector c;

    // объем ВП
    Double W;

    // избыточное давление ВП
    Double p;
    Double p_qs;
    Double p_damp;
    
    // тангаж
    Double phi;

    // вертикальное ускорение
    Double dV_y__dt;
    
    // вектор скорости
    Vector V;
    // вектор угловой скорости
    Vector w;

    // расход возуха в ВП
    Double Q_in;
    // расход возуха из ВП
    Double Q_out;

    // текущее время симуляции (для рассчета сдвига волны)
    Double t;
    // время на предыдущем шаге симуляции 
    Double t_prev;

    // счетчик для записи в файл
    uint64_t iteration {0};
};
} // namespace phisics::acv