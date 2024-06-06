#pragma once

#include <cmath>

#include "../common/utils.h"
#include "../common/mytypes.h"

namespace phisics {
namespace globals {
// ускорение свободного падения 
inline Double constexpr g = 9.8;

// показатель политропы
inline Double constexpr n = 1.4;

// коэффициент истечения
inline Double constexpr chi = 1;

// атмосферное давление
inline Double constexpr p_a = 1e5;

// плотность воздуха
inline Double constexpr rho_a = 1.2690;

// плотность воды
inline Double constexpr rho_w = 1000.0;

// константа для сравления чисел с плавающей точкой
inline Double constexpr eps = 1e-7;

// шаг времени симуляции
inline Double constexpr dt = 1e-7;
} // namespace globals

namespace wave {
// амплитуда волны
inline Double constexpr A = 0.5;

// высота волны
inline Double constexpr h = 2 * A;

// длина волны
inline Double constexpr lambda = 10 * h;

// волновой вектор
inline Double constexpr k = 2 * M_PI / lambda;

// скорость волны
inline Double constexpr c = utils::Sqrt(globals::g / k);

// циклическая частота
inline Double constexpr omega = c * k;
} // namespace wave

namespace acv {
// масса
inline Double constexpr m = 16000;

// расстояние от центра давления до центра тяжести, вдоль OX
inline Double constexpr l_AC = 0;

// длина воздушной подушки
inline Double constexpr L = 15;

// ширина воздушной подушки
inline Double constexpr b = 6;

// максимальная высота воздушной подушки
inline Double constexpr d_max = 0.7;

// площадь воздушной подушки
inline Double constexpr S = L * b;

// момент инерции судна
inline Double constexpr I_z = 250000;

// параметр влияния волны на судно
inline Double constexpr theta = 0.005;

// постоянная скорость буксира
inline Double constexpr V_x = 20;

// количество сечений ВП
inline Int    constexpr N = 80;

// ширина сечения ВП
inline Double constexpr delta_L = L / N; 
} // namespace acv

namespace damp {
// параметр A экспоненциальной функции A + B exp {C * x}
inline Double constexpr A = 31253.553;
// параметр B экспоненциальной функции A + B exp {C * x}
inline Double constexpr B = 6930673.352;
// параметр C экспоненциальной функции A + B exp {C * x}
inline Double constexpr C = -1025.178;
} // namespace damp

namespace compressor {
// параметр A квадратичной функции (A q^2 + B q + C = p)
inline Double constexpr A = -2.756;
// параметр B квадратичной функции (A q^2 + B q + C = p)
inline Double constexpr B = 48.46;
// параметр C квадратичной функции (A q^2 + B q + C = p)
inline Double constexpr C = 2771;

// минимальный нагнетаемый объем в секунду
inline Double constexpr Q_min = 0.0;

// максимальный нагнетаемый объем в секунду (вершина квадратичной функции)
inline Double constexpr Q_max = -B / 2 / A;

// минимальное избыточное давление ВП
inline Double constexpr p_min = 0.0;

// максимальное избыточное давление ВП
inline Double constexpr p_max = A * Q_max * Q_max + B * Q_max + C;
} // namespace compressor

namespace options {
// период сохранения дампа в csv файл
inline uint64_t constexpr period = static_cast<uint64_t>(1.0 / globals::dt);
} // namespace options
} // namespace::phisics