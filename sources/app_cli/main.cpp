#include <cassert>

#include "acv_phisics.h"
#include "tqdm.h"

int main()
{
    phisics::ACV acv;

    double const eps = 1e-8;

    double const dt = 1e-5;
    double const T_min = 0.0;
    double const T_max = 600.0;
    assert(T_min < T_max);

    size_t const total = (T_max - T_min + dt - eps) / dt;
    TQDM tqdm(total, 50);

    size_t i = 0;
    for (double t = T_min; t <= T_max; t += dt) {
        acv.Update(dt);
        tqdm.Update(i++);
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}