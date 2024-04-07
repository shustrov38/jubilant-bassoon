#include <cassert>

#include "constants.h"
#include "acv.h"
#include "tqdm.h"

int main()
{
    phisics::acv::ACV acv;
    acv.Init();

    double const eps = 1e-8;


    double const dt = phisics::globals::dt;
    double const T_min = 0.0;
    double const T_max = 100.0;
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