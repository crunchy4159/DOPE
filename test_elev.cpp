#include "dope/dope_api.h"
#include <cmath>
#include <iostream>


int main() {
    DOPE_Init();

    BulletProfile bullet = {};
    bullet.muzzle_velocity_ms = 850.0f;
    bullet.bc = 0.5f;
    bullet.mass_grains = 175.0f;
    bullet.drag_model = DOPE_DRAG_G1;
    DOPE_SetBulletProfile(&bullet);

    DOPE_SetWindManual(0.0f, 0.0f);

    SensorFrame frame = {};
    frame.lrf_range_m = 500.0f;
    frame.lrf_valid = true;

    for (float elev = 0.0f; elev <= 500.0f; elev += 100.0f) {
        frame.target_elevation_m = elev;
        DOPE_Update(&frame);

        FiringSolution sol;
        DOPE_GetSolution(&sol);

        float drop_inches =
            std::fabs(sol.hold_elevation_moa) * (3.14159f / 180.0f / 60.0f) * 500.0f * 39.3701f;
        std::cout << "Elev: " << elev << "m, Hold MOA: " << sol.hold_elevation_moa
                  << ", Drop In: " << drop_inches << std::endl;
    }

    return 0;
}
