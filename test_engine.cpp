#include <iostream>
#include <cmath>
#include ""bce_api.h""
#include ""bce_types.h""
#include ""bce_math_utils.h""

int main() {
    BCE_Init();

    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    // Firing North (0 deg)
    // Wind from East (90 deg), 10 m/s. Wind blows Right-To-Left (LEFTWARD).
    BCE_SetWindManual(10.0f, 90.0f);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = {};
        f.timestamp_us = (uint64_t)(i + 1) * 10000;
        f.accel_z = 9.81f; 
        f.imu_valid = true;
        f.baro_pressure_pa = 101325.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;
        f.mag_valid = false; 

        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    FiringSolution sol = {};
    BCE_GetSolution(&sol);

    std::cout << ""Hold Windage: "" << sol.hold_windage_moa << "" MOA\n"";
    std::cout << ""Target View dot maps center.x + hold (Positive=RIGHT): "" << (sol.hold_windage_moa > 0.0f ? ""RIGHT"" : ""LEFT"") << ""\n"";
    
    const float MOA_TO_RAD = 3.14159265f / 180.0f / 60.0f;
    const float lateral_m = sol.hold_windage_moa * MOA_TO_RAD * sol.range_m;
    std::cout << ""lateral_m (Aim Dir Offset): "" << lateral_m << ""\n"";
    std::cout << ""Top Down aim_dir maps X + lateral_m (Positive=RIGHT): "" << (lateral_m > 0.0f ? ""RIGHT"" : ""LEFT"") << ""\n"";
    
    return 0;
}
