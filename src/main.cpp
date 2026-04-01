/**
 * @file main.cpp
 * @brief Minimal embedded entry point that boots the DOPE engine.
 *
 * Platform/application-specific sensor plumbing is expected to be added by
 * firmware code around this skeleton.
 */

// Placeholder for ESP32 application main
// The DOPE library lives in lib/dope/ and is platform-agnostic.
// This file is only compiled for the esp32sim environment.

#include "dope/dope_api.h"
#include "dope/dope_types.h"
#include "../lib/dope/src/solver/solver.h"
#include "../lib/dope/src/atmo/atmosphere.h"
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

#if defined(DOPE_PLATFORM_NATIVE)

// Very simple argument parser
std::string getCmdOption(char ** begin, char ** end, const std::string & option) {
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) return std::string(*itr);
    return "";
}

bool cmdOptionExists(char** begin, char** end, const std::string& option) {
    return std::find(begin, end, option) != end;
}

int main(int argc, char* argv[]) {
    if (!cmdOptionExists(argv, argv + argc, "--headless")) {
        DOPE_Init();
        return 0;
    }

    SolverParams p = {};
    p.bc = std::stof(getCmdOption(argv, argv + argc, "--bc"));
    p.drag_model = static_cast<DragModel>(std::stoi(getCmdOption(argv, argv + argc, "--drag_model")));
    p.muzzle_velocity_ms = std::stof(getCmdOption(argv, argv + argc, "--mv"));
    p.bullet_mass_kg = std::stof(getCmdOption(argv, argv + argc, "--mass"));
    p.bullet_length_m = std::stof(getCmdOption(argv, argv + argc, "--length"));
    p.caliber_m = std::stof(getCmdOption(argv, argv + argc, "--caliber"));

    float temp = std::stof(getCmdOption(argv, argv + argc, "--temp"));
    float press = std::stof(getCmdOption(argv, argv + argc, "--pressure"));
    float hum = std::stof(getCmdOption(argv, argv + argc, "--humidity"));
    float range_max = std::stof(getCmdOption(argv, argv + argc, "--range"));
    float crosswind = cmdOptionExists(argv, argv + argc, "--crosswind") ? std::stof(getCmdOption(argv, argv + argc, "--crosswind")) : 0.0f;

    Atmosphere atmo;
    atmo.init();
    atmo.updateFromBaro(press, temp, hum);

    // Optional: simulate a shot event and feed it to the DOPE engine.
    if (cmdOptionExists(argv, argv + argc, "--simulate-shot")) {
        DOPE_Init();
        SensorFrame f = {};
        f.timestamp_us = 1000000ULL; // example timestamp
        f.shot_fired = true;
        f.shot_timestamp_us = f.timestamp_us;
        f.shot_ambient_temp_c = temp;
        DOPE_Update(&f);
        std::cout << "Simulated shot event sent to DOPE_Update()\n";
        return 0;
    }

    p.air_density = atmo.getAirDensity();
    p.speed_of_sound = atmo.getSpeedOfSound();
    p.bc = atmo.correctBC(p.bc);
    
    p.sight_height_m = 0.0f;
    p.drag_reference_scale = 1.0f;
    p.target_range_m = range_max;
    p.launch_angle_rad = 0.0f;
    p.crosswind_ms = crosswind;
    p.headwind_ms = 0.0f;
    p.spin_drift_enabled = false;
    p.coriolis_enabled = false;

    BallisticSolver solver;
    solver.init();
    SolverResult res = solver.integrate(p, true);

    if (!res.valid) {
        std::cerr << "Integration failed!" << std::endl;
        return 1;
    }

    std::cout << "{";
    std::cout << "\"range\": [";
    for(int i = 0; i <= (int)range_max; i++) {
        std::cout << solver.getPointAt(i)->range_m;
        if(i != (int)range_max) std::cout << ",";
    }
    std::cout << "], \"drop\": [";
    for(int i = 0; i <= (int)range_max; i++) {
        std::cout << solver.getPointAt(i)->drop_m;
        if(i != (int)range_max) std::cout << ",";
    }
    std::cout << "], \"velocity\": [";
    for(int i = 0; i <= (int)range_max; i++) {
        std::cout << solver.getPointAt(i)->velocity_ms;
        if(i != (int)range_max) std::cout << ",";
    }
    std::cout << "], \"tof\": [";
    for(int i = 0; i <= (int)range_max; i++) {
        std::cout << solver.getPointAt(i)->tof_s;
        if(i != (int)range_max) std::cout << ",";
    }
    std::cout << "], \"windage\": [";
    for(int i = 0; i <= (int)range_max; i++) {
        std::cout << solver.getPointAt(i)->windage_m;
        if(i != (int)range_max) std::cout << ",";
    }
    std::cout << "]}";

    return 0;
}
#else
extern "C" void app_main(void) {
    DOPE_Init();
    // Application layer would feed SensorFrames here
}
#endif
