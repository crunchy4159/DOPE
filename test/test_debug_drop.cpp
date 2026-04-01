// Debug test: reproduce drop for .308 Win 168gr Federal preset and log internals
#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "dope/dope_math_utils.h"
#include "test_helpers_v2.h"
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static AmmoDatasetV2 MakeDatasetFromJsonPreset(const std::string& path, const std::string& name) {
    AmmoDatasetV2 d = {};
    std::ifstream ifs(path);
    if (!ifs.good()) return d;
    json j; ifs >> j;
    json arr = j;
    if (j.is_object() && j.contains("ammo_v2") && j["ammo_v2"].is_array()) arr = j["ammo_v2"];
    if (!arr.is_array()) return d;
    for (const auto &entry : arr) {
        if (!entry.is_object()) continue;
        if (!entry.contains("name")) continue;
        const std::string entry_name = entry["name"].get<std::string>();
        int entry_family_count = 0;
        if (entry.contains("trajectory_families") && entry["trajectory_families"].is_array()) entry_family_count = (int)entry["trajectory_families"].size();
        fprintf(stderr, "Parsed cartridge entry: '%s' families=%d\n", entry_name.c_str(), entry_family_count);
        if (entry_name != name) continue;

        // Baseline env
        if (entry.contains("environmental_conditions") && entry["environmental_conditions"].is_object()) {
            const auto &env = entry["environmental_conditions"];
            if (env.contains("temperature_f")) {
                float tf = env["temperature_f"].get<float>();
                d.baseline_temperature_c = (tf - 32.0f) * 5.0f / 9.0f;
            }
            // Expect explicit Pascals in v2 data; default to standard pressure if absent
            d.baseline_pressure_pa = env.value("pressure_pa", 101325.0f);
            if (env.contains("humidity_pct")) d.baseline_humidity = env.value("humidity_pct", 0.0f) / 100.0f;
            d.baseline_convention = BaselineConvention::ICAO;
        }

        // velocities
        d.num_velocity_points = 0;
        if (entry.contains("velocity_profile") && entry["velocity_profile"].is_array()) {
            for (const auto &vp : entry["velocity_profile"]) {
                if (d.num_velocity_points >= DOPE_MAX_TABLE_POINTS) break;
                float dist = vp.value("distance_m", 0.0f);
                float v = vp.value("velocity_ms", 0.0f);
                d.velocity_by_range[d.num_velocity_points].range_m = dist;
                d.velocity_by_range[d.num_velocity_points].value = v;
                d.num_velocity_points++;
            }
        }
        if (d.num_velocity_points > 0) d.muzzle_velocity_ms = d.velocity_by_range[0].value;

        // trajectories
        d.num_trajectories = 0;
        if (entry.contains("trajectory_families") && entry["trajectory_families"].is_array()) {
            int fi = 0;
            for (const auto &fam : entry["trajectory_families"]) {
                if (fi >= DOPE_MAX_TRAJECTORY_FAMILIES) break;
                d.trajectories[fi].zero_range_m = fam.value("zero_range_m", 0.0f);
                d.trajectories[fi].num_points = 0;
                if (fam.contains("trajectory_profile") && fam["trajectory_profile"].is_array()) {
                    int pi = 0;
                    for (const auto &tp : fam["trajectory_profile"]) {
                        if (pi >= DOPE_MAX_TABLE_POINTS) break;
                        d.trajectories[fi].points[pi].range_m = tp.value("distance_m", 0.0f);
                        d.trajectories[fi].points[pi].drop_m = tp.value("trajectory_m", 0.0f);
                        pi++;
                    }
                    d.trajectories[fi].num_points = pi;
                }
                d.trajectories[fi].cached_table_present = false;
                fi++;
            }
            d.num_trajectories = fi;
            fprintf(stderr, "-> Matched cartridge '%s' with %d trajectories\n", entry["name"].get<std::string>().c_str(), d.num_trajectories);
        }

        // wind drift
        d.num_wind_drift_points = 0;
        if (entry.contains("wind_drift_profile") && entry["wind_drift_profile"].is_array()) {
            for (const auto &wd : entry["wind_drift_profile"]) {
                if (d.num_wind_drift_points >= DOPE_MAX_TABLE_POINTS) break;
                d.wind_drift_by_range[d.num_wind_drift_points].range_m = wd.value("distance_m", 0.0f);
                d.wind_drift_by_range[d.num_wind_drift_points].value = wd.value("wind_drift_m", 0.0f);
                d.num_wind_drift_points++;
            }
        }

        // other scalar fields
        d.bc = entry.contains("ballistic_coefficients") && entry["ballistic_coefficients"].contains("G1")
                   ? entry["ballistic_coefficients"]["G1"].get<float>()
                   : 0.0f;
        d.mass_grains = entry.value("mass_grains", 0.0f);
        d.length_mm = entry.value("length_inches", 0.0f) * dope::math::IN_TO_MM;
        d.caliber_inches = entry.value("caliber_inches", 0.0f);
        d.baseline_barrel_length_in = entry.value("reference_barrel_inches", 24.0f);

        return d;
    }
    return d;
}

static float InterpolateProfileValue(const DOPE_TrajectoryPoint* pts, int count, float r) {
    if (count <= 0) return std::numeric_limits<float>::quiet_NaN();
    if (r <= pts[0].range_m) return pts[0].drop_m;
    if (r >= pts[count-1].range_m) return pts[count-1].drop_m;
    for (int i = 0; i < count - 1; ++i) {
        if (r <= pts[i+1].range_m) {
            float dx = pts[i+1].range_m - pts[i].range_m;
            float a = (dx > 0.0f) ? ((r - pts[i].range_m) / dx) : 0.0f;
            return pts[i].drop_m + a * (pts[i+1].drop_m - pts[i].drop_m);
        }
    }
    return pts[count-1].drop_m;
}

TEST(DebugDrop, Federal308_200ydZero_500ydTarget_ICAO) {
    const std::string name = ".308 Win 168gr BTHP MatchKing Federal";
    AmmoDatasetV2 ds = MakeDatasetFromJsonPreset("tools/native_gui/dope_gui_cartridges_v2.json", name);
    ASSERT_GT(ds.num_trajectories, 0);

    DOPE_Init(); EnableSolverFallback();
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig z = {};
    z.zero_range_m = 182.88f; // 200 yd
    z.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&z);
    DOPE_SetWindManual(0.0f, 0.0f);

    // runtime atmosphere per your inputs: 59F, 101325.2 Pa, 0% humidity
    SensorFrame f = {};
    f.timestamp_us = 10000;
    f.imu_valid = true; f.mag_valid = false;
    f.baro_valid = true; f.baro_humidity_valid = true;
    f.baro_temperature_c = (59.0f - 32.0f) * 5.0f / 9.0f; // 15C
    f.baro_pressure_pa = 101325.2f;
    f.baro_humidity = 0.0f;
    f.lrf_valid = true;
    f.lrf_range_m = 457.2f; // 500 yd
    f.lrf_timestamp_us = f.timestamp_us;

    for (int i = 0; i < 120; ++i) {
        f.timestamp_us = (uint64_t)(i + 1) * 10000;
        f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }

    FiringSolution sol = {};
    DOPE_GetSolution(&sol);

    // Reconstruct engine drop_m from final hold and sight-line math used in engine
    const float horizontal_range_m = sol.horizontal_range_m;
    const float sight_h = z.sight_height_mm * dope::math::MM_TO_M;
    const float zero_range_m = z.zero_range_m;
    const float sight_line_drop = sight_h - (sight_h / zero_range_m) * horizontal_range_m;
    const float rel_drop = -(sol.hold_elevation_moa / dope::math::RAD_TO_MOA) * horizontal_range_m;
    const float engine_drop_m = rel_drop + sight_line_drop;

    // Compute dataset-interpolated drop (table-first family selection)
    int best_idx = 0;
    float best_err = std::fabs(ds.trajectories[0].zero_range_m - zero_range_m);
    for (int i = 1; i < ds.num_trajectories; ++i) {
        float err = std::fabs(ds.trajectories[i].zero_range_m - zero_range_m);
        if (err < best_err) { best_err = err; best_idx = i; }
    }
    const DOPE_TrajectoryFamily &fam = ds.trajectories[best_idx];
    const float dataset_interp_drop_m = InterpolateProfileValue(fam.points, fam.num_points, horizontal_range_m);

    // Solver table point (integer-meter index)
    TrajectoryPoint tp = {};
    bool got_tp = DOPE_GetTrajectoryPoint((int)std::lround(horizontal_range_m), &tp);

    fprintf(stderr, "DebugDrop: cartridge= %s\n", name.c_str());
    fprintf(stderr, "Dataset family zero_range_m= %f\n", fam.zero_range_m);
    fprintf(stderr, "Dataset interp drop_m = %f m (%f in)\n", dataset_interp_drop_m, dataset_interp_drop_m * 39.37007874f);
    if (got_tp) fprintf(stderr, "Solver table drop_m = %f m (%f in)\n", tp.drop_m, tp.drop_m * 39.37007874f);
    else fprintf(stderr, "Solver table not available\n");
    fprintf(stderr, "Engine final reconstructed drop_m = %f m (%f in)\n", engine_drop_m, engine_drop_m * 39.37007874f);
    fprintf(stderr, "Final hold elevation (MOA) = %f\n", sol.hold_elevation_moa);

    // Also write results to a workspace file for reliable retrieval
    std::ofstream ofs("debug_drop_out.txt", std::ios::app);
    if (ofs.good()) {
        ofs << "cartridge," << name << "\n";
        ofs << "dataset_family_zero_range_m," << fam.zero_range_m << "\n";
        ofs << "dataset_interp_drop_m," << dataset_interp_drop_m << "\n";
        if (got_tp) ofs << "solver_table_drop_m," << tp.drop_m << "\n";
        else ofs << "solver_table_drop_m,NA\n";
        ofs << "engine_reconstructed_drop_m," << engine_drop_m << "\n";
        ofs << "final_hold_moa," << sol.hold_elevation_moa << "\n";
        ofs.close();
    }

    // Basic assertions to ensure we produced a solution
    EXPECT_EQ(sol.solution_mode, static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY));
}
