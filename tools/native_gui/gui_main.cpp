/**
 * @file gui_main.cpp
 * @brief Native Windows + ImGui desktop harness for DOPE validation.
 *
 * This executable is a manual test console around the DOPE C API.
 * It owns window/device setup, editable presets, synthetic SensorFrame input,
 * and live rendering of solution output/target hold visualization.
 */

#include <d3d11.h>
#include <tchar.h>
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <map>
#include <ctime>

#include "backends/imgui_impl_dx11.h"
#include "backends/imgui_impl_win32.h"
#include "imgui.h"

#include "dope/dope_api.h"
#include "dope/dope_math_utils.h"
#include "nlohmann/json.hpp"

#ifdef _MSC_VER
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")
#endif

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam,
                                                             LPARAM lParam);

namespace {

// -----------------------------------------------------------------------------
// D3D11/ImGui host resources (process lifetime)
// -----------------------------------------------------------------------------

static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

using namespace dope::math;

// ---------------------------------------------------------------------------
// Engine ticker thread ΓÇö mirrors the FreeRTOS task pattern on the ESP32-P4.
// The ticker owns DOPE_Update() and RefreshOutput(). The render loop reads
// only the cached display snapshot, so it never blocks on the solver.
// ---------------------------------------------------------------------------
static std::mutex g_engine_mutex;  // guards all DOPE_* API calls
static std::mutex g_display_mutex; // guards display snapshot (held briefly)
static FiringSolution g_display_sol = {};
static std::string g_display_output;
static std::atomic<bool> g_ticker_running{false};
static std::atomic<bool> g_run_batch_in_flight{false}; // prevents stacking background runs
static std::atomic<bool> g_auto_tick_enabled{false};   // gate background DOPE ticker
static std::atomic<bool> g_uncertainty_job{false};     // prevents overlapping UC jobs

constexpr uint64_t FRAME_STEP_US = 10000;
constexpr int TARGET_RING_COUNT = 4;
constexpr float TARGET_CANVAS_MIN = 160.0f;
constexpr float TARGET_CANVAS_MAX = 560.0f;

enum class UnitSystem : int { IMPERIAL = 0, METRIC = 1 };

const char* kUnitSystemLabels[2] = {"Imperial", "Metric"};

const DragModel kDragModels[8] = {DragModel::G1, DragModel::G2, DragModel::G3, DragModel::G4,
                                  DragModel::G5, DragModel::G6, DragModel::G7, DragModel::G8};

const char* kDragModelLabels[8] = {"G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8"};

// Tag label lists for cartridge editor UI
static const char* kBulletTierLabels[] = {"match", "mil_spec", "commercial", "budget"};
static const int kNumBulletTiers = sizeof(kBulletTierLabels) / sizeof(kBulletTierLabels[0]);

static const char* kBulletConstructionLabels[] = {
    "hpbt", "polymer_tip", "solid", "monolithic", "fmj", "plated", "jhp", "soft_tip", "wc", "cast"};
static const int kNumBulletConstructions =
    sizeof(kBulletConstructionLabels) / sizeof(kBulletConstructionLabels[0]);

static const char* kBulletShapeLabels[] = {"standard", "flat_point"};
static const int kNumBulletShapes = sizeof(kBulletShapeLabels) / sizeof(kBulletShapeLabels[0]);

struct CartridgeProfile {
    const char* name;
    float bc;
    int drag_model_index;
    float muzzle_velocity_ms;
    float mass_grains;
    float caliber_inches;
    float length_mm;
};
// Cartridge demo profiles removed; use external JSON `dope_gui_cartridges_v2.json`.

struct EnvironmentalConditions {
    float temperature_f = 59.0f;
    float pressure_inhg = 29.921f;
    float altitude_ft = 0.0f;
    float humidity_pct = 0.0f;
    std::string standard = "ICAO";
};

struct CartridgePreset {
    std::string name;
    float bc = 0.0f;
    int drag_model_index = 0;
    uint8_t supported_drag_models_mask = 0; // bit0 => G1 ... bit7 => G8
    float muzzle_velocity_ms = 0.0f;
    float mass_grains = 0.0f;
    float caliber_inches = 0.0f;
    float length_inches = 0.0f;
    EnvironmentalConditions env;
    std::map<std::string, float> ballistic_coefficients;
    std::string preferred_drag_model;
    std::vector<std::string> cartridge_keys;
    float reference_barrel_inches = 24.0f;
    float sd_mv_fps = 0.0f; // Standard deviation of muzzle velocity (fps)
    float mv_adjustment_fps_per_in = 0.0f;
    std::vector<std::pair<float, float>> barrel_mv_profile;
    std::vector<std::pair<float, float>> velocity_profile;
    std::vector<std::pair<float, float>> trajectory_profile;
    std::vector<std::pair<float, float>> wind_drift_profile;
    float baseline_wind_speed_ms = 0.0f;
    struct TrajectoryFamilyPreset {
        float zero_range_m = 0.0f;
        std::vector<std::pair<float, float>> trajectory_profile;
    };
    std::vector<TrajectoryFamilyPreset> trajectory_families;
    std::vector<float> available_zero_ranges;
    // Cartridge-level zero range (meters). If 0.0, unspecified and the gun's
    // zero_range_m will be used for visualization unless overridden elsewhere.
    float zero_range_m = 0.0f;
    // tags removed
    float muzzle_diameter_in = 0.7f;
    float cold_bore_velocity_bias = 8.5f;
    float angular_sigma_moa = 0.4f;
    float measured_cep50_moa = 0.0f;
    float manufacturer_spec_moa = 0.0f;
    float category_radial_moa = 0.0f;
    float category_vertical_moa = 0.0f;
};

struct GunPreset {
    std::string name;
    float caliber_inches = 0.308f;
    std::vector<std::string> cartridge_keys; // Compatible cartridge identifiers (intersection with
                                             // cartridge preset required when present)
    float barrel_length_in = 24.0f;
    float muzzle_diameter_in = 0.7f;
    float cold_bore_velocity_bias = 8.5f;
    float angular_sigma_moa = 0.4f;
    BarrelMaterial barrel_material = BarrelMaterial::CMV;
    float twist_rate_inches = 10.0f;
    float zero_range_m = 91.44f; // 100 yd
    float sight_height_mm = 38.1f;
    bool free_floated = false;
    bool suppressor_attached = false;
    bool barrel_tuner_attached = false;
};

// ---------------------------------------------------------------------------
// Sensor hardware presets ΓÇö each entry specifies the 1-sigma accuracy for
// the sensor's output(s).  "Custom" (index 0) lets the user type a manual
// value; all other entries auto-populate the sigma fields.
// ---------------------------------------------------------------------------

// --- LRF accuracy tables (range-dependent) --------------------------------
// Each LRF model has a piecewise-linear table of (range_m, sigma_m) break-
// points; the effective sigma is interpolated at the current LRF range.

// ---------------------------------------------------------------------------
// Generic sensor preset ΓÇö name + DOPE_ErrorTable (x = sensor-specific unit,
// sigma = 1-sigma uncertainty in the same unit as x).
// index 0 is always the "Custom / manual" sentinel: {nullptr, 0}.
// ---------------------------------------------------------------------------
struct SensorPreset {
    const char* name;
    DOPE_ErrorTable table; // {nullptr, 0} for Custom
};

// Helper macro: build a DOPE_ErrorTable from a static array.
#define SENSOR_TABLE(arr) {arr, (int)(sizeof(arr) / sizeof(arr[0]))}
#define SENSOR_CUSTOM {nullptr, 0}

// ===========================================================================
// LRF accuracy tables  (x = range_m, sigma = sigma_m)
// ===========================================================================

// JRT D09C 2000 m rangefinder
//   0.15ΓÇô22 m  : ┬▒0.2 mm to ┬▒1 mm
//   5ΓÇô400 m    : Γëñ ┬▒1 m
//   400ΓÇô1000 m : ┬▒1.2 m to ┬▒3 m
//   1000ΓÇô2000 m: ┬▒3 m to ┬▒6 m
static const DOPE_ErrorPoint kLRF_D09C[] = {
    {0.15f, 0.0002f}, // 0.2 mm
    {22.0f, 0.001f},  // 1 mm (end of precision band)
    {400.0f, 1.0f},   // Γëñ ┬▒1 m
    {1000.0f, 3.0f},  // ┬▒3 m
    {2000.0f, 6.0f},  // ┬▒6 m
};

static const SensorPreset kLRFPresets[] = {
    {"Custom", SENSOR_CUSTOM},
    {"JRT D09C (2000 m)", SENSOR_TABLE(kLRF_D09C)},
};
static const int kNumLRFPresets = sizeof(kLRFPresets) / sizeof(kLRFPresets[0]);

// ===========================================================================
// Temperature sensor accuracy tables  (x = temp_c, sigma = sigma_c)
// Source: manufacturer datasheets.
// ===========================================================================

// Texas Instruments TMP117
//   ΓêÆ40 to ΓêÆ20 ┬░C : ┬▒0.15 ┬░C  (extended edge)
//   ΓêÆ20 to +50 ┬░C : ┬▒0.10 ┬░C  (main calibrated range)
//   +50 to +70 ┬░C : ┬▒0.15 ┬░C  (extended edge)
static const DOPE_ErrorPoint kTemp_TMP117[] = {
    {-40.0f, 0.15f}, {-20.0f, 0.15f}, {-19.9f, 0.10f}, // step at ΓêÆ20 ┬░C lower edge of main range
    {50.0f, 0.10f},  {50.1f, 0.15f},                   // step at +50 ┬░C upper edge of main range
    {70.0f, 0.15f},
};

// Bosch BMP581 ΓÇö onboard temperature sensor
//    ΓêÆ5 to +55 ┬░C : ┬▒0.52 ┬░C  (calibrated operating range)
//   outside range  : ┬▒1.52 ┬░C
static const DOPE_ErrorPoint kTemp_BMP581[] = {
    {-40.0f, 1.52f}, {-5.0f, 1.52f}, {-4.9f, 0.52f}, // step at ΓêÆ5 ┬░C lower edge of cal range
    {55.0f, 0.52f},  {55.1f, 1.52f},                 // step at +55 ┬░C upper edge of cal range
    {85.0f, 1.52f},
};

static const SensorPreset kTempSensorPresets[] = {
    {"Custom", SENSOR_CUSTOM},
    {"Texas Instruments TMP117", SENSOR_TABLE(kTemp_TMP117)},
    {"Bosch BMP581 (internal)", SENSOR_TABLE(kTemp_BMP581)},
};
static const int kNumTempSensorPresets = sizeof(kTempSensorPresets) / sizeof(kTempSensorPresets[0]);

// ===========================================================================
// Barometer pressure drift tables  (x = |delta_temp_c_since_cal|, sigma = Pa)
// Source: BMP581 profile specified by project requirements.
// ===========================================================================
static const DOPE_ErrorPoint kBaroPressure_BMP581[] = {
    {0.0f, 0.10f},  {0.5f, 0.25f},   {1.0f, 0.51f},   {2.0f, 1.01f},
    {3.0f, 1.50f},  {5.0f, 2.50f},   {8.0f, 4.00f},   {10.0f, 5.00f},
    {15.0f, 7.50f}, {20.0f, 10.00f}, {30.0f, 15.00f}, {40.0f, 20.00f},
};

static const SensorPreset kBarometerPresets[] = {
    {"Bosch BMP581", SENSOR_TABLE(kBaroPressure_BMP581)},
};
static const int kNumBarometerPresets = sizeof(kBarometerPresets) / sizeof(kBarometerPresets[0]);

// --- Cant sensor presets ---
struct CantSensorPreset {
    const char* name;
    float sigma_cant_deg;
};

static const CantSensorPreset kCantPresets[] = {
    {"Custom", 0.0f},
    {"ST ISM330DHCX IMU (1 deg)", 1.0f},
    {"Level bubble (2 deg)", 2.0f},
    {"Precision digital level (0.1 deg)", 0.1f},
};
static const int kNumCantPresets = sizeof(kCantPresets) / sizeof(kCantPresets[0]);
static const int kCantDefaultIndex = 1; // ST ISM330DHCX IMU (1 deg)

// --- Magnetometer latitude-estimation presets ---
// Used when the magnetometer dip angle is used to derive latitude autonomously.
// sigma_lat_deg reflects the expected worst-case error in the derived latitude.
struct MagLatPreset {
    const char* name;
    float sigma_lat_deg; // 1-sigma latitude uncertainty (degrees) from mag dip estimation
};

static const MagLatPreset kMagLatPresets[] = {
    {"Custom", 0.0f},
    {"PNI RM3100", 1.5f}, // flat ┬▒1.5┬░ from magnetic dip estimation
};
static const int kNumMagLatPresets = sizeof(kMagLatPresets) / sizeof(kMagLatPresets[0]);

// --- GPS / GNSS module presets ---
// Used when the application feeds latitude directly from a GNSS receiver.
// sigma_lat_deg reflects the receiver's position accuracy converted to degrees.
// No presets yet ΓÇö framework ready for specific modules to be added later.
struct GpsLatPreset {
    const char* name;
    float sigma_lat_deg; // 1-sigma latitude uncertainty (degrees)
};

static const GpsLatPreset kGpsLatPresets[] = {
    {"No GPS/GNSS Module", 0.0f},
    // Add GPS/GNSS module presets here as hardware is qualified
};
static const int kNumGpsLatPresets = sizeof(kGpsLatPresets) / sizeof(kGpsLatPresets[0]);

static const int kLrfDefaultIndex = 1;       // JRT D09C (2000 m)
static const int kTempDefaultIndex = 1;      // Texas Instruments TMP117
static const int kBarometerDefaultIndex = 0; // Bosch BMP581
static const int kGpsDefaultIndex = 0;       // No GPS/GNSS Module
static const int kMagLatDefaultIndex = 1;    // PNI RM3100

struct ErrorMultiplierPreset {
    const char* name;
    float multiplier;
};

static const ErrorMultiplierPreset kMassErrorPresets[] = {
    {"Monolithic Copper", 0.0005f},
    {"Match-Grade [DEFAULT]", 0.0012f},
    {"Soft Point", 0.0025f},
    {"Cheap", 0.005f},
};
static const int kNumMassErrorPresets = sizeof(kMassErrorPresets) / sizeof(kMassErrorPresets[0]);
static const int kMassErrorDefaultIndex = 1;

static const ErrorMultiplierPreset kLengthErrorPresets[] = {
    {"Polymer Tips", 0.002f},
    {"Soft Point", 0.008f},
    {"Standard [DEFAULT]", 0.005f},
    {"Match Sorted", 0.001f},
};
static const int kNumLengthErrorPresets =
    sizeof(kLengthErrorPresets) / sizeof(kLengthErrorPresets[0]);
static const int kLengthErrorDefaultIndex = 2;

static const ErrorMultiplierPreset kCaliberErrorPresets[] = {
    {"Monolithic Copper", 0.00015f},
    {"Lead Core Match-Grade [DEFAULT]", 0.00045f},
    {"Soft Point", 0.00075f},
    {"Cheap", 0.0015f},
};
static const int kNumCaliberErrorPresets =
    sizeof(kCaliberErrorPresets) / sizeof(kCaliberErrorPresets[0]);
static const int kCaliberErrorDefaultIndex = 1;

struct BarrelFinishPreset {
    const char* name;
    float sigma_mv_fps;
};

static const BarrelFinishPreset kBarrelFinishPresets[] = {
    {"Custom", 8.5f}, // editable
    {"Stainless (Match)", 5.0f},
    {"Nitride (QPQ)", 8.5f},
    {"Chrome", 20.0f},
};
static const int kNumBarrelFinishPresets =
    sizeof(kBarrelFinishPresets) / sizeof(kBarrelFinishPresets[0]);
static const int kBarrelFinishDefaultIndex = 2; // Nitride

struct BarrelMaterialPreset {
    const char* key;   // JSON key
    const char* label; // UI label
    BarrelMaterial material;
    float stiffness_scale; // relative deflection vs stainless baseline
    float density_kg_m3;
    float specific_heat_J_kgK;
    float cte_scale; // thermal expansion vs stainless baseline
};

static const BarrelMaterialPreset kBarrelMaterialPresets[] = {
    {"cmv", "CMV (4140/4150)", BarrelMaterial::CMV, 0.95f, 7850.0f, 460.0f, 0.75f},
    {"416ss", "416 Stainless", BarrelMaterial::STAINLESS_416, 1.0f, 8000.0f, 500.0f, 1.0f},
    {"carbon", "Carbon-Wrapped", BarrelMaterial::CARBON_WRAPPED, 1.30f, 5200.0f, 720.0f, 0.25f},
};
static const int kNumBarrelMaterialPresets =
    sizeof(kBarrelMaterialPresets) / sizeof(kBarrelMaterialPresets[0]);
static const int kDefaultBarrelMaterialIndex = 0; // CMV

struct GuiState {
    AmmoDatasetV2 bullet = {};
    GunProfile gun = {};
    ZeroConfig zero = {};
    UnitSystem unit_system = UnitSystem::IMPERIAL;
    bool override_drag_coefficient = false;
    float manual_drag_coefficient = 0.0f;

    // Barrel attachments / bedding
    bool free_floated = false;
    bool suppressor_attached = false;
    bool barrel_tuner_attached = false;

    float wind_speed_ms = 0.0f;
    float wind_heading = 0.0f;
    float latitude = 0.0f;

    float baro_pressure = 101325.0f;
    float baro_temp = 18.333f; // 65 ┬░F
    float baro_humidity = 0.65f;
    float lrf_range = 457.2f;        // 500 yd
    float target_elevation_m = 0.0f; // +up, -down from shooter to target

    bool imu_valid = true;
    bool mag_valid = true;
    bool baro_valid = true;
    bool baro_humidity_valid = true;
    bool lrf_valid = true;

    int drag_model_index = 0;
    uint64_t now_us = 0;

    char preset_path[260] = "dope_gui_preset.json";
    char profile_library_path[260] = "dope_gui_profile_library.json";
    char new_cartridge_preset_name[64] = "";
    char new_gun_preset_name[64] = "";
    int selected_cartridge_preset = -1;
    int selected_gun_preset = -1;
    int new_cartridge_tier_index = 0;
    int new_cartridge_construction_index = 0;
    int new_cartridge_shape_index = 0;
    float new_cartridge_sd_mv_fps = 0.0f;
    bool side_view_show_required_angle = false;
    bool top_down_show_required_angle = false;
    std::string output_text;
    std::string last_action;
    int barrel_finish_index = kBarrelFinishDefaultIndex;
    int barrel_material_index = kDefaultBarrelMaterialIndex;
    float barrel_finish_sigma_mv_fps = 8.5f;

    // Uncertainty / error-margin config (initialized from DOPE_GetDefaultUncertaintyConfig).
    UncertaintyConfig uc_config = {};

    // Hardware sensor preset indices (0 = Custom/manual).
    int hw_barometer_index = kBarometerDefaultIndex;
    int hw_lrf_index = kLrfDefaultIndex;
    int hw_cant_index = kCantDefaultIndex;
    int hw_temp_sensor_index = kTempDefaultIndex;
    int hw_gps_index = kGpsDefaultIndex;
    int hw_mag_lat_index = kMagLatDefaultIndex;
    bool baro_is_calibrated = false;
    bool baro_has_calibration_temp = false;
    float baro_calibration_temp_c = 0.0f;
    int mass_error_preset_index = kMassErrorDefaultIndex;
    int length_error_preset_index = kLengthErrorDefaultIndex;
    int caliber_error_preset_index = kCaliberErrorDefaultIndex;
};

GuiState g_state;
std::vector<CartridgePreset> g_cartridge_presets;
std::vector<GunPreset> g_gun_presets;

// ---------------------------------------------------------------------------
// Barrel-MV glue-layer state (owned here, not by the engine BulletProfile).
// ---------------------------------------------------------------------------
static float g_reference_mv_ms = 0.0f;          // MV at reference barrel length
static bool g_mv_adjustment_estimated = false;  // true when fallback tier estimator is active
static float g_active_mv_adj_fps_per_in = 0.0f; // fps/in used last time ComputeAdjustedMv ran
static std::vector<std::pair<float, float>> g_active_barrel_mv_profile; // (barrel_in, mv_fps)

using json = nlohmann::json;

static json g_bullet_tolerances;

// Legacy per-type tolerance tables can still be embedded; when absent we expect
// per-cartridge sigma fields to be present instead.

// Capture tolerances from any object that embeds a bullet_type_tolerances block.
static void CaptureBulletTolerances(const json& root) {
    if (!root.is_object())
        return;
    const auto tol_it = root.find("bullet_type_tolerances");
    if (tol_it == root.end() || !tol_it->is_object())
        return;

    json captured = json::object();
    if (root.contains("__comments"))
        captured["__comments"] = root["__comments"];
    if (root.contains("_comments"))
        captured["_comments"] = root["_comments"];
    if (root.contains("bullet_tolerance_comments"))
        captured["__comments"] = root["bullet_tolerance_comments"];
    captured["bullet_type_tolerances"] = *tol_it;
    g_bullet_tolerances = captured;
}

// Synthesize a minimal AmmoDatasetV2 from a CartridgePreset for runtime use.
static AmmoDatasetV2 MakeV2FromCartridgePreset(const CartridgePreset& cp) {
    AmmoDatasetV2 d = {};
    d.source_confidence = 0.5f;
    d.baseline_temperature_c = g_state.baro_temp;
    d.baseline_pressure_pa = g_state.baro_pressure;
    d.baseline_humidity = g_state.baro_humidity;
    d.baseline_altitude_m = 0.0f;
    d.baseline_barrel_length_in = cp.reference_barrel_inches;
    d.baseline_wind_speed_ms = g_state.wind_speed_ms;

    // velocity_profile -> velocity_by_range (distance_inches -> meters)
    d.num_velocity_points = 0;
    for (const auto& vp : cp.velocity_profile) {
        if (d.num_velocity_points >= DOPE_MAX_TABLE_POINTS)
            break;
        d.velocity_by_range[d.num_velocity_points].range_m = vp.first * INCHES_TO_M;
        d.velocity_by_range[d.num_velocity_points].value = vp.second;
        d.num_velocity_points++;
    }
    if (d.num_velocity_points == 0) {
        // fallback single-point from muzzle_velocity
        d.velocity_by_range[0].range_m = 0.0f;
        d.velocity_by_range[0].value = cp.muzzle_velocity_ms;
        d.num_velocity_points = 1;
    }

    // Trajectory profile (support per-family trajectory_tables when present)
    d.num_trajectories = 0;
    if (!cp.trajectory_families.empty()) {
        d.num_trajectories = static_cast<int>(std::min<size_t>(cp.trajectory_families.size(), DOPE_MAX_TRAJECTORY_FAMILIES));
        for (int fi = 0; fi < d.num_trajectories; ++fi) {
            const auto &fpre = cp.trajectory_families[fi];
            DOPE_TrajectoryFamily& fam = d.trajectories[fi];
            fam.zero_range_m = fpre.zero_range_m;
            fam.num_points = 0;
            for (const auto& tp : fpre.trajectory_profile) {
                if (fam.num_points >= DOPE_MAX_TABLE_POINTS)
                    break;
                fam.points[fam.num_points].range_m = tp.first * INCHES_TO_M;
                fam.points[fam.num_points].drop_m = tp.second * INCHES_TO_M; // inches -> meters
                fam.num_points++;
            }
            fam.cached_table_present = false;
            fam.cached_table_num_points = 0;
        }
    } else if (!cp.trajectory_profile.empty()) {
        // Backwards-compatible single-family behavior: use cartridge-level data.
        if (!cp.available_zero_ranges.empty()) {
            d.num_trajectories = static_cast<int>(std::min<size_t>(cp.available_zero_ranges.size(), DOPE_MAX_TRAJECTORY_FAMILIES));
            for (int fi = 0; fi < d.num_trajectories; ++fi) {
                DOPE_TrajectoryFamily& fam = d.trajectories[fi];
                fam.zero_range_m = cp.available_zero_ranges[fi];
                fam.num_points = 0;
                for (const auto& tp : cp.trajectory_profile) {
                    if (fam.num_points >= DOPE_MAX_TABLE_POINTS)
                        break;
                    fam.points[fam.num_points].range_m = tp.first * INCHES_TO_M;
                    fam.points[fam.num_points].drop_m = tp.second * INCHES_TO_M; // inches -> meters
                    fam.num_points++;
                }
                fam.cached_table_present = false;
                fam.cached_table_num_points = 0;
            }
        } else {
            d.num_trajectories = 1;
            DOPE_TrajectoryFamily& fam = d.trajectories[0];
            fam.zero_range_m = (cp.zero_range_m > 0.0f) ? cp.zero_range_m : 0.0f;
            fam.num_points = 0;
            for (const auto& tp : cp.trajectory_profile) {
                if (fam.num_points >= DOPE_MAX_TABLE_POINTS)
                    break;
                fam.points[fam.num_points].range_m = tp.first * INCHES_TO_M;
                fam.points[fam.num_points].drop_m = tp.second * INCHES_TO_M; // inches -> meters
                fam.num_points++;
            }
            fam.cached_table_present = false;
            fam.cached_table_num_points = 0;
        }
    }

    // Sparse fallback params
    d.bc = cp.bc;
    int dm_idx = cp.drag_model_index;
    if (dm_idx < 0)
        dm_idx = 0;
    if (dm_idx > 7)
        dm_idx = 7;
    d.drag_model = static_cast<DragModel>(dm_idx + 1); // DragModel enum is 1..8
    // Populate supported models mask from the cartridge preset when present;
    // otherwise fall back to the single `drag_model_index` for backward
    // compatibility.
    if (cp.supported_drag_models_mask != 0) {
        d.supported_drag_models_mask = cp.supported_drag_models_mask;
    } else {
        d.supported_drag_models_mask = static_cast<uint8_t>(1u << dm_idx);
    }
    d.muzzle_velocity_ms = (d.num_velocity_points > 0) ? d.velocity_by_range[0].value : cp.muzzle_velocity_ms;
    d.mass_grains = cp.mass_grains;
    d.length_mm = cp.length_inches * IN_TO_MM;
    d.caliber_inches = cp.caliber_inches;
    d.twist_rate_inches = g_state.bullet.twist_rate_inches;
    d.mv_adjustment_fps_per_in = cp.mv_adjustment_fps_per_in;
    d.baseline_wind_speed_ms = cp.baseline_wind_speed_ms;

    // wind_drift_profile -> wind_drift_by_range
    d.num_wind_drift_points = 0;
    for (const auto& wd : cp.wind_drift_profile) {
        if (d.num_wind_drift_points >= DOPE_MAX_TABLE_POINTS)
            break;
        d.wind_drift_by_range[d.num_wind_drift_points].range_m = wd.first * INCHES_TO_M;
        d.wind_drift_by_range[d.num_wind_drift_points].value = wd.second * INCHES_TO_M;
        d.num_wind_drift_points++;
    }
    
    if (d.num_wind_drift_points > 0) {
        std::printf("DEBUG: Converted dataset has %d wind drift points. First point: %.2fm at %.2fm\n", 
            d.num_wind_drift_points, d.wind_drift_by_range[0].value, d.wind_drift_by_range[0].range_m);
        // Write to a file for persistence in case stdout is suppressed
        std::ofstream df("debug_conversion.txt", std::ios::app);
        df << "Ammo: " << cp.name << " Points: " << d.num_wind_drift_points 
           << " First: " << d.wind_drift_by_range[0].value << "@" << d.wind_drift_by_range[0].range_m 
           << " Last: " << d.wind_drift_by_range[d.num_wind_drift_points-1].value << "@" << d.wind_drift_by_range[d.num_wind_drift_points-1].range_m 
           << "\n";
    }

    return d;
}

static bool CartridgesHaveSigmaValues(const std::vector<CartridgePreset>& presets) {
    for (const auto& p : presets) {
        if (p.sd_mv_fps > 0.0f) {
            return true;
        }
    }
    return false;
}

static void SanitizeCartridgePreset(CartridgePreset& preset);
static void SanitizeGunPreset(GunPreset& preset);
static json SerializeCartridgePreset(const CartridgePreset& input);
static json SerializeGunPreset(const GunPreset& input);
// Load external preset files (returns true if loaded and non-empty)
static bool LoadCartridgePresetsFromFile(const std::string& path);
static bool LoadGunPresetsFromFile(const std::string& path);
static bool LoadHardwarePresetsFromFile(const std::string& path);
static float GetSelectedCartridgeCaliberInches();
static bool IsGunPresetCompatibleWithSelectedCaliber(const GunPreset& preset);
static void EnsureSelectedGunPresetMatchesCurrentCaliber();
static void SelectDefaultGunPresetForSelectedCartridge();
static void ApplyGunPreset(const GunPreset& preset);
static float ComputeStiffnessMoa(float muzzle_diameter_in, float bore_diameter_in,
                                 BarrelMaterial material);
static CartridgePreset CaptureCurrentCartridgePreset(const std::string& name);
static void ComputeAdjustedMv(); // glue-layer barrel-length → MV interpolation

// Dynamic hardware preset labels / optional sigma tables loaded from JSON.
static std::vector<std::string> g_baro_labels;
static std::vector<std::string> g_temp_labels;
static std::vector<std::string> g_lrf_labels;
static std::vector<std::string> g_cant_labels;
static std::vector<float> g_cant_sigmas;
static std::vector<std::string> g_maglat_labels;
static std::vector<std::string> g_gps_labels;
// Mapping from dynamic list index -> kLRFPresets / kTempSensorPresets index.
// Needed because those static arrays have a Custom sentinel at index 0, so the
// dynamic index and the static index are not the same.
static std::vector<int> g_baro_preset_indices;
static std::vector<int> g_lrf_preset_indices;
static std::vector<int> g_temp_preset_indices;
static std::vector<int> g_maglat_preset_indices;
static std::vector<int> g_gps_preset_indices;

const BarrelMaterialPreset& GetBarrelMaterialPreset(BarrelMaterial material) {
    for (int i = 0; i < kNumBarrelMaterialPresets; ++i) {
        if (kBarrelMaterialPresets[i].material == material) {
            return kBarrelMaterialPresets[i];
        }
    }
    return kBarrelMaterialPresets[kDefaultBarrelMaterialIndex];
}

int ResolveBarrelMaterialIndex(BarrelMaterial material) {
    for (int i = 0; i < kNumBarrelMaterialPresets; ++i) {
        if (kBarrelMaterialPresets[i].material == material) {
            return i;
        }
    }
    return kDefaultBarrelMaterialIndex;
}

BarrelMaterial ParseBarrelMaterialKey(const std::string& key) {
    for (int i = 0; i < kNumBarrelMaterialPresets; ++i) {
        if (key == kBarrelMaterialPresets[i].key) {
            return kBarrelMaterialPresets[i].material;
        }
    }
    return kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material;
}

const char* BarrelMaterialKey(BarrelMaterial material) {
    return GetBarrelMaterialPreset(material).key;
}

bool LoadHardwarePresetsFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.good())
        return false;
    try {
        json j;
        ifs >> j;
        if (!j.is_object())
            return false;
        const auto sensors = j.find("sensors");
        if (sensors == j.end() || !sensors->is_object())
            return false;

        g_baro_labels.clear();
        g_temp_labels.clear();
        g_lrf_labels.clear();
        g_cant_labels.clear();
        g_cant_sigmas.clear();
        g_maglat_labels.clear();
        g_gps_labels.clear();
        g_baro_preset_indices.clear();
        g_lrf_preset_indices.clear();
        g_temp_preset_indices.clear();
        g_maglat_preset_indices.clear();
        g_gps_preset_indices.clear();

        // Resolve a name to its index in a static SensorPreset array.
        // Returns 0 (Custom) when no match is found.
        auto resolve_preset_index = [](const std::string& name, const SensorPreset* presets,
                                       int count) -> int {
            for (int k = 0; k < count; ++k) {
                if (presets[k].name && name == presets[k].name)
                    return k;
            }
            return 0; // Custom sentinel
        };

        auto it = sensors->find("barometers");
        if (it != sensors->end() && it->is_array()) {
            for (const auto& e : *it) {
                std::string nm = e.value("name", std::string(""));
                g_baro_labels.push_back(nm);
                int mapped = e.value("index", -1);
                if (mapped < 0 || mapped >= kNumBarometerPresets) {
                    mapped = resolve_preset_index(nm, kBarometerPresets, kNumBarometerPresets);
                }
                g_baro_preset_indices.push_back(mapped);
            }
        }
        it = sensors->find("temp_sensors");
        if (it != sensors->end() && it->is_array()) {
            for (const auto& e : *it) {
                std::string nm = e.value("name", std::string(""));
                g_temp_labels.push_back(nm);
                g_temp_preset_indices.push_back(
                    resolve_preset_index(nm, kTempSensorPresets, kNumTempSensorPresets));
            }
        }
        it = sensors->find("lrfs");
        if (it != sensors->end() && it->is_array()) {
            for (const auto& e : *it) {
                std::string nm = e.value("name", std::string(""));
                g_lrf_labels.push_back(nm);
                g_lrf_preset_indices.push_back(
                    resolve_preset_index(nm, kLRFPresets, kNumLRFPresets));
            }
        }
        it = sensors->find("gps");
        if (it != sensors->end() && it->is_array()) {
            for (const auto& e : *it) {
                std::string nm = e.value("name", std::string(""));
                g_gps_labels.push_back(nm);
                int mapped = e.value("index", -1);
                if (mapped < 0 || mapped >= kNumGpsLatPresets) {
                    mapped = 0;
                    for (int k = 0; k < kNumGpsLatPresets; ++k) {
                        if (kGpsLatPresets[k].name && nm == kGpsLatPresets[k].name) {
                            mapped = k;
                            break;
                        }
                    }
                }
                g_gps_preset_indices.push_back(mapped);
            }
        }
        it = sensors->find("mag_lat");
        if (it != sensors->end() && it->is_array()) {
            for (const auto& e : *it) {
                std::string nm = e.value("name", std::string(""));
                g_maglat_labels.push_back(nm);
                int mapped = e.value("index", -1);
                if (mapped < 0 || mapped >= kNumMagLatPresets) {
                    mapped = 0;
                    for (int k = 0; k < kNumMagLatPresets; ++k) {
                        if (kMagLatPresets[k].name && nm == kMagLatPresets[k].name) {
                            mapped = k;
                            break;
                        }
                    }
                }
                g_maglat_preset_indices.push_back(mapped);
            }
        }

        it = sensors->find("cant");
        if (it != sensors->end() && it->is_array()) {
            for (const auto& e : *it) {
                g_cant_labels.push_back(e.value("name", std::string("")));
                g_cant_sigmas.push_back(e.value("sigma_cant_deg", 0.0f));
            }
        }

        // Consider non-empty if any label group has entries.
        return !(g_baro_labels.empty() && g_temp_labels.empty() && g_lrf_labels.empty() &&
                 g_cant_labels.empty() && g_maglat_labels.empty() && g_gps_labels.empty());
    } catch (...) {
        return false;
    }
}

// Helper accessors: counts and labels; when dynamic labels are present they
// shadow the built-in static presets (which still provide detailed error tables).
inline int GetNumBarometerPresets() {
    return g_baro_labels.empty() ? kNumBarometerPresets : (int)g_baro_labels.size();
}
inline int GetNumTempSensorPresets() {
    return g_temp_labels.empty() ? kNumTempSensorPresets : (int)g_temp_labels.size();
}
inline int GetNumLRFPresets() {
    return g_lrf_labels.empty() ? kNumLRFPresets : (int)g_lrf_labels.size();
}
inline int GetNumCantPresets() {
    return g_cant_labels.empty() ? kNumCantPresets : (int)g_cant_labels.size();
}
inline int GetNumMagLatPresets() {
    return g_maglat_labels.empty() ? kNumMagLatPresets : (int)g_maglat_labels.size();
}
inline int GetNumGpsLatPresets() {
    return g_gps_labels.empty() ? kNumGpsLatPresets : (int)g_gps_labels.size();
}

inline const char* GetBarometerLabel(int i) {
    return g_baro_labels.empty()
               ? ((i >= 0 && i < kNumBarometerPresets) ? kBarometerPresets[i].name : "")
               : g_baro_labels[i].c_str();
}
inline const char* GetTempSensorLabel(int i) {
    return g_temp_labels.empty()
               ? ((i >= 0 && i < kNumTempSensorPresets) ? kTempSensorPresets[i].name : "")
               : g_temp_labels[i].c_str();
}
inline const char* GetLRFLabel(int i) {
    return g_lrf_labels.empty() ? ((i >= 0 && i < kNumLRFPresets) ? kLRFPresets[i].name : "")
                                : g_lrf_labels[i].c_str();
}
inline const char* GetCantLabel(int i) {
    return g_cant_labels.empty() ? ((i >= 0 && i < kNumCantPresets) ? kCantPresets[i].name : "")
                                 : g_cant_labels[i].c_str();
}
inline const char* GetMagLatLabel(int i) {
    return g_maglat_labels.empty()
               ? ((i >= 0 && i < kNumMagLatPresets) ? kMagLatPresets[i].name : "")
               : g_maglat_labels[i].c_str();
}
inline const char* GetGpsLabel(int i) {
    return g_gps_labels.empty() ? ((i >= 0 && i < kNumGpsLatPresets) ? kGpsLatPresets[i].name : "")
                                : g_gps_labels[i].c_str();
}

inline int GetMappedBarometerPresetIndex(int i) {
    if (!g_baro_labels.empty()) {
        return (i >= 0 && i < (int)g_baro_preset_indices.size()) ? g_baro_preset_indices[i] : 0;
    }
    return i;
}
inline int GetMappedMagLatPresetIndex(int i) {
    if (!g_maglat_labels.empty()) {
        return (i >= 0 && i < (int)g_maglat_preset_indices.size()) ? g_maglat_preset_indices[i] : 0;
    }
    return i;
}
inline int GetMappedGpsLatPresetIndex(int i) {
    if (!g_gps_labels.empty()) {
        return (i >= 0 && i < (int)g_gps_preset_indices.size()) ? g_gps_preset_indices[i] : 0;
    }
    return i;
}

inline const DOPE_ErrorTable* GetLRFTable(int i) {
    if (!g_lrf_labels.empty()) {
        // Dynamic mode: map dynamic index -> static preset index via name-resolution table.
        const int mapped =
            (i >= 0 && i < (int)g_lrf_preset_indices.size()) ? g_lrf_preset_indices[i] : 0;
        return (mapped >= 0 && mapped < kNumLRFPresets) ? &kLRFPresets[mapped].table : nullptr;
    }
    return (i >= 0 && i < kNumLRFPresets) ? &kLRFPresets[i].table : nullptr;
}
inline const DOPE_ErrorTable* GetTempTable(int i) {
    if (!g_temp_labels.empty()) {
        // Dynamic mode: map dynamic index -> static preset index via name-resolution table.
        const int mapped =
            (i >= 0 && i < (int)g_temp_preset_indices.size()) ? g_temp_preset_indices[i] : 0;
        return (mapped >= 0 && mapped < kNumTempSensorPresets) ? &kTempSensorPresets[mapped].table
                                                               : nullptr;
    }
    return (i >= 0 && i < kNumTempSensorPresets) ? &kTempSensorPresets[i].table : nullptr;
}
inline const DOPE_ErrorTable* GetBarometerTable(int i) {
    const int mapped = GetMappedBarometerPresetIndex(i);
    return (mapped >= 0 && mapped < kNumBarometerPresets) ? &kBarometerPresets[mapped].table
                                                          : nullptr;
}
inline float GetCantSigma(int i) {
    return g_cant_sigmas.empty()
               ? ((i >= 0 && i < kNumCantPresets) ? kCantPresets[i].sigma_cant_deg : 0.0f)
               : g_cant_sigmas[i];
}

// Implementation: load cartridge presets from a JSON array file
bool LoadCartridgePresetsFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.good()) {
        return false;
    }
    try {
        json j;
        ifs >> j;
        const json* preset_array = nullptr;

        if (j.is_object()) {
            CaptureBulletTolerances(j);
            if (j.contains("cartridge_presets") && j["cartridge_presets"].is_array()) {
                preset_array = &j["cartridge_presets"];
            } else if (j.contains("cartridges") && j["cartridges"].is_array()) {
                preset_array = &j["cartridges"];
            } else if (j.contains("presets") && j["presets"].is_array()) {
                preset_array = &j["presets"];
            } else if (j.contains("ammo_v2") && j["ammo_v2"].is_array()) {
                preset_array = &j["ammo_v2"];
            }
        } else if (j.is_array()) {
            preset_array = &j;
        }

        if (preset_array == nullptr)
            return false;
        std::vector<CartridgePreset> tmp;
        for (const auto& e : *preset_array) {
            CartridgePreset p;
            p.name = e.value("name", std::string(""));
            p.bc = e.value("bc", 0.0f);
            p.drag_model_index = e.value("drag_model_index", 0);
            // legacy single-point muzzle velocity (accept fps or m/s)
            if (e.contains("muzzle_velocity_fps") && e["muzzle_velocity_fps"].is_number()) {
                p.muzzle_velocity_ms = e["muzzle_velocity_fps"].get<float>() * FPS_TO_MPS;
            } else {
                p.muzzle_velocity_ms = e.value("muzzle_velocity_ms", 0.0f);
            }
            // Support V2 keys with SI units: mass_kg, caliber_m, length_m
            if (e.contains("mass_kg") && e["mass_kg"].is_number()) {
                const float mass_kg = e["mass_kg"].get<float>();
                // 1 kg = 15432.358352941 grains
                p.mass_grains = mass_kg * 15432.358352941f;
            } else {
                p.mass_grains = e.value("mass_grains", 0.0f);
            }
            if (e.contains("caliber_m") && e["caliber_m"].is_number()) {
                p.caliber_inches = e["caliber_m"].get<float>() * 39.37007874f;
            } else {
                p.caliber_inches = e.value("caliber_inches", 0.0f);
            }
            if (e.contains("length_m") && e["length_m"].is_number()) {
                p.length_inches = e["length_m"].get<float>() * 39.37007874f;
            } else {
                p.length_inches = e.value("length_inches", 0.0f);
            }
            
            // New V2 fields: environmental_conditions
            if (e.contains("environmental_conditions") && e["environmental_conditions"].is_object()) {
                const auto& env_j = e["environmental_conditions"];
                p.env.temperature_f = env_j.value("temperature_f", 59.0f);
                p.env.pressure_inhg = env_j.value("pressure_inhg", 29.921f);
                p.env.altitude_ft = env_j.value("altitude_ft", 0.0f);
                p.env.humidity_pct = env_j.value("humidity_pct", 0.0f);
                p.env.standard = env_j.value("standard", "ICAO");
            }

            // New V2 fields: multi-BC
            if (e.contains("ballistic_coefficients") && e["ballistic_coefficients"].is_object()) {
                for (auto it = e["ballistic_coefficients"].begin(); it != e["ballistic_coefficients"].end(); ++it) {
                    if (it.value().is_number()) {
                        p.ballistic_coefficients[it.key()] = it.value().get<float>();
                    }
                }
            }
            p.preferred_drag_model = e.value("preferred_drag_model", "G1");
            if (p.bc <= 0.0f && p.ballistic_coefficients.count(p.preferred_drag_model)) {
                p.bc = p.ballistic_coefficients[p.preferred_drag_model];
            }
            if (p.bc <= 0.0f) {
                p.bc = e.value("ballistic_coefficient", e.value("bc", 0.505f));
            }

            if (e.contains("drag_model_index")) {
                p.drag_model_index = e["drag_model_index"].get<int>();
            } else if (e.contains("drag_model") && e["drag_model"].is_string()) {
                const std::string dm = e["drag_model"].get<std::string>();
                for (int i = 0; i < 8; ++i) {
                    if (dm == kDragModelLabels[i]) {
                        p.drag_model_index = i;
                        break;
                    }
                }
            } else {
                for (int i = 0; i < 8; ++i) {
                    if (p.preferred_drag_model == kDragModelLabels[i]) {
                        p.drag_model_index = i;
                        break;
                    }
                }
            }
            p.cartridge_keys.clear();
            const auto ck_it = e.find("cartridge_keys");
            if (ck_it != e.end() && ck_it->is_array()) {
                for (const auto& k : *ck_it) {
                    if (k.is_string())
                        p.cartridge_keys.push_back(k.get<std::string>());
                }
            }
            p.reference_barrel_inches = e.value("reference_barrel_inches", 24.0f);
            p.muzzle_diameter_in = e.value("muzzle_diameter_in", 0.7f);
            p.cold_bore_velocity_bias = e.value("cold_bore_velocity_bias", 8.5f);
            p.angular_sigma_moa = e.value("angular_sigma_moa", 0.4f);
            p.measured_cep50_moa = e.value("measured_cep50_moa", 0.0f);
            p.manufacturer_spec_moa = e.value("manufacturer_spec_moa", 0.0f);
            p.category_radial_moa = e.value("category_radial_moa", 0.0f);
            p.category_vertical_moa = e.value("category_vertical_moa", 0.0f);
            // Handle velocity_sigma_fps (alternate field for MV uncertainty)
            if (e.contains("velocity_sigma_fps") && e["velocity_sigma_fps"].is_number()) {
                p.sd_mv_fps = e["velocity_sigma_fps"].get<float>();
            } else {
                p.sd_mv_fps = e.value("sd_mv_fps", 0.0f);
            }
            // tags removed
            // velocity_profile (optional)
            p.velocity_profile.clear();
            const auto vp_it = e.find("velocity_profile");
            if (vp_it != e.end() && vp_it->is_array()) {
                for (const auto& vp : *vp_it) {
                    float dist_inches = 0.0f;
                    if (vp.contains("distance_inches") && vp["distance_inches"].is_number()) {
                        dist_inches = vp["distance_inches"].get<float>();
                    } else if (vp.contains("distance_m") && vp["distance_m"].is_number()) {
                        dist_inches = vp["distance_m"].get<float>() * 39.37007874f;
                    }
                    float vel = 0.0f;
                    if (vp.contains("velocity_ms") && vp["velocity_ms"].is_number()) {
                        vel = vp["velocity_ms"].get<float>();
                    } else if (vp.contains("velocity_fps") && vp["velocity_fps"].is_number()) {
                        vel = vp["velocity_fps"].get<float>() * FPS_TO_MPS;
                    } else if (vp.contains("velocity_mps") && vp["velocity_mps"].is_number()) {
                        vel = vp["velocity_mps"].get<float>();
                    }
                    p.velocity_profile.emplace_back(dist_inches, vel);
                }
            } else if (p.muzzle_velocity_ms > 0.0f) {
                p.velocity_profile.emplace_back(0.0f, p.muzzle_velocity_ms);
            }
            // trajectory_profile (optional)
            p.trajectory_profile.clear();
            const auto tp_it = e.find("trajectory_profile");
            if (tp_it != e.end() && tp_it->is_array()) {
                for (const auto& tp : *tp_it) {
                    float dist_inches = 0.0f;
                    if (tp.contains("distance_inches") && tp["distance_inches"].is_number()) {
                        dist_inches = tp["distance_inches"].get<float>();
                    } else if (tp.contains("distance_m") && tp["distance_m"].is_number()) {
                        dist_inches = tp["distance_m"].get<float>() * 39.37007874f;
                    }
                    float traj_inches = 0.0f;
                    if (tp.contains("trajectory_inches") && tp["trajectory_inches"].is_number()) {
                        traj_inches = tp["trajectory_inches"].get<float>();
                    } else if (tp.contains("trajectory_in") && tp["trajectory_in"].is_number()) {
                        traj_inches = tp["trajectory_in"].get<float>();
                    } else if (tp.contains("trajectory_m") && tp["trajectory_m"].is_number()) {
                        traj_inches = tp["trajectory_m"].get<float>() * 39.37007874f;
                    }
                    p.trajectory_profile.emplace_back(dist_inches, traj_inches);
                }
                // trajectory_families (optional) - explicit per-zero trajectory tables
                p.trajectory_families.clear();
                const auto tf_it = e.find("trajectory_families");
                if (tf_it != e.end() && tf_it->is_array()) {
                    for (const auto &tf : *tf_it) {
                        CartridgePreset::TrajectoryFamilyPreset fam;
                        fam.zero_range_m = tf.value("zero_range_m", 0.0f);
                        const auto f_tp_it = tf.find("trajectory_profile");
                        if (f_tp_it != tf.end() && f_tp_it->is_array()) {
                            for (const auto &ftp : *f_tp_it) {
                                float dist_inches = 0.0f;
                                if (ftp.contains("distance_inches") && ftp["distance_inches"].is_number()) {
                                    dist_inches = ftp["distance_inches"].get<float>();
                                } else if (ftp.contains("distance_m") && ftp["distance_m"].is_number()) {
                                    dist_inches = ftp["distance_m"].get<float>() * 39.37007874f;
                                }
                                float traj_inches = 0.0f;
                                if (ftp.contains("trajectory_inches") && ftp["trajectory_inches"].is_number()) {
                                    traj_inches = ftp["trajectory_inches"].get<float>();
                                } else if (ftp.contains("trajectory_in") && ftp["trajectory_in"].is_number()) {
                                    traj_inches = ftp["trajectory_in"].get<float>();
                                } else if (ftp.contains("trajectory_m") && ftp["trajectory_m"].is_number()) {
                                    traj_inches = ftp["trajectory_m"].get<float>() * 39.37007874f;
                                }
                                fam.trajectory_profile.emplace_back(dist_inches, traj_inches);
                            }
                        }
                        p.trajectory_families.push_back(std::move(fam));
                    }
                }
            }
            // cartridge-level zero range (meters), optional
            p.zero_range_m = e.value("zero_range_m", 0.0f);
            // optional multiple zero ranges
            if (e.contains("available_zero_ranges") && e["available_zero_ranges"].is_array()) {
                for (const auto &zr : e["available_zero_ranges"]) {
                    if (zr.is_number())
                        p.available_zero_ranges.push_back(static_cast<float>(zr.get<double>()));
                }
            }
            // wind_drift_profile (optional)
            p.wind_drift_profile.clear();
            const auto wd_it = e.find("wind_drift_profile");
            if (wd_it != e.end() && wd_it->is_array()) {
                for (const auto& wd : *wd_it) {
                    float dist_inches = 0.0f;
                    if (wd.contains("distance_inches") && wd["distance_inches"].is_number()) {
                        dist_inches = wd["distance_inches"].get<float>();
                    } else if (wd.contains("distance_m") && wd["distance_m"].is_number()) {
                        dist_inches = wd["distance_m"].get<float>() * 39.37007874f;
                    }
                    float drift_inches = 0.0f;
                    if (wd.contains("wind_drift_inches") && wd["wind_drift_inches"].is_number()) {
                        drift_inches = wd["wind_drift_inches"].get<float>();
                    } else if (wd.contains("wind_drift_in") && wd["wind_drift_in"].is_number()) {
                        drift_inches = wd["wind_drift_in"].get<float>();
                    } else if (wd.contains("wind_drift_m") && wd["wind_drift_m"].is_number()) {
                        drift_inches = wd["wind_drift_m"].get<float>() * 39.37007874f;
                    }
                    p.wind_drift_profile.emplace_back(dist_inches, drift_inches);
                }
            }
            p.baseline_wind_speed_ms = e.value("baseline_wind_speed_ms", 4.4704f); // default 10 mph
            // cartridge CEP (optional)
            // Removed obsolete CEP fields
            // barrel-MV fields (optional)
            p.mv_adjustment_fps_per_in = e.value("mv_adjustment_fps_per_in", 0.0f);
            p.barrel_mv_profile.clear();
            const auto bmp_it = e.find("barrel_mv_profile");
            if (bmp_it != e.end() && bmp_it->is_array()) {
                for (const auto& bm : *bmp_it) {
                    const float barrel_in = bm.value("barrel_in", 0.0f);
                    const float mv_fps = bm.value("mv_fps", 0.0f);
                    if (barrel_in > 0.0f && mv_fps > 0.0f)
                        p.barrel_mv_profile.emplace_back(barrel_in, mv_fps);
                }
            }
            // Sanitize
            if (p.drag_model_index < 0 || p.drag_model_index >= 8) {
                p.drag_model_index = 0;
            }
            SanitizeCartridgePreset(p);
            tmp.push_back(p);
        }
        if (!tmp.empty()) {
            g_cartridge_presets = std::move(tmp);
            return true;
        }
    } catch (...) {
        return false;
    }
    return false;
}

// Implementation: load gun presets from a JSON array file
bool LoadGunPresetsFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.good()) {
        return false;
    }
    try {
        json j;
        ifs >> j;
        if (!j.is_array())
            return false;
        std::vector<GunPreset> tmp;
        for (const auto& e : j) {
            if (e.contains("_comments")) continue; // skip metadata entries
            GunPreset p;
            p.name = e.value("name", std::string(""));
            p.caliber_inches = e.value("caliber_inches", 0.308f);
            p.cartridge_keys.clear();
            const auto ck_it = e.find("cartridge_keys");
            if (ck_it != e.end() && ck_it->is_array()) {
                for (const auto& k : *ck_it) {
                    if (k.is_string())
                        p.cartridge_keys.push_back(k.get<std::string>());
                }
            }
            p.barrel_length_in = e.value("barrel_length_in", 24.0f);
            p.muzzle_diameter_in = e.value("muzzle_diameter_in", 0.7f);
            p.cold_bore_velocity_bias = e.value("cold_bore_velocity_bias", 8.5f);
            p.angular_sigma_moa = e.value("angular_sigma_moa", 0.4f);
            const std::string mat_key = e.value("barrel_material", std::string(""));
            p.barrel_material = mat_key.empty()
                                    ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
                                    : ParseBarrelMaterialKey(mat_key);
            p.twist_rate_inches = e.value("twist_rate_inches", 10.0f);
            p.zero_range_m = e.value("zero_range_m", 91.44f); // 100 yd
            p.sight_height_mm = e.value("sight_height_mm", 38.1f);
            p.free_floated = e.value("free_floated", false);
            p.suppressor_attached = e.value("suppressor_attached", false);
            p.barrel_tuner_attached = e.value("barrel_tuner_attached", false);
            SanitizeGunPreset(p);
            tmp.push_back(p);
        }
        if (!tmp.empty()) {
            g_gun_presets = std::move(tmp);
            return true;
        }
    } catch (...) {
        return false;
    }
    return false;
}



// Load bullet tolerances JSON file (returns true if loaded)
bool LoadBulletTolerancesFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.good())
        return false;
    try {
        json j;
        ifs >> j;
        if (!j.is_object())
            return false;
        g_bullet_tolerances = j;
        return true;
    } catch (...) {
        return false;
    }
}

int FindCartridgePresetByName(const std::string& name) {
    for (size_t i = 0; i < g_cartridge_presets.size(); ++i) {
        if (g_cartridge_presets[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

float GetSelectedCartridgeCaliberInches() {
    if (g_state.selected_cartridge_preset >= 0 &&
        g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
        return g_cartridge_presets[g_state.selected_cartridge_preset].caliber_inches;
    }
    return g_state.bullet.caliber_inches;
}

const CartridgePreset* GetSelectedCartridgePreset() {
    if (g_state.selected_cartridge_preset >= 0 &&
        g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
        return &g_cartridge_presets[g_state.selected_cartridge_preset];
    }
    return nullptr;
}

bool CartridgeKeysIntersect(const std::vector<std::string>& a, const std::vector<std::string>& b) {
    if (a.empty() || b.empty())
        return false;
    auto normalize_key = [](const std::string& s) {
        std::string out;
        out.reserve(s.size());
        for (unsigned char c : s) {
            if (std::isalnum(c)) {
                out.push_back(static_cast<char>(std::tolower(c)));
            }
        }
        return out;
    };
    auto is_9x19_family = [](const std::string& k) {
        return k == "9x19" || k == "9x19mm" || k == "9mm" || k == "9mmluger" ||
               k == "9mmparabellum";
    };
    for (const auto& ka_raw : a) {
        const std::string ka = normalize_key(ka_raw);
        for (const auto& kb_raw : b) {
            const std::string kb = normalize_key(kb_raw);
            if (ka == kb)
                return true;
            if (is_9x19_family(ka) && is_9x19_family(kb))
                return true;
        }
    }
    return false;
}

bool IsGunPresetCompatibleWithSelectedCaliber(const GunPreset& preset) {
    const float selected_caliber = std::fabs(GetSelectedCartridgeCaliberInches());
    const float preset_caliber = std::fabs(preset.caliber_inches);
    const CartridgePreset* selected = GetSelectedCartridgePreset();
    const bool keys_match =
        selected && CartridgeKeysIntersect(selected->cartridge_keys, preset.cartridge_keys);
    if (keys_match)
        return true;
    if (selected && !selected->cartridge_keys.empty() && !preset.cartridge_keys.empty()) {
        // Both provided keys but none matched.
        return false;
    }
    return std::fabs(selected_caliber - preset_caliber) <= 0.0005f;
}

void EnsureSelectedGunPresetMatchesCurrentCaliber() {
    if (g_state.selected_gun_preset >= 0 &&
        g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
        IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[g_state.selected_gun_preset])) {
        return;
    }

    g_state.selected_gun_preset = -1;
    for (int i = 0; i < static_cast<int>(g_gun_presets.size()); ++i) {
        if (IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[i])) {
            g_state.selected_gun_preset = i;
            break;
        }
    }
}

static bool IsDefaultGunPresetName(const std::string& name) {
    std::string lowered = name;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lowered.find("default") != std::string::npos;
}

void SelectDefaultGunPresetForSelectedCartridge() {
    const CartridgePreset* selected = GetSelectedCartridgePreset();

    int first_key_match = -1;
    int default_key_match = -1;
    if (selected && !selected->cartridge_keys.empty()) {
        for (int i = 0; i < static_cast<int>(g_gun_presets.size()); ++i) {
            const GunPreset& gun = g_gun_presets[i];
            if (!CartridgeKeysIntersect(selected->cartridge_keys, gun.cartridge_keys)) {
                continue;
            }
            if (first_key_match < 0) {
                first_key_match = i;
            }
            if (default_key_match < 0 && IsDefaultGunPresetName(gun.name)) {
                default_key_match = i;
            }
        }
    }

    if (default_key_match >= 0) {
        g_state.selected_gun_preset = default_key_match;
        return;
    }
    if (first_key_match >= 0) {
        g_state.selected_gun_preset = first_key_match;
        return;
    }

    int first_compatible = -1;
    int default_compatible = -1;
    for (int i = 0; i < static_cast<int>(g_gun_presets.size()); ++i) {
        const GunPreset& gun = g_gun_presets[i];
        if (!IsGunPresetCompatibleWithSelectedCaliber(gun)) {
            continue;
        }
        if (first_compatible < 0) {
            first_compatible = i;
        }
        if (default_compatible < 0 && IsDefaultGunPresetName(gun.name)) {
            default_compatible = i;
        }
    }

    if (default_compatible >= 0) {
        g_state.selected_gun_preset = default_compatible;
        return;
    }
    g_state.selected_gun_preset = first_compatible;
}

void EnsureProfileDefaults() {
    // Try loading external preset files; otherwise seed runtime-editable
    // preset lists from built-ins on first launch. This prevents malformed
    // JSON from poisoning GUI state by sanitizing every entry afterwards.
    // Try both the tools/native_gui path (where presets are stored in the repo)
    // and the current working directory so the GUI works when launched either
    // from the repo root or from the tools/native_gui folder.
    const std::string rel_dir = "tools/native_gui/";
    const std::string cartridge_file_repo_v2 = rel_dir + "dope_gui_cartridges_v2.json";
    const std::string gun_file_repo_v2 = rel_dir + "dope_gui_guns_v2.json";
    const std::string hw_file_repo = rel_dir + "dope_gui_hardware.json";
    // Only use *_v2.json files for presets.
    bool loaded_cartridges = LoadCartridgePresetsFromFile(cartridge_file_repo_v2);
    bool loaded_guns = LoadGunPresetsFromFile(gun_file_repo_v2);
    (void)LoadHardwarePresetsFromFile(hw_file_repo);

    // If neither embedded tables nor per-cartridge sigma values exist, legacy file fallback remains
    // optional.
    if (g_bullet_tolerances.is_null() && !CartridgesHaveSigmaValues(g_cartridge_presets)) {
        const std::string tol_file_repo = rel_dir + "dope_gui_bullet_tolerances.json";
        (void)LoadBulletTolerancesFromFile(tol_file_repo);
        (void)LoadBulletTolerancesFromFile("dope_gui_bullet_tolerances.json");
    }

    if (!loaded_cartridges && g_cartridge_presets.empty()) {
        CartridgePreset fallback;
        fallback.name = "Custom Cartridge";
        fallback.bc = 0.0f;
        fallback.drag_model_index = 0;
        fallback.muzzle_velocity_ms = 0.0f;
        fallback.mass_grains = 0.0f;
        fallback.caliber_inches = 0.0f;
        fallback.length_inches = 0.0f;
        SanitizeCartridgePreset(fallback);
        g_cartridge_presets.push_back(fallback);
    }

    if (!loaded_guns && g_gun_presets.empty()) {
        GunPreset fallback_gun;
        fallback_gun.name = "Custom Gun";
        fallback_gun.caliber_inches = 0.0f;
        fallback_gun.barrel_length_in = 0.0f;
        fallback_gun.twist_rate_inches = 0.0f;
        fallback_gun.zero_range_m = 0.0f;
        fallback_gun.sight_height_mm = 0.0f;
        SanitizeGunPreset(fallback_gun);
        g_gun_presets.push_back(fallback_gun);
    }

    for (auto& preset : g_cartridge_presets) {
        SanitizeCartridgePreset(preset);
    }
    for (auto& preset : g_gun_presets) {
        SanitizeGunPreset(preset);
    }

    if (g_state.selected_cartridge_preset < 0 && !g_cartridge_presets.empty()) {
        g_state.selected_cartridge_preset = 0;
    }
    if (g_state.selected_gun_preset < 0 && !g_gun_presets.empty()) {
        g_state.selected_gun_preset = 0;
    }
}

float ClampValue(float value, float lo, float hi) {
    if (!std::isfinite(value))
        return lo;
    if (!std::isfinite(lo))
        lo = 0.0f;
    if (!std::isfinite(hi))
        hi = lo;
    if (hi < lo) {
        const float tmp = lo;
        lo = hi;
        hi = tmp;
    }
    if (value < lo)
        return lo;
    if (value > hi)
        return hi;
    return value;
}

int ClampPresetIndex(int value, int count, int fallback) {
    if (count <= 0)
        return 0;
    if (value >= 0 && value < count)
        return value;
    if (fallback >= 0 && fallback < count)
        return fallback;
    return count - 1;
}

void SanitizeKeyList(std::vector<std::string>& keys) {
    auto trim_ascii = [](std::string& s) {
        while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
            s.erase(s.begin());
        while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
            s.pop_back();
    };
    for (auto& k : keys) {
        trim_ascii(k);
        std::transform(k.begin(), k.end(), k.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    }
    keys.erase(
        std::remove_if(keys.begin(), keys.end(), [](const std::string& s) { return s.empty(); }),
        keys.end());
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
}

// Removed unused GetPresetMultiplier function

void SyncDerivedInputUncertaintySigmas() {
    auto interpolate_error_sigma = [](const DOPE_ErrorTable* table, float x) -> float {
        if (!table || !table->points || table->count <= 0) {
            return 0.0f;
        }
        if (table->count == 1) {
            return table->points[0].sigma;
        }
        if (x <= table->points[0].x) {
            return table->points[0].sigma;
        }
        if (x >= table->points[table->count - 1].x) {
            return table->points[table->count - 1].sigma;
        }
        for (int i = 0; i < table->count - 1; ++i) {
            const DOPE_ErrorPoint& a = table->points[i];
            const DOPE_ErrorPoint& b = table->points[i + 1];
            if (x <= b.x) {
                const float dx = b.x - a.x;
                if (dx <= 0.0f) {
                    return b.sigma;
                }
                const float t = (x - a.x) / dx;
                return a.sigma + t * (b.sigma - a.sigma);
            }
        }
        return table->points[table->count - 1].sigma;
    };

    g_state.mass_error_preset_index = ClampPresetIndex(
        g_state.mass_error_preset_index, kNumMassErrorPresets, kMassErrorDefaultIndex);
    g_state.length_error_preset_index = ClampPresetIndex(
        g_state.length_error_preset_index, kNumLengthErrorPresets, kLengthErrorDefaultIndex);
    g_state.caliber_error_preset_index = ClampPresetIndex(
        g_state.caliber_error_preset_index, kNumCaliberErrorPresets, kCaliberErrorDefaultIndex);
    g_state.hw_barometer_index = ClampPresetIndex(g_state.hw_barometer_index,
                                                  GetNumBarometerPresets(), kBarometerDefaultIndex);
    g_state.hw_lrf_index =
        ClampPresetIndex(g_state.hw_lrf_index, GetNumLRFPresets(), kLrfDefaultIndex);
    g_state.hw_temp_sensor_index = ClampPresetIndex(g_state.hw_temp_sensor_index,
                                                    GetNumTempSensorPresets(), kTempDefaultIndex);
    g_state.hw_gps_index =
        ClampPresetIndex(g_state.hw_gps_index, GetNumGpsLatPresets(), kGpsDefaultIndex);
    g_state.hw_mag_lat_index =
        ClampPresetIndex(g_state.hw_mag_lat_index, GetNumMagLatPresets(), kMagLatDefaultIndex);

    UncertaintyConfig uc_defaults = {};
    DOPE_GetDefaultUncertaintyConfig(&uc_defaults);

    const DOPE_ErrorTable* lrf_table = GetLRFTable(g_state.hw_lrf_index);
    const bool has_lrf_table = (lrf_table && lrf_table->points != nullptr && lrf_table->count > 0);
    g_state.uc_config.use_range_error_table = has_lrf_table;
    g_state.uc_config.range_error_table = has_lrf_table ? *lrf_table : DOPE_ErrorTable{nullptr, 0};
    if (has_lrf_table && std::isfinite(g_state.lrf_range)) {
        g_state.uc_config.sigma_range_m = interpolate_error_sigma(lrf_table, g_state.lrf_range);
    }

    const DOPE_ErrorTable* temp_table = GetTempTable(g_state.hw_temp_sensor_index);
    const bool has_temp_table =
        (temp_table && temp_table->points != nullptr && temp_table->count > 0);
    g_state.uc_config.use_temperature_error_table = has_temp_table;
    g_state.uc_config.temperature_error_table =
        has_temp_table ? *temp_table : DOPE_ErrorTable{nullptr, 0};
    if (has_temp_table && std::isfinite(g_state.baro_temp)) {
        g_state.uc_config.sigma_temperature_c =
            interpolate_error_sigma(temp_table, g_state.baro_temp);
    }

    const DOPE_ErrorTable* pressure_table = GetBarometerTable(g_state.hw_barometer_index);
    const bool has_pressure_table =
        (pressure_table && pressure_table->points != nullptr && pressure_table->count > 0);
    g_state.uc_config.use_pressure_delta_temp_error_table = has_pressure_table;
    g_state.uc_config.pressure_delta_temp_error_table =
        has_pressure_table ? *pressure_table : DOPE_ErrorTable{nullptr, 0};

    if (!std::isfinite(g_state.uc_config.pressure_uncalibrated_sigma_pa) ||
        g_state.uc_config.pressure_uncalibrated_sigma_pa < 0.0f) {
        g_state.uc_config.pressure_uncalibrated_sigma_pa =
            uc_defaults.pressure_uncalibrated_sigma_pa;
    }

    g_state.uc_config.pressure_is_calibrated = g_state.baro_is_calibrated &&
                                               g_state.baro_has_calibration_temp &&
                                               std::isfinite(g_state.baro_calibration_temp_c);
    g_state.uc_config.pressure_has_calibration_temp =
        g_state.baro_has_calibration_temp && std::isfinite(g_state.baro_calibration_temp_c);
    g_state.uc_config.pressure_calibration_temp_c = g_state.baro_calibration_temp_c;

    if (has_pressure_table) {
        if (g_state.uc_config.pressure_is_calibrated && std::isfinite(g_state.baro_temp)) {
            const float delta_temp_c =
                std::fabs(g_state.baro_temp - g_state.baro_calibration_temp_c);
            g_state.uc_config.sigma_pressure_pa =
                interpolate_error_sigma(pressure_table, delta_temp_c);
        } else {
            g_state.uc_config.sigma_pressure_pa = g_state.uc_config.pressure_uncalibrated_sigma_pa;
        }
    }

    if (g_state.hw_cant_index >= 0 && g_state.hw_cant_index < GetNumCantPresets()) {
        g_state.uc_config.sigma_cant_deg = GetCantSigma(g_state.hw_cant_index);
    }

    const int gps_mapped = GetMappedGpsLatPresetIndex(g_state.hw_gps_index);
    const int mag_mapped = GetMappedMagLatPresetIndex(g_state.hw_mag_lat_index);
    if (gps_mapped > 0 && gps_mapped < kNumGpsLatPresets) {
        g_state.uc_config.sigma_latitude_deg = kGpsLatPresets[gps_mapped].sigma_lat_deg;
    } else if (mag_mapped > 0 && mag_mapped < kNumMagLatPresets) {
        g_state.uc_config.sigma_latitude_deg = kMagLatPresets[mag_mapped].sigma_lat_deg;
    }
}

void SanitizeCartridgePreset(CartridgePreset& preset) {
    if (preset.name.empty()) {
        preset.name = "Unnamed Cartridge";
    }
    SanitizeKeyList(preset.cartridge_keys);
    preset.bc = ClampValue(preset.bc, 0.001f, 1.20f);
    preset.drag_model_index =
        static_cast<int>(ClampValue(static_cast<float>(preset.drag_model_index), 0.0f, 7.0f));
    // Keep muzzle_velocity_ms as a legacy single-point value; prefer velocity_profile
    if (!preset.velocity_profile.empty()) {
        // ensure first point is reasonable and sync muzzle_velocity_ms
        preset.muzzle_velocity_ms = ClampValue(preset.velocity_profile[0].second, 10.0f, 1500.0f);
    } else {
        preset.muzzle_velocity_ms = ClampValue(preset.muzzle_velocity_ms, 10.0f, 1500.0f);
        // seed single-point velocity_profile for newer editors
        preset.velocity_profile.clear();
        preset.velocity_profile.emplace_back(0.0f, preset.muzzle_velocity_ms);
    }
    // clamp reference barrel length
    preset.reference_barrel_inches = ClampValue(preset.reference_barrel_inches, 1.0f, 40.0f);
    // sanitize velocity profile points
    for (auto& p : preset.velocity_profile) {
        if (!std::isfinite(p.first))
            p.first = 0.0f;
        if (!std::isfinite(p.second))
            p.second = preset.muzzle_velocity_ms;
        p.second = ClampValue(p.second, 5.0f, 1500.0f);
    }
    // sanitize trajectory profile points
    for (auto& p : preset.trajectory_profile) {
        if (!std::isfinite(p.first))
            p.first = 0.0f;
        if (!std::isfinite(p.second))
            p.second = 0.0f;
    }
    // sanitize cartridge zero range: allow 0.0 meaning unspecified, otherwise clamp.
    if (!std::isfinite(preset.zero_range_m) || preset.zero_range_m < 0.0f)
        preset.zero_range_m = 0.0f;
    preset.zero_range_m = ClampValue(preset.zero_range_m, 0.0f, static_cast<float>(DOPE_MAX_RANGE_M));
    // sanitize available zero ranges
    for (auto &zr : preset.available_zero_ranges) {
        if (!std::isfinite(zr) || zr < 0.0f)
            zr = 0.0f;
        zr = ClampValue(zr, 0.0f, static_cast<float>(DOPE_MAX_RANGE_M));
    }
    // sanitize explicit trajectory families
    for (auto &tf : preset.trajectory_families) {
        if (!std::isfinite(tf.zero_range_m) || tf.zero_range_m < 0.0f)
            tf.zero_range_m = 0.0f;
        tf.zero_range_m = ClampValue(tf.zero_range_m, 0.0f, static_cast<float>(DOPE_MAX_RANGE_M));
        for (auto &pt : tf.trajectory_profile) {
            if (!std::isfinite(pt.first)) pt.first = 0.0f;
            if (!std::isfinite(pt.second)) pt.second = 0.0f;
        }
    }
    // Removed obsolete CEP fields and logic
    if (!std::isfinite(preset.muzzle_diameter_in) || preset.muzzle_diameter_in <= 0.0f)
        preset.muzzle_diameter_in = 0.7f;
    preset.muzzle_diameter_in = ClampValue(preset.muzzle_diameter_in, 0.2f, 3.0f);
    if (!std::isfinite(preset.cold_bore_velocity_bias) || preset.cold_bore_velocity_bias <= 0.0f)
        preset.cold_bore_velocity_bias = 8.5f;
    preset.cold_bore_velocity_bias = ClampValue(preset.cold_bore_velocity_bias, 1.0f, 50.0f);
    if (!std::isfinite(preset.angular_sigma_moa) || preset.angular_sigma_moa <= 0.0f)
        preset.angular_sigma_moa = 0.4f;
    preset.angular_sigma_moa = ClampValue(preset.angular_sigma_moa, 0.01f, 5.0f);
    if (!std::isfinite(preset.measured_cep50_moa) || preset.measured_cep50_moa < 0.0f)
        preset.measured_cep50_moa = 0.0f;
    if (!std::isfinite(preset.manufacturer_spec_moa) || preset.manufacturer_spec_moa < 0.0f)
        preset.manufacturer_spec_moa = 0.0f;
    if (!std::isfinite(preset.category_radial_moa) || preset.category_radial_moa < 0.0f)
        preset.category_radial_moa = 0.0f;
    if (!std::isfinite(preset.category_vertical_moa) || preset.category_vertical_moa < 0.0f)
        preset.category_vertical_moa = 0.0f;
    // sanitize explicit sigma fields (0 = derive)
    if (!std::isfinite(preset.sd_mv_fps) || preset.sd_mv_fps < 0.0f)
        preset.sd_mv_fps = 0.0f;
    // sanitize barrel-MV fields
    if (!std::isfinite(preset.mv_adjustment_fps_per_in) || preset.mv_adjustment_fps_per_in < 0.0f)
        preset.mv_adjustment_fps_per_in = 0.0f;
    preset.mv_adjustment_fps_per_in = ClampValue(preset.mv_adjustment_fps_per_in, 0.0f, 200.0f);
    {
        auto& bmp = preset.barrel_mv_profile;
        bmp.erase(std::remove_if(bmp.begin(), bmp.end(),
                                 [](const std::pair<float, float>& p) {
                                     return p.first <= 0.0f || p.second <= 0.0f ||
                                            !std::isfinite(p.first) || !std::isfinite(p.second);
                                 }),
                  bmp.end());
        std::sort(bmp.begin(), bmp.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        bmp.erase(std::unique(bmp.begin(), bmp.end(),
                              [](const auto& a, const auto& b) { return a.first == b.first; }),
                  bmp.end());
    }
    preset.mass_grains = ClampValue(preset.mass_grains, 20.0f, 1200.0f);
    preset.caliber_inches = ClampValue(std::fabs(preset.caliber_inches), 0.10f, 1.00f);
    preset.length_inches = ClampValue(preset.length_inches, 5.0f / 25.4f, 100.0f / 25.4f);
}

void SanitizeGunPreset(GunPreset& preset) {
    if (preset.name.empty()) {
        preset.name = "Unnamed Gun";
    }
    SanitizeKeyList(preset.cartridge_keys);
    preset.caliber_inches = ClampValue(std::fabs(preset.caliber_inches), 0.10f, 1.00f);
    preset.barrel_length_in = ClampValue(preset.barrel_length_in, 2.0f, 40.0f);
    preset.muzzle_diameter_in = ClampValue(preset.muzzle_diameter_in, 0.2f, 3.0f);
    preset.cold_bore_velocity_bias = ClampValue(preset.cold_bore_velocity_bias, 1.0f, 50.0f);
    preset.angular_sigma_moa = ClampValue(preset.angular_sigma_moa, 0.01f, 5.0f);
    // Normalize material to a known preset.
    preset.barrel_material = GetBarrelMaterialPreset(preset.barrel_material).material;
    preset.twist_rate_inches = ClampValue(std::fabs(preset.twist_rate_inches), 1.0f, 30.0f);
    preset.zero_range_m = ClampValue(preset.zero_range_m, 10.0f, 2500.0f);
    preset.sight_height_mm = ClampValue(preset.sight_height_mm, 5.0f, 120.0f);
}

json SerializeCartridgePreset(const CartridgePreset& input) {
    CartridgePreset preset = input;
    SanitizeCartridgePreset(preset);

    json item;
    item["name"] = preset.name;
    item["bc"] = preset.bc;
    item["drag_model_index"] = preset.drag_model_index;
    if (preset.supported_drag_models_mask != 0) {
        item["supported_drag_models"] = json::array();
        for (int i = 0; i < 8; ++i) {
            if (preset.supported_drag_models_mask & (1u << i)) {
                item["supported_drag_models"].push_back(std::string(kDragModelLabels[i]));
            }
        }
    }
    item["muzzle_velocity_ms"] = preset.muzzle_velocity_ms;
    item["reference_barrel_inches"] = preset.reference_barrel_inches;
    item["muzzle_diameter_in"] = preset.muzzle_diameter_in;
    item["zero_range_m"] = preset.zero_range_m;
    if (!preset.available_zero_ranges.empty()) {
        item["available_zero_ranges"] = preset.available_zero_ranges;
    }
    if (!preset.trajectory_families.empty()) {
        item["trajectory_families"] = json::array();
        for (const auto &tf : preset.trajectory_families) {
            json tfj;
            tfj["zero_range_m"] = tf.zero_range_m;
            tfj["trajectory_profile"] = json::array();
            for (const auto &pt : tf.trajectory_profile) {
                json ptj;
                ptj["distance_in"] = pt.first;
                ptj["trajectory_in"] = pt.second;
                tfj["trajectory_profile"].push_back(ptj);
            }
            item["trajectory_families"].push_back(tfj);
        }
    }
    // Removed barrel_finish_sigma_mv_fps (obsolete)
    if (preset.measured_cep50_moa > 0.0f)
        item["measured_cep50_moa"] = preset.measured_cep50_moa;
    if (preset.manufacturer_spec_moa > 0.0f)
        item["manufacturer_spec_moa"] = preset.manufacturer_spec_moa;
    if (preset.category_radial_moa > 0.0f)
        item["category_radial_moa"] = preset.category_radial_moa;
    if (preset.category_vertical_moa > 0.0f)
        item["category_vertical_moa"] = preset.category_vertical_moa;
    if (preset.mv_adjustment_fps_per_in > 0.0f)
        item["mv_adjustment_fps_per_in"] = preset.mv_adjustment_fps_per_in;
    if (!preset.barrel_mv_profile.empty()) {
        item["barrel_mv_profile"] = json::array();
        for (const auto& p : preset.barrel_mv_profile) {
            json bmp;
            bmp["barrel_in"] = p.first;
            bmp["mv_fps"] = p.second;
            item["barrel_mv_profile"].push_back(bmp);
        }
    }
    // velocity_profile
    item["velocity_profile"] = json::array();
    for (const auto& p : preset.velocity_profile) {
        json vp;
        vp["distance_inches"] = p.first;
        vp["velocity_ms"] = p.second;
        item["velocity_profile"].push_back(vp);
    }
    // trajectory_profile
    item["trajectory_profile"] = json::array();
    for (const auto& p : preset.trajectory_profile) {
        json tp;
        tp["distance_inches"] = p.first;
        tp["trajectory_inches"] = p.second;
        item["trajectory_profile"].push_back(tp);
    }
    if (preset.sd_mv_fps > 0.0f)
        item["sd_mv_fps"] = preset.sd_mv_fps;
    item["mass_grains"] = preset.mass_grains;
    item["caliber_inches"] = preset.caliber_inches;
    item["length_in"] = preset.length_inches;
    // tags removed
    if (!preset.cartridge_keys.empty()) {
        item["cartridge_keys"] = json::array();
        for (const auto& k : preset.cartridge_keys)
            item["cartridge_keys"].push_back(k);
    }
    return item;
}

json SerializeGunPreset(const GunPreset& input) {
    GunPreset preset = input;
    SanitizeGunPreset(preset);

    json item;
    item["name"] = preset.name;
    item["caliber_inches"] = preset.caliber_inches;
    item["barrel_length_in"] = preset.barrel_length_in;
    item["muzzle_diameter_in"] = preset.muzzle_diameter_in;
    item["cold_bore_velocity_bias"] = preset.cold_bore_velocity_bias;
    item["angular_sigma_moa"] = preset.angular_sigma_moa;
    item["barrel_material"] = BarrelMaterialKey(preset.barrel_material);
    item["twist_rate_inches"] = preset.twist_rate_inches;
    item["zero_range_m"] = preset.zero_range_m;
    item["sight_height_mm"] = preset.sight_height_mm;
    item["free_floated"] = preset.free_floated;
    item["suppressor_attached"] = preset.suppressor_attached;
    item["barrel_tuner_attached"] = preset.barrel_tuner_attached;
    if (!preset.cartridge_keys.empty()) {
        item["cartridge_keys"] = json::array();
        for (const auto& k : preset.cartridge_keys)
            item["cartridge_keys"].push_back(k);
    }
    return item;
}

float ComputeAutoDragCoefficient(const AmmoDatasetV2& bullet) {
    // Lightweight BC estimate used by GUI when manual drag override is disabled.
    // This keeps the GUI self-contained without requiring additional external profile data.
    const float caliber_in = ClampValue(std::fabs(bullet.caliber_inches), 0.10f, 1.00f);
    const float mass_gr = ClampValue(bullet.mass_grains, 20.0f, 1200.0f);
    const float mass_lb = mass_gr / 7000.0f;
    const float sectional_density = mass_lb / (caliber_in * caliber_in);

    const float length_in = ClampValue(bullet.length_mm * MM_TO_IN, 0.10f, 3.50f);
    const float length_calibers = length_in / caliber_in;

    float base_form_factor = 0.58f;
    switch (bullet.drag_model) {
    case DragModel::G1:
        base_form_factor = 0.58f;
        break;
    case DragModel::G2:
        base_form_factor = 0.62f;
        break;
    case DragModel::G3:
        base_form_factor = 0.70f;
        break;
    case DragModel::G4:
        base_form_factor = 0.68f;
        break;
    case DragModel::G5:
        base_form_factor = 0.63f;
        break;
    case DragModel::G6:
        base_form_factor = 0.60f;
        break;
    case DragModel::G7:
        base_form_factor = 0.52f;
        break;
    case DragModel::G8:
        base_form_factor = 0.56f;
        break;
    default:
        break;
    }

    const float length_adjust = ClampValue(1.0f - 0.08f * (length_calibers - 3.2f), 0.75f, 1.25f);
    const float form_factor = ClampValue(base_form_factor * length_adjust, 0.35f, 1.10f);
    const float coeff = sectional_density / form_factor;
    return ClampValue(coeff, 0.05f, 1.20f);
}

void CreateRenderTarget() {
    ID3D11Texture2D* pBackBuffer = nullptr;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    if (pBackBuffer) {
        g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
        pBackBuffer->Release();
    }
}

void CleanupRenderTarget() {
    if (g_mainRenderTargetView) {
        g_mainRenderTargetView->Release();
        g_mainRenderTargetView = nullptr;
    }
}

HRESULT CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT createDeviceFlags = 0;
#ifdef _DEBUG
    createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[2] = {
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_0,
    };

    HRESULT res = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2,
        D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);

    if (res == DXGI_ERROR_UNSUPPORTED) {
        res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr,
                                            createDeviceFlags, featureLevelArray, 2,
                                            D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice,
                                            &featureLevel, &g_pd3dDeviceContext);
    }

    if (FAILED(res)) {
        return res;
    }

    CreateRenderTarget();
    return S_OK;
}

void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_pSwapChain) {
        g_pSwapChain->Release();
        g_pSwapChain = nullptr;
    }
    if (g_pd3dDeviceContext) {
        g_pd3dDeviceContext->Release();
        g_pd3dDeviceContext = nullptr;
    }
    if (g_pd3dDevice) {
        g_pd3dDevice->Release();
        g_pd3dDevice = nullptr;
    }
}

void ResetStateDefaults() {
    // Canonical GUI defaults. This is the first place to tweak startup behavior.
    g_state = {};
    g_state.unit_system = UnitSystem::IMPERIAL;
    // All ballistic fields now set via AmmoDatasetV2 or GunProfile.
    g_state.barrel_finish_index = kBarrelFinishDefaultIndex;
    g_state.barrel_material_index = kDefaultBarrelMaterialIndex;
    g_state.gun.measured_cep50_moa = 0.0f;
    g_state.gun.manufacturer_spec_moa = 0.0f;
    g_mv_adjustment_estimated = false;
    g_active_mv_adj_fps_per_in = 0.0f;
    g_active_barrel_mv_profile.clear();

    g_state.zero.zero_range_m = 91.44f; // 100 yd
    g_state.zero.sight_height_mm = 38.1f;

    g_state.wind_speed_ms = 0.0f;
    g_state.wind_heading = 0.0f;
    g_state.latitude = 37.0f;

    g_state.free_floated = false;
    g_state.suppressor_attached = false;
    g_state.barrel_tuner_attached = false;

    g_state.baro_pressure = 101325.0f;
    g_state.baro_temp = 18.333f; // 65 ┬░F
    g_state.baro_humidity = 0.65f;
    g_state.lrf_range = 457.2f; // 500 yd

    g_state.imu_valid = true;
    g_state.mag_valid = true;
    g_state.baro_valid = true;
    g_state.baro_humidity_valid = true;
    g_state.lrf_valid = true;

    g_state.drag_model_index = 0;
    g_state.now_us = 0;
    g_state.last_action = "Startup";
    g_state.hw_barometer_index = kBarometerDefaultIndex;
    g_state.hw_lrf_index = kLrfDefaultIndex;
    g_state.hw_temp_sensor_index = kTempDefaultIndex;
    g_state.hw_gps_index = kGpsDefaultIndex;
    g_state.hw_mag_lat_index = kMagLatDefaultIndex;
    g_state.baro_is_calibrated = false;
    g_state.baro_has_calibration_temp = false;
    g_state.baro_calibration_temp_c = 0.0f;
    g_state.mass_error_preset_index = kMassErrorDefaultIndex;
    g_state.length_error_preset_index = kLengthErrorDefaultIndex;
    g_state.caliber_error_preset_index = kCaliberErrorDefaultIndex;
    DOPE_GetDefaultUncertaintyConfig(&g_state.uc_config);
    SyncDerivedInputUncertaintySigmas();
    std::snprintf(g_state.preset_path, sizeof(g_state.preset_path), "%s", "dope_gui_preset.json");
    std::snprintf(g_state.profile_library_path, sizeof(g_state.profile_library_path), "%s",
                  "dope_gui_profile_library.json");
    std::snprintf(g_state.new_cartridge_preset_name, sizeof(g_state.new_cartridge_preset_name),
                  "%s", "New Cartridge Preset");
    std::snprintf(g_state.new_gun_preset_name, sizeof(g_state.new_gun_preset_name), "%s",
                  "New Gun Preset");
    g_state.selected_cartridge_preset = -1;
    g_state.selected_gun_preset = -1;

    g_cartridge_presets.clear();
    g_gun_presets.clear();
    EnsureProfileDefaults();
    SelectDefaultGunPresetForSelectedCartridge();
}

void ApplyConfig() {
    // Push current GUI inputs into the DOPE engine.
    g_state.bullet.drag_model = kDragModels[g_state.drag_model_index];
    g_state.bullet.bc = g_state.override_drag_coefficient
                            ? ClampValue(g_state.manual_drag_coefficient, 0.001f, 1.20f)
                            : ComputeAutoDragCoefficient(g_state.bullet);
    g_state.gun.stiffness_moa = ComputeStiffnessMoa(g_state.gun.muzzle_diameter_in,
                                                   std::fabs(g_state.bullet.caliber_inches),
                                                   g_state.gun.barrel_material);
    g_state.gun.free_floated = g_state.free_floated;
    g_state.gun.suppressor_attached = g_state.suppressor_attached;
    g_state.gun.barrel_tuner_attached = g_state.barrel_tuner_attached;
    DOPE_SetGunProfile(&g_state.gun);
    DOPE_SetZeroConfig(&g_state.zero);
    DOPE_SetWindManual(g_state.wind_speed_ms, g_state.wind_heading);
    DOPE_SetLatitude(g_state.latitude);
    SyncDerivedInputUncertaintySigmas();
    DOPE_SetUncertaintyConfig(&g_state.uc_config);
    DOPE_SetDeferUncertainty(true);
}

SensorFrame BuildFrame() {
    // Build a synthetic sensor frame for the desktop harness.
    // Inputs mirror fields from the "Sensor Frame" panel.
    SensorFrame frame = {};
    g_state.now_us += FRAME_STEP_US;

    frame.timestamp_us = g_state.now_us;

    frame.accel_x = 0.0f;
    frame.accel_y = 0.0f;
    frame.accel_z = 9.81f;
    frame.gyro_x = 0.0f;
    frame.gyro_y = 0.0f;
    frame.gyro_z = 0.0f;
    frame.imu_valid = g_state.imu_valid;

    frame.mag_x = 25.0f;
    frame.mag_y = 0.0f;
    frame.mag_z = 40.0f;
    frame.mag_valid = g_state.mag_valid;

    frame.baro_pressure_pa = g_state.baro_pressure;
    frame.baro_temperature_c = g_state.baro_temp;
    frame.baro_humidity = g_state.baro_humidity;
    frame.baro_valid = g_state.baro_valid;
    frame.baro_humidity_valid = g_state.baro_humidity_valid;

    frame.lrf_range_m = g_state.lrf_range;
    frame.lrf_timestamp_us = g_state.now_us;
    frame.lrf_valid = g_state.lrf_valid;
    frame.target_elevation_m = g_state.target_elevation_m;
    frame.target_elevation_valid = true;

    frame.encoder_focal_length_mm = 0.0f;
    frame.encoder_valid = false;
    return frame;
}

void RunFrameUpdates(int frame_count) {
    for (int i = 0; i < frame_count; ++i) {
        SensorFrame frame = BuildFrame();
        DOPE_Update(&frame);
    }
}

// Forward declaration for async callbacks that refresh the displayed solution snapshot.
void RefreshOutput();

// Launch a background uncertainty computation if one is pending.
void KickUncertaintyJob() {
    if (g_uncertainty_job.exchange(true, std::memory_order_acq_rel))
        return;
    std::thread([] {
        {
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            DOPE_ComputeUncertainty();
            RefreshOutput();
        }
        g_uncertainty_job.store(false, std::memory_order_release);
    }).detach();
}

void RefreshOutput() {
    // Produce the human-readable diagnostics/solution text displayed in "DOPE Output".
    FiringSolution sol = {};
    DOPE_GetSolution(&sol);

    DOPE_Mode mode = DOPE_GetMode();
    uint32_t fault = DOPE_GetFaultFlags();
    uint32_t diag = DOPE_GetDiagFlags();

    const char* mode_text = "UNKNOWN";
    if (mode == DOPE_Mode::IDLE)
        mode_text = "IDLE";
    if (mode == DOPE_Mode::SOLUTION_READY)
        mode_text = "SOLUTION_READY";
    if (mode == DOPE_Mode::FAULT)
        mode_text = "FAULT";

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);

    auto appendFaultNames = [&](uint32_t flags) {
        bool first = true;
        auto add = [&](const char* name) {
            if (!first)
                ss << ", ";
            ss << name;
            first = false;
        };
        if (flags == DOPE_Fault::NONE) {
            ss << "none";
            return;
        }
        if (flags & DOPE_Fault::NO_RANGE)
            add("NO_RANGE");
        if (flags & DOPE_Fault::NO_BULLET)
            add("NO_BULLET");
        if (flags & DOPE_Fault::NO_MV)
            add("NO_MV");
        if (flags & DOPE_Fault::NO_BC)
            add("NO_BC");
        if (flags & DOPE_Fault::ZERO_UNSOLVABLE)
            add("ZERO_UNSOLVABLE");
        if (flags & DOPE_Fault::AHRS_UNSTABLE)
            add("AHRS_UNSTABLE");
        if (flags & DOPE_Fault::SENSOR_INVALID)
            add("SENSOR_INVALID");
    };

    auto appendDiagNames = [&](uint32_t flags) {
        bool first = true;
        auto add = [&](const char* name) {
            if (!first)
                ss << ", ";
            ss << name;
            first = false;
        };
        if (flags == DOPE_Diag::NONE) {
            ss << "none";
            return;
        }
        if (flags & DOPE_Diag::CORIOLIS_DISABLED)
            add("CORIOLIS_DISABLED");
        if (flags & DOPE_Diag::DEFAULT_PRESSURE)
            add("DEFAULT_PRESSURE");
        if (flags & DOPE_Diag::DEFAULT_TEMP)
            add("DEFAULT_TEMP");
        if (flags & DOPE_Diag::DEFAULT_HUMIDITY)
            add("DEFAULT_HUMIDITY");
        if (flags & DOPE_Diag::DEFAULT_ALTITUDE)
            add("DEFAULT_ALTITUDE");
        if (flags & DOPE_Diag::DEFAULT_WIND)
            add("DEFAULT_WIND");
        if (flags & DOPE_Diag::MAG_SUPPRESSED)
            add("MAG_SUPPRESSED");
        if (flags & DOPE_Diag::LRF_STALE)
            add("LRF_STALE");
    };

    ss << "Action: " << g_state.last_action << "\n";
    ss << "Mode: " << mode_text << "\n";
    ss << "Fault Flags: 0x" << std::hex << fault << std::dec << "\n";
    ss << "Fault Decode: ";
    appendFaultNames(fault);
    ss << "\n";
    ss << "Diag Flags:  0x" << std::hex << diag << std::dec << "\n";
    ss << "Diag Decode: ";
    appendDiagNames(diag);
    ss << "\n\n";

    if (mode == DOPE_Mode::IDLE) {
        ss << "Hint: IDLE means no valid firing solution yet. Apply config then feed frames with "
              "Step/Run 100.\n";
    }
    if (fault & DOPE_Fault::AHRS_UNSTABLE) {
        ss << "Hint: AHRS_UNSTABLE is expected in early frames. Use Run 100 or enough Step updates "
              "while IMU is static.\n";
    }
    if (fault & DOPE_Fault::NO_RANGE) {
        ss << "Hint: NO_RANGE means no accepted LRF sample (check LRF valid/range).\n";
    }
    if (diag & DOPE_Diag::DEFAULT_ALTITUDE) {
        ss << "Hint: DEFAULT_ALTITUDE is informational (not a fault).\n";
    }
    ss << "\n";

    const float velocity_at_target_fps = sol.velocity_at_target_ms * MPS_TO_FPS;
    const float range_yd = sol.range_m * M_TO_YD;
    const float horiz_range_yd = sol.horizontal_range_m * M_TO_YD;
    const float energy_ftlb = sol.energy_at_target_j * J_TO_FTLB;
    auto direction_label = [](float moa) -> const char* {
        if (moa > 0.01f)
            return "RIGHT";
        if (moa < -0.01f)
            return "LEFT";
        return "CENTER";
    };

    ss << "---[ ENGINE DEBUG ]---\n";
    ss << "Active Unit Sys: " << (g_state.unit_system == UnitSystem::IMPERIAL ? "IMPERIAL" : "METRIC") << "\n";
    ss << "Raw Wind Input: " << std::fixed << std::setprecision(2) << g_state.wind_speed_ms << " m/s\n";
    ss << "----------------------\n";

    if (g_state.unit_system == UnitSystem::IMPERIAL) {
        ss << "Drag Coefficient (G-model): " << g_state.bullet.bc;
        ss << (g_state.override_drag_coefficient ? " [manual]" : " [auto]") << "\n";
        ss << "Muzzle Velocity (fps): " << (g_state.bullet.muzzle_velocity_ms * MPS_TO_FPS) << "\n";
        ss << "Wind Speed (mph): " << (g_state.wind_speed_ms * MPS_TO_MPH) << "\n";
        ss << "Range (yd): " << range_yd << "\n";
        ss << "Horiz Range (yd): " << horiz_range_yd << "\n";
    } else {
        ss << "Drag Coefficient (G-model): " << g_state.bullet.bc;
        ss << (g_state.override_drag_coefficient ? " [manual]" : " [auto]") << "\n";
        ss << "Muzzle Velocity (m/s): " << g_state.bullet.muzzle_velocity_ms << "\n";
        ss << "Wind Speed (m/s): " << g_state.wind_speed_ms << "\n";
        ss << "Range (m): " << sol.range_m << "\n";
        ss << "Horiz Range (m): " << sol.horizontal_range_m << "\n";
    }
    ss << "Elevation Hold (MOA): " << sol.hold_elevation_moa << "\n";
    ss << "Windage Hold (MOA):   " << sol.hold_windage_moa << "\n";
    ss << "TOF (ms): " << sol.tof_ms << "\n";
    if (g_state.unit_system == UnitSystem::IMPERIAL) {
        ss << "Velocity @ target (fps): " << velocity_at_target_fps << "\n";
        ss << "Energy @ target (ft-lb): " << energy_ftlb << "\n";
    } else {
        ss << "Velocity @ target (m/s): " << sol.velocity_at_target_ms << "\n";
        ss << "Energy @ target (J): " << sol.energy_at_target_j << "\n";
    }
    ss << "Air density (kg/m^3): " << sol.air_density_kgm3 << "\n";
    ss << "Heading true (deg): " << sol.heading_deg_true << "\n";
    ss << "Cant angle (deg): " << sol.cant_angle_deg << "\n";
    ss << "Coriolis elev/wind (MOA): " << sol.coriolis_elevation_moa << " / "
       << sol.coriolis_windage_moa << "\n";
    ss << "Spin drift (MOA): " << sol.spin_drift_moa << "\n";
    ss << "Windage Breakdown (MOA, + = right hold):\n";
    ss << "  Wind only:        " << sol.wind_only_windage_moa << " ["
       << direction_label(sol.wind_only_windage_moa) << "]\n";
    ss << "  Earth spin total: " << sol.earth_spin_windage_moa << " (Coriolis "
       << sol.coriolis_windage_moa << ", Spin " << sol.spin_drift_moa << ") ["
       << direction_label(sol.earth_spin_windage_moa) << "]\n";
    ss << "  Offsets total:    " << sol.offsets_windage_moa << " ["
       << direction_label(sol.offsets_windage_moa) << "]\n";
    ss << "  Cant added:       " << sol.cant_windage_moa << " ["
       << direction_label(sol.cant_windage_moa) << "]\n";
    ss << "  Total windage:    " << sol.hold_windage_moa << " ["
       << direction_label(sol.hold_windage_moa) << "]\n";

    if (sol.uncertainty_valid) {
        const float rms_sigma = std::sqrt((sol.sigma_elevation_moa * sol.sigma_elevation_moa +
                                           sol.sigma_windage_moa * sol.sigma_windage_moa) *
                                          0.5f);
        const float cep50 = 1.1774f * rms_sigma;
        const float cep95 = 2.4477f * rms_sigma;
        ss << "\nUncertainty (1-sigma propagation):\n";
        ss << "  Elevation: " << sol.sigma_elevation_moa << " MOA\n";
        ss << "  Windage:   " << sol.sigma_windage_moa << " MOA\n";
        ss << "  50% CEP:   " << cep50 << " MOA\n";
        ss << "  95% CEP:   " << cep95 << " MOA\n";

        // Per-input variance breakdown
        const char* input_names[FiringSolution::kNumUncertaintyInputs] = {
            "MV",     "Range", "WindSpd",  "WindDir", "Temp",    "Press", "Humid",
            "SightH", "Cant",  "Latitude", "Twist",   "ZeroRng", "MVAdj"};
        const float total_var_e = sol.sigma_elevation_moa * sol.sigma_elevation_moa;
        const float total_var_w = sol.sigma_windage_moa * sol.sigma_windage_moa;
        ss << "  --- Per-input variance (MOA^2):  Elev / Wind ---\n";
        for (int i = 0; i < FiringSolution::kNumUncertaintyInputs; ++i) {
            char buf[128];
            float pct_e =
                (total_var_e > 1e-12f) ? (sol.uc_var_elev[i] / total_var_e * 100.0f) : 0.0f;
            float pct_w =
                (total_var_w > 1e-12f) ? (sol.uc_var_wind[i] / total_var_w * 100.0f) : 0.0f;
            std::snprintf(buf, sizeof(buf), "  %-8s %8.5f (%5.1f%%)  %8.5f (%5.1f%%)\n",
                          input_names[i], sol.uc_var_elev[i], pct_e, sol.uc_var_wind[i], pct_w);
            ss << buf;
        }
    } else if (g_state.uc_config.enabled) {
        ss << "\nUncertainty: enabled but not yet computed (need valid solution).\n";
    } else {
        ss << "\nUncertainty: disabled.\n";
    }

    g_state.output_text = ss.str();

    // Publish to display snapshot so the render loop can read it without
    // ever blocking on g_engine_mutex (which may be held during a long solve).
    {
        std::lock_guard<std::mutex> dlk(g_display_mutex);
        g_display_output = g_state.output_text;
        DOPE_GetSolution(&g_display_sol);
    }
}

// ---------------------------------------------------------------------------
// Barrel-length → MV helpers (glue layer only — engine not modified).
// ---------------------------------------------------------------------------

// Linear interpolation over a sorted (barrel_in, mv_fps) table.
static float InterpolateBarrelMvProfile(const std::vector<std::pair<float, float>>& profile,
                                        float barrel_in) {
    if (profile.empty())
        return 0.0f;
    if (barrel_in <= profile.front().first)
        return profile.front().second;
    if (barrel_in >= profile.back().first)
        return profile.back().second;
    for (size_t i = 1; i < profile.size(); ++i) {
        if (barrel_in <= profile[i].first) {
            const float t =
                (barrel_in - profile[i - 1].first) / (profile[i].first - profile[i - 1].first);
            return profile[i - 1].second + t * (profile[i].second - profile[i - 1].second);
        }
    }
    return profile.back().second;
}

// Tiered fps/in estimator — fallback when cartridge has no explicit value.
static float FallbackMvAdjFpsPerIn(float ref_mv_ms) {
    const float fps = ref_mv_ms * MPS_TO_FPS;
    if (fps < 550.0f)
        return 8.0f; // pistol / subsonic
    if (fps < 760.0f)
        return 12.0f; // pistol / SMG
    if (fps < 915.0f)
        return 20.0f; // intermediate / carbine
    if (fps < 1067.0f)
        return 28.0f; // rifle carbine
    return 38.0f;     // full-power rifle
}

static float ComputeStiffnessMoa(float muzzle_diameter_in, float bore_diameter_in,
                                 BarrelMaterial material) {
    if (!std::isfinite(muzzle_diameter_in) || muzzle_diameter_in <= 0.0f ||
        !std::isfinite(bore_diameter_in) || bore_diameter_in <= 0.0f) {
        return 0.75f; // mid-range default
    }
    const float wall = 0.5f * std::fmax(0.0f, muzzle_diameter_in - bore_diameter_in);
    float base = 1.25f;
    if (wall > 0.35f) {
        base = 0.20f; // Bull / Heavy
    } else if (wall >= 0.20f) {
        base = 0.75f; // Medium / Standard
    }
    const float scale = GetBarrelMaterialPreset(material).stiffness_scale;
    const float clamp_scale = std::fmax(0.5f, std::fmin(scale, 2.0f));
    return base * clamp_scale;
}

static int ResolveBarrelFinishIndex(float sigma_fps) {
    for (int i = 0; i < kNumBarrelFinishPresets; ++i) {
        if (std::fabs(kBarrelFinishPresets[i].sigma_mv_fps - sigma_fps) < 0.05f) {
            return i;
        }
    }
    return 0; // Custom
}

// Resolve MV for the current barrel length.  Three-tier priority:
//   1. g_active_barrel_mv_profile non-empty  → profile table interpolation
//   2. g_active_mv_adj_fps_per_in > 0        → linear fps/in from reference barrel
//   3. fallback estimator                     → tier-based fps/in from ref MV
void ComputeAdjustedMv() {
    const float barrel_in = g_state.gun.barrel_length_in;
    g_mv_adjustment_estimated = false;

    if (!g_active_barrel_mv_profile.empty()) {
        // Tier 1: profile lookup — MV is fully determined by barrel length.
        const float mv_fps = InterpolateBarrelMvProfile(g_active_barrel_mv_profile, barrel_in);
        g_state.bullet.muzzle_velocity_ms = mv_fps * FPS_TO_MPS;
        g_state.gun.reference_barrel_length_in = barrel_in; // no engine delta
        g_state.bullet.mv_adjustment_fps_per_in = 0.0f;
    } else {
        // Tier 2/3: linear adjustment from the cartridge's reference barrel.
        float fps_per_in = g_active_mv_adj_fps_per_in;
        if (fps_per_in <= 0.0f) {
            fps_per_in = FallbackMvAdjFpsPerIn(g_reference_mv_ms);
            g_mv_adjustment_estimated = true;
        }
        // Engine computes delta: actual_mv = muzzle_velocity_ms + (barrel - ref) * factor *
        // FPS_TO_MPS
        g_state.bullet.muzzle_velocity_ms = g_reference_mv_ms;
        g_state.bullet.mv_adjustment_fps_per_in = fps_per_in;
        // reference_barrel_length_in was set in ApplyCartridgePreset; leave it alone here.
    }
}

void ApplyCartridgePreset(const CartridgePreset& preset) {
    // Applying a preset intentionally enables manual drag override so the
    // exact saved BC is preserved instead of being recomputed heuristically.
    CartridgePreset normalized = preset;
    SanitizeCartridgePreset(normalized);

    g_state.bullet.bc = normalized.bc;
    g_state.drag_model_index = normalized.drag_model_index;

    // Capture reference MV from first velocity-profile point.
    if (!normalized.velocity_profile.empty()) {
        g_reference_mv_ms = normalized.velocity_profile[0].second;
    } else {
        g_reference_mv_ms = normalized.muzzle_velocity_ms;
    }

    // Populate glue-layer barrel-MV state from the cartridge preset.
    g_active_mv_adj_fps_per_in = normalized.mv_adjustment_fps_per_in;
    g_active_barrel_mv_profile = normalized.barrel_mv_profile;

    g_state.bullet.mass_grains = normalized.mass_grains;
    g_state.bullet.caliber_inches = normalized.caliber_inches;
    g_state.bullet.length_mm = normalized.length_inches * IN_TO_MM;
    g_state.gun.muzzle_diameter_in = normalized.muzzle_diameter_in;
    // Removed barrel_finish_sigma_mv_fps logic (no longer used)
    g_state.gun.measured_cep50_moa = normalized.measured_cep50_moa;
    g_state.gun.manufacturer_spec_moa = normalized.manufacturer_spec_moa;
    g_state.gun.category_radial_moa = normalized.category_radial_moa;
    g_state.gun.category_vertical_moa = normalized.category_vertical_moa;
    g_state.gun.stiffness_moa =
        ComputeStiffnessMoa(normalized.muzzle_diameter_in, std::fabs(normalized.caliber_inches),
                            g_state.gun.barrel_material);

    // Sync atmospheric baseline from cartridge preset
    if (normalized.env.temperature_f > -100.0f) {
        g_state.baro_temp = (normalized.env.temperature_f - 32.0f) * 5.0f / 9.0f;
    }
    if (normalized.env.pressure_inhg > 0.0f) {
        g_state.baro_pressure = normalized.env.pressure_inhg * 3386.39f;
    }
    if (normalized.env.humidity_pct >= 0.0f) {
        g_state.baro_humidity = normalized.env.humidity_pct / 100.0f;
    }
    g_state.baro_has_calibration_temp = false; // reset calibration state to new baseline
    g_state.baro_is_calibrated = false;

    // Start with the reference barrel as both actual and reference length.
    if (normalized.reference_barrel_inches > 0.0f) {
        g_state.gun.barrel_length_in = normalized.reference_barrel_inches;
        g_state.gun.reference_barrel_length_in = normalized.reference_barrel_inches;
    }

    // Removed obsolete cartridge CEP fields and logic
    g_state.override_drag_coefficient = true;
    g_state.manual_drag_coefficient = normalized.bc;

    // Resolve MV for the reference barrel length.
    ComputeAdjustedMv();

    SelectDefaultGunPresetForSelectedCartridge();
    if (g_state.selected_gun_preset >= 0 &&
        g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
        IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[g_state.selected_gun_preset])) {
        ApplyGunPreset(g_gun_presets[g_state.selected_gun_preset]);
    }
    // Debug: report glue-layer values set by applying the cartridge preset.
    std::printf("[DEBUG] ApplyCartridgePreset: name= %s, mass=%.2f gr, caliber=%.4f in, length=%.2f in, mv_ms=%.2f, bc=%.4f, drag_idx=%d\n",
                normalized.name.c_str(), normalized.mass_grains, normalized.caliber_inches,
                normalized.length_inches, g_state.bullet.muzzle_velocity_ms, g_state.bullet.bc,
                g_state.drag_model_index);
}

void ApplyGunPreset(const GunPreset& preset) {
    GunPreset normalized = preset;
    SanitizeGunPreset(normalized);

    g_state.gun.barrel_length_in = normalized.barrel_length_in;
    g_state.gun.muzzle_diameter_in = normalized.muzzle_diameter_in;
    g_state.gun.cold_bore_velocity_bias = normalized.cold_bore_velocity_bias;
    g_state.gun.angular_sigma_moa = normalized.angular_sigma_moa;
    g_state.gun.barrel_material = normalized.barrel_material;
    g_state.barrel_material_index = ResolveBarrelMaterialIndex(normalized.barrel_material);
    g_state.bullet.twist_rate_inches = normalized.twist_rate_inches;
    g_state.zero.zero_range_m = normalized.zero_range_m;
    g_state.zero.sight_height_mm = normalized.sight_height_mm;
    g_state.free_floated = normalized.free_floated;
    g_state.suppressor_attached = normalized.suppressor_attached;
    g_state.barrel_tuner_attached = normalized.barrel_tuner_attached;
    g_state.gun.free_floated = normalized.free_floated;
    g_state.gun.suppressor_attached = normalized.suppressor_attached;
    g_state.gun.barrel_tuner_attached = normalized.barrel_tuner_attached;
    g_state.gun.stiffness_moa =
        ComputeStiffnessMoa(normalized.muzzle_diameter_in, std::fabs(normalized.caliber_inches),
                            normalized.barrel_material);
    ComputeAdjustedMv();
}

CartridgePreset CaptureCurrentCartridgePreset(const std::string& name) {
    CartridgePreset preset;
    preset.name = name;
    preset.bc = g_state.override_drag_coefficient
                    ? ClampValue(g_state.manual_drag_coefficient, 0.001f, 1.20f)
                    : ComputeAutoDragCoefficient(g_state.bullet);
    preset.drag_model_index = g_state.drag_model_index;
    // Use reference MV and barrel from glue-layer state (not the adjusted/current values).
    const float ref_mv =
        (g_reference_mv_ms > 0.0f) ? g_reference_mv_ms : g_state.bullet.muzzle_velocity_ms;
    preset.muzzle_velocity_ms = ref_mv;
    preset.reference_barrel_inches = (g_state.gun.reference_barrel_length_in > 0.0f)
                                         ? g_state.gun.reference_barrel_length_in
                                         : g_state.gun.barrel_length_in;
    preset.velocity_profile.clear();
    preset.velocity_profile.emplace_back(0.0f, ref_mv);
    preset.trajectory_profile.clear();
    preset.mass_grains = g_state.bullet.mass_grains;
    preset.caliber_inches = g_state.bullet.caliber_inches;
    preset.length_inches = g_state.bullet.length_mm * MM_TO_IN;
    preset.muzzle_diameter_in = g_state.gun.muzzle_diameter_in;
    // Removed barrel_finish_sigma_mv_fps (no longer used)
    preset.measured_cep50_moa = g_state.gun.measured_cep50_moa;
    preset.manufacturer_spec_moa = g_state.gun.manufacturer_spec_moa;
    preset.category_radial_moa = g_state.gun.category_radial_moa;
    preset.category_vertical_moa = g_state.gun.category_vertical_moa;
    preset.sd_mv_fps = g_state.new_cartridge_sd_mv_fps;
    // barrel-MV data from glue-layer globals
    preset.mv_adjustment_fps_per_in = g_active_mv_adj_fps_per_in;
    preset.barrel_mv_profile = g_active_barrel_mv_profile;
    // tags removed

    if (g_state.selected_cartridge_preset >= 0 &&
        g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
        preset.cartridge_keys =
            g_cartridge_presets[g_state.selected_cartridge_preset].cartridge_keys;
    }

    SanitizeCartridgePreset(preset);
    return preset;
}

void SaveProfileLibrary() {
    EnsureProfileDefaults();

    json root;
    root["cartridge_presets"] = json::array();
    root["gun_presets"] = json::array();

    for (const auto& preset : g_cartridge_presets) {
        root["cartridge_presets"].push_back(SerializeCartridgePreset(preset));
    }

    for (const auto& preset : g_gun_presets) {
        root["gun_presets"].push_back(SerializeGunPreset(preset));
    }

    std::ofstream out(g_state.profile_library_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        g_state.last_action = "Save Profile Library (failed)";
        g_state.output_text = "Profile library save failed: cannot open file.";
        return;
    }
    out << root.dump(2);
    g_state.last_action = "Save Profile Library";
}

void LoadProfileLibrary() {
    std::ifstream in(g_state.profile_library_path, std::ios::binary);
    if (!in) {
        g_state.last_action = "Load Profile Library (failed)";
        g_state.output_text = "Profile library load failed: cannot open file.";
        return;
    }

    try {
        json root;
        in >> root;

        // Load into temporary containers first; only commit once parsing and
        // sanitization succeed to avoid partially-applied profile libraries.
        std::vector<CartridgePreset> loaded_cartridge;
        std::vector<GunPreset> loaded_gun;

        if (root.contains("cartridge_presets") && root["cartridge_presets"].is_array()) {
            for (const auto& item : root["cartridge_presets"]) {
                if (!item.contains("name") || !item["name"].is_string()) {
                    continue;
                }
                CartridgePreset preset;
                preset.name = item["name"].get<std::string>();
                preset.bc = item.value("bc", 0.0f);
                preset.drag_model_index = item.value("drag_model_index", 0);
                if (item.contains("ballistic_coefficients") && item["ballistic_coefficients"].is_object()) {
                    for (auto it = item["ballistic_coefficients"].begin(); it != item["ballistic_coefficients"].end(); ++it) {
                        if (it.value().is_number()) {
                            preset.ballistic_coefficients[it.key()] = it.value().get<float>();
                        }
                    }
                }
                preset.preferred_drag_model = item.value("preferred_drag_model", "G1");
                if (preset.bc <= 0.0f && preset.ballistic_coefficients.count(preset.preferred_drag_model)) {
                    preset.bc = preset.ballistic_coefficients[preset.preferred_drag_model];
                }
                if (preset.bc <= 0.0f) {
                    preset.bc = item.value("ballistic_coefficient", 0.505f);
                }

                if (item.contains("drag_model_index")) {
                    preset.drag_model_index = item["drag_model_index"].get<int>();
                } else if (item.contains("drag_model") && item["drag_model"].is_string()) {
                    std::string dm = item["drag_model"].get<std::string>();
                    for (int i = 0; i < 8; ++i) {
                        if (dm == kDragModelLabels[i]) {
                            preset.drag_model_index = i;
                            break;
                        }
                    }
                } else {
                    for (int i = 0; i < 8; ++i) {
                        if (preset.preferred_drag_model == kDragModelLabels[i]) {
                            preset.drag_model_index = i;
                            break;
                        }
                    }
                }

                if (preset.drag_model_index < 0 || preset.drag_model_index >= 8) {
                    preset.drag_model_index = 0;
                }
                // Parse explicit supported-drag-models array (optional). Expected
                // format: ["G1","G7"] or similar. If absent, fall back to the
                // legacy single-index field.
                preset.supported_drag_models_mask = 0;
                if (item.contains("supported_drag_models") && item["supported_drag_models"].is_array()) {
                    for (const auto& mv : item["supported_drag_models"]) {
                        if (!mv.is_string()) continue;
                        std::string s = mv.get<std::string>();
                        if (s.size() >= 2 && (s[0] == 'G' || s[0] == 'g')) {
                            try {
                                int m = std::stoi(s.substr(1));
                                if (m >= 1 && m <= 8) {
                                    preset.supported_drag_models_mask |= static_cast<uint8_t>(1u << (m - 1));
                                }
                            } catch (...) {
                                continue;
                            }
                        }
                    }
                }
                if (preset.supported_drag_models_mask == 0) {
                    preset.supported_drag_models_mask = static_cast<uint8_t>(1u << preset.drag_model_index);
                }
                if (item.contains("muzzle_velocity_fps") &&
                    item["muzzle_velocity_fps"].is_number()) {
                    preset.muzzle_velocity_ms =
                        item["muzzle_velocity_fps"].get<float>() * FPS_TO_MPS;
                } else {
                    preset.muzzle_velocity_ms = item.value("muzzle_velocity_ms", 792.0f);
                }
                preset.mass_grains = item.value("mass_grains", 175.0f);
                preset.caliber_inches = item.value("caliber_inches", 0.308f);
                if (item.contains("length_in") && item["length_in"].is_number()) {
                    preset.length_inches = item["length_in"].get<float>();
                } else {
                    preset.length_inches = item.value("length_mm", 31.2f) * MM_TO_IN;
                }
                preset.reference_barrel_inches = item.value("reference_barrel_inches", 24.0f);
                preset.velocity_profile.clear();
                if (item.contains("velocity_profile") && item["velocity_profile"].is_array()) {
                    for (const auto& vp : item["velocity_profile"]) {
                        const float dist = vp.value("distance_inches", 0.0f);
                        float vel = 0.0f;
                        if (vp.contains("velocity_ms") && vp["velocity_ms"].is_number()) {
                            vel = vp["velocity_ms"].get<float>();
                        } else if (vp.contains("velocity_fps") && vp["velocity_fps"].is_number()) {
                            vel = vp["velocity_fps"].get<float>() * FPS_TO_MPS;
                        } else {
                            vel = vp.value("velocity_mps", 0.0f);
                        }
                        preset.velocity_profile.emplace_back(dist, vel);
                    }
                } else {
                    preset.velocity_profile.emplace_back(0.0f, preset.muzzle_velocity_ms);
                }
                preset.trajectory_profile.clear();
                if (item.contains("trajectory_profile") && item["trajectory_profile"].is_array()) {
                    for (const auto& tp : item["trajectory_profile"]) {
                        const float dist = tp.value("distance_inches", 0.0f);
                        const float traj =
                            tp.value("trajectory_inches", tp.value("trajectory_in", 0.0f));
                        preset.trajectory_profile.emplace_back(dist, traj);
                    }
                }
                preset.muzzle_diameter_in = item.value("muzzle_diameter_in", 0.7f);
                preset.cold_bore_velocity_bias = item.value("cold_bore_velocity_bias", 8.5f);
                preset.angular_sigma_moa = item.value("angular_sigma_moa", 0.4f);
                preset.measured_cep50_moa = item.value("measured_cep50_moa", 0.0f);
                preset.manufacturer_spec_moa = item.value("manufacturer_spec_moa", 0.0f);
                preset.category_radial_moa = item.value("category_radial_moa", 0.0f);
                preset.category_vertical_moa = item.value("category_vertical_moa", 0.0f);
                SanitizeCartridgePreset(preset);
                loaded_cartridge.push_back(preset);
            }
        }

        if (root.contains("gun_presets") && root["gun_presets"].is_array()) {
            for (const auto& item : root["gun_presets"]) {
                if (!item.contains("name") || !item["name"].is_string()) {
                    continue;
                }
                GunPreset preset;
                preset.name = item["name"].get<std::string>();
                preset.caliber_inches = item.value("caliber_inches", 0.308f);
                preset.barrel_length_in = item.value("barrel_length_in", 24.0f);
                preset.muzzle_diameter_in = item.value("muzzle_diameter_in", 0.7f);
                preset.cold_bore_velocity_bias = item.value("cold_bore_velocity_bias", 8.5f);
                preset.angular_sigma_moa = item.value("angular_sigma_moa", 0.4f);
                const std::string mat_key = item.value("barrel_material", std::string(""));
                preset.barrel_material =
                    mat_key.empty() ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
                                    : ParseBarrelMaterialKey(mat_key);
                preset.twist_rate_inches = item.value("twist_rate_inches", 10.0f);
                preset.zero_range_m = item.value("zero_range_m", 100.0f);
                preset.sight_height_mm = item.value("sight_height_mm", 38.1f);
                preset.free_floated = item.value("free_floated", false);
                preset.suppressor_attached = item.value("suppressor_attached", false);
                preset.barrel_tuner_attached = item.value("barrel_tuner_attached", false);
                SanitizeGunPreset(preset);
                loaded_gun.push_back(preset);
            }
        } else if (root.contains("rifle_presets") && root["rifle_presets"].is_array()) {
            for (const auto& item : root["rifle_presets"]) {
                if (!item.contains("name") || !item["name"].is_string()) {
                    continue;
                }
                GunPreset preset;
                preset.name = item["name"].get<std::string>();
                preset.caliber_inches = GetSelectedCartridgeCaliberInches();
                preset.barrel_length_in = item.value("barrel_length_in", 24.0f);
                preset.muzzle_diameter_in = item.value("muzzle_diameter_in", 0.7f);
                // Removed: preset.barrel_finish_sigma_mv_fps (GunPreset does not have this member)
                const std::string mat_key = item.value("barrel_material", std::string(""));
                preset.barrel_material =
                    mat_key.empty() ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
                                    : ParseBarrelMaterialKey(mat_key);
                preset.twist_rate_inches = item.value("twist_rate_inches", 10.0f);
                preset.zero_range_m = item.value("zero_range_m", 100.0f);
                preset.sight_height_mm = item.value("sight_height_mm", 38.1f);
                preset.free_floated = item.value("free_floated", false);
                preset.suppressor_attached = item.value("suppressor_attached", false);
                preset.barrel_tuner_attached = item.value("barrel_tuner_attached", false);
                SanitizeGunPreset(preset);
                loaded_gun.push_back(preset);
            }
        }

        if (!loaded_cartridge.empty()) {
            g_cartridge_presets = loaded_cartridge;
            g_state.selected_cartridge_preset = 0;
        }
        if (!loaded_gun.empty()) {
            g_gun_presets = loaded_gun;
            g_state.selected_gun_preset = 0;
        }

        EnsureProfileDefaults();
        SelectDefaultGunPresetForSelectedCartridge();
        g_state.last_action = "Load Profile Library";
    } catch (const json::parse_error& e) {
        g_state.last_action = "Load Profile Library (failed)";
        std::string error_msg = "Profile library load failed: JSON parse error: ";
        error_msg += e.what();
        g_state.output_text = error_msg;
    }
}

void SavePreset() {
    // Persist full GUI state to JSON so users can iterate on profiles quickly.
    json j;
    j["unit_system"] = (g_state.unit_system == UnitSystem::IMPERIAL ? "imperial" : "metric");
    j["drag_coefficient_override"] = g_state.override_drag_coefficient;
    j["drag_coefficient"] = g_state.manual_drag_coefficient;
    j["bc"] = g_state.bullet.bc;
    j["drag_model_index"] = g_state.drag_model_index;
    j["muzzle_velocity_ms"] = g_state.bullet.muzzle_velocity_ms;
    j["mass_grains"] = g_state.bullet.mass_grains;
    j["length_mm"] = g_state.bullet.length_mm;
    j["caliber_inches"] = g_state.bullet.caliber_inches;
    j["twist_rate_inches"] = g_state.bullet.twist_rate_inches;
    j["barrel_length_in"] = g_state.gun.barrel_length_in;
    j["muzzle_diameter_in"] = g_state.gun.muzzle_diameter_in;
    j["barrel_material"] = BarrelMaterialKey(g_state.gun.barrel_material);
    j["free_floated"] = g_state.free_floated;
    j["suppressor_attached"] = g_state.suppressor_attached;
    j["barrel_tuner_attached"] = g_state.barrel_tuner_attached;
    j["cold_bore_velocity_bias"] = g_state.gun.cold_bore_velocity_bias;
    j["angular_sigma_moa"] = g_state.gun.angular_sigma_moa;
    j["measured_cep50_moa"] = g_state.gun.measured_cep50_moa;
    j["manufacturer_spec_moa"] = g_state.gun.manufacturer_spec_moa;
    j["category_radial_moa"] = g_state.gun.category_radial_moa;
    j["category_vertical_moa"] = g_state.gun.category_vertical_moa;
    j["stiffness_moa"] = g_state.gun.stiffness_moa;
    j["reference_mv_ms"] = g_reference_mv_ms;
    j["mv_adj_fps_per_in_active"] = g_active_mv_adj_fps_per_in;
    j["mv_adj_estimated"] = g_mv_adjustment_estimated;
    j["zero_range_m"] = g_state.zero.zero_range_m;
    j["sight_height_mm"] = g_state.zero.sight_height_mm;
    j["wind_speed_ms"] = g_state.wind_speed_ms;
    j["wind_heading_deg"] = g_state.wind_heading;
    j["latitude_deg"] = g_state.latitude;
    j["baro_pressure_pa"] = g_state.baro_pressure;
    j["baro_temperature_c"] = g_state.baro_temp;
    j["baro_humidity"] = g_state.baro_humidity;
    j["lrf_range_m"] = g_state.lrf_range;
    j["target_elevation_m"] = g_state.target_elevation_m;
    j["uc_enabled"] = g_state.uc_config.enabled;
    j["uc_sigma_mv_ms"] = g_state.uc_config.sigma_muzzle_velocity_ms;
    j["hw_barometer_index"] = g_state.hw_barometer_index;
    j["hw_lrf_index"] = g_state.hw_lrf_index;
    j["hw_cant_index"] = g_state.hw_cant_index;
    j["hw_temp_sensor_index"] = g_state.hw_temp_sensor_index;
    j["hw_gps_index"] = g_state.hw_gps_index;
    j["hw_mag_lat_index"] = g_state.hw_mag_lat_index;
    j["baro_calibrated"] = g_state.baro_is_calibrated;
    j["baro_calibration_temp_valid"] = g_state.baro_has_calibration_temp;
    j["baro_calibration_temp_c"] = g_state.baro_calibration_temp_c;
    j["uc_mass_error_preset_index"] = g_state.mass_error_preset_index;
    j["uc_length_error_preset_index"] = g_state.length_error_preset_index;
    j["uc_caliber_error_preset_index"] = g_state.caliber_error_preset_index;
    j["imu_valid"] = g_state.imu_valid;
    j["mag_valid"] = g_state.mag_valid;
    j["baro_valid"] = g_state.baro_valid;
    j["baro_humidity_valid"] = g_state.baro_humidity_valid;
    j["lrf_valid"] = g_state.lrf_valid;

    std::ofstream out(g_state.preset_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        g_state.last_action = "Save Preset (failed)";
        g_state.output_text = "Preset save failed: cannot open file.";
        return;
    }
    out << j.dump(2);
}

void LoadPreset() {
    // Backward-compatible preset loading:
    // supports both SI and legacy imperial keys where available.
    std::ifstream in(g_state.preset_path, std::ios::binary);
    if (!in) {
        g_state.last_action = "Load Preset (failed)";
        g_state.output_text = "Preset load failed: cannot open file.";
        return;
    }

    try {
        json j;
        in >> j;

        if (j.contains("unit_system") && j["unit_system"].is_string()) {
            if (j["unit_system"] == "metric") {
                g_state.unit_system = UnitSystem::METRIC;
            } else {
                g_state.unit_system = UnitSystem::IMPERIAL;
            }
        }

        g_state.override_drag_coefficient = j.value("drag_coefficient_override", false);
        g_state.manual_drag_coefficient = j.value("drag_coefficient", 0.0f);

        bool has_bc = false;
        if (j.contains("bc") && j["bc"].is_number()) {
            g_state.bullet.bc = j["bc"];
            has_bc = true;
            if (!j.contains("drag_coefficient_override")) {
                g_state.override_drag_coefficient = true;
                g_state.manual_drag_coefficient = g_state.bullet.bc;
            }
        }

        g_state.drag_model_index = j.value("drag_model_index", 0);
        if (g_state.drag_model_index < 0 || g_state.drag_model_index >= 8) {
            g_state.drag_model_index = 0;
        }

        // Accept legacy imperial keys first, then modern SI fields.
        if (j.contains("muzzle_velocity_fps") && j["muzzle_velocity_fps"].is_number()) {
            g_state.bullet.muzzle_velocity_ms = j["muzzle_velocity_fps"].get<float>() * FPS_TO_MPS;
        } else {
            g_state.bullet.muzzle_velocity_ms = j.value("muzzle_velocity_ms", 0.0f);
        }

        g_state.bullet.mass_grains = j.value("mass_grains", 0.0f);
        if (j.contains("length_in") && j["length_in"].is_number()) {
            g_state.bullet.length_mm = j["length_in"].get<float>() * IN_TO_MM;
        } else {
            g_state.bullet.length_mm = j.value("length_mm", 0.0f);
        }
        g_state.bullet.caliber_inches = j.value("caliber_inches", 0.0f);
        g_state.bullet.twist_rate_inches = j.value("twist_rate_inches", 0.0f);
        g_state.gun.barrel_length_in = j.value("barrel_length_in", 24.0f);
        g_state.gun.muzzle_diameter_in = j.value("muzzle_diameter_in", 0.7f);
        const std::string barrel_material_key = j.value("barrel_material", std::string(""));
        g_state.gun.barrel_material =
            barrel_material_key.empty()
                ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
                : ParseBarrelMaterialKey(barrel_material_key);
        g_state.barrel_material_index = ResolveBarrelMaterialIndex(g_state.gun.barrel_material);
        g_state.free_floated = j.value("free_floated", false);
        g_state.suppressor_attached = j.value("suppressor_attached", false);
        g_state.barrel_tuner_attached = j.value("barrel_tuner_attached", false);
        g_state.gun.free_floated = g_state.free_floated;
        g_state.gun.suppressor_attached = g_state.suppressor_attached;
        g_state.gun.barrel_tuner_attached = g_state.barrel_tuner_attached;
        g_state.barrel_finish_sigma_mv_fps = j.value("barrel_finish_sigma_mv_fps", 8.5f);
        g_state.gun.measured_cep50_moa = j.value("measured_cep50_moa", 0.0f);
        g_state.gun.manufacturer_spec_moa = j.value("manufacturer_spec_moa", 0.0f);
        g_state.gun.category_radial_moa = j.value("category_radial_moa", 0.0f);
        g_state.gun.category_vertical_moa = j.value("category_vertical_moa", 0.0f);
        g_state.gun.stiffness_moa =
            j.value("stiffness_moa", ComputeStiffnessMoa(g_state.gun.muzzle_diameter_in,
                                                         std::fabs(g_state.bullet.caliber_inches),
                                                         g_state.gun.barrel_material));
        if (j.contains("barrel_finish_index")) {
            g_state.barrel_finish_index = j["barrel_finish_index"];
        } else {
            g_state.barrel_finish_index =
                ResolveBarrelFinishIndex(g_state.barrel_finish_sigma_mv_fps);
        }
        g_reference_mv_ms = j.value("reference_mv_ms", g_state.bullet.muzzle_velocity_ms);
        g_active_mv_adj_fps_per_in = j.value("mv_adj_fps_per_in_active", 0.0f);
        g_mv_adjustment_estimated = j.value("mv_adj_estimated", false);
        g_state.zero.zero_range_m = j.value("zero_range_m", 0.0f);
        g_state.zero.sight_height_mm = j.value("sight_height_mm", 0.0f);

        if (j.contains("wind_speed_mph") && j["wind_speed_mph"].is_number()) {
            g_state.wind_speed_ms = j["wind_speed_mph"].get<float>() * MPH_TO_MPS;
        } else if (j.contains("wind_speed_fps") && j["wind_speed_fps"].is_number()) {
            g_state.wind_speed_ms = j["wind_speed_fps"].get<float>() * FPS_TO_MPS;
        } else {
            g_state.wind_speed_ms = j.value("wind_speed_ms", 0.0f);
        }

        g_state.wind_heading = j.value("wind_heading_deg", 0.0f);
        if (j.contains("latitude_deg") && j["latitude_deg"].is_number()) {
            g_state.latitude = j["latitude_deg"].get<float>();
            if (std::fabs(g_state.latitude) < 0.0001f) {
                g_state.latitude = 37.0f;
            }
        } else {
            g_state.latitude = 37.0f;
        }
        g_state.baro_pressure = j.value("baro_pressure_pa", 101325.0f);
        g_state.baro_temp = j.value("baro_temperature_c", 18.333f);
        g_state.baro_humidity = j.value("baro_humidity", 0.5f);
        g_state.lrf_range = j.value("lrf_range_m", 500.0f);
        g_state.target_elevation_m = j.value("target_elevation_m", 0.0f);

        // Uncertainty config ΓÇö load with per-field defaults matching DOPE defaults
        UncertaintyConfig uc_def = {};
        DOPE_GetDefaultUncertaintyConfig(&uc_def);
        g_state.uc_config.enabled = j.value("uc_enabled", uc_def.enabled);
        g_state.uc_config.sigma_muzzle_velocity_ms =
            j.value("uc_sigma_mv_ms", uc_def.sigma_muzzle_velocity_ms);
        // Removed obsolete sigma_bc field
        g_state.uc_config.sigma_range_m = j.value("uc_sigma_range_m", uc_def.sigma_range_m);
        g_state.uc_config.sigma_wind_speed_ms =
            j.value("uc_sigma_wind_speed_ms", uc_def.sigma_wind_speed_ms);
        g_state.uc_config.sigma_wind_heading_deg =
            j.value("uc_sigma_wind_heading_deg", uc_def.sigma_wind_heading_deg);
        g_state.uc_config.sigma_temperature_c =
            j.value("uc_sigma_temperature_c", uc_def.sigma_temperature_c);
        g_state.uc_config.sigma_pressure_pa =
            j.value("uc_sigma_pressure_pa", uc_def.sigma_pressure_pa);
        g_state.uc_config.sigma_humidity = j.value("uc_sigma_humidity", uc_def.sigma_humidity);
        g_state.uc_config.sigma_sight_height_mm =
            j.value("uc_sigma_sight_height_mm", uc_def.sigma_sight_height_mm);
        g_state.uc_config.sigma_cant_deg = j.value("uc_sigma_cant_deg", uc_def.sigma_cant_deg);
        g_state.uc_config.sigma_latitude_deg =
            j.value("uc_sigma_latitude_deg", uc_def.sigma_latitude_deg);
        // Removed obsolete sigma_mass_grains field
        // Removed obsolete sigma_length_mm field
        // Removed obsolete sigma_caliber_inches field
        g_state.uc_config.sigma_twist_rate_inches =
            j.value("uc_sigma_twist_inches", uc_def.sigma_twist_rate_inches);
        g_state.uc_config.sigma_zero_range_m =
            j.value("uc_sigma_zero_range_m", uc_def.sigma_zero_range_m);
        g_state.uc_config.sigma_mv_adjustment_fps_per_in =
            j.value("uc_sigma_mv_adj_fps_per_in", uc_def.sigma_mv_adjustment_fps_per_in);

        // Removed obsolete cartridge CEP fields and logic

        // Hardware sensor selections (backwards-compatible: 0 = Custom)
        g_state.hw_barometer_index = j.value("hw_barometer_index", kBarometerDefaultIndex);
        g_state.hw_lrf_index = j.value("hw_lrf_index", kLrfDefaultIndex);
        g_state.hw_cant_index = j.value("hw_cant_index", 0);
        g_state.hw_temp_sensor_index = j.value("hw_temp_sensor_index", kTempDefaultIndex);
        g_state.hw_gps_index = j.value("hw_gps_index", kGpsDefaultIndex);
        g_state.hw_mag_lat_index = j.value("hw_mag_lat_index", kMagLatDefaultIndex);
        if (g_state.hw_barometer_index < 0 ||
            g_state.hw_barometer_index >= GetNumBarometerPresets())
            g_state.hw_barometer_index = kBarometerDefaultIndex;
        if (g_state.hw_lrf_index < 0 || g_state.hw_lrf_index >= GetNumLRFPresets())
            g_state.hw_lrf_index = kLrfDefaultIndex;
        if (g_state.hw_cant_index < 0 || g_state.hw_cant_index >= GetNumCantPresets())
            g_state.hw_cant_index = kCantDefaultIndex;
        if (g_state.hw_temp_sensor_index < 0 ||
            g_state.hw_temp_sensor_index >= GetNumTempSensorPresets())
            g_state.hw_temp_sensor_index = kTempDefaultIndex;
        if (g_state.hw_gps_index < 0 || g_state.hw_gps_index >= GetNumGpsLatPresets())
            g_state.hw_gps_index = kGpsDefaultIndex;
        if (g_state.hw_mag_lat_index < 0 || g_state.hw_mag_lat_index >= GetNumMagLatPresets())
            g_state.hw_mag_lat_index = kMagLatDefaultIndex;

        g_state.baro_is_calibrated = j.value("baro_calibrated", false);
        g_state.baro_has_calibration_temp = j.value("baro_calibration_temp_valid", false);
        g_state.baro_calibration_temp_c = j.value("baro_calibration_temp_c", 0.0f);

        g_state.mass_error_preset_index =
            ClampPresetIndex(j.value("uc_mass_error_preset_index", kMassErrorDefaultIndex),
                             kNumMassErrorPresets, kMassErrorDefaultIndex);
        g_state.length_error_preset_index =
            ClampPresetIndex(j.value("uc_length_error_preset_index", kLengthErrorDefaultIndex),
                             kNumLengthErrorPresets, kLengthErrorDefaultIndex);
        g_state.caliber_error_preset_index =
            ClampPresetIndex(j.value("uc_caliber_error_preset_index", kCaliberErrorDefaultIndex),
                             kNumCaliberErrorPresets, kCaliberErrorDefaultIndex);

        g_state.imu_valid = j.value("imu_valid", true);
        g_state.mag_valid = j.value("mag_valid", true);
        g_state.baro_valid = j.value("baro_valid", true);
        g_state.baro_humidity_valid = j.value("baro_humidity_valid", true);
        g_state.lrf_valid = j.value("lrf_valid", true);

        if (!has_bc && !g_state.override_drag_coefficient) {
            g_state.manual_drag_coefficient = ComputeAutoDragCoefficient(g_state.bullet);
        }

        SyncDerivedInputUncertaintySigmas();
    } catch (const json::parse_error& e) {
        g_state.last_action = "Load Preset (failed)";
        std::string error_msg = "Preset load failed: JSON parse error: ";
        error_msg += e.what();
        g_state.output_text = error_msg;
    }
}

void ResetEngineAndState() {
    DOPE_Init();
    DOPE_SetDeferUncertainty(true);
    g_state.now_us = 0;
    g_state.last_action = "Reset Engine";
    ApplyConfig();
    RefreshOutput();
}

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
    case WM_SIZE:
        if (g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED) {
            // Recreate render target whenever swap-chain buffers resize.
            CleanupRenderTarget();
            g_pSwapChain->ResizeBuffers(0, static_cast<UINT>(LOWORD(lParam)),
                                        static_cast<UINT>(HIWORD(lParam)), DXGI_FORMAT_UNKNOWN, 0);
            CreateRenderTarget();
        }
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU)
            return 0;
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

} // namespace

// ---------------------------------------------------------------------------
// Engine ticker ΓÇö mirrors the FreeRTOS DOPE task on the ESP32-P4.
// Runs DOPE_Update + RefreshOutput at FRAME_STEP_US intervals on a background
// thread so the render loop never blocks on the solver.
// ---------------------------------------------------------------------------
void EngineTickerThread() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    while (g_ticker_running.load(std::memory_order_relaxed)) {
        next += microseconds(static_cast<long long>(FRAME_STEP_US));
        std::this_thread::sleep_until(next);
        if (!g_ticker_running.load(std::memory_order_relaxed))
            break;
        if (!g_auto_tick_enabled.load(std::memory_order_relaxed))
            continue; // disabled: keep window responsive without engine churn
        std::lock_guard<std::mutex> lk(g_engine_mutex);
        RunFrameUpdates(1);
        RefreshOutput();
    }
    KickUncertaintyJob();
}

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    std::printf("DEBUG: WinMain started\n");
    std::fflush(stdout);
    // Desktop harness startup sequence: defaults -> window/device -> ImGui -> DOPE init.
    ResetStateDefaults();

    WNDCLASSEX wc = {sizeof(WNDCLASSEX),       CS_CLASSDC, WndProc, 0L,      0L,
                     GetModuleHandle(nullptr), nullptr,    nullptr, nullptr, nullptr,
                     _T("DOPE_ImGui_Test"),    nullptr};
    RegisterClassEx(&wc);

    HWND hwnd =
        CreateWindow(wc.lpszClassName, _T("DOPE Basic Test GUI (ImGui)"), WS_OVERLAPPEDWINDOW, 100,
                     100, 1280, 800, nullptr, nullptr, wc.hInstance, nullptr);

    if (CreateDeviceD3D(hwnd) < 0) {
        CleanupDeviceD3D();
        UnregisterClass(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    ResetEngineAndState();

    // Start engine ticker thread ΓÇö mirrors the FreeRTOS DOPE task on ESP32-P4.
    g_ticker_running = true;
    std::thread ticker_thread(EngineTickerThread);

    bool done = false;
    while (!done) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done) {
            break;
        }

        // Snapshot display state from the ticker thread. g_display_mutex is
        // held only briefly (string copy), so this never blocks on the solver.
        FiringSolution frame_sol = {};
        std::string frame_output;
        {
            std::lock_guard<std::mutex> dlk(g_display_mutex);
            frame_sol = g_display_sol;
            frame_output = g_display_output;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("DOPE Inputs");
        int unit_mode = static_cast<int>(g_state.unit_system);
        if (ImGui::Combo("Unit System", &unit_mode, kUnitSystemLabels,
                         IM_ARRAYSIZE(kUnitSystemLabels))) {
            g_state.unit_system = static_cast<UnitSystem>(unit_mode);
            g_state.last_action = "Unit Mode Changed";
            // ticker will refresh output at next tick
        }

        const bool is_imperial = (g_state.unit_system == UnitSystem::IMPERIAL);
        const bool uc_on = g_state.uc_config.enabled;

        if (ImGui::CollapsingHeader("Cartridge Presets (GUI-only)")) {
            ImGui::PushID("cartridge_presets");
            ImGui::TextUnformatted(
                "Presets are for test-harness convenience and do not alter engine internals.");

            ImGui::InputText("Cartridge Preset Name", g_state.new_cartridge_preset_name,
                             IM_ARRAYSIZE(g_state.new_cartridge_preset_name));
            // Preset tag UI removed
            if (ImGui::Button("Add/Update Cartridge Preset")) {
                const std::string preset_name = g_state.new_cartridge_preset_name;
                if (!preset_name.empty()) {
                    CartridgePreset preset = CaptureCurrentCartridgePreset(preset_name);
                    // tags removed from preset
                    const int existing = FindCartridgePresetByName(preset_name);
                    if (existing >= 0) {
                        g_cartridge_presets[existing] = preset;
                        g_state.selected_cartridge_preset = existing;
                    } else {
                        g_cartridge_presets.push_back(preset);
                        g_state.selected_cartridge_preset =
                            static_cast<int>(g_cartridge_presets.size()) - 1;
                    }
                    SelectDefaultGunPresetForSelectedCartridge();
                    g_state.last_action = "Cartridge Preset Saved";
                    // ticker will refresh output at next tick
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Cartridge (V2)") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& cp = g_cartridge_presets[g_state.selected_cartridge_preset];
                // Update GUI glue-layer fields from the cartridge preset so ApplyConfig
                // pushes consistent bullet/gun state to the engine.
                ApplyCartridgePreset(cp);
                AmmoDatasetV2 ds = MakeV2FromCartridgePreset(cp);
                g_state.last_action = "Applied Cartridge as V2 Dataset";
                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    DOPE_SetAmmoDatasetV2(&ds);
                    std::printf("[DEBUG] Applied V2 ammo: name= %s, muzzle_vel=%.2f m/s, bc=%.3f, drag_model=%d, vel_points=%d\n",
                                cp.name.c_str(), ds.muzzle_velocity_ms, ds.bc, static_cast<int>(ds.drag_model), ds.num_velocity_points);
                    ApplyConfig();
                    std::printf("[DEBUG] After ApplyConfig (V2 apply): glue mass=%.2f gr, caliber=%.4f in, length=%.2f mm, gmv_ms=%.2f, gun_barrel_in=%.1f\n",
                                g_state.bullet.mass_grains, g_state.bullet.caliber_inches, g_state.bullet.length_mm,
                                g_state.bullet.muzzle_velocity_ms, g_state.gun.barrel_length_in);
                    RefreshOutput();
                    // If the provided V2 trajectory is sparse or doesn't cover the
                    // visual range we care about, synthesize a cached full-table
                    // from the solver's 1-metre trajectory table and attach it to
                    // the dataset. This keeps GUI visuals and engine holds
                    // consistent without reverting to legacy behavior.
                    if (ds.num_trajectories > 0) {
                        // Desired visual coverage: use current LRF/target range or
                        // a default 600 yd (~548.64 m) cap, whichever is larger.
                        const float k_default_visual_max_m = 548.64f; // 600 yd
                        const float desired_visual_max_m = std::max(g_state.lrf_range, k_default_visual_max_m);
                        const int sparse_threshold_points = 12; // heuristic threshold
                        const int max_cache_pts = DOPE_MAX_TABLE_POINTS;
                        // For each trajectory family, generate a per-family cached table
                        // by setting the zero config to the family's zero and sampling
                        // the engine's solver table via DOPE_GetTrajectoryPoint().
                        for (int fi = 0; fi < ds.num_trajectories; ++fi) {
                            DOPE_TrajectoryFamily& fam = ds.trajectories[fi];
                            const float fam_max_range = (fam.num_points > 0) ? fam.points[fam.num_points - 1].range_m : 0.0f;
                            const bool need_expand = (fam.num_points < sparse_threshold_points) || (fam_max_range < desired_visual_max_m);
                            if (!need_expand)
                                continue;
                            float step_m = std::ceil(desired_visual_max_m / static_cast<float>(max_cache_pts));
                            if (step_m < 1.0f)
                                step_m = 1.0f;
                            int num_cache_pts = static_cast<int>(std::floor(desired_visual_max_m / step_m)) + 1;
                            if (num_cache_pts > max_cache_pts)
                                num_cache_pts = max_cache_pts;

                            // Apply dataset so engine has latest data
                            DOPE_SetAmmoDatasetV2(&ds);
                            // Set zero config for this family so DOPE_GetTrajectoryPoint
                            // reflects the requested zero-range variant.
                            ZeroConfig zc = {};
                            zc.zero_range_m = fam.zero_range_m;
                            zc.sight_height_mm = g_state.zero.sight_height_mm;
                            DOPE_SetZeroConfig(&zc);

                            fam.cached_table_present = true;
                            fam.cached_table_step_m = step_m;
                            fam.cached_table_num_points = num_cache_pts;
                            for (int i = 0; i < num_cache_pts; ++i) {
                                int r = static_cast<int>(std::round(i * step_m));
                                if (r < 0)
                                    r = 0;
                                if (r > DOPE_MAX_RANGE_M)
                                    r = DOPE_MAX_RANGE_M;
                                TrajectoryPoint tp = {};
                                const bool ok = DOPE_GetTrajectoryPoint(r, &tp);
                                fam.cached_table[i].range_m = static_cast<float>(r);
                                fam.cached_table[i].drop_m = ok ? tp.drop_m : 0.0f;
                            }
                            fam.cached_checksum = 0u;
                            std::printf("[DEBUG] Expanded V2 family %d cached_table: points=%d step=%.2f m zero=%.2f m\n",
                                        fi, fam.cached_table_num_points, fam.cached_table_step_m, fam.zero_range_m);
                        }
                        // Re-apply dataset once more to ensure engine sees any cached metadata
                        DOPE_SetAmmoDatasetV2(&ds);
                        ApplyConfig();
                        RefreshOutput();
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Cartridge (Legacy)") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                ApplyCartridgePreset(g_cartridge_presets[g_state.selected_cartridge_preset]);
                g_state.last_action = "Cartridge Preset Applied (legacy)";
                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    ApplyConfig();
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Cartridge as V2") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& cp = g_cartridge_presets[g_state.selected_cartridge_preset];
                // Keep GUI glue-layer in sync with the selected cartridge preset.
                ApplyCartridgePreset(cp);
                AmmoDatasetV2 ds = MakeV2FromCartridgePreset(cp);
                g_state.last_action = "Applied Cartridge as V2 Dataset";
                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    DOPE_SetAmmoDatasetV2(&ds);
                    ApplyConfig();
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Preset editing disabled (use JSON files directly)");

            ImGui::Separator();
            ImGui::TextUnformatted("Drag");
            ImGui::Checkbox("Override Drag Coefficient", &g_state.override_drag_coefficient);
            {
                const float avail_bc = ImGui::GetContentRegionAvail().x;
                if (g_state.override_drag_coefficient) {
                    ImGui::SetNextItemWidth(uc_on ? avail_bc * 0.45f : -1.0f);
                    ImGui::InputFloat("Drag Coefficient (G-model)",
                                      &g_state.manual_drag_coefficient, 0.001f, 0.01f, "%.4f");
                } else {
                    const float auto_drag = ComputeAutoDragCoefficient(g_state.bullet);
                    ImGui::Text("Drag Coefficient (auto): %.4f", auto_drag);
                }
                if (uc_on) {
                    // Removed obsolete BC sigma UI
                }
            }
            // Show only drag models supported by the selected cartridge preset.
            uint8_t visible_mask = 0xFFu;
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                visible_mask = g_cartridge_presets[g_state.selected_cartridge_preset].supported_drag_models_mask;
                if (visible_mask == 0) {
                    // Legacy presets without an explicit mask fall back to their
                    // single drag_model_index.
                    const int dm = g_cartridge_presets[g_state.selected_cartridge_preset].drag_model_index;
                    visible_mask = static_cast<uint8_t>(1u << dm);
                }
            }

            std::vector<const char*> labels;
            std::vector<int> indices;
            for (int i = 0; i < 8; ++i) {
                if (visible_mask & (1u << i)) {
                    labels.push_back(kDragModelLabels[i]);
                    indices.push_back(i);
                }
            }

            if (labels.empty()) {
                // Fallback to showing all models if mask computation fails.
                ImGui::Combo("Drag Model", &g_state.drag_model_index, kDragModelLabels,
                             IM_ARRAYSIZE(kDragModelLabels));
            } else {
                int visible_sel = 0;
                for (int vi = 0; vi < static_cast<int>(indices.size()); ++vi) {
                    if (indices[vi] == g_state.drag_model_index) {
                        visible_sel = vi;
                        break;
                    }
                }
                if (ImGui::Combo("Drag Model", &visible_sel, labels.data(), (int)labels.size())) {
                    g_state.drag_model_index = indices[visible_sel];
                }
            }
            ImGui::Separator();

            if (ImGui::BeginListBox("##cartridge_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g_cartridge_presets.size()); ++i) {
                    const bool is_selected = (g_state.selected_cartridge_preset == i);
                    if (ImGui::Selectable(g_cartridge_presets[i].name.c_str(), is_selected)) {
                        g_state.selected_cartridge_preset = i;
                        SelectDefaultGunPresetForSelectedCartridge();
                        // populate tag UI selections from selected preset
                        const CartridgePreset& cp = g_cartridge_presets[i];
                        // reset to defaults (tag UI removed)
                        g_state.new_cartridge_tier_index = 0;
                        g_state.new_cartridge_construction_index = 0;
                        g_state.new_cartridge_shape_index = 0;
                        g_state.new_cartridge_sd_mv_fps = cp.sd_mv_fps;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
            // Editable barrel-MV fields for the selected cartridge preset.
            // Editable sd_mv_fps field for the selected cartridge preset.
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& selected_cp =
                    g_cartridge_presets[g_state.selected_cartridge_preset];
                ImGui::Separator();
                ImGui::TextDisabled("Cartridge MV SD (fps): %.2f", selected_cp.sd_mv_fps);
            }
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& selected_cp =
                    g_cartridge_presets[g_state.selected_cartridge_preset];
                ImGui::Separator();
                ImGui::TextDisabled("Cartridge Barrel-MV: %s", selected_cp.name.c_str());
                ImGui::TextDisabled("Ref Barrel (in): %.1f", selected_cp.reference_barrel_inches);
                if (!selected_cp.barrel_mv_profile.empty()) {
                    ImGui::TextDisabled("Profile (%d pts — edit JSON to modify):",
                                        static_cast<int>(selected_cp.barrel_mv_profile.size()));
                    for (const auto& bp : selected_cp.barrel_mv_profile) {
                        ImGui::TextDisabled("  %.0f\" \u2192 %.0f fps", bp.first, bp.second);
                    }
                } else {
                    ImGui::TextDisabled("MV Adj (fps/in): %.1f", selected_cp.mv_adjustment_fps_per_in);
                }
            }
            // -- Bullet geometry (mass, length, caliber) --
            ImGui::Separator();
            {
                const float avail_mass = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_mass * 0.35f : -1.0f);
                float mass_display = g_state.bullet.mass_grains;
                if (g_state.selected_cartridge_preset >= 0 &&
                    g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                    mass_display = g_cartridge_presets[g_state.selected_cartridge_preset].mass_grains;
                }
                ImGui::TextDisabled("Mass (gr): %.2f", mass_display);
                if (uc_on) {
                    // Removed obsolete mass sigma UI
                }
            }
            float length_display = is_imperial ? (g_state.bullet.length_mm * MM_TO_IN) : g_state.bullet.length_mm;
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& cp = g_cartridge_presets[g_state.selected_cartridge_preset];
                length_display = is_imperial ? cp.length_inches : (cp.length_inches * IN_TO_MM);
            }
            {
                const float avail_len = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_len * 0.35f : -1.0f);
                if (is_imperial)
                    ImGui::TextDisabled("Length (in): %.3f", length_display);
                else
                    ImGui::TextDisabled("Length (mm): %.2f", length_display);
                if (uc_on) {
                    // Removed obsolete length sigma UI
                }
            }
            float caliber_display = is_imperial ? g_state.bullet.caliber_inches
                                                : (g_state.bullet.caliber_inches * IN_TO_MM);
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& cp = g_cartridge_presets[g_state.selected_cartridge_preset];
                caliber_display = is_imperial ? cp.caliber_inches : (cp.caliber_inches * IN_TO_MM);
            }
            {
                const float avail_cal = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_cal * 0.35f : -1.0f);
                if (is_imperial)
                    ImGui::TextDisabled("Caliber (in): %.3f", caliber_display);
                else
                    ImGui::TextDisabled("Caliber (mm): %.2f", caliber_display);
                if (uc_on) {
                    // Removed obsolete caliber sigma UI
                }
            }
            // Removed obsolete CEP table UI
            ImGui::PopID();
        }

        if (ImGui::CollapsingHeader("Gun Presets (GUI-only)")) {
            ImGui::PushID("gun_presets");
            EnsureSelectedGunPresetMatchesCurrentCaliber();
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                ImGui::TextDisabled("Selected Gun Preset: %s",
                                    g_gun_presets[g_state.selected_gun_preset].name.c_str());
            } else {
                ImGui::TextDisabled("Selected Gun Preset: (none)");
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Gun") && g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
                IsGunPresetCompatibleWithSelectedCaliber(
                    g_gun_presets[g_state.selected_gun_preset])) {
                ApplyGunPreset(g_gun_presets[g_state.selected_gun_preset]);
                g_state.last_action = "Gun Preset Applied";
                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    ApplyConfig();
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("Preset editing disabled (use JSON files directly)");

            if (ImGui::BeginListBox("##gun_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g_gun_presets.size()); ++i) {
                    if (!IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[i])) {
                        continue;
                    }
                    const bool is_selected = (g_state.selected_gun_preset == i);
                    if (ImGui::Selectable(g_gun_presets[i].name.c_str(), is_selected)) {
                        g_state.selected_gun_preset = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
            // -- Gun geometry (twist rate, barrel length) --
            ImGui::Separator();
            float twist_display = is_imperial ? g_state.bullet.twist_rate_inches
                                              : (g_state.bullet.twist_rate_inches * IN_TO_MM);
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                const GunPreset& gp = g_gun_presets[g_state.selected_gun_preset];
                twist_display = is_imperial ? gp.twist_rate_inches : (gp.twist_rate_inches * IN_TO_MM);
            }
            ImGui::TextDisabled(is_imperial ? "Twist (in/turn): %.2f" : "Twist (mm/turn): %.1f", twist_display);
            if (uc_on) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                const float twist_sigma_display =
                    is_imperial ? g_state.uc_config.sigma_twist_rate_inches
                                : (g_state.uc_config.sigma_twist_rate_inches * IN_TO_MM);
                ImGui::Text(is_imperial ? "+/-Twist (1%%) = %.2f in"
                                        : "+/-Twist (1%%) = %.1f mm",
                            twist_sigma_display);
            }
            float barrel_len_display = g_state.gun.barrel_length_in;
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                barrel_len_display = g_gun_presets[g_state.selected_gun_preset].barrel_length_in;
            }
            ImGui::TextDisabled("Barrel Length (in): %.1f", barrel_len_display);
            float muzzle_od_display = g_state.gun.muzzle_diameter_in;
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                muzzle_od_display = g_gun_presets[g_state.selected_gun_preset].muzzle_diameter_in;
            }
            ImGui::TextDisabled("Muzzle OD (in): %.3f", muzzle_od_display);
            const char* material_label = kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].label;
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                const GunPreset& gp = g_gun_presets[g_state.selected_gun_preset];
                for (int i = 0; i < kNumBarrelMaterialPresets; ++i) {
                    if (kBarrelMaterialPresets[i].material == gp.barrel_material) {
                        material_label = kBarrelMaterialPresets[i].label;
                        break;
                    }
                }
            } else if (g_state.barrel_material_index >= 0 &&
                       g_state.barrel_material_index < kNumBarrelMaterialPresets) {
                material_label = kBarrelMaterialPresets[g_state.barrel_material_index].label;
            }
            ImGui::TextDisabled("Barrel Material: %s", material_label);
            ImGui::TextDisabled("Stiffness MOA: %.2f", g_state.gun.stiffness_moa);
            const char* current_finish =
                (g_state.barrel_finish_index >= 0 && g_state.barrel_finish_index < kNumBarrelFinishPresets)
                    ? kBarrelFinishPresets[g_state.barrel_finish_index].name
                    : "Custom";
            ImGui::TextDisabled("Barrel Finish: %s", current_finish);
            float bias_display = g_state.gun.cold_bore_velocity_bias;
            float angular_display = g_state.gun.angular_sigma_moa;
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                const GunPreset& gp = g_gun_presets[g_state.selected_gun_preset];
                bias_display = gp.cold_bore_velocity_bias;
                angular_display = gp.angular_sigma_moa;
            }
            ImGui::TextDisabled("Cold Bore MV Bias (fps): %.2f", bias_display);
            ImGui::TextDisabled("Angular Sigma (MOA): %.2f", angular_display);
            ImGui::SeparatorText("Barrel Attachments");
            bool ff = g_state.free_floated;
            bool sup = g_state.suppressor_attached;
            bool tuner = g_state.barrel_tuner_attached;
            if (g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size())) {
                const GunPreset& gp = g_gun_presets[g_state.selected_gun_preset];
                ff = gp.free_floated;
                sup = gp.suppressor_attached;
                tuner = gp.barrel_tuner_attached;
            }
            ImGui::TextDisabled("Free-Floated: %s", ff ? "Yes" : "No");
            ImGui::SameLine();
            ImGui::TextDisabled("Suppressor: %s", sup ? "Yes" : "No");
            ImGui::SameLine();
            ImGui::TextDisabled("Barrel Tuner: %s", tuner ? "Yes" : "No");
            // Keep gun struct in sync for ApplyConfig hot-path.
            g_state.gun.free_floated = g_state.free_floated;
            g_state.gun.suppressor_attached = g_state.suppressor_attached;
            g_state.gun.barrel_tuner_attached = g_state.barrel_tuner_attached;
            {
                const float avail_mva = ImGui::GetContentRegionAvail().x;
                // MV-per-inch is automatically computed from the active cartridge preset.
                // Show read-only; append "(est.)" when the fallback tier estimator is active.
                const float active_fps_per_in = g_state.bullet.mv_adjustment_fps_per_in;
                ImGui::SetNextItemWidth(uc_on ? avail_mva * 0.45f : -1.0f);
                if (!g_active_barrel_mv_profile.empty()) {
                    ImGui::TextDisabled("MV Adj: profile (curr %.0f fps)",
                                        g_state.bullet.muzzle_velocity_ms * MPS_TO_FPS);
                } else if (g_mv_adjustment_estimated) {
                    ImGui::TextDisabled("MV Adj (fps/in): %.1f (est.)", active_fps_per_in);
                } else {
                    ImGui::TextDisabled("MV Adj (fps/in): %.1f", active_fps_per_in);
                }
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("+/-MVAdj(fps/in): %.2f", g_state.uc_config.sigma_mv_adjustment_fps_per_in);
                }
            }
            ImGui::PopID();
        }

        ImGui::Checkbox("Enable Uncertainty Propagation", &g_state.uc_config.enabled);
        if (g_state.uc_config.enabled) {
            ImGui::SameLine();
            if (ImGui::SmallButton("Reset Sigmas")) {
                bool was_enabled = g_state.uc_config.enabled;
                DOPE_GetDefaultUncertaintyConfig(&g_state.uc_config);
                g_state.uc_config.enabled = was_enabled;
                g_state.hw_barometer_index = kBarometerDefaultIndex;
                g_state.hw_lrf_index = kLrfDefaultIndex;
                g_state.hw_cant_index = kCantDefaultIndex;
                g_state.hw_temp_sensor_index = kTempDefaultIndex;
                g_state.hw_gps_index = kGpsDefaultIndex;
                g_state.hw_mag_lat_index = kMagLatDefaultIndex;
                g_state.baro_is_calibrated = false;
                g_state.baro_has_calibration_temp = false;
                g_state.baro_calibration_temp_c = 0.0f;
                g_state.mass_error_preset_index = kMassErrorDefaultIndex;
                g_state.length_error_preset_index = kLengthErrorDefaultIndex;
                g_state.caliber_error_preset_index = kCaliberErrorDefaultIndex;
                SyncDerivedInputUncertaintySigmas();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(+/- shown inline with each value)");
        }

        ImGui::TextUnformatted("Manual Inputs");
        float muzzle_display = is_imperial ? (g_state.bullet.muzzle_velocity_ms * MPS_TO_FPS)
                                           : g_state.bullet.muzzle_velocity_ms;
        // Effective MV sigma is the larger of manual MV sigma and barrel-finish sigma.
        const float mv_sigma_effective_ms =
            std::max(g_state.uc_config.sigma_muzzle_velocity_ms,
                     g_state.barrel_finish_sigma_mv_fps * FPS_TO_MPS);
        float mv_sig_display =
            is_imperial ? (mv_sigma_effective_ms * MPS_TO_FPS) : mv_sigma_effective_ms;
        {
            const float avail_mv = ImGui::GetContentRegionAvail().x;
            ImGui::BeginDisabled(true);
            ImGui::SetNextItemWidth(uc_on ? avail_mv * 0.45f : -1.0f);
            ImGui::InputFloat(is_imperial ? "Muzzle Velocity (fps)" : "Muzzle Velocity (m/s)",
                              &muzzle_display, 0.0f, 0.0f, "%.1f", ImGuiInputTextFlags_ReadOnly);
            if (uc_on) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputFloat(is_imperial ? "+/-MV(fps)" : "+/-MV(m/s)##mv", &mv_sig_display,
                                  0.0f, 0.0f, "%.1f", ImGuiInputTextFlags_ReadOnly);
            }
            ImGui::EndDisabled();
        }
        ImGui::TextDisabled("Muzzle velocity is derived from cartridge + barrel; sigma = "
                            "max(manual MV sigma, barrel finish sigma).");
        ImGui::Separator();
        float zero_display =
            is_imperial ? (g_state.zero.zero_range_m * M_TO_YD) : g_state.zero.zero_range_m;
        {
            const float avail_zr = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_zr * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Zero Range (yd)" : "Zero Range (m)", &zero_display,
                                  is_imperial ? 1.0f : 1.0f, is_imperial ? 10.0f : 10.0f, "%.2f")) {
                g_state.zero.zero_range_m = is_imperial ? (zero_display * YD_TO_M) : zero_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float zr_sig = is_imperial ? (g_state.uc_config.sigma_zero_range_m * M_TO_YD)
                                            : g_state.uc_config.sigma_zero_range_m;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Zero(yd)##zr" : "+/-Zero(m)##zr", &zr_sig,
                                      0.0f, 0.0f, "%.2f")) {
                    g_state.uc_config.sigma_zero_range_m =
                        is_imperial ? (zr_sig * YD_TO_M) : zr_sig;
                    if (g_state.uc_config.sigma_zero_range_m < 0.0f)
                        g_state.uc_config.sigma_zero_range_m = 0.0f;
                }
            }
        }
        float sight_display =
            is_imperial ? (g_state.zero.sight_height_mm * MM_TO_IN) : g_state.zero.sight_height_mm;
        {
            const float avail_sh = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_sh * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Sight Height (in)" : "Sight Height (mm)",
                                  &sight_display, is_imperial ? 0.01f : 0.1f,
                                  is_imperial ? 0.1f : 1.0f, is_imperial ? "%.3f" : "%.2f")) {
                g_state.zero.sight_height_mm =
                    is_imperial ? (sight_display * IN_TO_MM) : sight_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float sht_sig = is_imperial ? (g_state.uc_config.sigma_sight_height_mm * MM_TO_IN)
                                            : g_state.uc_config.sigma_sight_height_mm;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-SightH(in)##sh" : "+/-SightH(mm)##sh",
                                      &sht_sig, 0.0f, 0.0f, is_imperial ? "%.3f" : "%.2f")) {
                    g_state.uc_config.sigma_sight_height_mm =
                        is_imperial ? (sht_sig * IN_TO_MM) : sht_sig;
                    if (g_state.uc_config.sigma_sight_height_mm < 0.0f)
                        g_state.uc_config.sigma_sight_height_mm = 0.0f;
                }
            }
        }

        float wind_display =
            is_imperial ? (g_state.wind_speed_ms * MPS_TO_MPH) : g_state.wind_speed_ms;
        {
            const float avail_ws = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_ws * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Wind Speed (mph)" : "Wind Speed (m/s)",
                                  &wind_display, is_imperial ? 0.5f : 0.1f,
                                  is_imperial ? 5.0f : 1.0f, "%.2f")) {
                g_state.wind_speed_ms = is_imperial ? (wind_display * MPH_TO_MPS) : wind_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float wsp_sig = is_imperial ? (g_state.uc_config.sigma_wind_speed_ms * MPS_TO_MPH)
                                            : g_state.uc_config.sigma_wind_speed_ms;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Wind(mph)##ws" : "+/-Wind(m/s)##ws",
                                      &wsp_sig, 0.0f, 0.0f, "%.2f")) {
                    g_state.uc_config.sigma_wind_speed_ms =
                        is_imperial ? (wsp_sig * MPH_TO_MPS) : wsp_sig;
                    if (g_state.uc_config.sigma_wind_speed_ms < 0.0f)
                        g_state.uc_config.sigma_wind_speed_ms = 0.0f;
                }
            }
        }
        {
            const float avail_wh = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_wh * 0.45f : -1.0f);
            ImGui::InputFloat("Wind Heading (deg)", &g_state.wind_heading, 1.0f, 5.0f, "%.1f");
            if (uc_on) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-WindDir(deg)##wd",
                                      &g_state.uc_config.sigma_wind_heading_deg, 0.0f, 0.0f,
                                      "%.1f")) {
                    if (g_state.uc_config.sigma_wind_heading_deg < 0.0f)
                        g_state.uc_config.sigma_wind_heading_deg = 0.0f;
                }
            }
        }
        // Latitude source / uncertainty presets (shown when uncertainty propagation is on)
        if (uc_on) {
            // GPS / GNSS module combo ΓÇö direct latitude feed; highest priority source
            {
                const int gps_count = GetNumGpsLatPresets();
                const char* gps_labels[16];
                for (int i = 0; i < gps_count && i < 16; ++i)
                    gps_labels[i] = GetGpsLabel(i);
                if (ImGui::Combo("GPS / GNSS Module", &g_state.hw_gps_index, gps_labels,
                                 gps_count)) {
                    const int gps_mapped = GetMappedGpsLatPresetIndex(g_state.hw_gps_index);
                    if (gps_mapped > 0) {
                        if (gps_mapped >= 0 && gps_mapped < kNumGpsLatPresets) {
                            g_state.uc_config.sigma_latitude_deg =
                                kGpsLatPresets[gps_mapped].sigma_lat_deg;
                        }
                        g_state.hw_mag_lat_index = 0; // GPS overrides mag estimator selection
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "GPS/GNSS receiver feeding latitude directly.\n"
                        "Takes priority over magnetometer estimation and manual entry.\n"
                        "Set to Custom and enter sigma manually if module is not listed.");
            }
            // Magnetometer preset ΓÇö shown only when no GPS module is selected
            if (GetMappedGpsLatPresetIndex(g_state.hw_gps_index) == 0) {
                const int mag_count = GetNumMagLatPresets();
                const char* mag_labels[16];
                for (int i = 0; i < mag_count && i < 16; ++i)
                    mag_labels[i] = GetMagLatLabel(i);
                if (ImGui::Combo("Magnetometer (lat est.)", &g_state.hw_mag_lat_index, mag_labels,
                                 mag_count)) {
                    const int mag_mapped = GetMappedMagLatPresetIndex(g_state.hw_mag_lat_index);
                    if (mag_mapped > 0) {
                        if (mag_mapped >= 0 && mag_mapped < kNumMagLatPresets) {
                            g_state.uc_config.sigma_latitude_deg =
                                kMagLatPresets[mag_mapped].sigma_lat_deg;
                        }
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Magnetometer used for autonomous latitude estimation via "
                                      "magnetic dip angle.\n"
                                      "Only active when no GPS/GNSS module is present.\n"
                                      "Manual entry calibrates the estimator when a known position "
                                      "is available.");
            }
        }
        {
            // Sigma locked when any hardware preset is providing the value
            const int gps_mapped = GetMappedGpsLatPresetIndex(g_state.hw_gps_index);
            const int mag_mapped = GetMappedMagLatPresetIndex(g_state.hw_mag_lat_index);
            const bool lat_locked = uc_on && (gps_mapped > 0 || mag_mapped > 0);
            const float avail_lat = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_lat * 0.45f : -1.0f);
            ImGui::InputFloat("Latitude (deg)", &g_state.latitude, 0.1f, 1.0f, "%.3f");
            if (uc_on) {
                ImGui::SameLine();
                if (lat_locked)
                    ImGui::BeginDisabled();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-Lat(deg)##lat", &g_state.uc_config.sigma_latitude_deg,
                                      0.0f, 0.0f, "%.2f")) {
                    if (g_state.uc_config.sigma_latitude_deg < 0.0f)
                        g_state.uc_config.sigma_latitude_deg = 0.0f;
                }
                if (lat_locked)
                    ImGui::EndDisabled();
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Sensor Frame");

        if (uc_on) {
            const int baro_count = GetNumBarometerPresets();
            const char* baro_labels[16];
            for (int i = 0; i < baro_count && i < 16; ++i)
                baro_labels[i] = GetBarometerLabel(i);
            if (ImGui::Combo("Barometer", &g_state.hw_barometer_index, baro_labels, baro_count)) {
                SyncDerivedInputUncertaintySigmas();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Barometer pressure uncertainty profile used for sigma pressure propagation.");

            // Single calibration button; default calibration temperature is current baro temp.
            {
                if (!g_state.baro_is_calibrated) {
                    if (ImGui::Button("Calibrate Barometer")) {
                        g_state.baro_is_calibrated = true;
                        g_state.baro_has_calibration_temp = true;
                        if (std::isfinite(g_state.baro_temp)) {
                            g_state.baro_calibration_temp_c = g_state.baro_temp;
                        }
                        SyncDerivedInputUncertaintySigmas();
                    }
                } else {
                    if (ImGui::Button("Barometer: Calibrated  (click to clear)")) {
                        g_state.baro_is_calibrated = false;
                        g_state.baro_has_calibration_temp = false;
                        SyncDerivedInputUncertaintySigmas();
                    }
                }
            }

            if (g_state.baro_is_calibrated) {
                float cal_temp_display =
                    is_imperial ? ((g_state.baro_calibration_temp_c * 9.0f / 5.0f) + 32.0f)
                                : g_state.baro_calibration_temp_c;
                if (ImGui::InputFloat(is_imperial ? "Cal Temp (F, internal)"
                                                  : "Cal Temp (C, internal)",
                                      &cal_temp_display, is_imperial ? 0.5f : 0.1f,
                                      is_imperial ? 2.0f : 1.0f, "%.2f")) {
                    g_state.baro_calibration_temp_c =
                        is_imperial ? ((cal_temp_display - 32.0f) * 5.0f / 9.0f) : cal_temp_display;
                    SyncDerivedInputUncertaintySigmas();
                }
            }

            const float delta_temp_c =
                (g_state.baro_is_calibrated && g_state.baro_has_calibration_temp)
                    ? std::fabs(g_state.baro_temp - g_state.baro_calibration_temp_c)
                    : 0.0f;
            ImGui::Text("Barometer sigma pressure: %.2f Pa (\u0394T used: %.2f C)",
                        g_state.uc_config.sigma_pressure_pa, delta_temp_c);
        }

        // Thermometer hardware preset ΓÇö shown above temp row when uc_on.
        if (uc_on) {
            const int tmp_count = GetNumTempSensorPresets();
            const char* tmp_labels[16];
            for (int i = 0; i < tmp_count && i < 16; ++i)
                tmp_labels[i] = GetTempSensorLabel(i);
            if (ImGui::Combo("Thermometer", &g_state.hw_temp_sensor_index, tmp_labels, tmp_count)) {
                SyncDerivedInputUncertaintySigmas();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Thermometer sensor: accuracy varies with temperature.\n"
                                  "Grays out the manual sigma field when table data is loaded.");
        }
        // Lock manual entry only when a preset with actual table data is selected.
        const DOPE_ErrorTable* tmp_table_lock = GetTempTable(g_state.hw_temp_sensor_index);
        const bool tmp_locked = uc_on && (g_state.hw_temp_sensor_index > 0 && tmp_table_lock &&
                                          tmp_table_lock->points != nullptr);
        const DOPE_ErrorTable* pr_table_lock = GetBarometerTable(g_state.hw_barometer_index);
        const bool pressure_locked = uc_on &&
                                     (g_state.hw_barometer_index >= 0 &&
                                      g_state.hw_barometer_index < GetNumBarometerPresets()) &&
                                     (pr_table_lock && pr_table_lock->points != nullptr);
        float pressure_display = g_state.baro_pressure;
        {
            const float avail_pr = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_pr * 0.45f : -1.0f);
            if (ImGui::InputFloat("Baro Pressure (Pa)", &pressure_display, 10.0f, 100.0f, "%.1f")) {
                // Pressure value itself changed; sigma remains derived from barometer calibration
                // drift model.
                g_state.baro_pressure = pressure_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                if (pressure_locked)
                    ImGui::BeginDisabled();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-Press(Pa)##pr", &g_state.uc_config.sigma_pressure_pa,
                                      0.0f, 0.0f, "%.0f")) {
                    if (g_state.uc_config.sigma_pressure_pa < 0.0f)
                        g_state.uc_config.sigma_pressure_pa = 0.0f;
                }
                if (pressure_locked)
                    ImGui::EndDisabled();
            }
        }
        float temp_display =
            is_imperial ? ((g_state.baro_temp * 9.0f / 5.0f) + 32.0f) : g_state.baro_temp;
        {
            const float avail_tm = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_tm * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Temp (F)" : "Temp (C)", &temp_display,
                                  is_imperial ? 0.5f : 0.1f, is_imperial ? 2.0f : 1.0f, "%.2f")) {
                g_state.baro_temp =
                    is_imperial ? ((temp_display - 32.0f) * 5.0f / 9.0f) : temp_display;
                if (uc_on) {
                    SyncDerivedInputUncertaintySigmas();
                }
            }
            if (uc_on) {
                ImGui::SameLine();
                if (tmp_locked)
                    ImGui::BeginDisabled();
                float tmp_sig = is_imperial ? (g_state.uc_config.sigma_temperature_c * 9.0f / 5.0f)
                                            : g_state.uc_config.sigma_temperature_c;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Temp(F)##tm" : "+/-Temp(C)##tm", &tmp_sig,
                                      0.0f, 0.0f, "%.2f")) {
                    g_state.uc_config.sigma_temperature_c =
                        is_imperial ? (tmp_sig * 5.0f / 9.0f) : tmp_sig;
                    if (g_state.uc_config.sigma_temperature_c < 0.0f)
                        g_state.uc_config.sigma_temperature_c = 0.0f;
                }
                if (tmp_locked)
                    ImGui::EndDisabled();
            }
        }
        {
            const float avail_hm = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_hm * 0.45f : -1.0f);
            ImGui::InputFloat("Humidity (0..1)", &g_state.baro_humidity, 0.01f, 0.1f, "%.2f");
            if (uc_on) {
                ImGui::SameLine();
                if (tmp_locked)
                    ImGui::BeginDisabled();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-Humid(frac)##hm", &g_state.uc_config.sigma_humidity, 0.0f,
                                      0.0f, "%.3f")) {
                    if (g_state.uc_config.sigma_humidity < 0.0f)
                        g_state.uc_config.sigma_humidity = 0.0f;
                }
                if (tmp_locked)
                    ImGui::EndDisabled();
            }
        }

        // --- LRF hardware + range ---
        if (uc_on) {
            const int lrf_count = GetNumLRFPresets();
            const char* lrf_labels[16];
            for (int i = 0; i < lrf_count && i < 16; ++i)
                lrf_labels[i] = GetLRFLabel(i);
            if (ImGui::Combo("Rangefinder", &g_state.hw_lrf_index, lrf_labels, lrf_count)) {
                SyncDerivedInputUncertaintySigmas();
            }
        }
        const bool lrf_locked = uc_on && (g_state.hw_lrf_index > 0);
        float lrf_display = is_imperial ? (g_state.lrf_range * M_TO_YD) : g_state.lrf_range;
        {
            const float avail_lrf = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_lrf * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "LRF Range (yd)" : "LRF Range (m)", &lrf_display,
                                  1.0f, 10.0f, "%.2f")) {
                g_state.lrf_range = is_imperial ? (lrf_display * YD_TO_M) : lrf_display;
                if (uc_on) {
                    SyncDerivedInputUncertaintySigmas();
                }
            }
            if (uc_on) {
                ImGui::SameLine();
                if (lrf_locked)
                    ImGui::BeginDisabled();
                float range_sigma_display = is_imperial
                                                ? (g_state.uc_config.sigma_range_m * M_TO_YD)
                                                : g_state.uc_config.sigma_range_m;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Range(yd)##lrf" : "+/-Range(m)##lrf",
                                      &range_sigma_display, 0.0f, 0.0f, "%.2f")) {
                    g_state.uc_config.sigma_range_m =
                        is_imperial ? (range_sigma_display * YD_TO_M) : range_sigma_display;
                    if (g_state.uc_config.sigma_range_m < 0.0f)
                        g_state.uc_config.sigma_range_m = 0.0f;
                }
                if (lrf_locked)
                    ImGui::EndDisabled();
            }
        }
        {
            constexpr float kMToFt = 3.280839895f;
            constexpr float kFtToM = 0.3048f;
            float target_elev_display =
                is_imperial ? (g_state.target_elevation_m * kMToFt) : g_state.target_elevation_m;
            // Input first (no visible label), then place the human-friendly label to the right.
            const float avail_elev = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(avail_elev * 0.55f);
            if (ImGui::InputFloat("##target_elev", &target_elev_display,
                                  is_imperial ? 0.5f : 0.1f,
                                  is_imperial ? 5.0f : 1.0f, "%.2f")) {
                g_state.target_elevation_m =
                    is_imperial ? (target_elev_display * kFtToM) : target_elev_display;
                // Keep geometry sane for slant-range conversion.
                const float max_abs = std::fmax(g_state.lrf_range - 0.01f, 0.0f);
                if (std::fabs(g_state.target_elevation_m) > max_abs) {
                    g_state.target_elevation_m = std::copysign(max_abs, g_state.target_elevation_m);
                }
            }
            ImGui::SameLine();
            ImGui::TextUnformatted(is_imperial ? "Elevation Delta (ft, +up/-down)"
                                               : "Elevation Delta (m, +up/-down)");
        }

        if (uc_on) {
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Cant Uncertainty (1-sigma)")) {
                ImGui::Indent();

                // Cant comes from IMU; this preset controls sigma only.
                {
                    const int cant_count = GetNumCantPresets();
                    const char* cant_labels[16];
                    for (int i = 0; i < cant_count && i < 16; ++i)
                        cant_labels[i] = GetCantLabel(i);
                    if (ImGui::Combo("Cant Sensor", &g_state.hw_cant_index, cant_labels,
                                     cant_count)) {
                        if (g_state.hw_cant_index > 0) {
                            g_state.uc_config.sigma_cant_deg = GetCantSigma(g_state.hw_cant_index);
                        }
                    }
                }
                const bool cant_locked = (g_state.hw_cant_index > 0);
                if (cant_locked)
                    ImGui::BeginDisabled();
                if (ImGui::InputFloat("+/- Cant (deg)", &g_state.uc_config.sigma_cant_deg, 0.1f,
                                      1.0f, "%.2f")) {
                    if (g_state.uc_config.sigma_cant_deg < 0.0f)
                        g_state.uc_config.sigma_cant_deg = 0.0f;
                }
                if (cant_locked)
                    ImGui::EndDisabled();

                ImGui::Unindent();
            }
        }

        if (ImGui::Button("Apply Config")) {
            g_state.last_action = "Apply Config";
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            ApplyConfig();
            RefreshOutput();
            KickUncertaintyJob();
        }
        ImGui::SameLine();
        if (ImGui::Button("Step Update")) {
            g_state.last_action = "Step Update";
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            ApplyConfig();
            RunFrameUpdates(1);
            RefreshOutput();
            KickUncertaintyJob();
        }
        ImGui::SameLine();
        if (ImGui::Button("Shot Fired")) {
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            ApplyConfig();
            DOPE_NotifyShotFired(g_state.now_us, g_state.baro_temp);
            RefreshOutput();
            KickUncertaintyJob();
        }
        ImGui::SameLine();
        const bool run_100_busy = g_run_batch_in_flight.load(std::memory_order_relaxed);
        ImGui::BeginDisabled(run_100_busy);
        if (ImGui::Button("Run 100")) {
            // Kick off the heavier batch on a worker so the render loop stays responsive.
            g_run_batch_in_flight.store(true, std::memory_order_relaxed);
            std::thread([] {
                g_state.last_action = "Run 100";
                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    ApplyConfig();
                }

                // Run in small chunks so the render thread is not blocked for the full batch.
                constexpr int kTotalFrames = 100;
                constexpr int kChunk = 10;
                int remaining = kTotalFrames;
                while (remaining > 0) {
                    const int to_run = std::min(kChunk, remaining);
                    {
                        std::lock_guard<std::mutex> lk(g_engine_mutex);
                        for (int i = 0; i < to_run; ++i) {
                            SensorFrame frame = BuildFrame();
                            DOPE_Update(&frame);
                        }
                    }
                    remaining -= to_run;
                    std::this_thread::yield();
                }

                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    RefreshOutput();
                }
                g_run_batch_in_flight.store(false, std::memory_order_relaxed);
                KickUncertaintyJob();
            }).detach();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        bool auto_tick = g_auto_tick_enabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Auto Tick", &auto_tick)) {
            g_auto_tick_enabled.store(auto_tick, std::memory_order_relaxed);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Engine")) {
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            ResetEngineAndState();
            KickUncertaintyJob();
        }

        ImGui::Separator();
        ImGui::InputText("Profile Library Path", g_state.profile_library_path,
                         IM_ARRAYSIZE(g_state.profile_library_path));
        if (ImGui::Button("Save Profile Library")) {
            SaveProfileLibrary();
            // ticker will refresh output at next tick
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Profile Library")) {
            LoadProfileLibrary();
            // ticker will refresh output at next tick
        }

        ImGui::Separator();
        ImGui::InputText("Preset Path", g_state.preset_path, IM_ARRAYSIZE(g_state.preset_path));
        if (ImGui::Button("Save Preset")) {
            g_state.last_action = "Save Preset";
            SavePreset();
            // ticker will refresh output at next tick
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Preset")) {
            g_state.last_action = "Load Preset";
            LoadPreset();
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            ApplyConfig();
            RefreshOutput();
        }

        ImGui::End();

        ImGui::Begin("DOPE Output");
        ImGui::InputTextMultiline("##output", frame_output.data(), frame_output.size() + 1,
                                  ImVec2(-FLT_MIN, -FLT_MIN), ImGuiInputTextFlags_ReadOnly);
        ImGui::End();

        ImGui::Begin("Target View");
        FiringSolution sol = frame_sol;

        const float hold_windage_moa = sol.hold_windage_moa;
        const float hold_elevation_moa = sol.hold_elevation_moa;

        // Use the solver's final hold values directly so the bullseye dot matches the
        // textual solution output even if the trajectory table hasn't been updated yet.
        const float impact_distance_moa = std::sqrt(hold_windage_moa * hold_windage_moa +
                                                    hold_elevation_moa * hold_elevation_moa);

        float axis1_moa = 0.0f, axis2_moa = 0.0f, cos_r = 1.0f, sin_r = 0.0f;
        if (sol.uncertainty_valid &&
            (sol.sigma_elevation_moa > 0.001f || sol.sigma_windage_moa > 0.001f)) {
            const float se = std::max(sol.sigma_elevation_moa, 0.001f);
            const float sw = std::max(sol.sigma_windage_moa, 0.001f);
            const float cov = sol.covariance_elev_wind;
            const float tr_half = (se * se + sw * sw) * 0.5f;
            const float disc = std::sqrt(std::max(
                ((se * se - sw * sw) * 0.5f) * ((se * se - sw * sw) * 0.5f) + cov * cov, 0.0f));
            const float lambda1 = tr_half + disc;
            const float lambda2 = std::max(tr_half - disc, 0.0f);
            axis1_moa = std::sqrt(lambda1);
            axis2_moa = std::sqrt(lambda2);
            const float rot_rad = std::atan2(lambda1 - se * se, cov);
            cos_r = std::cos(rot_rad);
            sin_r = std::sin(rot_rad);
        }

        const float ring_count_val = static_cast<float>(TARGET_RING_COUNT);
        const float required_span_moa = impact_distance_moa;
        float moa_per_ring = std::max(std::round(required_span_moa / ring_count_val), 1.0f);
        if ((moa_per_ring * ring_count_val) < required_span_moa) {
            moa_per_ring = std::max(moa_per_ring + 1.0f, 1.0f);
        }
        const float max_span_moa_val = moa_per_ring * ring_count_val;
        const float display_range_m = (sol.range_m > 0.0f) ? sol.range_m : g_state.lrf_range;
        const float display_range_yd = display_range_m * M_TO_YD;
        const float inches_per_moa_val = 1.047f * (display_range_yd / 100.0f);
        const float elev_offset_in = hold_elevation_moa * inches_per_moa_val;
        const float wind_offset_in = hold_windage_moa * inches_per_moa_val;
        // inch-conversion values removed — wind breakdown moved to textual output

        auto direction_label = [](float moa) -> const char* {
            if (moa > 0.01f)
                return "RIGHT";
            if (moa < -0.01f)
                return "LEFT";
            return "CENTER";
        };
        auto direction_color = [](float moa) -> ImVec4 {
            if (moa > 0.01f)
                return ImGui::GetStyleColorVec4(ImGuiCol_PlotHistogram);
            if (moa < -0.01f)
                return ImGui::GetStyleColorVec4(ImGuiCol_PlotLines);
            return ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
        };

        // (Removed single-line bullseye header — information is shown on-canvas)
        ImGui::Text("Offset: Elev %.2f MOA (%.2f in), Wind %.2f MOA (%.2f in)", hold_elevation_moa,
                    elev_offset_in, hold_windage_moa, wind_offset_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(hold_windage_moa), "[%s]",
                           direction_label(hold_windage_moa));
        // Wind breakdown moved to textual DOPE Output (see RefreshOutput()).

        if (sol.uncertainty_valid &&
            (sol.sigma_elevation_moa > 0.001f || sol.sigma_windage_moa > 0.001f)) {
            ImGui::Text("Confidence - Horizontal: 1SD %.2f MOA  2SD %.2f MOA",
                        sol.sigma_windage_moa, sol.sigma_windage_moa * 2.0f);
            ImGui::Text("           Vertical:   1SD %.2f MOA  2SD %.2f MOA",
                        sol.sigma_elevation_moa, sol.sigma_elevation_moa * 2.0f);
        }
        ImGui::Separator();

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float canvas_side = (avail.x < avail.y) ? avail.x : avail.y;
        if (canvas_side > TARGET_CANVAS_MAX)
            canvas_side = TARGET_CANVAS_MAX;
        if (canvas_side < TARGET_CANVAS_MIN)
            canvas_side = TARGET_CANVAS_MIN;
        const ImVec2 canvas_size(canvas_side, canvas_side);

        float pad_x = (avail.x - canvas_size.x) * 0.5f;
        if (pad_x < 0.0f)
            pad_x = 0.0f;
        float pad_y = (avail.y - canvas_size.y) * 0.5f;
        if (pad_y < 0.0f)
            pad_y = 0.0f;
        const ImVec2 draw_origin = ImGui::GetCursorScreenPos();
        ImGui::SetCursorScreenPos(ImVec2(draw_origin.x + pad_x, draw_origin.y + pad_y));

        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##target_canvas", canvas_size);

        ImVec2 canvas_end(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
        draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(24, 24, 28, 255));
        draw_list->AddRect(canvas_pos, canvas_end, IM_COL32(80, 80, 90, 255));

        // Draw the target range label in the canvas top-right corner.
        {
            char range_label[64];
            std::snprintf(range_label, sizeof(range_label), "Range: %.0f yd", display_range_yd);
            const ImVec2 range_sz = ImGui::CalcTextSize(range_label);
            const ImVec2 range_pos(canvas_end.x - range_sz.x - 6.0f, canvas_pos.y + 6.0f);
            draw_list->AddText(range_pos, IM_COL32(200, 200, 210, 255), range_label);
        }

        const ImVec2 center(canvas_pos.x + canvas_size.x * 0.5f,
                            canvas_pos.y + canvas_size.y * 0.5f);
        const float bullseye_radius = canvas_size.x * 0.45f;
        const float moa_to_px = bullseye_radius / max_span_moa_val;

        for (int ring = 1; ring <= TARGET_RING_COUNT; ++ring) {
            const float ring_radius = bullseye_radius * (static_cast<float>(ring) / ring_count_val);
            draw_list->AddCircle(center, ring_radius, IM_COL32(210, 210, 210, 255), 0, 1.0f);

            char ring_label[64];
            const float ring_moa = moa_per_ring * static_cast<float>(ring);
            const float ring_inches = ring_moa * inches_per_moa_val;
            std::snprintf(ring_label, sizeof(ring_label), "%.0f MOA (%.1f in)", ring_moa,
                          ring_inches);
            draw_list->AddText(ImVec2(center.x + 6.0f, center.y - ring_radius - 14.0f),
                               IM_COL32(200, 200, 210, 255), ring_label);
        }

        draw_list->AddLine(ImVec2(center.x - bullseye_radius, center.y),
                           ImVec2(center.x + bullseye_radius, center.y),
                           IM_COL32(180, 180, 180, 180), 1.0f);
        draw_list->AddLine(ImVec2(center.x, center.y - bullseye_radius),
                           ImVec2(center.x, center.y + bullseye_radius),
                           IM_COL32(180, 180, 180, 180), 1.0f);

        // Positive elevation hold draws up on screen (negative Y in ImGui).
        // Position impact dot using the solver's final hold values so visual and text
        // are always synchronized.
        const ImVec2 impact_point(center.x + hold_windage_moa * moa_to_px,
                      center.y - hold_elevation_moa * moa_to_px);


        // Analytic Gaussian uncertainty ellipse contours (no shot simulation/sampling).
        if (sol.uncertainty_valid &&
            (sol.sigma_elevation_moa > 0.001f || sol.sigma_windage_moa > 0.001f)) {
            // axis1_moa, axis2_moa, cos_r, sin_r already computed above for auto-scaling.

            constexpr int kSegments = 48;

            // Labeled contour outlines at 1σ / 2σ (no 3σ)
            const float sigma_levels[] = {1.0f, 2.0f};
            const ImU32 sigma_colors[] = {IM_COL32(255, 200, 80, 230), IM_COL32(180, 120, 255, 180)};
            for (int si = 0; si < 2; ++si) {
                const float k = sigma_levels[si];
                ImVec2 pts[kSegments];
                for (int s = 0; s < kSegments; ++s) {
                    const float ang = 2.0f * 3.14159265f * s / static_cast<float>(kSegments);
                    const float u = k * axis1_moa * std::cos(ang);
                    const float v = k * axis2_moa * std::sin(ang);
                    const float elev_moa = u * cos_r - v * sin_r;
                    const float wind_moa = u * sin_r + v * cos_r;
                    pts[s] = ImVec2(impact_point.x + wind_moa * moa_to_px,
                                    impact_point.y - elev_moa * moa_to_px);
                }
                draw_list->AddPolyline(pts, kSegments, sigma_colors[si], ImDrawFlags_Closed, 1.2f);
            }
        }

        draw_list->AddCircleFilled(impact_point, 2.0f, IM_COL32(255, 70, 70, 255));

        ImGui::End(); // End "Target View"

        auto nice_ceil_fn = [](float value) -> float {
            if (value <= 0.0f)
                return 1.0f;
            const float exponent = std::floor(std::log10(value));
            const float base = std::pow(10.0f, exponent);
            const float normalized = value / base;
            float snapped = 1.0f;
            if (normalized <= 1.0f)
                snapped = 1.0f;
            else if (normalized <= 1.25f)
                snapped = 1.25f;
            else if (normalized <= 2.0f)
                snapped = 2.0f;
            else if (normalized <= 2.5f)
                snapped = 2.5f;
            else if (normalized <= 5.0f)
                snapped = 5.0f;
            else
                snapped = 10.0f;
            return snapped * base;
        };

        ImGui::Begin("Side View Arc");
        // --- Restored bullet arc, muzzle, and impact point rendering ---
        ImDrawList* side_draw_list = ImGui::GetWindowDrawList();
        FiringSolution side_sol = frame_sol;
        float side_range_m = side_sol.horizontal_range_m;
        if (side_range_m <= 0.0f) {
            side_range_m = (side_sol.range_m > 0.0f) ? side_sol.range_m : g_state.lrf_range;
        }
        side_range_m = ClampValue(side_range_m, 1.0f, 5000.0f);
        const float range_m_to_display = is_imperial ? M_TO_YD : 1.0f;
        const float range_display_to_m = is_imperial ? YD_TO_M : 1.0f;
        const float offset_m_to_display = is_imperial ? 39.3700787f : 100.0f;

        const float side_range_display = side_range_m * range_m_to_display;
        const float side_range_scale_display = [&]() -> float {
            // Find the smallest "nice" scale that keeps the shot at ≥85% of the window width.
            // Uses finer increments than nice_ceil_fn to avoid the 500→1000 jump.
            const float min_scale = side_range_display * 1.05f; // 5% right-edge headroom
            const float exp = std::floor(std::log10(std::max(min_scale, 1.0f)));
            const float base = std::pow(10.0f, exp);
            const float steps[] = {1.0f, 1.2f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f,
                                   5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
            for (float s : steps) {
                if (s * base >= min_scale)
                    return ClampValue(s * base, 10.0f, is_imperial ? 6000.0f : 5000.0f);
            }
            return ClampValue(10.0f * base, 10.0f, is_imperial ? 6000.0f : 5000.0f);
        }();
        const float side_range_step_display = side_range_scale_display / 5.0f;
        const float side_range_scale_m = side_range_scale_display * range_display_to_m;
        const float side_range_step_m = side_range_step_display * range_display_to_m;
        const char* range_units = is_imperial ? "yd" : "m";
        const float elevation_angle_rad = side_sol.hold_elevation_moa * MOA_TO_RAD;
        const float elevation_angle_deg = side_sol.hold_elevation_moa / 60.0f;

        // Use solved geometry from engine output so target marker and arc endpoint share
        // the same reference frame even when UI input is clamped/filtered in-engine.
        // Account for Look Angle (incline). Compute slant->horizontal projection first.
        const float PI = 3.14159265f;
        const float look_angle_rad = side_sol.look_angle_deg * (PI / 180.0f);
        const float target_range_m = (side_sol.range_m > 0.0f) ? side_sol.range_m : g_state.lrf_range;
        const float target_dist_horiz_m =
            (side_sol.horizontal_range_m > 0.0f) ? side_sol.horizontal_range_m
                                                 : (target_range_m * std::cos(look_angle_rad));
        const float target_elev_m_engine = target_dist_horiz_m * std::tan(look_angle_rad);
        const float target_elev_m =
            std::isfinite(target_elev_m_engine) ? target_elev_m_engine : g_state.target_elevation_m;

        // Shared drop interpolation at horizontal distance.
        auto interpolate_drop_at_horizontal = [&](float dist_horiz, float& out_drop_m) -> bool {
            // Prefer GUI-side V2 trajectory profile (from selected cartridge preset)
            // when present so visuals match the engine's V2-path hold computation.
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                const CartridgePreset& cpres = g_cartridge_presets[g_state.selected_cartridge_preset];
                if (!cpres.trajectory_profile.empty()) {
                    // Build a small profile array similar to DOPE_ProfilePoint used by engine
                    std::vector<std::pair<float,float>> pts;
                    pts.reserve(cpres.trajectory_profile.size());
                    for (const auto &pp : cpres.trajectory_profile) {
                        pts.emplace_back(pp.first * INCHES_TO_M, pp.second * INCHES_TO_M);
                    }
                    // Interpolate piecewise-linearly (clamp to ends)
                    if (pts.size() == 1) {
                        out_drop_m = pts[0].second;
                        return true;
                    }
                    if (dist_horiz <= pts.front().first) {
                        out_drop_m = pts.front().second;
                        return true;
                    }
                    if (dist_horiz >= pts.back().first) {
                        out_drop_m = pts.back().second;
                        return true;
                    }
                    for (size_t i = 0; i + 1 < pts.size(); ++i) {
                        if (dist_horiz <= pts[i+1].first) {
                            const float dx = pts[i+1].first - pts[i].first;
                            const float t = (dx > 0.0f) ? ((dist_horiz - pts[i].first) / dx) : 0.0f;
                            out_drop_m = pts[i].second + t * (pts[i+1].second - pts[i].second);
                            return true;
                        }
                    }
                }
            }
            const int idx = static_cast<int>(std::floor(dist_horiz));
            const float frac = dist_horiz - static_cast<float>(idx);
            TrajectoryPoint tp0{}, tp1{};
            const bool h0 = DOPE_GetTrajectoryPoint(idx, &tp0);
            const bool h1 = DOPE_GetTrajectoryPoint(idx + 1, &tp1);
            if (h0 && h1) {
                out_drop_m = tp0.drop_m * (1.0f - frac) + tp1.drop_m * frac;
                return true;
            }
            if (h0) {
                out_drop_m = tp0.drop_m;
                return true;
            }
            if (h1) {
                out_drop_m = tp1.drop_m;
                return true;
            }
            out_drop_m = 0.0f;
            return false;
        };

        // Fetch solver trajectory state at target horizontal distance.
        float drop_at_target_interp = 0.0f;
        const bool have_traj_at_target =
            interpolate_drop_at_horizontal(target_dist_horiz_m, drop_at_target_interp);
        float impact_y_at_target_m = 0.0f;
        if (have_traj_at_target) {
            impact_y_at_target_m =
                target_dist_horiz_m * std::tan(side_sol.launch_angle_rad) + drop_at_target_interp;
        }
        // Keep this as raw trajectory-table drop (not impact height). The side-view
        // endpoint constraint is y(x)=x*q_a+x^2*q_b+drop(x), so q_b must be solved
        // against drop(target_x) to guarantee arc->target closure.
        const float traj_drop_at_target_m = drop_at_target_interp;

        // Legacy/proven side-view shaping:
        // - Aim (POA) comes from hold at target distance.
        // - Arc is sheared quadratically so it is tangent-consistent near muzzle and
        //   still lands on the selected target at target distance.
        const float geom_tan_theta = (have_traj_at_target && target_dist_horiz_m > 0.01f)
            ? ((target_elev_m - traj_drop_at_target_m) / target_dist_horiz_m)
            : (std::isfinite(std::tan(elevation_angle_rad)) ? std::tan(elevation_angle_rad) : 0.0f);
        const float hold_m = side_sol.hold_elevation_moa * MOA_TO_RAD * target_dist_horiz_m;
        const float bore_tan_theta = (target_dist_horiz_m > 0.01f)
            ? ((target_elev_m + hold_m) / target_dist_horiz_m)
            : geom_tan_theta;
        const float bore_proj_y_m = target_dist_horiz_m * bore_tan_theta;
        float table_slope_at_0 = 0.0f;
        {
            // Compute a numeric near-muzzle slope (dy/dx) using the 0m->1m points
            // when available. Use a safe clamp so low-velocity / short-barrel
            // pistol profiles don't produce extreme quadratic shearing.
            // Compute a numeric near-muzzle slope (dy/dx).
            // Prefer a 0->5m slope when available for stability on short/low-velocity
            // trajectories (pistols), else fall back to 0->1m.
            TrajectoryPoint tp0{};
            TrajectoryPoint tpN{};
            bool h0 = DOPE_GetTrajectoryPoint(0, &tp0);
            const int prefer_n = 5;
            bool hN = DOPE_GetTrajectoryPoint(prefer_n, &tpN);
            if (h0 && hN) {
                table_slope_at_0 = (tpN.drop_m - tp0.drop_m) / static_cast<float>(prefer_n);
            } else {
                TrajectoryPoint tp1{};
                bool h1 = DOPE_GetTrajectoryPoint(1, &tp1);
                if (h0 && h1) {
                    table_slope_at_0 = (tp1.drop_m - tp0.drop_m);
                } else if (h1) {
                    table_slope_at_0 = tp1.drop_m;
                } else {
                    table_slope_at_0 = 0.0f;
                }
            }
            // Clamp the slope to a physically-reasonable range to avoid
            // creating large q_a that bend the arc unrealistically near muzzle.
            // Tighter clamp to avoid excessive quadratic shearing on short/slow
            // trajectories (pistols). ~0.005 m/m ≈ 0.29°
            const float kMaxNearMuzzleSlope = 0.005f; // m vertical per m horizontal
            if (table_slope_at_0 > kMaxNearMuzzleSlope) table_slope_at_0 = kMaxNearMuzzleSlope;
            if (table_slope_at_0 < -kMaxNearMuzzleSlope) table_slope_at_0 = -kMaxNearMuzzleSlope;
        }
        const float q_a = bore_tan_theta - table_slope_at_0;
        const float q_b = (target_dist_horiz_m > 0.01f)
            ? ((target_elev_m - (traj_drop_at_target_m + q_a * target_dist_horiz_m)) /
               (target_dist_horiz_m * target_dist_horiz_m))
            : 0.0f;

        // Pre-pass: scan the engine trajectory table to find the real arc extents using the
        // corrected geometric slope (not hold_elevation_moa which would shift the extents wrong).
        float arc_y_hi = 0.0f;
        float arc_y_lo = 0.0f;
        {
            const int scan_max = static_cast<int>(std::ceil(target_dist_horiz_m));
            for (int r = 0; r <= scan_max; ++r) {
                TrajectoryPoint tp{};
                if (DOPE_GetTrajectoryPoint(r, &tp)) {
                    const float rf = static_cast<float>(r);
                    const float y_world = rf * q_a + rf * rf * q_b + tp.drop_m;
                    arc_y_hi = std::max(arc_y_hi, y_world);
                    arc_y_lo = std::min(arc_y_lo, y_world);
                }
            }
        }

        // Build Y window: include target, arc, and bore/POA line.
        const float y_lo_raw = std::min({0.0f, target_elev_m, arc_y_lo, bore_proj_y_m});
        const float y_hi_raw = std::max({0.0f, target_elev_m, arc_y_hi, bore_proj_y_m});
        const float span_raw = std::max(y_hi_raw - y_lo_raw, 0.01f);
        float y_lo = y_lo_raw - span_raw * 0.18f;   // 18% headroom bottom
        float y_hi = y_hi_raw + span_raw * 0.18f;   // 18% headroom top
        float y_span_m = y_hi - y_lo;

        // Keep the top-line metric consistent with the engine "Offset: Elev ..." readout:
        // hold distance at target derived from hold_elevation_moa.
        const float range_for_hold_m = (side_sol.range_m > 0.0f) ? side_sol.range_m : target_range_m;
        const float moa_linear_m = range_for_hold_m * std::tan(MOA_TO_RAD);
        const float drop_at_target_m = std::fabs(side_sol.hold_elevation_moa) * moa_linear_m;
        const float drop_display = drop_at_target_m * offset_m_to_display;
        const char* drop_units = is_imperial ? "in" : "cm";
        constexpr float kMToFt = 3.280839895f;
        float elevation_delta_display = is_imperial ? (g_state.target_elevation_m * kMToFt)
                               : g_state.target_elevation_m;
        float zero_range_display = is_imperial ? (g_state.zero.zero_range_m * M_TO_YD)
                               : g_state.zero.zero_range_m;
        ImGui::Text("Range: %.1f %s    Elev Delta: %.2f %s    Zero: %.1f %s", side_range_display,
                range_units, elevation_delta_display, is_imperial ? "ft" : "m",
                zero_range_display, range_units);
        ImGui::Text("Drop at target: %.2f %s    Required elev: %.4f deg", drop_display,
                drop_units, elevation_angle_deg);
        ImGui::Separator();
        ImVec2 side_avail = ImGui::GetContentRegionAvail();
        float side_canvas_w = side_avail.x;
        if (side_canvas_w < 260.0f)
            side_canvas_w = 260.0f;
        if (side_canvas_w > 900.0f)
            side_canvas_w = 900.0f;
        float side_canvas_h = side_avail.y;
        if (side_canvas_h < 180.0f)
            side_canvas_h = 180.0f;
        if (side_canvas_h > 320.0f)
            side_canvas_h = 320.0f;
        const ImVec2 side_canvas_size(side_canvas_w, side_canvas_h);
        const ImVec2 side_canvas_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##side_view_arc_canvas", side_canvas_size);
        const ImVec2 side_canvas_end(side_canvas_pos.x + side_canvas_size.x, side_canvas_pos.y + side_canvas_size.y);
        side_draw_list->AddRectFilled(side_canvas_pos, side_canvas_end, IM_COL32(24, 24, 28, 255));
        side_draw_list->AddRect(side_canvas_pos, side_canvas_end, IM_COL32(80, 80, 90, 255));
        const float left_margin = 82.0f;
        const float right_margin = 20.0f;
        const float top_margin = 20.0f;
        const float bottom_margin = 24.0f;
        const float plot_left = side_canvas_pos.x + left_margin;
        const float plot_right = side_canvas_end.x - right_margin;
        const float plot_top = side_canvas_pos.y + top_margin;
        const float plot_bottom = side_canvas_end.y - bottom_margin;
        const float plot_width = plot_right - plot_left;
        const float plot_height = plot_bottom - plot_top;

        // Stick-figure scale enforcement: if the 72in reference figure would render taller
        // than 80px at the current Y window, expand the Y window (not shrink the figure).
        // This keeps the pixel:metre scale physically honest — 80px always = 72 inches —
        // while ensuring the arc and aim point stay within view.
        // Centred on the content midpoint so both muzzle and target remain visible.
        {
            const float shooter_height_m_ref = 72.0f * (IN_TO_MM / 1000.0f); // 1.8288 m
            const float h_px_uncapped = shooter_height_m_ref * (plot_height / y_span_m);
            if (h_px_uncapped > 80.0f) {
                const float new_y_span_m = shooter_height_m_ref * plot_height / 80.0f;
                const float y_center = (y_lo + y_hi) * 0.5f;
                y_lo = y_center - new_y_span_m * 0.5f;
                y_hi = y_center + new_y_span_m * 0.5f;
                y_span_m = new_y_span_m;
            }
        }

        auto map_x = [&](float x_m) -> float { return plot_left + (x_m / side_range_scale_m) * plot_width; };
        auto map_y = [&](float y_m) -> float { const float t = (y_hi - y_m) / y_span_m; return plot_top + t * plot_height; };
        for (int i = 0; i <= 5; ++i) {
            const float grid_x_m = side_range_step_m * static_cast<float>(i);
            const float gx = map_x(grid_x_m);
            side_draw_list->AddLine(ImVec2(gx, plot_top), ImVec2(gx, plot_bottom), IM_COL32(60, 60, 70, 150), 1.0f);
            char tick_label[32];
            const float tick_display = is_imperial ? (grid_x_m * M_TO_YD) : grid_x_m;
            std::snprintf(tick_label, sizeof(tick_label), "%.0f", tick_display);
            side_draw_list->AddText(ImVec2(gx - 8.0f, plot_bottom + 3.0f), IM_COL32(180, 180, 190, 255), tick_label);
        }

        // Y-axis ticks and labels (height in m or ft)
        {
            // Choose a nice tick interval that produces ~5 ticks across the window
            const float raw_step = (y_hi - y_lo) / 5.0f;
            // Round to a clean value: find order of magnitude and snap
            const float mag = std::pow(10.0f, std::floor(std::log10(std::fabs(raw_step) + 1e-9f)));
            float y_tick_step = mag;
            if      (raw_step / mag >= 5.0f) y_tick_step = mag * 5.0f;
            else if (raw_step / mag >= 2.0f) y_tick_step = mag * 2.0f;
            y_tick_step = std::max(y_tick_step, 0.001f);

            float tick_y = std::floor(y_lo / y_tick_step) * y_tick_step;
            const char* y_unit_lbl = is_imperial ? "ft" : "m";
            // Unit label at top-left corner
            side_draw_list->AddText(ImVec2(side_canvas_pos.x + 2.0f, plot_top - 2.0f),
                                    IM_COL32(140, 140, 155, 200), y_unit_lbl);
            while (tick_y <= y_hi + y_tick_step * 0.01f) {
                const float sy = map_y(tick_y);
                if (sy >= plot_top - 1.0f && sy <= plot_bottom + 1.0f) {
                    // Tick mark
                    side_draw_list->AddLine(ImVec2(plot_left - 5.0f, sy), ImVec2(plot_left, sy),
                                            IM_COL32(120, 120, 130, 200), 1.0f);
                    // Horizontal grid line (very dim)
                    side_draw_list->AddLine(ImVec2(plot_left, sy), ImVec2(plot_right, sy),
                                            IM_COL32(55, 55, 65, 80), 1.0f);
                    // Label
                    char y_lbl[32];
                    const float disp_val = is_imperial ? (tick_y * 3.28084f) : tick_y;
                    std::snprintf(y_lbl, sizeof(y_lbl), "%.1f", disp_val);
                    side_draw_list->AddText(ImVec2(plot_left - 44.0f, sy - 6.0f),
                                            IM_COL32(160, 160, 170, 220), y_lbl);
                }
                tick_y += y_tick_step;
            }
        }
        
        // Small scale markers (10, 25, 50 yd/m)
        {
            const float m_list[] = {10.0f, 25.0f, 50.0f};
            for (float m_val : m_list) {
                const float m_m = is_imperial ? (m_val * YD_TO_M) : m_val;
                if (m_m < side_range_scale_m * 0.98f && m_m > 0.1f) {
                    const float mx = map_x(m_m);
                    bool redundant = false;
                    for (int i = 0; i <= 5; ++i) {
                        if (std::fabs(m_m - side_range_step_m * i) < (side_range_step_m * 0.02f)) {
                            redundant = true;
                            break;
                        }
                    }
                    if (!redundant) {
                        side_draw_list->AddLine(ImVec2(mx, plot_bottom - 5.0f), ImVec2(mx, plot_bottom), IM_COL32(130, 130, 145, 120), 1.0f);
                        char m_label[16];
                        std::snprintf(m_label, sizeof(m_label), "%.0f", m_val);
                        side_draw_list->AddText(ImVec2(mx - 6.0f, plot_bottom + 3.0f), IM_COL32(130, 130, 145, 180), m_label);
                    }
                }
            }
        }
        // Y-axis rule
        side_draw_list->AddLine(ImVec2(plot_left, plot_top), ImVec2(plot_left, plot_bottom),
                                IM_COL32(120, 120, 130, 180), 1.0f);
        const float sight_height_m = g_state.zero.sight_height_mm / 1000.0f;
        const ImVec2 muzzle_pt(map_x(0.0f), map_y(sight_height_m));

        // --- Stick figure (shooter silhouette) ---
        {
            const float shooter_height_in = 72.0f;
            const float shooter_height_m = shooter_height_in * (IN_TO_MM / 1000.0f);
            const float px_per_m_y = plot_height / y_span_m;
            const float h_px_true = shooter_height_m * px_per_m_y;
            const float h_px = std::fmin(std::fmax(h_px_true, 6.0f), 80.0f);
            const float shoulder_y = muzzle_pt.y;
            const float hip_down = 0.30f * h_px;
            const float knee_down = 0.52f * h_px;
            const float feet_down = 0.77f * h_px;
            const float head_r = std::fmax(0.10f * h_px, 4.5f);
            const float head_cy = shoulder_y - head_r * 1.4f;
            const float hip_y = shoulder_y + hip_down;
            const float knee_y = shoulder_y + knee_down;
            const float feet_y = shoulder_y + feet_down;
            const float sx = muzzle_pt.x - 22.0f;
            const float splay = std::fmax(0.08f * h_px, 4.0f);
            const ImU32 sc = IM_COL32(160, 175, 200, 220);
            const float sw = 1.6f;
            side_draw_list->AddCircle(ImVec2(sx, head_cy), head_r, sc, 16, sw);
            side_draw_list->AddLine(ImVec2(sx, head_cy + head_r), ImVec2(sx, hip_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx, hip_y), ImVec2(sx - splay, knee_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx, hip_y), ImVec2(sx + splay, knee_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx - splay, knee_y), ImVec2(sx - splay, feet_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx + splay, knee_y), ImVec2(sx + splay, feet_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx - splay, feet_y), ImVec2(sx - splay - 5.0f, feet_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx + splay, feet_y), ImVec2(sx + splay + 5.0f, feet_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx, shoulder_y), muzzle_pt, IM_COL32(175, 188, 210, 220), sw);
            side_draw_list->AddLine(ImVec2(sx, shoulder_y), ImVec2(sx - 12.0f, shoulder_y + 0.07f * h_px), sc, sw);
            const float brace_x = sx - splay - 18.0f;
            const float top_y = head_cy - head_r;
            side_draw_list->AddLine(ImVec2(brace_x, top_y), ImVec2(brace_x, feet_y),
                                    IM_COL32(110, 125, 150, 140), 1.0f);
            side_draw_list->AddLine(ImVec2(brace_x, top_y), ImVec2(brace_x + 3.0f, top_y),
                                    IM_COL32(110, 125, 150, 140), 1.0f);
            side_draw_list->AddLine(ImVec2(brace_x, feet_y), ImVec2(brace_x + 3.0f, feet_y),
                                    IM_COL32(110, 125, 150, 140), 1.0f);
            side_draw_list->AddText(
                ImVec2(brace_x - 33.0f, top_y + (feet_y - top_y) * 0.5f - 5.0f),
                IM_COL32(130, 140, 165, 210), "72in\n(ref)");
        }

        // Side-view geometry: one selected target, one POA at target distance,
        // sightline from sight marker, and trajectory from muzzle marker.
        const float launch_rad = side_sol.launch_angle_rad;
        const float target_x_m = target_dist_horiz_m;
        const float target_y_m = target_elev_m;
        const float hold_angle_rad = side_sol.hold_elevation_moa * MOA_TO_RAD;
        const float poa_y_m = target_y_m + target_x_m * std::tan(hold_angle_rad);
        const float zero_range_m = std::max(1.0f, g_state.zero.zero_range_m);
        const float sightline_tan =
            (target_x_m > 1e-6f) ? ((poa_y_m - sight_height_m) / target_x_m) : 0.0f;
        const float bore_visual_tan = sightline_tan + (sight_height_m / zero_range_m);

        // Draw trajectory arc in muzzle/world frame:
        // y(x) = x*q_a + x^2*q_b + drop_interp(x)
        const int sample_count2 = 100;
        ImVec2 arc_points2[sample_count2 + 1];
        for (int i = 0; i <= sample_count2; ++i) {
            const float dist_horiz = (target_x_m * i) / sample_count2;
            float drop_m_interp = 0.0f;
            if (!interpolate_drop_at_horizontal(dist_horiz, drop_m_interp)) {
                arc_points2[i] = ImVec2(map_x(dist_horiz), map_y(0.0f));
                continue;
            }
            const float y_w = dist_horiz * q_a + dist_horiz * dist_horiz * q_b + drop_m_interp;
            arc_points2[i] = ImVec2(map_x(dist_horiz), map_y(y_w));
        }
        side_draw_list->AddPolyline(arc_points2, sample_count2 + 1, IM_COL32(98, 203, 255, 255),
                                    ImDrawFlags_None, 2.0f);

        const ImVec2 muzzle_pixel(map_x(0.0f), map_y(0.0f));
        const ImVec2 sight_pixel(map_x(0.0f), map_y(sight_height_m));
        const ImVec2 target_pixel_pos(map_x(target_x_m), map_y(target_y_m));
        const ImVec2 aim_pixel(map_x(target_x_m), map_y(poa_y_m));

        // Marker semantics: muzzle, sight, target, and POA.
        side_draw_list->AddCircleFilled(muzzle_pixel, 3.5f, IM_COL32(130, 255, 130, 255));
        side_draw_list->AddCircle(sight_pixel, 6.0f, IM_COL32(150, 175, 220, 220), 0, 1.8f);
        side_draw_list->AddCircleFilled(target_pixel_pos, 5.0f, IM_COL32(255, 90, 90, 255));
        side_draw_list->AddCircleFilled(aim_pixel, 4.5f, IM_COL32(255, 200, 60, 255));

        // Bore direction line from muzzle using sight-height + zero-range convergence.
        // Extend the bore line to the far right of the plot (end of the grid).
        const ImVec2 bore_end_pixel(map_x(side_range_scale_m), map_y(side_range_scale_m * bore_visual_tan));
        side_draw_list->AddLine(muzzle_pixel, bore_end_pixel, IM_COL32(205, 170, 95, 150), 1.2f);

        // Draw full sightline from sight marker to POA at target distance.
        side_draw_list->AddLine(sight_pixel, aim_pixel, IM_COL32(255, 200, 60, 200), 1.6f);

        side_draw_list->AddText(ImVec2(muzzle_pixel.x + 6.0f, muzzle_pixel.y + 4.0f),
                                IM_COL32(140, 245, 140, 220), "Muzzle");
        side_draw_list->AddText(ImVec2(sight_pixel.x + 8.0f, sight_pixel.y - 12.0f),
                                IM_COL32(170, 190, 230, 220), "Sight");
        side_draw_list->AddText(ImVec2(target_pixel_pos.x + 6.0f, target_pixel_pos.y - 10.0f),
                                IM_COL32(255, 120, 120, 220), "Target");
        side_draw_list->AddText(ImVec2(aim_pixel.x + 6.0f, aim_pixel.y - 10.0f),
                                IM_COL32(255, 200, 140, 220), "Aim pt");
        // Label the bore line: move label slightly upward and match the line color.
        side_draw_list->AddText(ImVec2(bore_end_pixel.x - 40.0f, bore_end_pixel.y - 12.0f),
                    IM_COL32(205, 170, 95, 150), "Bore Line");

        // Debug diagnostics for deterministic side-view geometry.
        static bool side_view_debug = false;
        ImGui::Checkbox("Show Side View Debug", &side_view_debug);
        if (side_view_debug) {
            ImGui::Indent();
            ImGui::Text("DBG: target_x_m=%.3f m, target_y_m=%.3f m, poa_y_m=%.3f m",
                        target_x_m, target_y_m, poa_y_m);
            ImGui::Text("DBG: launch_angle_rad=%.6f, hold_moa=%.3f",
                        launch_rad, side_sol.hold_elevation_moa);
            ImGui::Text("DBG: bore_tan=%.6f, q_a=%.6f, q_b=%.9f",
                        bore_tan_theta, q_a, q_b);
            ImGui::Text("DBG: sightline_tan=%.6f, bore_visual_tan=%.6f",
                        sightline_tan, bore_visual_tan);
            ImGui::Text("DBG: zero_range_m=%.3f m, sight_height_m=%.3f m",
                        zero_range_m, sight_height_m);
            ImGui::Text("DBG: impact_y_at_target_m=%.3f m, drop_at_target_m=%.3f m",
                        impact_y_at_target_m, drop_at_target_m);

            // Print the near-muzzle table slope used for shaping
            ImGui::Text("DBG: table_slope_at_0=%.6f m/m", table_slope_at_0);

            // Show the raw trajectory table samples used to compute drop interpolation
            int sample_idx = static_cast<int>(std::floor(target_dist_horiz_m));
            TrajectoryPoint stp0{}; TrajectoryPoint stp1{};
            bool sh0 = DOPE_GetTrajectoryPoint(sample_idx, &stp0);
            bool sh1 = DOPE_GetTrajectoryPoint(sample_idx + 1, &stp1);
            const float sfrac = target_dist_horiz_m - static_cast<float>(sample_idx);
            float interp_check = 0.0f;
            if (sh0 && sh1) interp_check = stp0.drop_m * (1.0f - sfrac) + stp1.drop_m * sfrac;
            else if (sh0) interp_check = stp0.drop_m;
            else if (sh1) interp_check = stp1.drop_m;
            ImGui::Text("DBG: traj_idx=%d frac=%.3f h0=%d h1=%d", sample_idx, sfrac, sh0 ? 1 : 0, sh1 ? 1 : 0);
            ImGui::Text("DBG: tp0.drop_m=%.6f m, tp1.drop_m=%.6f m, interp_calc=%.6f m",
                        stp0.drop_m, stp1.drop_m, interp_check);

            // Dump the full solver trajectory table for deeper inspection.
            ImGui::Text("DBG: Full trajectory table (index range drop):");
            for (int ti = 0; ti < DOPE_MAX_TABLE_POINTS; ++ti) {
                TrajectoryPoint ttp{};
                if (!DOPE_GetTrajectoryPoint(ti, &ttp))
                    break;
                ImGui::Text("DBG: table[%d] drop=%.6f m", ti, ttp.drop_m);
            }
            ImGui::Unindent();

            // Add a button to print the diagnostics once to stdout (avoids flooding the console).
            if (ImGui::Button("Print Side View Debug to Console")) {
                std::printf("SVDBG: target_x_m=%.3f m, target_y_m=%.3f m, poa_y_m=%.3f m\n",
                            target_x_m, target_y_m, poa_y_m);
                std::printf("SVDBG: launch_angle_rad=%.6f, hold_moa=%.3f\n",
                            launch_rad, side_sol.hold_elevation_moa);
                std::printf("SVDBG: bore_tan=%.6f, q_a=%.6f, q_b=%.9f\n",
                            bore_tan_theta, q_a, q_b);
                std::printf("SVDBG: sightline_tan=%.6f, bore_visual_tan=%.6f\n",
                            sightline_tan, bore_visual_tan);
                std::printf("SVDBG: zero_range_m=%.3f m, sight_height_m=%.3f m\n",
                            zero_range_m, sight_height_m);
                std::printf("SVDBG: impact_y_at_target_m=%.3f m, drop_at_target_m=%.3f m\n",
                            impact_y_at_target_m, drop_at_target_m);
                std::printf("SVDBG: table_slope_at_0=%.6f m/m\n", table_slope_at_0);
                std::printf("SVDBG: traj_idx=%d frac=%.3f h0=%d h1=%d\n",
                            sample_idx, sfrac, sh0 ? 1 : 0, sh1 ? 1 : 0);
                std::printf("SVDBG: tp0.drop_m=%.6f m, tp1.drop_m=%.6f m, interp_calc=%.6f m\n",
                            stp0.drop_m, stp1.drop_m, interp_check);
                std::printf("SVDBG: Full trajectory table (index drop):\n");
                for (int ti = 0; ti < DOPE_MAX_TABLE_POINTS; ++ti) {
                    TrajectoryPoint ttp{};
                    if (!DOPE_GetTrajectoryPoint(ti, &ttp))
                        break;
                    std::printf("SVDBG: table[%d] drop=%.6f m\n", ti, ttp.drop_m);
                }
                std::fflush(stdout);
            }
        }
        ImGui::End();

        ImGui::Begin("Top Down Drift");
        {
            ImDrawList* drift_draw_list = ImGui::GetWindowDrawList();
            FiringSolution d_sol = frame_sol;
            float d_range_m =
                d_sol.horizontal_range_m > 0.0f ? d_sol.horizontal_range_m : g_state.lrf_range;
            d_range_m = ClampValue(d_range_m, 1.0f, 5000.0f);

            // Flip lateral sign so positive `hold_windage_moa` (right) maps to
            // positive X to the right in the top-down canvas.
            float lateral_m = -d_sol.hold_windage_moa * MOA_TO_RAD * d_range_m;
            // d_range_scale_m will be set later after plot bounds

            // More sensitive scale for small drifts
            const float lat_abs_m = std::fabs(lateral_m);
            const float min_lat_scale_m =
                (is_imperial ? 6.0f * IN_TO_MM / 1000.0f : 0.15f); // 6 inches or 15cm
            float lateral_scale_m = std::max(lat_abs_m * 1.5f, min_lat_scale_m);

            // Aim offset text removed from top-down view per user request.
            ImGui::Separator();

            ImVec2 d_avail = ImGui::GetContentRegionAvail();
            const ImVec2 d_canvas_size(ClampValue(d_avail.x, 260.0f, 900.0f),
                                       ClampValue(d_avail.y, 180.0f, 380.0f));
            const ImVec2 d_canvas_pos = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton("##drift_canvas", d_canvas_size);
            const ImVec2 d_canvas_end(d_canvas_pos.x + d_canvas_size.x,
                                      d_canvas_pos.y + d_canvas_size.y);
            // --- Enforce minimum 1:2 scale ratio for shot length ---
            // y-axis scaling
            // Use d_range_m and d_range_scale_m, define off_disp_to_m and min_ratio locally
            const float elevation_angle_rad = 0.0f; // Set to 0 or replace with actual value if available
            const float arc_peak_m = std::fabs(std::tan(elevation_angle_rad)) * d_range_m * 0.5f;
            const float off_disp_to_m = is_imperial ? IN_TO_MM / 1000.0f : 0.01f;
            const float target_elev_m = g_state.target_elevation_m;
            float max_val = arc_peak_m;
            max_val = std::max(max_val, std::fabs(target_elev_m));
            max_val = std::max(max_val, off_disp_to_m * 5.0f);
            // removed unused vertical scale
            // Minimum horizontal scale: must be at least half the vertical scale

            drift_draw_list->AddRectFilled(d_canvas_pos, d_canvas_end, IM_COL32(24, 24, 28, 255));
            drift_draw_list->AddRect(d_canvas_pos, d_canvas_end, IM_COL32(80, 80, 90, 255));

            const float d_plot_left = d_canvas_pos.x + 35.0f;
            const float d_plot_right = d_canvas_end.x - 35.0f;
            const float d_plot_top = d_canvas_pos.y + 20.0f;
            const float d_plot_bottom = d_canvas_end.y - 25.0f;
            const float d_center_x = d_plot_left + (d_plot_right - d_plot_left) * 0.5f;

            // Set and enforce horizontal scale here
            const float stick_height_m = 2.0f;
            float shot_length_m = d_range_m; // horizontal shot length
            float min_horizontal_scale = std::max(stick_height_m, shot_length_m);
            float d_range_scale_m = min_horizontal_scale;

            auto d_map_x = [&](float lat_m) {
                return d_center_x + (lat_m / lateral_scale_m) * (d_plot_right - d_plot_left) * 0.5f;
            };
            auto d_map_y = [&](float fwd_m) {
                return d_plot_bottom - (fwd_m / d_range_scale_m) * (d_plot_bottom - d_plot_top);
            };

            // Grid - Range lines
            // Adjust range step to match forced scaling
            const float d_range_step_m = (d_range_scale_m / 5.0f);
            for (int i = 0; i <= 5; ++i) {
                float gy_m = i * d_range_step_m;
                float gy = d_map_y(gy_m);
                drift_draw_list->AddLine(ImVec2(d_plot_left, gy), ImVec2(d_plot_right, gy),
                                         IM_COL32(60, 60, 70, 100), 1.0f);
                char tick[32];
                std::snprintf(tick, sizeof(tick), "%.0f", gy_m * (is_imperial ? M_TO_YD : 1.0f));
                drift_draw_list->AddText(ImVec2(d_plot_right + 4.0f, gy - 7.0f),
                                         IM_COL32(140, 140, 150, 255), tick);
            }

            // Grid - Lateral lines
            // Adjust lateral step to match forced scaling
            const float lat_step_m = nice_ceil_fn(d_range_scale_m * (is_imperial ? 39.37f : 100.0f) / 3.0f) / (is_imperial ? 39.37f : 100.0f);
            for (float lx_m = -std::floor(lateral_scale_m / lat_step_m) * lat_step_m;
                 lx_m <= lateral_scale_m + 0.001f; lx_m += lat_step_m) {
                float gx = d_map_x(lx_m);
                if (gx < d_plot_left || gx > d_plot_right)
                    continue;
                drift_draw_list->AddLine(ImVec2(gx, d_plot_top), ImVec2(gx, d_plot_bottom),
                                         IM_COL32(60, 60, 70, 80), 1.0f);
                char tick[32];
                std::snprintf(tick, sizeof(tick), "%.0f", lx_m * (is_imperial ? 39.37f : 100.0f));
                drift_draw_list->AddText(ImVec2(gx - 8.0f, d_plot_bottom + 4.0f),
                                         IM_COL32(150, 150, 160, 255), tick);
            }

            drift_draw_list->AddLine(ImVec2(d_center_x, d_plot_top),
                                     ImVec2(d_center_x, d_plot_bottom),
                                     IM_COL32(170, 170, 180, 150), 1.0f);

            // Trajectory
            ImVec2 d_pts[65];
            for (int i = 0; i <= 64; ++i) {
                float t = (float)i / 64.0f;
                // Simple quadratic drift approximation for visual: x = t^2 * drift
                // But we aim at -drift to hit center: x_pos = -drift*t + drift*t^2
                float x = lateral_m * (t * t - t);
                d_pts[i] = ImVec2(d_map_x(x), d_map_y(t * d_range_m));
            }
            drift_draw_list->AddPolyline(d_pts, 65, IM_COL32(255, 180, 90, 255), 0, 2.2f);

            const ImVec2 d_muzzle(d_map_x(0.0f), d_map_y(0.0f));
            // Offset impact vertically by target_elev_m in pixels
            const ImVec2 d_impact(d_map_x(0.0f), d_map_y(d_range_m) - target_elev_m * ((d_plot_bottom - d_plot_top) / d_range_scale_m));
            const ImVec2 d_aim(d_map_x(-lateral_m), d_map_y(d_range_m)); // Aim point is offset

            // Target: single known shot point — draw a crosshair, not a wall.
            drift_draw_list->AddCircle(d_impact, 5.0f, IM_COL32(255, 100, 100, 220), 0, 1.5f);
            drift_draw_list->AddLine(ImVec2(d_impact.x - 8.0f, d_impact.y),
                                     ImVec2(d_impact.x + 8.0f, d_impact.y),
                                     IM_COL32(255, 100, 100, 180), 1.0f);
            drift_draw_list->AddLine(ImVec2(d_impact.x, d_impact.y - 8.0f),
                                     ImVec2(d_impact.x, d_impact.y + 8.0f),
                                     IM_COL32(255, 100, 100, 180), 1.0f);

            drift_draw_list->AddCircleFilled(d_muzzle, 3.5f, IM_COL32(130, 255, 130, 255));
            drift_draw_list->AddCircleFilled(d_impact, 4.0f, IM_COL32(255, 90, 90, 255));
            drift_draw_list->AddCircleFilled(d_aim, 3.0f, IM_COL32(255, 210, 120, 180));
            drift_draw_list->AddLine(d_muzzle, d_aim, IM_COL32(200, 200, 200, 100),
                                     1.0f); // Line of aim

            // Wind indicators (Top Down)
            {
                auto draw_arrow_fn = [&](const ImVec2& from, const ImVec2& to, ImU32 color) {
                    drift_draw_list->AddLine(from, to, color, 1.2f);
                    const float dx = to.x - from.x;
                    const float dy = to.y - from.y;
                    const float len = std::sqrt(dx * dx + dy * dy);
                    if (len < 0.1f)
                        return;
                    const float ux = dx / len;
                    const float uy = dy / len;
                    const float px = -uy;
                    const float py = ux;
                    const ImVec2 p1(to.x - ux * 4.0f + px * 2.5f, to.y - uy * 4.0f + py * 2.5f);
                    const ImVec2 p2(to.x - ux * 4.0f - px * 2.5f, to.y - uy * 4.0f - py * 2.5f);
                    drift_draw_list->AddTriangleFilled(to, p1, p2, color);
                };

                const ImVec2 axis_origin(d_plot_right - 40.0f, d_plot_top + 40.0f);
                const float wind_relative_rad_td =
                    (g_state.wind_heading - d_sol.heading_deg_true) * dope::math::DEG_TO_RAD;
                const float cw_ms = g_state.wind_speed_ms * std::sin(wind_relative_rad_td);
                const float hw_ms = g_state.wind_speed_ms * std::cos(wind_relative_rad_td);
                const float w_scale = (is_imperial ? MPS_TO_MPH : 1.0f);

                // Crosswind arrow
                if (std::fabs(cw_ms) > 0.05f) {
                    const float adir = (cw_ms > 0) ? -1.0f : 1.0f;
                    draw_arrow_fn(ImVec2(axis_origin.x - adir * 15.0f, axis_origin.y),
                                  ImVec2(axis_origin.x + adir * 15.0f, axis_origin.y),
                                  IM_COL32(180, 215, 255, 220));
                    char cw_txt[16];
                    std::snprintf(cw_txt, sizeof(cw_txt), "%.1f", std::fabs(cw_ms) * w_scale);
                    drift_draw_list->AddText(ImVec2(axis_origin.x - 10.0f, axis_origin.y + 5.0f),
                                             IM_COL32(180, 215, 255, 220), cw_txt);
                }
                // Headwind arrow
                if (std::fabs(hw_ms) > 0.05f) {
                    const float adir = (hw_ms > 0) ? 1.0f : -1.0f;
                    draw_arrow_fn(ImVec2(axis_origin.x, axis_origin.y + adir * 15.0f),
                                  ImVec2(axis_origin.x, axis_origin.y - adir * 15.0f),
                                  IM_COL32(255, 175, 120, 220));
                }

                // Windage bracket: horizontal line at target range between aim point and target center
                {
                    const float wind_in  = std::fabs(d_sol.hold_windage_moa) * inches_per_moa_val;
                    const ImVec2 td_target_ctr(d_map_x(0.0f), d_map_y(d_range_m));
                    const float left_x   = std::min(d_aim.x, td_target_ctr.x);
                    const float right_x  = std::max(d_aim.x, td_target_ctr.x);
                    const float mid_x    = (left_x + right_x) * 0.5f;
                    const float by       = d_aim.y - 10.0f; // bracket sits just above the aim point
                    if (right_x - left_x > 4.0f) {
                        drift_draw_list->AddLine(ImVec2(left_x,  by), ImVec2(right_x, by), IM_COL32(200, 200, 90, 180), 1.0f);
                        drift_draw_list->AddLine(ImVec2(left_x,  by - 3.0f), ImVec2(left_x,  by + 3.0f), IM_COL32(200, 200, 90, 180), 1.0f);
                        drift_draw_list->AddLine(ImVec2(right_x, by - 3.0f), ImVec2(right_x, by + 3.0f), IM_COL32(200, 200, 90, 180), 1.0f);
                    }
                    char wind_bracket_lbl[64];
                    std::snprintf(wind_bracket_lbl, sizeof(wind_bracket_lbl), "%.2fin\n%.2f MOA",
                                  wind_in, d_sol.hold_windage_moa);
                    drift_draw_list->AddText(ImVec2(mid_x - 20.0f, by - 22.0f), IM_COL32(220, 220, 100, 230), wind_bracket_lbl);
                }

                // Windage annotation (measure text and clamp inside canvas)
                const float wy = d_plot_bottom - 28.0f;
                const float wind_in = std::fabs(d_sol.hold_windage_moa) * inches_per_moa_val;
                char wind_label_txt[64];
                std::snprintf(wind_label_txt, sizeof(wind_label_txt), "Wind: %.2f MOA (%.2f in)",
                              d_sol.hold_windage_moa, wind_in);

                // Measure text width and add small horizontal padding
                ImVec2 txt_sz = ImGui::CalcTextSize(wind_label_txt);
                const float pad_h = 8.0f;
                const float label_w = txt_sz.x + pad_h;
                const float max_right = d_canvas_end.x - 4.0f;
                const float min_left = d_plot_left + 4.0f;

                // Anchor near plot right but ensure fully inside canvas
                float rect_right = std::min(d_plot_right - 4.0f, max_right);
                float rect_left = rect_right - label_w;
                if (rect_left < min_left) {
                    rect_left = min_left;
                    rect_right = rect_left + label_w;
                    if (rect_right > max_right)
                        rect_right = max_right;
                }

                drift_draw_list->AddRectFilled(ImVec2(rect_left, wy + 6.0f),
                                               ImVec2(rect_right, wy + 20.0f),
                                               IM_COL32(10, 10, 14, 210));
                drift_draw_list->AddText(ImVec2(rect_left + pad_h * 0.5f, wy + 8.0f),
                                         IM_COL32(110, 235, 255, 255), wind_label_txt);
            }
        }
        ImGui::End();

        // Rendering
        ImGui::Render();
        const float clear_color_with_alpha[4] = {0.10f, 0.10f, 0.12f, 1.00f};
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        g_pSwapChain->Present(1, 0);
    }

    g_ticker_running = false;
    if (ticker_thread.joinable()) {
        ticker_thread.join();
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    DestroyWindow(hwnd);
    UnregisterClass(wc.lpszClassName, wc.hInstance);

    return 0;
}
