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

#include <atomic>
#include <algorithm>
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
#include <vector>
#include <utility>

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
// Engine ticker thread ╬ô├ç├╢ mirrors the FreeRTOS task pattern on the ESP32-P4.
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

struct CartridgeProfile {
    const char* name;
    float bc;
    int drag_model_index;
    float muzzle_velocity_ms;
    float mass_grains;
    float caliber_inches;
    float length_mm;
};
// Cartridge demo profiles removed; use external JSON `dope_gui_cartridges.json`.

struct CartridgePreset {
    std::string name;
    float bc = 0.0f;
    int drag_model_index = 0;
    float muzzle_velocity_ms = 0.0f;
    float mass_grains = 0.0f;
    float caliber_inches = 0.0f;
    float length_mm = 0.0f;
    std::vector<std::string> cartridge_keys;
    float reference_barrel_inches = 24.0f;
    float sd_mv_fps = 0.0f; // Standard deviation of muzzle velocity (fps)
    float mv_adjustment_fps_per_in = 0.0f;
    std::vector<std::pair<float,float>> barrel_mv_profile;
    std::vector<std::pair<float,float>> velocity_profile;
    std::vector<std::pair<float,float>> trajectory_profile;
    std::vector<std::string> tags;
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
    std::vector<std::string> cartridge_keys; // Compatible cartridge identifiers (intersection with cartridge preset required when present)
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
// Sensor hardware presets ╬ô├ç├╢ each entry specifies the 1-sigma accuracy for
// the sensor's output(s).  "Custom" (index 0) lets the user type a manual
// value; all other entries auto-populate the sigma fields.
// ---------------------------------------------------------------------------

// --- LRF accuracy tables (range-dependent) --------------------------------
// Each LRF model has a piecewise-linear table of (range_m, sigma_m) break-
// points; the effective sigma is interpolated at the current LRF range.

// ---------------------------------------------------------------------------
// Generic sensor preset ╬ô├ç├╢ name + DOPE_ErrorTable (x = sensor-specific unit,
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
//   0.15╬ô├ç├┤22 m  : Γö¼ΓûÆ0.2 mm to Γö¼ΓûÆ1 mm
//   5╬ô├ç├┤400 m    : ╬ô├½├▒ Γö¼ΓûÆ1 m
//   400╬ô├ç├┤1000 m : Γö¼ΓûÆ1.2 m to Γö¼ΓûÆ3 m
//   1000╬ô├ç├┤2000 m: Γö¼ΓûÆ3 m to Γö¼ΓûÆ6 m
static const DOPE_ErrorPoint kLRF_D09C[] = {
    {0.15f, 0.0002f}, // 0.2 mm
    {22.0f, 0.001f},  // 1 mm (end of precision band)
    {400.0f, 1.0f},   // ╬ô├½├▒ Γö¼ΓûÆ1 m
    {1000.0f, 3.0f},  // Γö¼ΓûÆ3 m
    {2000.0f, 6.0f},  // Γö¼ΓûÆ6 m
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
//   ╬ô├¬├å40 to ╬ô├¬├å20 Γö¼ΓûæC : Γö¼ΓûÆ0.15 Γö¼ΓûæC  (extended edge)
//   ╬ô├¬├å20 to +50 Γö¼ΓûæC : Γö¼ΓûÆ0.10 Γö¼ΓûæC  (main calibrated range)
//   +50 to +70 Γö¼ΓûæC : Γö¼ΓûÆ0.15 Γö¼ΓûæC  (extended edge)
static const DOPE_ErrorPoint kTemp_TMP117[] = {
    {-40.0f, 0.15f}, {-20.0f, 0.15f}, {-19.9f, 0.10f}, // step at ╬ô├¬├å20 Γö¼ΓûæC lower edge of main range
    {50.0f, 0.10f},  {50.1f, 0.15f},                   // step at +50 Γö¼ΓûæC upper edge of main range
    {70.0f, 0.15f},
};

// Bosch BMP581 ╬ô├ç├╢ onboard temperature sensor
//    ╬ô├¬├å5 to +55 Γö¼ΓûæC : Γö¼ΓûÆ0.52 Γö¼ΓûæC  (calibrated operating range)
//   outside range  : Γö¼ΓûÆ1.52 Γö¼ΓûæC
static const DOPE_ErrorPoint kTemp_BMP581[] = {
    {-40.0f, 1.52f}, {-5.0f, 1.52f}, {-4.9f, 0.52f}, // step at ╬ô├¬├å5 Γö¼ΓûæC lower edge of cal range
    {55.0f, 0.52f},  {55.1f, 1.52f},                 // step at +55 Γö¼ΓûæC upper edge of cal range
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
    {"PNI RM3100", 1.5f}, // flat Γö¼ΓûÆ1.5Γö¼Γûæ from magnetic dip estimation
};
static const int kNumMagLatPresets = sizeof(kMagLatPresets) / sizeof(kMagLatPresets[0]);

// --- GPS / GNSS module presets ---
// Used when the application feeds latitude directly from a GNSS receiver.
// sigma_lat_deg reflects the receiver's position accuracy converted to degrees.
// No presets yet ╬ô├ç├╢ framework ready for specific modules to be added later.
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
    {"Custom", 8.5f},      // editable
    {"Stainless (Match)", 5.0f},
    {"Nitride (QPQ)", 8.5f},
    {"Chrome", 20.0f},
};
static const int kNumBarrelFinishPresets = sizeof(kBarrelFinishPresets) / sizeof(kBarrelFinishPresets[0]);
static const int kBarrelFinishDefaultIndex = 2; // Nitride

struct BarrelMaterialPreset {
    const char* key;   // JSON key
    const char* label; // UI label
    BarrelMaterial material;
    float stiffness_scale;      // relative deflection vs stainless baseline
    float density_kg_m3;
    float specific_heat_J_kgK;
    float cte_scale;            // thermal expansion vs stainless baseline
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
    BulletProfile bullet = {};
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
    float baro_temp = 18.333f; // 65 Γö¼ΓûæF
    float baro_humidity = 0.65f;
    float lrf_range = 457.2f; // 500 yd
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
static bool  g_mv_adjustment_estimated = false;  // true when fallback tier estimator is active
static float g_active_mv_adj_fps_per_in = 0.0f;  // fps/in used last time ComputeAdjustedMv ran
static std::vector<std::pair<float,float>> g_active_barrel_mv_profile; // (barrel_in, mv_fps)

using json = nlohmann::json;

static json g_bullet_tolerances;

// Legacy per-type tolerance tables can still be embedded; when absent we expect
// per-cartridge sigma fields to be present instead.

// Capture tolerances from any object that embeds a bullet_type_tolerances block.
static void CaptureBulletTolerances(const json& root) {
    if (!root.is_object()) return;
    const auto tol_it = root.find("bullet_type_tolerances");
    if (tol_it == root.end() || !tol_it->is_object()) return;

    json captured = json::object();
    if (root.contains("__comments")) captured["__comments"] = root["__comments"];
    if (root.contains("_comments"))  captured["_comments"] = root["_comments"];
    if (root.contains("bullet_tolerance_comments")) captured["__comments"] = root["bullet_tolerance_comments"];
    captured["bullet_type_tolerances"] = *tol_it;
    g_bullet_tolerances = captured;
}

static bool CartridgesHaveSigmaValues(const std::vector<CartridgePreset>& presets) {
    for (const auto& p : presets) {
        if (p.sd_mv_fps > 0.0f) {
            return true;
        }
    }
    return false;
}


void SanitizeCartridgePreset(CartridgePreset& preset);
void SanitizeGunPreset(GunPreset& preset);
json SerializeCartridgePreset(const CartridgePreset& input);
json SerializeGunPreset(const GunPreset& input);
// Load external preset files (returns true if loaded and non-empty)
bool LoadCartridgePresetsFromFile(const std::string& path);
bool LoadGunPresetsFromFile(const std::string& path);
bool LoadHardwarePresetsFromFile(const std::string& path);
float GetSelectedCartridgeCaliberInches();
bool IsGunPresetCompatibleWithSelectedCaliber(const GunPreset& preset);
void EnsureSelectedGunPresetMatchesCurrentCaliber();
void SelectDefaultGunPresetForSelectedCartridge();
void ApplyGunPreset(const GunPreset& preset);
static float ComputeStiffnessMoa(float muzzle_diameter_in, float bore_diameter_in,
                                 BarrelMaterial material);
void ComputeAdjustedMv(); // glue-layer barrel-length ΓåÆ MV interpolation

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
    if (!ifs.good()) return false;
    try {
        json j;
        ifs >> j;
        if (!j.is_object()) return false;
        const auto sensors = j.find("sensors");
        if (sensors == j.end() || !sensors->is_object()) return false;

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
        auto resolve_preset_index = [](const std::string& name, const SensorPreset* presets, int count) -> int {
            for (int k = 0; k < count; ++k) {
                if (presets[k].name && name == presets[k].name) return k;
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
inline int GetNumBarometerPresets() { return g_baro_labels.empty() ? kNumBarometerPresets : (int)g_baro_labels.size(); }
inline int GetNumTempSensorPresets() { return g_temp_labels.empty() ? kNumTempSensorPresets : (int)g_temp_labels.size(); }
inline int GetNumLRFPresets() { return g_lrf_labels.empty() ? kNumLRFPresets : (int)g_lrf_labels.size(); }
inline int GetNumCantPresets() { return g_cant_labels.empty() ? kNumCantPresets : (int)g_cant_labels.size(); }
inline int GetNumMagLatPresets() { return g_maglat_labels.empty() ? kNumMagLatPresets : (int)g_maglat_labels.size(); }
inline int GetNumGpsLatPresets() { return g_gps_labels.empty() ? kNumGpsLatPresets : (int)g_gps_labels.size(); }

inline const char* GetBarometerLabel(int i) { return g_baro_labels.empty() ? ((i>=0 && i<kNumBarometerPresets) ? kBarometerPresets[i].name : "") : g_baro_labels[i].c_str(); }
inline const char* GetTempSensorLabel(int i) { return g_temp_labels.empty() ? ((i>=0 && i<kNumTempSensorPresets) ? kTempSensorPresets[i].name : "") : g_temp_labels[i].c_str(); }
inline const char* GetLRFLabel(int i) { return g_lrf_labels.empty() ? ((i>=0 && i<kNumLRFPresets) ? kLRFPresets[i].name : "") : g_lrf_labels[i].c_str(); }
inline const char* GetCantLabel(int i) { return g_cant_labels.empty() ? ((i>=0 && i<kNumCantPresets) ? kCantPresets[i].name : "") : g_cant_labels[i].c_str(); }
inline const char* GetMagLatLabel(int i) { return g_maglat_labels.empty() ? ((i>=0 && i<kNumMagLatPresets) ? kMagLatPresets[i].name : "") : g_maglat_labels[i].c_str(); }
inline const char* GetGpsLabel(int i) { return g_gps_labels.empty() ? ((i>=0 && i<kNumGpsLatPresets) ? kGpsLatPresets[i].name : "") : g_gps_labels[i].c_str(); }

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
        const int mapped = (i >= 0 && i < (int)g_lrf_preset_indices.size()) ? g_lrf_preset_indices[i] : 0;
        return (mapped >= 0 && mapped < kNumLRFPresets) ? &kLRFPresets[mapped].table : nullptr;
    }
    return (i>=0 && i<kNumLRFPresets) ? &kLRFPresets[i].table : nullptr;
}
inline const DOPE_ErrorTable* GetTempTable(int i) {
    if (!g_temp_labels.empty()) {
        // Dynamic mode: map dynamic index -> static preset index via name-resolution table.
        const int mapped = (i >= 0 && i < (int)g_temp_preset_indices.size()) ? g_temp_preset_indices[i] : 0;
        return (mapped >= 0 && mapped < kNumTempSensorPresets) ? &kTempSensorPresets[mapped].table : nullptr;
    }
    return (i>=0 && i<kNumTempSensorPresets) ? &kTempSensorPresets[i].table : nullptr;
}
inline const DOPE_ErrorTable* GetBarometerTable(int i) {
    const int mapped = GetMappedBarometerPresetIndex(i);
    return (mapped>=0 && mapped<kNumBarometerPresets) ? &kBarometerPresets[mapped].table : nullptr;
}
inline float GetCantSigma(int i) { return g_cant_sigmas.empty() ? ((i>=0 && i<kNumCantPresets) ? kCantPresets[i].sigma_cant_deg : 0.0f) : g_cant_sigmas[i]; }

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
            }
        } else if (j.is_array()) {
            preset_array = &j;
        }

        if (preset_array == nullptr) return false;
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
            p.mass_grains = e.value("mass_grains", 0.0f);
            p.caliber_inches = e.value("caliber_inches", 0.0f);
            p.length_mm = e.value("length_mm", 0.0f);
            p.cartridge_keys.clear();
            const auto ck_it = e.find("cartridge_keys");
            if (ck_it != e.end() && ck_it->is_array()) {
                for (const auto& k : *ck_it) {
                    if (k.is_string()) p.cartridge_keys.push_back(k.get<std::string>());
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
            p.sd_mv_fps = e.value("sd_mv_fps", 0.0f);
            // tags (optional): ["tier","construction"]
            p.tags.clear();
            const auto tags_it = e.find("tags");
            if (tags_it != e.end() && tags_it->is_array()) {
                for (const auto& t : *tags_it) {
                    if (t.is_string()) p.tags.push_back(t.get<std::string>());
                }
            }
            // velocity_profile (optional)
            p.velocity_profile.clear();
            const auto vp_it = e.find("velocity_profile");
            if (vp_it != e.end() && vp_it->is_array()) {
                for (const auto& vp : *vp_it) {
                    const float dist = vp.value("distance_inches", 0.0f);
                    float vel = 0.0f;
                    if (vp.contains("velocity_ms") && vp["velocity_ms"].is_number()) {
                        vel = vp["velocity_ms"].get<float>();
                    } else if (vp.contains("velocity_fps") && vp["velocity_fps"].is_number()) {
                        vel = vp["velocity_fps"].get<float>() * FPS_TO_MPS;
                    } else {
                        vel = vp.value("velocity_mps", 0.0f);
                    }
                    p.velocity_profile.emplace_back(dist, vel);
                }
            } else if (p.muzzle_velocity_ms > 0.0f) {
                p.velocity_profile.emplace_back(0.0f, p.muzzle_velocity_ms);
            }
            // trajectory_profile (optional)
            p.trajectory_profile.clear();
            const auto tp_it = e.find("trajectory_profile");
            if (tp_it != e.end() && tp_it->is_array()) {
                for (const auto& tp : *tp_it) {
                    const float dist = tp.value("distance_inches", 0.0f);
                    const float traj = tp.value("trajectory_inches", tp.value("trajectory_in", 0.0f));
                    p.trajectory_profile.emplace_back(dist, traj);
                }
            }
            // cartridge CEP (optional)
            // Removed obsolete CEP fields
            // barrel-MV fields (optional)
            p.mv_adjustment_fps_per_in = e.value("mv_adjustment_fps_per_in", 0.0f);
            p.barrel_mv_profile.clear();
            const auto bmp_it = e.find("barrel_mv_profile");
            if (bmp_it != e.end() && bmp_it->is_array()) {
                for (const auto& bm : *bmp_it) {
                    const float barrel_in = bm.value("barrel_in", 0.0f);
                    const float mv_fps    = bm.value("mv_fps", 0.0f);
                    if (barrel_in > 0.0f && mv_fps > 0.0f)
                        p.barrel_mv_profile.emplace_back(barrel_in, mv_fps);
                }
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
        if (!j.is_array()) return false;
        std::vector<GunPreset> tmp;
        for (const auto& e : j) {
            GunPreset p;
            p.name = e.value("name", std::string(""));
            p.caliber_inches = e.value("caliber_inches", 0.308f);
            p.cartridge_keys.clear();
            const auto ck_it = e.find("cartridge_keys");
            if (ck_it != e.end() && ck_it->is_array()) {
                for (const auto& k : *ck_it) {
                    if (k.is_string()) p.cartridge_keys.push_back(k.get<std::string>());
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

const DragModel kDragModels[8] = {DragModel::G1, DragModel::G2, DragModel::G3, DragModel::G4,
                                  DragModel::G5, DragModel::G6, DragModel::G7, DragModel::G8};

const char* kDragModelLabels[8] = {"G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8"};

// Tag label lists for cartridge editor UI
static const char* kBulletTierLabels[] = {"match", "mil_spec", "commercial", "budget"};
static const int kNumBulletTiers = sizeof(kBulletTierLabels) / sizeof(kBulletTierLabels[0]);

static const char* kBulletConstructionLabels[] = {"hpbt", "polymer_tip", "solid", "monolithic", "fmj", "plated", "jhp", "soft_tip", "wc", "cast"};
static const int kNumBulletConstructions = sizeof(kBulletConstructionLabels) / sizeof(kBulletConstructionLabels[0]);

static const char* kBulletShapeLabels[] = {"standard", "flat_point"};
static const int kNumBulletShapes = sizeof(kBulletShapeLabels) / sizeof(kBulletShapeLabels[0]);

// Load bullet tolerances JSON file (returns true if loaded)
bool LoadBulletTolerancesFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.good()) return false;
    try {
        json j;
        ifs >> j;
        if (!j.is_object()) return false;
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

int FindGunPresetByName(const std::string& name) {
    for (size_t i = 0; i < g_gun_presets.size(); ++i) {
        if (g_gun_presets[i].name == name) {
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
    if (a.empty() || b.empty()) return false;
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
            if (ka == kb) return true;
            if (is_9x19_family(ka) && is_9x19_family(kb)) return true;
        }
    }
    return false;
}

bool IsGunPresetCompatibleWithSelectedCaliber(const GunPreset& preset) {
    const float selected_caliber = std::fabs(GetSelectedCartridgeCaliberInches());
    const float preset_caliber = std::fabs(preset.caliber_inches);
    const CartridgePreset* selected = GetSelectedCartridgePreset();
    const bool keys_match = selected && CartridgeKeysIntersect(selected->cartridge_keys, preset.cartridge_keys);
    if (keys_match) return true;
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
    const std::string cartridge_file_repo = rel_dir + "dope_gui_cartridges.json";
    const std::string gun_file_repo = rel_dir + "dope_gui_guns.json";
    const std::string hw_file_repo = rel_dir + "dope_gui_hardware.json";
    bool loaded_cartridges = LoadCartridgePresetsFromFile(cartridge_file_repo) || LoadCartridgePresetsFromFile("dope_gui_cartridges.json");
    bool loaded_guns = LoadGunPresetsFromFile(gun_file_repo) || LoadGunPresetsFromFile("dope_gui_guns.json");
    (void)LoadHardwarePresetsFromFile(hw_file_repo);
    (void)LoadHardwarePresetsFromFile("dope_gui_hardware.json");

    // If neither embedded tables nor per-cartridge sigma values exist, legacy file fallback remains optional.
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
        fallback.length_mm = 0.0f;
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
    if (count <= 0) return 0;
    if (value >= 0 && value < count) return value;
    if (fallback >= 0 && fallback < count) return fallback;
    return count - 1;
}

void SanitizeKeyList(std::vector<std::string>& keys) {
    auto trim_ascii = [](std::string& s) {
        while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
        while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    };
    for (auto& k : keys) {
        trim_ascii(k);
        std::transform(k.begin(), k.end(), k.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    }
    keys.erase(std::remove_if(keys.begin(), keys.end(), [](const std::string& s) { return s.empty(); }), keys.end());
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
    g_state.hw_barometer_index =
        ClampPresetIndex(g.state.hw_barometer_index, GetNumBarometerPresets(), kBarometerDefaultIndex);
    g_state.hw_lrf_index = ClampPresetIndex(g.state.hw_lrf_index, GetNumLRFPresets(), kLrfDefaultIndex);
    g_state.hw_temp_sensor_index =
        ClampPresetIndex(g.state.hw_temp_sensor_index, GetNumTempSensorPresets(), kTempDefaultIndex);
    g_state.hw_gps_index =
        ClampPresetIndex(g.state.hw_gps_index, GetNumGpsLatPresets(), kGpsDefaultIndex);
    g_state.hw_mag_lat_index =
        ClampPresetIndex(g.state.hw_mag_lat_index, GetNumMagLatPresets(), kMagLatDefaultIndex);

    UncertaintyConfig uc_defaults = {};
    DOPE_GetDefaultUncertaintyConfig(&uc_defaults);

    const DOPE_ErrorTable* lrf_table = GetLRFTable(g.state.hw_lrf_index);
    const bool has_lrf_table =
        (lrf_table && lrf_table->points != nullptr && lrf_table->count > 0);
    g.state.uc_config.use_range_error_table = has_lrf_table;
    g.state.uc_config.range_error_table = has_lrf_table ? *lrf_table : DOPE_ErrorTable{nullptr, 0};
    if (has_lrf_table && std::isfinite(g.state.lrf_range)) {
        g.state.uc_config.sigma_range_m = interpolate_error_sigma(lrf_table, g.state.lrf_range);
    }

    const DOPE_ErrorTable* temp_table = GetTempTable(g.state.hw_temp_sensor_index);
    const bool has_temp_table =
        (temp_table && temp_table->points != nullptr && temp_table->count > 0);
    g.state.uc_config.use_temperature_error_table = has_temp_table;
    g.state.uc_config.temperature_error_table =
        has_temp_table ? *temp_table : DOPE_ErrorTable{nullptr, 0};
    if (has_temp_table && std::isfinite(g.state.baro_temp)) {
        g.state.uc_config.sigma_temperature_c =
            interpolate_error_sigma(temp_table, g.state.baro_temp);
    }

    const DOPE_ErrorTable* pressure_table = GetBarometerTable(g.state.hw_barometer_index);
    const bool has_pressure_table =
        (pressure_table && pressure_table->points != nullptr && pressure_table->count > 0);
    g.state.uc_config.use_pressure_delta_temp_error_table = has_pressure_table;
    g.state.uc_config.pressure_delta_temp_error_table =
        has_pressure_table ? *pressure_table : DOPE_ErrorTable{nullptr, 0};

    if (!std::isfinite(g.state.uc_config.pressure_uncalibrated_sigma_pa) ||
        g.state.uc_config.pressure_uncalibrated_sigma_pa < 0.0f) {
        g.state.uc_config.pressure_uncalibrated_sigma_pa = uc_defaults.pressure_uncalibrated_sigma_pa;
    }

    g.state.uc_config.pressure_is_calibrated =
        g.state.baro_is_calibrated && g.state.baro_has_calibration_temp &&
        std::isfinite(g.state.baro_calibration_temp_c);
    g.state.uc_config.pressure_has_calibration_temp =
        g.state.baro_has_calibration_temp && std::isfinite(g.state.baro_calibration_temp_c);
    g.state.uc_config.pressure_calibration_temp_c = g.state.baro_calibration_temp_c;

    if (has_pressure_table) {
        if (g.state.uc_config.pressure_is_calibrated && std::isfinite(g.state.baro_temp)) {
            const float delta_temp_c =
                std::fabs(g.state.baro_temp - g.state.baro_calibration_temp_c);
            g.state.uc_config.sigma_pressure_pa =
                interpolate_error_sigma(pressure_table, delta_temp_c);
        } else {
            g.state.uc_config.sigma_pressure_pa = g.state.uc_config.pressure_uncalibrated_sigma_pa;
        }
    }

    if (g.state.hw_cant_index >= 0 && g.state.hw_cant_index < GetNumCantPresets()) {
        g.state.uc_config.sigma_cant_deg = GetCantSigma(g.state.hw_cant_index);
    }

    const int gps_mapped = GetMappedGpsLatPresetIndex(g.state.hw_gps_index);
    const int mag_mapped = GetMappedMagLatPresetIndex(g.state.hw_mag_lat_index);
    if (gps_mapped > 0 && gps_mapped < kNumGpsLatPresets) {
        g.state.uc_config.sigma_latitude_deg = kGpsLatPresets[gps_mapped].sigma_lat_deg;
    } else if (mag_mapped > 0 && mag_mapped < kNumMagLatPresets) {
        g.state.uc_config.sigma_latitude_deg = kMagLatPresets[mag_mapped].sigma_lat_deg;
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
        if (!std::isfinite(p.first)) p.first = 0.0f;
        if (!std::isfinite(p.second)) p.second = preset.muzzle_velocity_ms;
        p.second = ClampValue(p.second, 5.0f, 1500.0f);
    }
    // sanitize trajectory profile points
    for (auto& p : preset.trajectory_profile) {
        if (!std::isfinite(p.first)) p.first = 0.0f;
        if (!std::isfinite(p.second)) p.second = 0.0f;
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
        bmp.erase(std::remove_if(bmp.begin(), bmp.end(), [](const std::pair<float,float>& p) {
                      return p.first <= 0.0f || p.second <= 0.0f ||
                             !std::isfinite(p.first) || !std::isfinite(p.second);
                  }), bmp.end());
        std::sort(bmp.begin(), bmp.end(), [](const auto& a, const auto& b){ return a.first < b.first; });
        bmp.erase(std::unique(bmp.begin(), bmp.end(),
                              [](const auto& a, const auto& b){ return a.first == b.first; }), bmp.end());
    }
    preset.mass_grains = ClampValue(preset.mass_grains, 20.0f, 1200.0f);
    preset.caliber_inches = ClampValue(std::fabs(preset.caliber_inches), 0.10f, 1.00f);
    preset.length_mm = ClampValue(preset.length_mm, 5.0f, 100.0f);
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
    item["muzzle_velocity_ms"] = preset.muzzle_velocity_ms;
    item["reference_barrel_inches"] = preset.reference_barrel_inches;
    item["muzzle_diameter_in"] = preset.muzzle_diameter_in;
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
    item["length_mm"] = preset.length_mm;
    if (!preset.tags.empty()) {
        item["tags"] = json::array();
        for (const auto& t : preset.tags) item["tags"].push_back(t);
    }
    if (!preset.cartridge_keys.empty()) {
        item["cartridge_keys"] = json::array();
        for (const auto& k : preset.cartridge_keys) item["cartridge_keys"].push_back(k);
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
        for (const auto& k : preset.cartridge_keys) item["cartridge_keys"].push_back(k);
    }
    return item;
}

float ComputeAutoDragCoefficient(const BulletProfile& bullet) {
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
    g_state.bullet.bc = 0.505f;
    g_state.manual_drag_coefficient = g_state.bullet.bc;
    g_state.override_drag_coefficient = false;
    g_state.bullet.drag_model = DragModel::G1;
    g_state.bullet.muzzle_velocity_ms = 792.0f;
    g_state.bullet.mass_grains = 175.0f;
    g_state.bullet.length_mm = 31.2f;
    g_state.bullet.caliber_inches = 0.308f;
    g_state.bullet.twist_rate_inches = 10.0f;
    g_state.bullet.barrel_length_in = 24.0f;
    g_state.bullet.muzzle_diameter_in = 0.7f;
    g_state.bullet.barrel_material = kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material;
    g_state.bullet.free_floated = false;
    g_state.bullet.suppressor_attached = false;
    g_state.bullet.barrel_tuner_attached = false;
    g_state.barrel_finish_index = kBarrelFinishDefaultIndex;
    g_state.barrel_material_index = kDefaultBarrelMaterialIndex;
    g_state.bullet.cold_bore_velocity_bias = kBarrelFinishPresets[kBarrelFinishDefaultIndex].sigma_mv_fps;
    g_state.bullet.angular_sigma_moa = 0.4f;
    g_state.bullet.measured_cep50_moa = 0.0f;
    g_state.bullet.manufacturer_spec_moa = 0.0f;
    g_state.bullet.category_radial_moa = 0.0f;
    g_state.bullet.category_vertical_moa = 0.0f;
    g_state.bullet.stiffness_moa = 0.75f;
    g_reference_mv_ms = g_state.bullet.muzzle_velocity_ms;
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
    g_state.baro_temp = 18.333f; // 65 Γö¼ΓûæF
    g_state.baro_humidity = 0.65f;
    g.state.lrf_range = 457.2f; // 500 yd

    g.state.imu_valid = true;
    g.state.mag_valid = true;
    g.state.baro_valid = true;
    g.state.baro_humidity_valid = true;
    g.state.lrf_valid = true;

    g.state.drag_model_index = 0;
    g.state.now_us = 0;
    g.state.last_action = "Startup";
    g.state.hw_barometer_index = kBarometerDefaultIndex;
    g.state.hw_lrf_index = kLrfDefaultIndex;
    g.state.hw_temp_sensor_index = kTempDefaultIndex;
    g.state.hw_gps_index = kGpsDefaultIndex;
    g.state.hw_mag_lat_index = kMagLatDefaultIndex;
    g.state.baro_is_calibrated = false;
    g.state.baro_has_calibration_temp = false;
    g.state.baro_calibration_temp_c = 0.0f;
    g.state.mass_error_preset_index = kMassErrorDefaultIndex;
    g.state.length_error_preset_index = kLengthErrorDefaultIndex;
    g.state.caliber_error_preset_index = kCaliberErrorDefaultIndex;
    DOPE_GetDefaultUncertaintyConfig(&g.state.uc_config);
    SyncDerivedInputUncertaintySigmas();
    std::snprintf(g.state.preset_path, sizeof(g.state.preset_path), "%s", "dope_gui_preset.json");
    std::snprintf(g.state.profile_library_path, sizeof(g.state.profile_library_path), "%s",
                  "dope_gui_profile_library.json");
    std::snprintf(g.state.new_cartridge_preset_name, sizeof(g.state.new_cartridge_preset_name),
                  "%s", "New Cartridge Preset");
    std::snprintf(g.state.new_gun_preset_name, sizeof(g.state.new_gun_preset_name), "%s",
                  "New Gun Preset");
    g.state.selected_cartridge_preset = -1;
    g.state.selected_gun_preset = -1;

    g_cartridge_presets.clear();
    g_gun_presets.clear();
    EnsureProfileDefaults();
    SelectDefaultGunPresetForSelectedCartridge();
}

void ApplyConfig() {
    // Push current GUI inputs into the DOPE engine.
    g_state.bullet.drag_model = kDragModels[g.state.drag_model_index];
    g_state.bullet.bc = g.state.override_drag_coefficient
                            ? ClampValue(g.state.manual_drag_coefficient, 0.001f, 1.20f)
                            : ComputeAutoDragCoefficient(g.state.bullet);
    g.state.bullet.stiffness_moa = ComputeStiffnessMoa(g.state.bullet.muzzle_diameter_in,
                                                      std::fabs(g.state.bullet.caliber_inches),
                                                      g.state.bullet.barrel_material);
    g.state.bullet.free_floated = g.state.free_floated;
    g.state.bullet.suppressor_attached = g.state.suppressor_attached;
    g.state.bullet.barrel_tuner_attached = g.state.barrel_tuner_attached;
    DOPE_SetBulletProfile(&g.state.bullet);
    DOPE_SetZeroConfig(&g.state.zero);
    DOPE_SetWindManual(g.state.wind_speed_ms, g.state.wind_heading);
    DOPE_SetLatitude(g.state.latitude);
    SyncDerivedInputUncertaintySigmas();
    DOPE_SetUncertaintyConfig(&g.state.uc_config);
    DOPE_SetDeferUncertainty(true);
}

SensorFrame BuildFrame() {
    // Build a synthetic sensor frame for the desktop harness.
    // Inputs mirror fields from the "Sensor Frame" panel.
    SensorFrame frame = {};
    g_state.now_us += FRAME_STEP_US;

    frame.timestamp_us = g.state.now_us;

    frame.accel_x = 0.0f;
    frame.accel_y = 0.0f;
    frame.accel_z = 9.81f;
    frame.gyro_x = 0.0f;
    frame.gyro_y = 0.0f;
    frame.gyro_z = 0.0f;
    frame.imu_valid = g.state.imu_valid;

    frame.mag_x = 25.0f;
    frame.mag_y = 0.0f;
    frame.mag_z = 40.0f;
    frame.mag_valid = g.state.mag_valid;

    frame.baro_pressure_pa = g.state.baro_pressure;
    frame.baro_temperature_c = g.state.baro_temp;
    frame.baro_humidity = g.state.baro_humidity;
    frame.baro_valid = g.state.baro_valid;
    frame.baro_humidity_valid = g.state.baro_humidity_valid;

    frame.lrf_range_m = g.state.lrf_range;
    frame.lrf_timestamp_us = g.state.now_us;
    frame.lrf_valid = g.state.lrf_valid;
    frame.target_elevation_m = g.state.target_elevation_m;
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

    ss << "Action: " << g.state.last_action << "\n";
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

    if (g.state.unit_system == UnitSystem::IMPERIAL) {
        ss << "Drag Coefficient (G-model): " << g.state.bullet.bc;
        ss << (g.state.override_drag_coefficient ? " [manual]" : " [auto]") << "\n";
        ss << "Muzzle Velocity (fps): " << (g.state.bullet.muzzle_velocity_ms * MPS_TO_FPS) << "\n";
        ss << "Wind Speed (mph): " << (g.state.wind_speed_ms * MPS_TO_MPH) << "\n";
        ss << "Range (yd): " << range_yd << "\n";
        ss << "Horiz Range (yd): " << horiz_range_yd << "\n";
    } else {
        ss << "Drag Coefficient (G-model): " << g.state.bullet.bc;
        ss << (g.state.override_drag_coefficient ? " [manual]" : " [auto]") << "\n";
        ss << "Muzzle Velocity (m/s): " << g.state.bullet.muzzle_velocity_ms << "\n";
        ss << "Wind Speed (m/s): " << g.state.wind_speed_ms << "\n";
        ss << "Range (m): " << sol.range_m << "\n";
        ss << "Horiz Range (m): " << sol.horizontal_range_m << "\n";
    }
    ss << "Elevation Hold (MOA): " << sol.hold_elevation_moa << "\n";
    ss << "Windage Hold (MOA):   " << sol.hold_windage_moa << "\n";
    ss << "TOF (ms): " << sol.tof_ms << "\n";
    if (g.state.unit_system == UnitSystem::IMPERIAL) {
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
            "MV", "Range", "WindSpd", "WindDir", "Temp", "Press", "Humid", "SightH", "Cant", "Latitude", "Twist", "ZeroRng", "MVAdj"
        };
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
    } else if (g.state.uc_config.enabled) {
        ss << "\nUncertainty: enabled but not yet computed (need valid solution).\n";
    } else {
        ss << "\nUncertainty: disabled.\n";
    }

    g.state.output_text = ss.str();

    // Publish to display snapshot so the render loop can read it without
    // ever blocking on g_engine_mutex (which may be held during a long solve).
    {
        std::lock_guard<std::mutex> dlk(g.display_mutex);
        g.display_output = g.state.output_text;
        DOPE_GetSolution(&g.display_sol);
    }
}

// ---------------------------------------------------------------------------
// Barrel-length ΓåÆ MV helpers (glue layer only ΓÇö engine not modified).
// ---------------------------------------------------------------------------

// Linear interpolation over a sorted (barrel_in, mv_fps) table.
static float InterpolateBarrelMvProfile(const std::vector<std::pair<float,float>>& profile,
                                        float barrel_in) {
    if (profile.empty()) return 0.0f;
    if (barrel_in <= profile.front().first) return profile.front().second;
    if (barrel_in >= profile.back().first)  return profile.back().second;
    for (size_t i = 1; i < profile.size(); ++i) {
        if (barrel_in <= profile[i].first) {
            const float t = (barrel_in - profile[i-1].first) /
                            (profile[i].first - profile[i-1].first);
            return profile[i-1].second + t * (profile[i].second - profile[i-1].second);
        }
    }
    return profile.back().second;
}

// Tiered fps/in estimator ΓÇö fallback when cartridge has no explicit value.
static float FallbackMvAdjFpsPerIn(float ref_mv_ms) {
    const float fps = ref_mv_ms * MPS_TO_FPS;
    if (fps < 550.0f)  return 8.0f;    // pistol / subsonic
    if (fps < 760.0f)  return 12.0f;   // pistol / SMG
    if (fps < 915.0f)  return 20.0f;   // intermediate / carbine
    if (fps < 1067.0f) return 28.0f;   // rifle carbine
    return 38.0f;                       // full-power rifle
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
        base = 0.20f;                  // Bull / Heavy
    } else if (wall >= 0.20f) {
        base = 0.75f;                  // Medium / Standard
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
//   1. g_active_barrel_mv_profile non-empty  ΓåÆ profile table interpolation
//   2. g_active_mv_adj_fps_per_in > 0        ΓåÆ linear fps/in from reference barrel
//   3. fallback estimator                     ΓåÆ tier-based fps/in from ref MV
void ComputeAdjustedMv() {
    const float barrel_in = g.state.bullet.barrel_length_in;
    g_mv_adjustment_estimated = false;

    if (!g_active_barrel_mv_profile.empty()) {
        // Tier 1: profile lookup ΓÇö MV is fully determined by barrel length.
        const float mv_fps = InterpolateBarrelMvProfile(g_active_barrel_mv_profile, barrel_in);
        g.state.bullet.muzzle_velocity_ms        = mv_fps * FPS_TO_MPS;
        g.state.bullet.reference_barrel_length_in = barrel_in; // no engine delta
        g.state.bullet.mv_adjustment_factor       = 0.0f;
    } else {
        // Tier 2/3: linear adjustment from the cartridge's reference barrel.
        float fps_per_in = g_active_mv_adj_fps_per_in;
        if (fps_per_in <= 0.0f) {
            fps_per_in = FallbackMvAdjFpsPerIn(g_reference_mv_ms);
            g_mv_adjustment_estimated = true;
        }
        // Engine computes delta: actual_mv = muzzle_velocity_ms + (barrel - ref) * factor * FPS_TO_MPS
        g.state.bullet.muzzle_velocity_ms  = g_reference_mv_ms;
        g.state.bullet.mv_adjustment_factor = fps_per_in;
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

    g_state.bullet.mass_grains    = normalized.mass_grains;
    g_state.bullet.caliber_inches = normalized.caliber_inches;
    g_state.bullet.length_mm      = normalized.length_mm;
    g_state.bullet.muzzle_diameter_in = normalized.muzzle_diameter_in;
    // Removed barrel_finish_sigma_mv_fps logic (no longer used)
    g_state.bullet.measured_cep50_moa = normalized.measured_cep50_moa;
    g_state.bullet.manufacturer_spec_moa = normalized.manufacturer_spec_moa;
    g_state.bullet.category_radial_moa = normalized.category_radial_moa;
    g_state.bullet.category_vertical_moa = normalized.category_vertical_moa;
    g_state.bullet.stiffness_moa = ComputeStiffnessMoa(normalized.muzzle_diameter_in,
                                                      std::fabs(normalized.caliber_inches),
                                                      g.state.bullet.barrel_material);

    // Start with the reference barrel as both actual and reference length.
    if (normalized.reference_barrel_inches > 0.0f) {
        g_state.bullet.barrel_length_in           = normalized.reference_barrel_inches;
        g_state.bullet.reference_barrel_length_in = normalized.reference_barrel_inches;
    }

    // Removed obsolete cartridge CEP fields and logic
    g_state.override_drag_coefficient = true;
    g.state.manual_drag_coefficient   = normalized.bc;

    // Resolve MV for the reference barrel length.
    ComputeAdjustedMv();

    SelectDefaultGunPresetForSelectedCartridge();
    if (g.state.selected_gun_preset >= 0 &&
        g.state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
        IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[g.state.selected_gun_preset])) {
        ApplyGunPreset(g_gun_presets[g.state.selected_gun_preset]);
    }
}

void ApplyGunPreset(const GunPreset& preset) {
    GunPreset normalized = preset;
    SanitizeGunPreset(normalized);

    g_state.bullet.barrel_length_in = normalized.barrel_length_in;
    g_state.bullet.muzzle_diameter_in = normalized.muzzle_diameter_in;
    g.state.bullet.cold_bore_velocity_bias = normalized.cold_bore_velocity_bias;
    g.state.bullet.angular_sigma_moa = normalized.angular_sigma_moa;
    g.state.bullet.barrel_material = normalized.barrel_material;
    g.state.barrel_material_index = ResolveBarrelMaterialIndex(normalized.barrel_material);
    g.state.bullet.twist_rate_inches = normalized.twist_rate_inches;
    g.state.zero.zero_range_m = normalized.zero_range_m;
    g.state.zero.sight_height_mm = normalized.sight_height_mm;
    g.state.free_floated = normalized.free_floated;
    g.state.suppressor_attached = normalized.suppressor_attached;
    g.state.barrel_tuner_attached = normalized.barrel_tuner_attached;
    g.state.bullet.free_floated = normalized.free_floated;
    g.state.bullet.suppressor_attached = normalized.suppressor_attached;
    g.state.bullet.barrel_tuner_attached = normalized.barrel_tuner_attached;
    g.state.bullet.stiffness_moa = ComputeStiffnessMoa(normalized.muzzle_diameter_in,
                                                      std::fabs(normalized.caliber_inches),
                                                      normalized.barrel_material);
    ComputeAdjustedMv();
}

CartridgePreset CaptureCurrentCartridgePreset(const std::string& name) {
    CartridgePreset preset;
    preset.name = name;
    preset.bc = g.state.override_drag_coefficient
                    ? ClampValue(g.state.manual_drag_coefficient, 0.001f, 1.20f)
                    : ComputeAutoDragCoefficient(g.state.bullet);
    preset.drag_model_index = g.state.drag_model_index;
    // Use reference MV and barrel from glue-layer state (not the adjusted/current values).
    const float ref_mv = (g_reference_mv_ms > 0.0f) ? g_reference_mv_ms
                                                      : g.state.bullet.muzzle_velocity_ms;
    preset.muzzle_velocity_ms = ref_mv;
    preset.reference_barrel_inches = (g.state.bullet.reference_barrel_length_in > 0.0f)
                                         ? g.state.bullet.reference_barrel_length_in
                                         : g.state.bullet.barrel_length_in;
    preset.velocity_profile.clear();
    preset.velocity_profile.emplace_back(0.0f, ref_mv);
    preset.trajectory_profile.clear();
    preset.mass_grains = g.state.bullet.mass_grains;
    preset.caliber_inches = g.state.bullet.caliber_inches;
    preset.length_mm = g.state.bullet.length_mm;
    preset.muzzle_diameter_in = g.state.bullet.muzzle_diameter_in;
    // Removed barrel_finish_sigma_mv_fps (no longer used)
    preset.measured_cep50_moa = g.state.bullet.measured_cep50_moa;
    preset.manufacturer_spec_moa = g.state.bullet.manufacturer_spec_moa;
    preset.category_radial_moa = g.state.bullet.category_radial_moa;
    preset.category_vertical_moa = g.state.bullet.category_vertical_moa;
    preset.sd_mv_fps = g.state.new_cartridge_sd_mv_fps;
    // barrel-MV data from glue-layer globals
    preset.mv_adjustment_fps_per_in = g_active_mv_adj_fps_per_in;
    preset.barrel_mv_profile        = g_active_barrel_mv_profile;
    // attach tags from current UI selections where applicable
    preset.tags.clear();
    if (g.state.new_cartridge_tier_index >= 0 && g.state.new_cartridge_tier_index < kNumBulletTiers)
        preset.tags.push_back(kBulletTierLabels[g.state.new_cartridge_tier_index]);
    if (g.state.new_cartridge_construction_index >= 0 && g.state.new_cartridge_construction_index < kNumBulletConstructions)
        preset.tags.push_back(kBulletConstructionLabels[g.state.new_cartridge_construction_index]);
    if (g.state.new_cartridge_shape_index > 0 && g.state.new_cartridge_shape_index < kNumBulletShapes)
        preset.tags.push_back(kBulletShapeLabels[g.state.new_cartridge_shape_index]);

    if (g.state.selected_cartridge_preset >= 0 &&
        g.state.selected_cartridge_preset < static_cast<int>(g.cartridge_presets.size())) {
        preset.cartridge_keys = g.cartridge_presets[g.state.selected_cartridge_preset].cartridge_keys;
    }

    SanitizeCartridgePreset(preset);
    return preset;
}

GunPreset CaptureCurrentGunPreset(const std::string& name) {
    GunPreset preset;
    preset.name = name;
    preset.caliber_inches = g.state.bullet.caliber_inches;
    preset.barrel_length_in = g.state.bullet.barrel_length_in;
    preset.muzzle_diameter_in = g.state.bullet.muzzle_diameter_in;
    preset.cold_bore_velocity_bias = g.state.bullet.cold_bore_velocity_bias;
    preset.angular_sigma_moa = g.state.bullet.angular_sigma_moa;
    preset.barrel_material = g.state.bullet.barrel_material;
    preset.twist_rate_inches = g.state.bullet.twist_rate_inches;
    preset.zero_range_m = g.state.zero.zero_range_m;
    preset.sight_height_mm = g.state.zero.sight_height_mm;
    preset.free_floated = g.state.free_floated;
    preset.suppressor_attached = g.state.suppressor_attached;
    preset.barrel_tuner_attached = g.state.barrel_tuner_attached;
    if (g.state.selected_gun_preset >= 0 &&
        g.state.selected_gun_preset < static_cast<int>(g.gun_presets.size())) {
        preset.cartridge_keys = g.gun_presets[g.state.selected_gun_preset].cartridge_keys;
    } else if (const CartridgePreset* cart = GetSelectedCartridgePreset()) {
        preset.cartridge_keys = cart->cartridge_keys;
    }
    SanitizeGunPreset(preset);
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

    std::ofstream out(g.state.profile_library_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        g.state.last_action = "Save Profile Library (failed)";
        g.state.output_text = "Profile library save failed: cannot open file.";
        return;
    }
    out << root.dump(2);
    g.state.last_action = "Save Profile Library";
}

// Save cartridge presets (JSON array) to given path. Returns true on success.
bool SaveCartridgePresetsToFile(const std::string& path) {
    EnsureProfileDefaults();
    json root = json::object();

    if (!g_bullet_tolerances.is_null() && g_bullet_tolerances.contains("bullet_type_tolerances")) {
        root["bullet_type_tolerances"] = g_bullet_tolerances["bullet_type_tolerances"];
        if (g_bullet_tolerances.contains("__comments")) {
            root["bullet_tolerance_comments"] = g_bullet_tolerances["__comments"];
        } else if (g_bullet_tolerances.contains("_comments")) {
            root["bullet_tolerance_comments"] = g_bullet_tolerances["_comments"];
        }
    }

    root["cartridge_presets"] = json::array();
    for (const auto& p : g_cartridge_presets) {
        root["cartridge_presets"].push_back(SerializeCartridgePreset(p));
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << root.dump(2);
    return true;
}

// Save gun presets (JSON array) to given path. Returns true on success.
bool SaveGunPresetsToFile(const std::string& path) {
    EnsureProfileDefaults();
    json j = json::array();
    for (const auto& p : g_gun_presets) {
        j.push_back(SerializeGunPreset(p));
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << j.dump(2);
    return true;
}

void LoadProfileLibrary() {
    std::ifstream in(g.state.profile_library_path, std::ios::binary);
    if (!in) {
        g.state.last_action = "Load Profile Library (failed)";
        g.state.output_text = "Profile library load failed: cannot open file.";
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
                preset.bc = item.value("bc", 0.505f);
                preset.drag_model_index = item.value("drag_model_index", 0);
                if (preset.drag_model_index < 0 || preset.drag_model_index >= 8) {
                    preset.drag_model_index = 0;
                }
                if (item.contains("muzzle_velocity_fps") && item["muzzle_velocity_fps"].is_number()) {
                    preset.muzzle_velocity_ms = item["muzzle_velocity_fps"].get<float>() * FPS_TO_MPS;
                } else {
                    preset.muzzle_velocity_ms = item.value("muzzle_velocity_ms", 792.0f);
                }
                preset.mass_grains = item.value("mass_grains", 175.0f);
                preset.caliber_inches = item.value("caliber_inches", 0.308f);
                preset.length_mm = item.value("length_mm", 31.2f);
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
                        const float traj = tp.value("trajectory_inches", tp.value("trajectory_in", 0.0f));
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
                preset.barrel_material = mat_key.empty()
                                             ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
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
                preset.barrel_finish_sigma_mv_fps = item.value("barrel_finish_sigma_mv_fps", 8.5f);
                const std::string mat_key = item.value("barrel_material", std::string(""));
                preset.barrel_material = mat_key.empty()
                                             ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
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
            g.state.selected_cartridge_preset = 0;
        }
        if (!loaded_gun.empty()) {
            g_gun_presets = loaded_gun;
            g.state.selected_gun_preset = 0;
        }

        EnsureProfileDefaults();
        SelectDefaultGunPresetForSelectedCartridge();
        g.state.last_action = "Load Profile Library";
    } catch (const json::parse_error& e) {
        g.state.last_action = "Load Profile Library (failed)";
        std::string error_msg = "Profile library load failed: JSON parse error: ";
        error_msg += e.what();
        g.state.output_text = error_msg;
    }
}

void SavePreset() {
    // Persist full GUI state to JSON so users can iterate on profiles quickly.
    json j;
    j["unit_system"] = (g.state.unit_system == UnitSystem::IMPERIAL ? "imperial" : "metric");
    j["drag_coefficient_override"] = g.state.override_drag_coefficient;
    j["drag_coefficient"] = g.state.manual_drag_coefficient;
    j["bc"] = g.state.bullet.bc;
    j["drag_model_index"] = g.state.drag_model_index;
    j["muzzle_velocity_ms"] = g.state.bullet.muzzle_velocity_ms;
    j["mass_grains"] = g.state.bullet.mass_grains;
    j["length_mm"] = g.state.bullet.length_mm;
    j["caliber_inches"] = g.state.bullet.caliber_inches;
    j["twist_rate_inches"] = g.state.bullet.twist_rate_inches;
    j["barrel_length_in"] = g.state.bullet.barrel_length_in;
    j["muzzle_diameter_in"] = g.state.bullet.muzzle_diameter_in;
    j["barrel_material"] = BarrelMaterialKey(g.state.bullet.barrel_material);
    j["free_floated"] = g.state.free_floated;
    j["suppressor_attached"] = g.state.suppressor_attached;
    j["barrel_tuner_attached"] = g.state.barrel_tuner_attached;
    j["cold_bore_velocity_bias"] = g.state.bullet.cold_bore_velocity_bias;
    j["angular_sigma_moa"] = g.state.bullet.angular_sigma_moa;
    j["measured_cep50_moa"] = g.state.bullet.measured_cep50_moa;
    j["manufacturer_spec_moa"] = g.state.bullet.manufacturer_spec_moa;
    j["category_radial_moa"] = g.state.bullet.category_radial_moa;
    j["category_vertical_moa"] = g.state.bullet.category_vertical_moa;
    j["stiffness_moa"] = g.state.bullet.stiffness_moa;
    j["reference_mv_ms"] = g_reference_mv_ms;
    j["mv_adj_fps_per_in_active"] = g_active_mv_adj_fps_per_in;
    j["mv_adj_estimated"] = g_mv_adjustment_estimated;
    j["zero_range_m"] = g.state.zero.zero_range_m;
    j["sight_height_mm"] = g.state.zero.sight_height_mm;
    j["wind_speed_ms"] = g.state.wind_speed_ms;
    j["wind_heading_deg"] = g.state.wind_heading;
    j["latitude_deg"] = g.state.latitude;
    j["baro_pressure_pa"] = g.state.baro_pressure;
    j["baro_temperature_c"] = g.state.baro_temp;
    j["baro_humidity"] = g.state.baro_humidity;
    j["lrf_range_m"] = g.state.lrf_range;
    j["target_elevation_m"] = g.state.target_elevation_m;
    j["uc_enabled"] = g.state.uc_config.enabled;
    j["uc_sigma_mv_ms"] = g.state.uc_config.sigma_muzzle_velocity_ms;
    j["hw_barometer_index"] = g.state.hw_barometer_index;
    j["hw_lrf_index"] = g.state.hw_lrf_index;
    j["hw_cant_index"] = g.state.hw_cant_index;
    j["hw_temp_sensor_index"] = g.state.hw_temp_sensor_index;
    j["hw_gps_index"] = g.state.hw_gps_index;
    j["hw_mag_lat_index"] = g.state.hw_mag_lat_index;
    j["baro_calibrated"] = g.state.baro_is_calibrated;
    j["baro_calibration_temp_valid"] = g.state.baro_has_calibration_temp;
    j["baro_calibration_temp_c"] = g.state.baro_calibration_temp_c;
    j["uc_mass_error_preset_index"] = g.state.mass_error_preset_index;
    j["uc_length_error_preset_index"] = g.state.length_error_preset_index;
    j["uc_caliber_error_preset_index"] = g.state.caliber_error_preset_index;
    j["imu_valid"] = g.state.imu_valid;
    j["mag_valid"] = g.state.mag_valid;
    j["baro_valid"] = g.state.baro_valid;
    j["baro_humidity_valid"] = g.state.baro_humidity_valid;
    j["lrf_valid"] = g.state.lrf_valid;

    std::ofstream out(g.state.preset_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        g.state.last_action = "Save Preset (failed)";
        g.state.output_text = "Preset save failed: cannot open file.";
        return;
    }
    out << j.dump(2);
}

void LoadPreset() {
    // Backward-compatible preset loading:
    // supports both SI and legacy imperial keys where available.
    std::ifstream in(g.state.preset_path, std::ios::binary);
    if (!in) {
        g.state.last_action = "Load Preset (failed)";
        g.state.output_text = "Preset load failed: cannot open file.";
        return;
    }

    try {
        json j;
        in >> j;

        if (j.contains("unit_system") && j["unit_system"].is_string()) {
            if (j["unit_system"] == "metric") {
                g.state.unit_system = UnitSystem::METRIC;
            } else {
                g.state.unit_system = UnitSystem::IMPERIAL;
            }
        }

        g.state.override_drag_coefficient = j.value("drag_coefficient_override", false);
        g.state.manual_drag_coefficient = j.value("drag_coefficient", 0.0f);

        bool has_bc = false;
        if (j.contains("bc") && j["bc"].is_number()) {
            g.state.bullet.bc = j["bc"];
            has_bc = true;
            if (!j.contains("drag_coefficient_override")) {
                g.state.override_drag_coefficient = true;
                g.state.manual_drag_coefficient = g.state.bullet.bc;
            }
        }

        g.state.drag_model_index = j.value("drag_model_index", 0);
        if (g.state.drag_model_index < 0 || g.state.drag_model_index >= 8) {
            g.state.drag_model_index = 0;
        }

        // Accept legacy imperial keys first, then modern SI fields.
        if (j.contains("muzzle_velocity_fps") && j["muzzle_velocity_fps"].is_number()) {
            g.state.bullet.muzzle_velocity_ms = j["muzzle_velocity_fps"].get<float>() * FPS_TO_MPS;
        } else {
            g.state.bullet.muzzle_velocity_ms = j.value("muzzle_velocity_ms", 0.0f);
        }

        g.state.bullet.mass_grains = j.value("mass_grains", 0.0f);
        g.state.bullet.length_mm = j.value("length_mm", 0.0f);
        g.state.bullet.caliber_inches = j.value("caliber_inches", 0.0f);
        g.state.bullet.twist_rate_inches = j.value("twist_rate_inches", 0.0f);
        g.state.bullet.barrel_length_in = j.value("barrel_length_in", 24.0f);
        g.state.bullet.muzzle_diameter_in = j.value("muzzle_diameter_in", 0.7f);
        const std::string barrel_material_key = j.value("barrel_material", std::string(""));
        g.state.bullet.barrel_material = barrel_material_key.empty()
                             ? kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].material
                             : ParseBarrelMaterialKey(barrel_material_key);
        g.state.barrel_material_index = ResolveBarrelMaterialIndex(g.state.bullet.barrel_material);
        g.state.free_floated = j.value("free_floated", false);
        g.state.suppressor_attached = j.value("suppressor_attached", false);
        g.state.barrel_tuner_attached = j.value("barrel_tuner_attached", false);
        g.state.bullet.free_floated = g.state.free_floated;
        g.state.bullet.suppressor_attached = g.state.suppressor_attached;
        g.state.bullet.barrel_tuner_attached = g.state.barrel_tuner_attached;
        g.state.bullet.barrel_finish_sigma_mv_fps = j.value("barrel_finish_sigma_mv_fps", 8.5f);
        g.state.bullet.measured_cep50_moa = j.value("measured_cep50_moa", 0.0f);
        g.state.bullet.manufacturer_spec_moa = j.value("manufacturer_spec_moa", 0.0f);
        g.state.bullet.category_radial_moa = j.value("category_radial_moa", 0.0f);
        g.state.bullet.category_vertical_moa = j.value("category_vertical_moa", 0.0f);
        g.state.bullet.stiffness_moa = j.value("stiffness_moa",
                              ComputeStiffnessMoa(g.state.bullet.muzzle_diameter_in,
                          std::fabs(g.state.bullet.caliber_inches),
                          g.state.bullet.barrel_material));
        g.state.barrel_finish_index = ResolveBarrelFinishIndex(g.state.bullet.barrel_finish_sigma_mv_fps);
        g_reference_mv_ms = j.value("reference_mv_ms", g.state.bullet.muzzle_velocity_ms);
        g_active_mv_adj_fps_per_in = j.value("mv_adj_fps_per_in_active", 0.0f);
        g_mv_adjustment_estimated = j.value("mv_adj_estimated", false);
        g.state.zero.zero_range_m = j.value("zero_range_m", 0.0f);
        g.state.zero.sight_height_mm = j.value("sight_height_mm", 0.0f);

        if (j.contains("wind_speed_mph") && j["wind_speed_mph"].is_number()) {
            g.state.wind_speed_ms = j["wind_speed_mph"].get<float>() * MPH_TO_MPS;
        } else if (j.contains("wind_speed_fps") && j["wind_speed_fps"].is_number()) {
            g.state.wind_speed_ms = j["wind_speed_fps"].get<float>() * FPS_TO_MPS;
        } else {
            g.state.wind_speed_ms = j.value("wind_speed_ms", 0.0f);
        }

        g.state.wind_heading = j.value("wind_heading_deg", 0.0f);
        if (j.contains("latitude_deg") && j["latitude_deg"].is_number()) {
            g.state.latitude = j["latitude_deg"].get<float>();
            if (std::fabs(g.state.latitude) < 0.0001f) {
                g.state.latitude = 37.0f;
            }
        } else {
            g.state.latitude = 37.0f;
        }
        g.state.baro_pressure = j.value("baro_pressure_pa", 101325.0f);
        g.state.baro_temp = j.value("baro_temperature_c", 18.333f);
        g.state.baro_humidity = j.value("baro_humidity", 0.5f);
        g.state.lrf_range = j.value("lrf_range_m", 500.0f);
        g.state.target_elevation_m = j.value("target_elevation_m", 0.0f);

        // Uncertainty config ╬ô├ç├╢ load with per-field defaults matching DOPE defaults
        UncertaintyConfig uc_def = {};
        DOPE_GetDefaultUncertaintyConfig(&uc_def);
        g.state.uc_config.enabled = j.value("uc_enabled", uc_def.enabled);
        g.state.uc_config.sigma_muzzle_velocity_ms =
            j.value("uc_sigma_mv_ms", uc_def.sigma_muzzle_velocity_ms);
        // Removed obsolete sigma_bc field
        g.state.uc_config.sigma_range_m = j.value("uc_sigma_range_m", uc_def.sigma_range_m);
        g.state.uc_config.sigma_wind_speed_ms =
            j.value("uc_sigma_wind_speed_ms", uc_def.sigma_wind_speed_ms);
        g.state.uc_config.sigma_wind_heading_deg =
            j.value("uc_sigma_wind_heading_deg", uc_def.sigma_wind_heading_deg);
        g.state.uc_config.sigma_temperature_c =
            j.value("uc_sigma_temperature_c", uc_def.sigma_temperature_c);
        g.state.uc_config.sigma_pressure_pa =
            j.value("uc_sigma_pressure_pa", uc_def.sigma_pressure_pa);
        g.state.uc_config.sigma_humidity = j.value("uc_sigma_humidity", uc_def.sigma_humidity);
        g.state.uc_config.sigma_sight_height_mm =
            j.value("uc_sigma_sight_height_mm", uc_def.sigma_sight_height_mm);
        g.state.uc_config.sigma_cant_deg = j.value("uc_sigma_cant_deg", uc_def.sigma_cant_deg);
        g.state.uc_config.sigma_latitude_deg =
            j.value("uc_sigma_latitude_deg", uc_def.sigma_latitude_deg);
        // Removed obsolete sigma_mass_grains field
        // Removed obsolete sigma_length_mm field
        // Removed obsolete sigma_caliber_inches field
        g.state.uc_config.sigma_twist_rate_inches =
            j.value("uc_sigma_twist_inches", uc_def.sigma_twist_rate_inches);
        g.state.uc_config.sigma_zero_range_m =
            j.value("uc_sigma_zero_range_m", uc_def.sigma_zero_range_m);
        g.state.uc_config.sigma_mv_adjustment_fps_per_in =
            j.value("uc_sigma_mv_adj_fps_per_in", uc_def.sigma_mv_adjustment_fps_per_in);

        // Removed obsolete cartridge CEP fields and logic

        // Hardware sensor selections (backwards-compatible: 0 = Custom)
        g.state.hw_barometer_index = j.value("hw_barometer_index", kBarometerDefaultIndex);
        g.state.hw_lrf_index = j.value("hw_lrf_index", kLrfDefaultIndex);
        g.state.hw_cant_index = j.value("hw_cant_index", 0);
        g.state.hw_temp_sensor_index = j.value("hw_temp_sensor_index", kTempDefaultIndex);
        g.state.hw_gps_index = j.value("hw_gps_index", kGpsDefaultIndex);
        g.state.hw_mag_lat_index = j.value("hw_mag_lat_index", kMagLatDefaultIndex);
        if (g.state.hw_barometer_index < 0 || g.state.hw_barometer_index >= GetNumBarometerPresets())
            g.state.hw_barometer_index = kBarometerDefaultIndex;
        if (g.state.hw_lrf_index < 0 || g.state.hw_lrf_index >= GetNumLRFPresets())
            g.state.hw_lrf_index = kLrfDefaultIndex;
        if (g.state.hw_cant_index < 0 || g.state.hw_cant_index >= GetNumCantPresets())
            g.state.hw_cant_index = kCantDefaultIndex;
        if (g.state.hw_temp_sensor_index < 0 || g.state.hw_temp_sensor_index >= GetNumTempSensorPresets())
            g.state.hw_temp_sensor_index = kTempDefaultIndex;
        if (g.state.hw_gps_index < 0 || g.state.hw_gps_index >= GetNumGpsLatPresets())
            g.state.hw_gps_index = kGpsDefaultIndex;
        if (g.state.hw_mag_lat_index < 0 || g.state.hw_mag_lat_index >= GetNumMagLatPresets())
            g.state.hw_mag_lat_index = kMagLatDefaultIndex;

        g.state.baro_is_calibrated = j.value("baro_calibrated", false);
        g.state.baro_has_calibration_temp = j.value("baro_calibration_temp_valid", false);
        g.state.baro_calibration_temp_c = j.value("baro_calibration_temp_c", 0.0f);

        g.state.mass_error_preset_index =
            ClampPresetIndex(j.value("uc_mass_error_preset_index", kMassErrorDefaultIndex),
                             kNumMassErrorPresets, kMassErrorDefaultIndex);
        g.state.length_error_preset_index =
            ClampPresetIndex(j.value("uc_length_error_preset_index", kLengthErrorDefaultIndex),
                             kNumLengthErrorPresets, kLengthErrorDefaultIndex);
        g.state.caliber_error_preset_index =
            ClampPresetIndex(j.value("uc_caliber_error_preset_index", kCaliberErrorDefaultIndex),
                             kNumCaliberErrorPresets, kCaliberErrorDefaultIndex);

        g.state.imu_valid = j.value("imu_valid", true);
        g.state.mag_valid = j.value("mag_valid", true);
        g.state.baro_valid = j.value("baro_valid", true);
        g.state.baro_humidity_valid = j.value("baro_humidity_valid", true);
        g.state.lrf_valid = j.value("lrf_valid", true);

        if (!has_bc && !g.state.override_drag_coefficient) {
            g.state.manual_drag_coefficient = ComputeAutoDragCoefficient(g.state.bullet);
        }

        SyncDerivedInputUncertaintySigmas();
    } catch (const json::parse_error& e) {
        g.state.last_action = "Load Preset (failed)";
        std::string error_msg = "Preset load failed: JSON parse error: ";
        error_msg += e.what();
        g.state.output_text = error_msg;
    }
}

void ResetEngineAndState() {
    DOPE_Init();
    DOPE_SetDeferUncertainty(true);
    g.state.now_us = 0;
    g.state.last_action = "Reset Engine";
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
// Engine ticker ╬ô├ç├╢ mirrors the FreeRTOS DOPE task on the ESP32-P4.
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
    // Desktop harness startup sequence: defaults -> window/device -> ImGui -> DOPE init.
    ResetStateDefaults();

    WNDCLASSEX wc = {sizeof(WNDCLASSEX),       CS_CLASSDC, WndProc, 0L,      0L,
                     GetModuleHandle(nullptr), nullptr,    nullptr, nullptr, nullptr,
                     _T("DOPE_ImGui_Test"),     nullptr};
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

    // Start engine ticker thread ╬ô├ç├╢ mirrors the FreeRTOS DOPE task on ESP32-P4.
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
            frame_output = g.display_output;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("DOPE Inputs");
        int unit_mode = static_cast<int>(g.state.unit_system);
        if (ImGui::Combo("Unit System", &unit_mode, kUnitSystemLabels,
                         IM_ARRAYSIZE(kUnitSystemLabels))) {
            g.state.unit_system = static_cast<UnitSystem>(unit_mode);
            g.state.last_action = "Unit Mode Changed";
            // ticker will refresh output at next tick
        }

        const bool is_imperial = (g.state.unit_system == UnitSystem::IMPERIAL);
        const bool uc_on = g.state.uc_config.enabled;

        if (ImGui::CollapsingHeader("Cartridge Presets (GUI-only)")) {
            ImGui::PushID("cartridge_presets");
            ImGui::TextUnformatted(
                "Presets are for test-harness convenience and do not alter engine internals.");

            ImGui::InputText("Cartridge Preset Name", g.state.new_cartridge_preset_name,
                             IM_ARRAYSIZE(g.state.new_cartridge_preset_name));
            // Tag selection: Tier, Construction, optional Shape
            ImGui::TextUnformatted("Preset Tags:");
            bool tags_changed = false;
            int tier_idx = g.state.new_cartridge_tier_index;
            if (ImGui::Combo("Tier", &tier_idx, kBulletTierLabels, kNumBulletTiers)) {
                g.state.new_cartridge_tier_index = tier_idx;
                tags_changed = true;
            }
            int cons_idx = g.state.new_cartridge_construction_index;
            if (ImGui::Combo("Construction", &cons_idx, kBulletConstructionLabels, kNumBulletConstructions)) {
                g.state.new_cartridge_construction_index = cons_idx;
                tags_changed = true;
            }
            int shape_idx = g.state.new_cartridge_shape_index;
            if (ImGui::Combo("Shape", &shape_idx, kBulletShapeLabels, kNumBulletShapes)) {
                g.state.new_cartridge_shape_index = shape_idx;
                tags_changed = true;
            }
            if (tags_changed) {
                CartridgePreset tmp;
                tmp.tags.clear();
                if (g.state.new_cartridge_tier_index >= 0 && g.state.new_cartridge_tier_index < kNumBulletTiers)
                    tmp.tags.push_back(kBulletTierLabels[g.state.new_cartridge_tier_index]);
                if (g.state.new_cartridge_construction_index >= 0 && g.state.new_cartridge_construction_index < kNumBulletConstructions)
                    tmp.tags.push_back(kBulletConstructionLabels[g.state.new_cartridge_construction_index]);
                if (g.state.new_cartridge_shape_index > 0 && g.state.new_cartridge_shape_index < kNumBulletShapes)
                    tmp.tags.push_back(kBulletShapeLabels[g.state.new_cartridge_shape_index]);
                const int existing = FindCartridgePresetByName(tmp.name);
                if (existing >= 0) {
                    g.cartridge_presets[existing] = tmp;
                    g.state.selected_cartridge_preset = existing;
                } else {
                    g.cartridge_presets.push_back(tmp);
                    g.state.selected_cartridge_preset =
                        static_cast<int>(g.cartridge_presets.size()) - 1;
                }
                SelectDefaultGunPresetForSelectedCartridge();
                g.state.last_action = "Cartridge Preset Saved";
                // ticker will refresh output at next tick
            }
            if (ImGui::Button("Add/Update Cartridge Preset")) {
                const std::string preset_name = g.state.new_cartridge_preset_name;
                if (!preset_name.empty()) {
                    CartridgePreset preset = CaptureCurrentCartridgePreset(preset_name);
                    // attach tags from UI selection
                    preset.tags.clear();
                    if (g.state.new_cartridge_tier_index >= 0 && g.state.new_cartridge_tier_index < kNumBulletTiers)
                        preset.tags.push_back(kBulletTierLabels[g.state.new_cartridge_tier_index]);
                    if (g.state.new_cartridge_construction_index >= 0 && g.state.new_cartridge_construction_index < kNumBulletConstructions)
                        preset.tags.push_back(kBulletConstructionLabels[g.state.new_cartridge_construction_index]);
                    if (g.state.new_cartridge_shape_index > 0 && g.state.new_cartridge_shape_index < kNumBulletShapes)
                        preset.tags.push_back(kBulletShapeLabels[g.state.new_cartridge_shape_index]);
                    const int existing = FindCartridgePresetByName(preset_name);
                    if (existing >= 0) {
                        g.cartridge_presets[existing] = preset;
                        g.state.selected_cartridge_preset = existing;
                    } else {
                        g.cartridge_presets.push_back(preset);
                        g.state.selected_cartridge_preset =
                            static_cast<int>(g.cartridge_presets.size()) - 1;
                    }
                    SelectDefaultGunPresetForSelectedCartridge();
                    g.state.last_action = "Cartridge Preset Saved";
                    // ticker will refresh output at next tick
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Cartridge") &&
                g.state.selected_cartridge_preset >= 0 &&
                g.state.selected_cartridge_preset < static_cast<int>(g.cartridge_presets.size())) {
                ApplyCartridgePreset(g.cartridge_presets[g.state.selected_cartridge_preset]);
                g.state.last_action = "Cartridge Preset Applied";
                {
                    std::lock_guard<std::mutex> lk(g.engine_mutex);
                    ApplyConfig();
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Selected Cartridge") &&
                g.state.selected_cartridge_preset >= 0 &&
                g.state.selected_cartridge_preset < static_cast<int>(g.cartridge_presets.size())) {
                g.cartridge_presets.erase(g.cartridge_presets.begin() +
                                          g.state.selected_cartridge_preset);
                if (g.cartridge_presets.empty()) {
                    g.state.selected_cartridge_preset = -1;
                } else if (g.state.selected_cartridge_preset >=
                           static_cast<int>(g.cartridge_presets.size())) {
                    g.state.selected_cartridge_preset =
                        static_cast<int>(g.cartridge_presets.size()) - 1;
                }
                SelectDefaultGunPresetForSelectedCartridge();
                g.state.last_action = "Cartridge Preset Removed";
                // ticker will refresh output at next tick
            }

            ImGui::SameLine();
            if (ImGui::Button("Save Presets to File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_cartridges.json";
                const bool ok = SaveCartridgePresetsToFile(repo_path) || SaveCartridgePresetsToFile("dope_gui_cartridges.json");
                g.state.last_action = ok ? "Saved Cartridge Presets" : "Save Cartridge Presets (failed)";
                if (!ok) g.state.output_text = "Failed to save cartridge presets.";
            }
            ImGui::SameLine();
            if (ImGui::Button("Load Presets from File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_cartridges.json";
                const bool ok = LoadCartridgePresetsFromFile(repo_path) || LoadCartridgePresetsFromFile("dope_gui_cartridges.json");
                g.state.last_action = ok ? "Loaded Cartridge Presets" : "Load Cartridge Presets (failed)";
                if (ok) {
                    EnsureProfileDefaults();
                    g.state.selected_cartridge_preset = (g.cartridge_presets.empty() ? -1 : 0);
                    SelectDefaultGunPresetForSelectedCartridge();
                } else {
                    g.state.output_text = "Failed to load cartridge presets.";
                }
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Drag");
            ImGui::Checkbox("Override Drag Coefficient", &g.state.override_drag_coefficient);
            {
                const float avail_bc = ImGui::GetContentRegionAvail().x;
                if (g.state.override_drag_coefficient) {
                    ImGui::SetNextItemWidth(uc_on ? avail_bc * 0.45f : -1.0f);
                    ImGui::InputFloat("Drag Coefficient (G-model)", &g.state.manual_drag_coefficient,
                                      0.001f, 0.01f, "%.4f");
                } else {
                    const float auto_drag = ComputeAutoDragCoefficient(g.state.bullet);
                    ImGui::Text("Drag Coefficient (auto): %.4f", auto_drag);
                }
                if (uc_on) {
                    // Removed obsolete BC sigma UI
                }
            }
            ImGui::Combo("Drag Model", &g.state.drag_model_index, kDragModelLabels,
                         IM_ARRAYSIZE(kDragModelLabels));
            ImGui::Separator();

            if (ImGui::BeginListBox("##cartridge_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g.cartridge_presets.size()); ++i) {
                    const bool is_selected = (g.state.selected_cartridge_preset == i);
                    if (ImGui::Selectable(g.cartridge_presets[i].name.c_str(), is_selected)) {
                        g.state.selected_cartridge_preset = i;
                        SelectDefaultGunPresetForSelectedCartridge();
                        // populate tag UI selections from selected preset
                        const CartridgePreset& cp = g.cartridge_presets[i];
                        // reset to defaults
                        g.state.new_cartridge_tier_index = 0;
                        g.state.new_cartridge_construction_index = 0;
                        g.state.new_cartridge_shape_index = 0;
                        for (const auto& t : cp.tags) {
                            for (int ti = 0; ti < kNumBulletTiers; ++ti) {
                                if (t == kBulletTierLabels[ti]) g.state.new_cartridge_tier_index = ti;
                            }
                            for (int ci = 0; ci < kNumBulletConstructions; ++ci) {
                                if (t == kBulletConstructionLabels[ci]) g.state.new_cartridge_construction_index = ci;
                            }
                            for (int si = 1; si < kNumBulletShapes; ++si) {
                                if (t == kBulletShapeLabels[si]) g.state.new_cartridge_shape_index = si;
                            }
                        }
                        g.state.new_cartridge_sd_mv_fps = cp.sd_mv_fps;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
            // Editable barrel-MV fields for the selected cartridge preset.
                        // Editable sd_mv_fps field for the selected cartridge preset.
                        if (g.state.selected_cartridge_preset >= 0 &&
                            g.state.selected_cartridge_preset < static_cast<int>(g.cartridge_presets.size())) {
                            CartridgePreset& selected_cp =
                                g.cartridge_presets[g.state.selected_cartridge_preset];
                            ImGui::Separator();
                            ImGui::TextDisabled("Cartridge MV SD (fps):");
                            ImGui::SetNextItemWidth(120.0f);
                            ImGui::InputFloat("SD MV (fps)", &selected_cp.sd_mv_fps, 0.1f, 1.0f, "%.2f");
                        }
            if (g.state.selected_cartridge_preset >= 0 &&
                g.state.selected_cartridge_preset < static_cast<int>(g.cartridge_presets.size())) {
                CartridgePreset& selected_cp =
                    g.cartridge_presets[g.state.selected_cartridge_preset];
                ImGui::Separator();
                ImGui::TextDisabled("Cartridge Barrel-MV: %s", selected_cp.name.c_str());
                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::InputFloat("Ref Barrel (in)##cp_ref_barrel",
                                      &selected_cp.reference_barrel_inches, 0.5f, 1.0f, "%.1f")) {
                    selected_cp.reference_barrel_inches =
                        ClampValue(selected_cp.reference_barrel_inches, 1.0f, 40.0f);
                }
                if (!selected_cp.barrel_mv_profile.empty()) {
                    ImGui::TextDisabled("Profile (%d pts ΓÇö edit JSON to modify):",
                                        static_cast<int>(selected_cp.barrel_mv_profile.size()));
                    for (const auto& bp : selected_cp.barrel_mv_profile) {
                        ImGui::TextDisabled("  %.0f\" \u2192 %.0f fps", bp.first, bp.second);
                    }
                } else {
                    ImGui::SetNextItemWidth(120.0f);
                    if (ImGui::InputFloat("MV Adj (fps/in)##cp_mv_adj",
                                          &selected_cp.mv_adjustment_fps_per_in, 1.0f, 5.0f, "%.1f")) {
                        selected_cp.mv_adjustment_fps_per_in =
                            std::fmax(0.0f, selected_cp.mv_adjustment_fps_per_in);
                    }
                }
            }
            // -- Bullet geometry (mass, length, caliber) --
            ImGui::Separator();
            {
                const float avail_mass = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_mass * 0.35f : -1.0f);
                ImGui::InputFloat("Mass (gr)", &g.state.bullet.mass_grains, 1.0f, 10.0f, "%.2f");
                if (uc_on) {
                    // Removed obsolete mass sigma UI
                }
            }
            float length_display =
                is_imperial ? (g.state.bullet.length_mm * MM_TO_IN) : g.state.bullet.length_mm;
            {
                const float avail_len = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_len * 0.35f : -1.0f);
                if (ImGui::InputFloat(is_imperial ? "Length (in)" : "Length (mm)", &length_display,
                                      is_imperial ? 0.01f : 0.1f, is_imperial ? 0.1f : 1.0f,
                                      is_imperial ? "%.3f" : "%.2f")) {
                    g.state.bullet.length_mm =
                        is_imperial ? (length_display * IN_TO_MM) : length_display;
                }
                if (uc_on) {
                    // Removed obsolete length sigma UI
                }
            }
            float caliber_display = is_imperial ? g.state.bullet.caliber_inches
                                                : (g.state.bullet.caliber_inches * IN_TO_MM);
            {
                const float avail_cal = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_cal * 0.35f : -1.0f);
                if (ImGui::InputFloat(is_imperial ? "Caliber (in)" : "Caliber (mm)", &caliber_display,
                                      is_imperial ? 0.001f : 0.01f, is_imperial ? 0.01f : 0.1f,
                                      is_imperial ? "%.3f" : "%.2f")) {
                    g.state.bullet.caliber_inches =
                        is_imperial ? caliber_display : (caliber_display * MM_TO_IN);
                }
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
            ImGui::InputText("Gun Preset Name", g.state.new_gun_preset_name,
                             IM_ARRAYSIZE(g.state.new_gun_preset_name));
            if (ImGui::Button("Add/Update Gun Preset")) {
                const std::string preset_name = g.state.new_gun_preset_name;
                if (!preset_name.empty()) {
                    GunPreset preset = CaptureCurrentGunPreset(preset_name);
                    const int existing = FindGunPresetByName(preset_name);
                    if (existing >= 0) {
                        g.gun_presets[existing] = preset;
                        g.state.selected_gun_preset = existing;
                    } else {
                        g.gun_presets.push_back(preset);
                        g.state.selected_gun_preset = static_cast<int>(g.gun_presets.size()) - 1;
                    }
                    g.state.last_action = "Gun Preset Saved";
                    // ticker will refresh output at next tick
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Gun") && g.state.selected_gun_preset >= 0 &&
                g.state.selected_gun_preset < static_cast<int>(g.gun_presets.size()) &&
                IsGunPresetCompatibleWithSelectedCaliber(
                    g.gun_presets[g.state.selected_gun_preset])) {
                ApplyGunPreset(g.gun_presets[g.state.selected_gun_preset]);
                g.state.last_action = "Gun Preset Applied";
                {
                    std::lock_guard<std::mutex> lk(g.engine_mutex);
                    ApplyConfig();
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Selected Gun") && g.state.selected_gun_preset >= 0 &&
                g.state.selected_gun_preset < static_cast<int>(g.gun_presets.size()) &&
                IsGunPresetCompatibleWithSelectedCaliber(
                    g.gun_presets[g.state.selected_gun_preset])) {
                g.gun_presets.erase(g.gun_presets.begin() + g.state.selected_gun_preset);
                if (g.gun_presets.empty()) {
                    g.state.selected_gun_preset = -1;
                } else if (g.state.selected_gun_preset >= static_cast<int>(g.gun_presets.size())) {
                    g.state.selected_gun_preset = static_cast<int>(g.gun_presets.size()) - 1;
                }
                g.state.last_action = "Gun Preset Removed";
                // ticker will refresh output at next tick
            }

            ImGui::SameLine();
            if (ImGui::Button("Save Presets to File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_guns.json";
                const bool ok = SaveGunPresetsToFile(repo_path) || SaveGunPresetsToFile("dope_gui_guns.json");
                g.state.last_action = ok ? "Saved Gun Presets" : "Save Gun Presets (failed)";
                if (!ok) g.state.output_text = "Failed to save gun presets.";
            }
            ImGui::SameLine();
            if (ImGui::Button("Load Presets from File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_guns.json";
                const bool ok = LoadGunPresetsFromFile(repo_path) || LoadGunPresetsFromFile("dope_gui_guns.json");
                g.state.last_action = ok ? "Loaded Gun Presets" : "Load Gun Presets (failed)";
                if (ok) {
                    EnsureProfileDefaults();
                    EnsureSelectedGunPresetMatchesCurrentCaliber();
                } else {
                    g.state.output_text = "Failed to load gun presets.";
                }
            }

            if (ImGui::BeginListBox("##gun_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g.gun_presets.size()); ++i) {
                    if (!IsGunPresetCompatibleWithSelectedCaliber(g.gun_presets[i])) {
                        continue;
                    }
                    const bool is_selected = (g.state.selected_gun_preset == i);
                    if (ImGui::Selectable(g.gun_presets[i].name.c_str(), is_selected)) {
                        g.state.selected_gun_preset = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
            // -- Gun geometry (twist rate, barrel length) --
            ImGui::Separator();
            float twist_display = is_imperial ? g.state.bullet.twist_rate_inches
                                              : (g.state.bullet.twist_rate_inches * IN_TO_MM);
            {
                const float avail_tw = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_tw * 0.45f : -1.0f);
                if (ImGui::InputFloat(is_imperial ? "Twist (in/turn)" : "Twist (mm/turn)",
                                      &twist_display, is_imperial ? 0.1f : 1.0f,
                                      is_imperial ? 1.0f : 10.0f, is_imperial ? "%.2f" : "%.1f")) {
                    g.state.bullet.twist_rate_inches =
                        is_imperial ? twist_display : (twist_display * MM_TO_IN);
                }
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    const float twist_sigma_display =
                        is_imperial ? g.state.uc_config.sigma_twist_rate_inches
                                    : (g.state.uc_config.sigma_twist_rate_inches * IN_TO_MM);
                    ImGui::Text(is_imperial ? "+/-Twist (1%%) = %.2f in" : "+/-Twist (1%%) = %.1f mm",
                                twist_sigma_display);
                }
            }
            if (ImGui::InputFloat("Barrel Length (in)", &g.state.bullet.barrel_length_in, 0.1f, 1.0f,
                                  "%.1f")) {
                g.state.bullet.barrel_length_in = ClampValue(g.state.bullet.barrel_length_in, 2.0f, 40.0f);
                ComputeAdjustedMv();
            }
            if (ImGui::InputFloat("Muzzle OD (in)", &g.state.bullet.muzzle_diameter_in, 0.01f, 0.1f,
                                  "%.3f")) {
                g.state.bullet.muzzle_diameter_in = ClampValue(g.state.bullet.muzzle_diameter_in, 0.2f, 3.0f);
                g.state.bullet.stiffness_moa = ComputeStiffnessMoa(g.state.bullet.muzzle_diameter_in,
                                                                  std::fabs(g.state.bullet.caliber_inches),
                                                                  g.state.bullet.barrel_material);
            }
            const char* current_material =
                (g.state.barrel_material_index >= 0 &&
                 g.state.barrel_material_index < kNumBarrelMaterialPresets)
                    ? kBarrelMaterialPresets[g.state.barrel_material_index].label
                    : kBarrelMaterialPresets[kDefaultBarrelMaterialIndex].label;
            if (ImGui::BeginCombo("Barrel Material", current_material)) {
                for (int i = 0; i < kNumBarrelMaterialPresets; ++i) {
                    bool selected = (g.state.barrel_material_index == i);
                    if (ImGui::Selectable(kBarrelMaterialPresets[i].label, selected)) {
                        g.state.barrel_material_index = i;
                        g.state.bullet.barrel_material = kBarrelMaterialPresets[i].material;
                        g.state.bullet.stiffness_moa = ComputeStiffnessMoa(
                            g.state.bullet.muzzle_diameter_in, std::fabs(g.state.bullet.caliber_inches),
                            g.state.bullet.barrel_material);
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::TextDisabled("Stiffness MOA: %.2f", g.state.bullet.stiffness_moa);
            {
                const char* current_finish =
                    (g.state.barrel_finish_index >= 0 && g.state.barrel_finish_index < kNumBarrelFinishPresets)
                        ? kBarrelFinishPresets[g.state.barrel_finish_index].name
                        : "Custom";
                if (ImGui::BeginCombo("Barrel Finish", current_finish)) {
                    for (int i = 0; i < kNumBarrelFinishPresets; ++i) {
                        bool selected = (g.state.barrel_finish_index == i);
                        if (ImGui::Selectable(kBarrelFinishPresets[i].name, selected)) {
                            g.state.barrel_finish_index = i;
                            if (i > 0) {
                                g.state.bullet.barrel_finish_sigma_mv_fps = kBarrelFinishPresets[i].sigma_mv_fps;
                            }
                        }
                        if (selected) ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                float input_bias = g.state.bullet.cold_bore_velocity_bias;
                if (ImGui::InputFloat("Cold Bore MV Bias (fps)", &input_bias, 0.1f, 1.0f, "%.2f")) {
                    g.state.bullet.cold_bore_velocity_bias = ClampValue(input_bias, 1.0f, 50.0f);
                }
                float input_angular = g.state.bullet.angular_sigma_moa;
                if (ImGui::InputFloat("Angular Sigma (MOA)", &input_angular, 0.01f, 0.1f, "%.2f")) {
                    g.state.bullet.angular_sigma_moa = ClampValue(input_angular, 0.01f, 5.0f);
                }
            }
            ImGui::SeparatorText("Barrel Attachments");
            ImGui::Checkbox("Free-Floated", &g.state.free_floated);
            ImGui::SameLine();
            ImGui::Checkbox("Suppressor", &g.state.suppressor_attached);
            ImGui::SameLine();
            ImGui::Checkbox("Barrel Tuner", &g.state.barrel_tuner_attached);
            // Keep bullet struct in sync for ApplyConfig hot-path.
            g.state.bullet.free_floated = g.state.free_floated;
            g.state.bullet.suppressor_attached = g.state.suppressor_attached;
            g.state.bullet.barrel_tuner_attached = g.state.barrel_tuner_attached;
            {
                const float avail_mva = ImGui::GetContentRegionAvail().x;
                // MV-per-inch is automatically computed from the active cartridge preset.
                // Show read-only; append "(est.)" when the fallback tier estimator is active.
                const float active_fps_per_in = g.state.bullet.mv_adjustment_factor;
                ImGui::SetNextItemWidth(uc_on ? avail_mva * 0.45f : -1.0f);
                if (!g.active_barrel_mv_profile.empty()) {
                    ImGui::TextDisabled("MV Adj: profile (curr %.0f fps)",
                                        g.state.bullet.muzzle_velocity_ms * MPS_TO_FPS);
                } else if (g_mv_adjustment_estimated) {
                    ImGui::TextDisabled("MV Adj (fps/in): %.1f (est.)", active_fps_per_in);
                } else {
                    ImGui::TextDisabled("MV Adj (fps/in): %.1f", active_fps_per_in);
                }
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputFloat("+/-MVAdj(fps/in)##mva",
                                          &g.state.uc_config.sigma_mv_adjustment_fps_per_in, 0.0f, 0.0f,
                                          "%.2f")) {
                        if (g.state.uc_config.sigma_mv_adjustment_fps_per_in < 0.0f)
                            g.state.uc_config.sigma_mv_adjustment_fps_per_in = 0.0f;
                    }
                }
            }
            ImGui::PopID();
        }

        ImGui::Checkbox("Enable Uncertainty Propagation", &g.state.uc_config.enabled);
        if (g.state.uc_config.enabled) {
            ImGui::SameLine();
            if (ImGui::SmallButton("Reset Sigmas")) {
                bool was_enabled = g.state.uc_config.enabled;
                DOPE_GetDefaultUncertaintyConfig(&g.state.uc_config);
                g.state.uc_config.enabled = was_enabled;
                g.state.hw_barometer_index = kBarometerDefaultIndex;
                g.state.hw_lrf_index = kLrfDefaultIndex;
                g.state.hw_cant_index = kCantDefaultIndex;
                g.state.hw_temp_sensor_index = kTempDefaultIndex;
                g.state.hw_gps_index = kGpsDefaultIndex;
                g.state.hw_mag_lat_index = kMagLatDefaultIndex;
                g.state.baro_is_calibrated = false;
                g.state.baro_has_calibration_temp = false;
                g.state.baro_calibration_temp_c = 0.0f;
                g.state.mass_error_preset_index = kMassErrorDefaultIndex;
                g.state.length_error_preset_index = kLengthErrorDefaultIndex;
                g.state.caliber_error_preset_index = kCaliberErrorDefaultIndex;
                SyncDerivedInputUncertaintySigmas();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(+/- shown inline with each value)");
        }

        ImGui::TextUnformatted("Manual Inputs");
        float muzzle_display = is_imperial ? (g.state.bullet.muzzle_velocity_ms * MPS_TO_FPS)
                                           : g.state.bullet.muzzle_velocity_ms;
        // Effective MV sigma is the larger of manual MV sigma and barrel-finish sigma.
        const float mv_sigma_effective_ms = std::max(g.state.uc_config.sigma_muzzle_velocity_ms,
                                                     g.state.bullet.barrel_finish_sigma_mv_fps * FPS_TO_MPS);
        float mv_sig_display = is_imperial ? (mv_sigma_effective_ms * MPS_TO_FPS)
                                           : mv_sigma_effective_ms;
        {
            const float avail_mv = ImGui::GetContentRegionAvail().x;
            ImGui::BeginDisabled(true);
            ImGui::SetNextItemWidth(uc_on ? avail_mv * 0.45f : -1.0f);
            ImGui::InputFloat(is_imperial ? "Muzzle Velocity (fps)" : "Muzzle Velocity (m/s)",
                              &muzzle_display, 0.0f, 0.0f, "%.1f",
                              ImGuiInputTextFlags_ReadOnly);
            if (uc_on) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputFloat(is_imperial ? "+/-MV(fps)" : "+/-MV(m/s)##mv",
                                  &mv_sig_display, 0.0f, 0.0f, "%.1f",
                                  ImGuiInputTextFlags_ReadOnly);
            }
            ImGui::EndDisabled();
        }
        ImGui::TextDisabled("Muzzle velocity is derived from cartridge + barrel; sigma = max(manual MV sigma, barrel finish sigma).");
        ImGui::Separator();
        float zero_display =
            is_imperial ? (g.state.zero.zero_range_m * M_TO_YD) : g.state.zero.zero_range_m;
        {
            const float avail_zr = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_zr * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Zero Range (yd)" : "Zero Range (m)", &zero_display,
                                  is_imperial ? 1.0f : 1.0f, is_imperial ? 10.0f : 10.0f, "%.2f")) {
                g.state.zero.zero_range_m = is_imperial ? (zero_display * YD_TO_M) : zero_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float zr_sig = is_imperial ? (g.state.uc_config.sigma_zero_range_m * M_TO_YD)
                                            : g.state.uc_config.sigma_zero_range_m;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Zero(yd)##zr" : "+/-Zero(m)##zr", &zr_sig,
                                      0.0f, 0.0f, "%.2f")) {
                    g.state.uc_config.sigma_zero_range_m =
                        is_imperial ? (zr_sig * YD_TO_M) : zr_sig;
                    if (g.state.uc_config.sigma_zero_range_m < 0.0f)
                        g.state.uc_config.sigma_zero_range_m = 0.0f;
                }
            }
        }
        float sight_display =
            is_imperial ? (g.state.zero.sight_height_mm * MM_TO_IN) : g.state.zero.sight_height_mm;
        {
            const float avail_sh = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_sh * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Sight Height (in)" : "Sight Height (mm)",
                                  &sight_display, is_imperial ? 0.01f : 0.1f,
                                  is_imperial ? 0.1f : 1.0f, is_imperial ? "%.3f" : "%.2f")) {
                g.state.zero.sight_height_mm =
                    is_imperial ? (sight_display * IN_TO_MM) : sight_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float sht_sig = is_imperial ? (g.state.uc_config.sigma_sight_height_mm * MM_TO_IN)
                                            : g.state.uc_config.sigma_sight_height_mm;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-SightH(in)##sh" : "+/-SightH(mm)##sh",
                                      &sht_sig, 0.0f, 0.0f, is_imperial ? "%.3f" : "%.2f")) {
                    g.state.uc_config.sigma_sight_height_mm =
                        is_imperial ? (sht_sig * IN_TO_MM) : sht_sig;
                    if (g.state.uc_config.sigma_sight_height_mm < 0.0f)
                        g.state.uc_config.sigma_sight_height_mm = 0.0f;
                }
            }
        }

        float wind_display =
            is_imperial ? (g.state.wind_speed_ms * MPS_TO_MPH) : g.state.wind_speed_ms;
        {
            const float avail_ws = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_ws * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Wind Speed (mph)" : "Wind Speed (m/s)",
                                  &wind_display, is_imperial ? 0.5f : 0.1f,
                                  is_imperial ? 5.0f : 1.0f, "%.2f")) {
                g.state.wind_speed_ms = is_imperial ? (wind_display * MPH_TO_MPS) : wind_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float wsp_sig = is_imperial ? (g.state.uc_config.sigma_wind_speed_ms * MPS_TO_MPH)
                                            : g.state.uc_config.sigma_wind_speed_ms;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Wind(mph)##ws" : "+/-Wind(m/s)##ws",
                                      &wsp_sig, 0.0f, 0.0f, "%.2f")) {
                    g.state.uc_config.sigma_wind_speed_ms =
                        is_imperial ? (wsp_sig * MPH_TO_MPS) : wsp_sig;
                    if (g.state.uc_config.sigma_wind_speed_ms < 0.0f)
                        g.state.uc_config.sigma_wind_speed_ms = 0.0f;
                }
            }
        }
        {
            const float avail_wh = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_wh * 0.45f : -1.0f);
            ImGui::InputFloat("Wind Heading (deg)", &g.state.wind_heading, 1.0f, 5.0f, "%.1f");
            if (uc_on) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-WindDir(deg)##wd",
                                      &g.state.uc_config.sigma_wind_heading_deg, 0.0f, 0.0f,
                                      "%.1f")) {
                    if (g.state.uc_config.sigma_wind_heading_deg < 0.0f)
                        g.state.uc_config.sigma_wind_heading_deg = 0.0f;
                }
            }
        }
        // Latitude source / uncertainty presets (shown when uncertainty propagation is on)
        if (uc_on) {
            // GPS / GNSS module combo ╬ô├ç├╢ direct latitude feed; highest priority source
            {
                const int gps_count = GetNumGpsLatPresets();
                const char* gps_labels[16];
                for (int i = 0; i < gps_count && i < 16; ++i)
                    gps_labels[i] = GetGpsLabel(i);
                if (ImGui::Combo("GPS / GNSS Module", &g.state.hw_gps_index, gps_labels, gps_count)) {
                    const int gps_mapped = GetMappedGpsLatPresetIndex(g.state.hw_gps_index);
                    if (gps_mapped > 0) {
                        if (gps_mapped >= 0 && gps_mapped < kNumGpsLatPresets) {
                            g.state.uc_config.sigma_latitude_deg =
                                kGpsLatPresets[gps_mapped].sigma_lat_deg;
                        }
                        g.state.hw_mag_lat_index = 0; // GPS overrides mag estimator selection
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "GPS/GNSS receiver feeding latitude directly.\n"
                        "Takes priority over magnetometer estimation and manual entry.\n"
                        "Set to Custom and enter sigma manually if module is not listed.");
            }
            // Magnetometer preset ╬ô├ç├╢ shown only when no GPS module is selected
            if (GetMappedGpsLatPresetIndex(g.state.hw_gps_index) == 0) {
                const int mag_count = GetNumMagLatPresets();
                const char* mag_labels[16];
                for (int i = 0; i < mag_count && i < 16; ++i)
                    mag_labels[i] = GetMagLatLabel(i);
                if (ImGui::Combo("Magnetometer (lat est.)", &g.state.hw_mag_lat_index, mag_labels,
                                 mag_count)) {
                    const int mag_mapped = GetMappedMagLatPresetIndex(g.state.hw_mag_lat_index);
                    if (mag_mapped > 0) {
                        if (mag_mapped >= 0 && mag_mapped < kNumMagLatPresets) {
                            g.state.uc_config.sigma_latitude_deg =
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
            const int gps_mapped = GetMappedGpsLatPresetIndex(g.state.hw_gps_index);
            const int mag_mapped = GetMappedMagLatPresetIndex(g.state.hw_mag_lat_index);
            const bool lat_locked =
                uc_on && (gps_mapped > 0 || mag_mapped > 0);
            const float avail_lat = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_lat * 0.45f : -1.0f);
            ImGui::InputFloat("Latitude (deg)", &g.state.latitude, 0.1f, 1.0f, "%.3f");
            if (uc_on) {
                ImGui::SameLine();
                if (lat_locked)
                    ImGui::BeginDisabled();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-Lat(deg)##lat", &g.state.uc_config.sigma_latitude_deg,
                                      0.0f, 0.0f, "%.2f")) {
                    if (g.state.uc_config.sigma_latitude_deg < 0.0f)
                        g.state.uc_config.sigma_latitude_deg = 0.0f;
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
            if (ImGui::Combo("Barometer", &g.state.hw_barometer_index, baro_labels, baro_count)) {
                SyncDerivedInputUncertaintySigmas();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Barometer pressure uncertainty profile used for sigma pressure propagation.");

            // Single calibration button; default calibration temperature is current baro temp.
            {
                if (!g.state.baro_is_calibrated) {
                    if (ImGui::Button("Calibrate Barometer")) {
                        g.state.baro_is_calibrated = true;
                        g.state.baro_has_calibration_temp = true;
                        if (std::isfinite(g.state.baro_temp)) {
                            g.state.baro_calibration_temp_c = g.state.baro_temp;
                        }
                        SyncDerivedInputUncertaintySigmas();
                    }
                } else {
                    if (ImGui::Button("Barometer: Calibrated  (click to clear)")) {
                        g.state.baro_is_calibrated = false;
                        g.state.baro_has_calibration_temp = false;
                        SyncDerivedInputUncertaintySigmas();
                    }
                }
            }

            if (g.state.baro_is_calibrated) {
                float cal_temp_display =
                    is_imperial ? ((g.state.baro_calibration_temp_c * 9.0f / 5.0f) + 32.0f)
                                : g.state.baro_calibration_temp_c;
                if (ImGui::InputFloat(is_imperial ? "Cal Temp (F, internal)"
                                                  : "Cal Temp (C, internal)",
                                      &cal_temp_display, is_imperial ? 0.5f : 0.1f,
                                      is_imperial ? 2.0f : 1.0f, "%.2f")) {
                    g.state.baro_calibration_temp_c =
                        is_imperial ? ((cal_temp_display - 32.0f) * 5.0f / 9.0f) : cal_temp_display;
                    SyncDerivedInputUncertaintySigmas();
                }
            }

            const float delta_temp_c =
                (g.state.baro_is_calibrated && g.state.baro_has_calibration_temp)
                    ? std::fabs(g.state.baro_temp - g.state.baro_calibration_temp_c)
                    : 0.0f;
            ImGui::Text("Barometer sigma pressure: %.2f Pa (\u0394T used: %.2f C)",
                        g.state.uc_config.sigma_pressure_pa, delta_temp_c);
        }

        // Thermometer hardware preset ╬ô├ç├╢ shown above temp row when uc_on.
        if (uc_on) {
            const int tmp_count = GetNumTempSensorPresets();
            const char* tmp_labels[16];
            for (int i = 0; i < tmp_count && i < 16; ++i)
                tmp_labels[i] = GetTempSensorLabel(i);
            if (ImGui::Combo("Thermometer", &g.state.hw_temp_sensor_index, tmp_labels, tmp_count)) {
                SyncDerivedInputUncertaintySigmas();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Thermometer sensor: accuracy varies with temperature.\n"
                                  "Grays out the manual sigma field when table data is loaded.");
        }
        // Lock manual entry only when a preset with actual table data is selected.
        const DOPE_ErrorTable* tmp_table_lock = GetTempTable(g.state.hw_temp_sensor_index);
        const bool tmp_locked = uc_on && (g.state.hw_temp_sensor_index > 0 && tmp_table_lock && tmp_table_lock->points != nullptr);
        const DOPE_ErrorTable* pr_table_lock = GetBarometerTable(g.state.hw_barometer_index);
        const bool pressure_locked = uc_on && (g.state.hw_barometer_index >= 0 && g.state.hw_barometer_index < GetNumBarometerPresets()) && (pr_table_lock && pr_table_lock->points != nullptr);
        float pressure_display = g.state.baro_pressure;
        {
            const float avail_pr = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_pr * 0.45f : -1.0f);
            if (ImGui::InputFloat("Baro Pressure (Pa)", &pressure_display, 10.0f, 100.0f, "%.1f")) {
                // Pressure value itself changed; sigma remains derived from barometer calibration
                // drift model.
                g.state.baro_pressure = pressure_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                if (pressure_locked)
                    ImGui::BeginDisabled();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-Press(Pa)##pr", &g.state.uc_config.sigma_pressure_pa,
                                      0.0f, 0.0f, "%.0f")) {
                    if (g.state.uc_config.sigma_pressure_pa < 0.0f)
                        g.state.uc_config.sigma_pressure_pa = 0.0f;
                }
                if (pressure_locked)
                    ImGui::EndDisabled();
            }
        }
        float temp_display =
            is_imperial ? ((g.state.baro_temp * 9.0f / 5.0f) + 32.0f) : g.state.baro_temp;
        {
            const float avail_tm = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_tm * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Temp (F)" : "Temp (C)", &temp_display,
                                  is_imperial ? 0.5f : 0.1f, is_imperial ? 2.0f : 1.0f, "%.2f")) {
                g.state.baro_temp =
                    is_imperial ? ((temp_display - 32.0f) * 5.0f / 9.0f) : temp_display;
                if (uc_on) {
                    SyncDerivedInputUncertaintySigmas();
                }
            }
            if (uc_on) {
                ImGui::SameLine();
                if (tmp_locked)
                    ImGui::BeginDisabled();
                float tmp_sig = is_imperial ? (g.state.uc_config.sigma_temperature_c * 9.0f / 5.0f)
                                            : g.state.uc_config.sigma_temperature_c;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Temp(F)##tm" : "+/-Temp(C)##tm", &tmp_sig,
                                      0.0f, 0.0f, "%.2f")) {
                    g.state.uc_config.sigma_temperature_c =
                        is_imperial ? (tmp_sig * 5.0f / 9.0f) : tmp_sig;
                    if (g.state.uc_config.sigma_temperature_c < 0.0f)
                        g.state.uc_config.sigma_temperature_c = 0.0f;
                }
                if (tmp_locked)
                    ImGui::EndDisabled();
            }
        }
        {
            const float avail_hm = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_hm * 0.45f : -1.0f);
            ImGui::InputFloat("Humidity (0..1)", &g.state.baro_humidity, 0.01f, 0.1f, "%.2f");
            if (uc_on) {
                ImGui::SameLine();
                if (tmp_locked)
                    ImGui::BeginDisabled();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat("+/-Humid(frac)##hm", &g.state.uc_config.sigma_humidity, 0.0f,
                                      0.0f, "%.3f")) {
                    if (g.state.uc_config.sigma_humidity < 0.0f)
                        g.state.uc_config.sigma_humidity = 0.0f;
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
            if (ImGui::Combo("Rangefinder", &g.state.hw_lrf_index, lrf_labels, lrf_count)) {
                SyncDerivedInputUncertaintySigmas();
            }
        }
        const bool lrf_locked = uc_on && (g.state.hw_lrf_index > 0);
        float lrf_display = is_imperial ? (g.state.lrf_range * M_TO_YD) : g.state.lrf_range;
        {
            const float avail_lrf = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_lrf * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "LRF Range (yd)" : "LRF Range (m)", &lrf_display,
                                  1.0f, 10.0f, "%.2f")) {
                g.state.lrf_range = is_imperial ? (lrf_display * YD_TO_M) : lrf_display;
                if (uc_on) {
                    SyncDerivedInputUncertaintySigmas();
                }
            }
            if (uc_on) {
                ImGui::SameLine();
                if (lrf_locked)
                    ImGui::BeginDisabled();
                float range_sigma_display = is_imperial
                                                ? (g.state.uc_config.sigma_range_m * M_TO_YD)
                                                : g.state.uc_config.sigma_range_m;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-Range(yd)##lrf" : "+/-Range(m)##lrf",
                                      &range_sigma_display, 0.0f, 0.0f, "%.2f")) {
                    g.state.uc_config.sigma_range_m =
                        is_imperial ? (range_sigma_display * YD_TO_M) : range_sigma_display;
                    if (g.state.uc_config.sigma_range_m < 0.0f)
                        g.state.uc_config.sigma_range_m = 0.0f;
                }
                if (lrf_locked)
                    ImGui::EndDisabled();
            }
        }
        {
            constexpr float kMToFt = 3.280839895f;
            constexpr float kFtToM = 0.3048f;
            float target_elev_display =
                is_imperial ? (g.state.target_elevation_m * kMToFt) : g.state.target_elevation_m;
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::InputFloat(is_imperial ? "Elevation Delta (ft, +up/-down)"
                                              : "Elevation Delta (m, +up/-down)",
                                  &target_elev_display,
                                  is_imperial ? 0.5f : 0.1f,
                                  is_imperial ? 5.0f : 1.0f,
                                  "%.2f")) {
                g.state.target_elevation_m =
                    is_imperial ? (target_elev_display * kFtToM) : target_elev_display;
                // Keep geometry sane for slant-range conversion.
                const float max_abs = std::fmax(g.state.lrf_range - 0.01f, 0.0f);
                if (std::fabs(g.state.target_elevation_m) > max_abs) {
                    g.state.target_elevation_m =
                        std::copysign(max_abs, g.state.target_elevation_m);
                }
            }
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
                    if (ImGui::Combo("Cant Sensor", &g.state.hw_cant_index, cant_labels, cant_count)) {
                        if (g.state.hw_cant_index > 0) {
                            g.state.uc_config.sigma_cant_deg = GetCantSigma(g.state.hw_cant_index);
                        }
                    }
                }
                const bool cant_locked = (g.state.hw_cant_index > 0);
                if (cant_locked)
                    ImGui::BeginDisabled();
                if (ImGui::InputFloat("+/- Cant (deg)", &g.state.uc_config.sigma_cant_deg, 0.1f,
                                      1.0f, "%.2f")) {
                    if (g.state.uc_config.sigma_cant_deg < 0.0f)
                        g.state.uc_config.sigma_cant_deg = 0.0f;
                }
                if (cant_locked)
                    ImGui::EndDisabled();

                ImGui::Unindent();
            }
        }

        if (ImGui::Button("Apply Config")) {
            g.state.last_action = "Apply Config";
            std::lock_guard<std::mutex> lk(g.engine_mutex);
            ApplyConfig();
            RefreshOutput();
            KickUncertaintyJob();
        }
        ImGui::SameLine();
        if (ImGui::Button("Step Update")) {
            g.state.last_action = "Step Update";
            std::lock_guard<std::mutex> lk(g.engine_mutex);
            ApplyConfig();
            RunFrameUpdates(1);
            RefreshOutput();
            KickUncertaintyJob();
        }
        ImGui::SameLine();
        if (ImGui::Button("Shot Fired")) {
            std::lock_guard<std::mutex> lk(g.engine_mutex);
            ApplyConfig();
            DOPE_NotifyShotFired(g.state.now_us, g.state.baro_temp);
            RefreshOutput();
            KickUncertaintyJob();
        }
        ImGui::SameLine();
        const bool run_100_busy = g.run_batch_in_flight.load(std::memory_order_relaxed);
        ImGui::BeginDisabled(run_100_busy);
        if (ImGui::Button("Run 100")) {
            // Kick off the heavier batch on a worker so the render loop stays responsive.
            g.run_batch_in_flight.store(true, std::memory_order_relaxed);
            std::thread([] {
                g.state.last_action = "Run 100";
                {
                    std::lock_guard<std::mutex> lk(g.engine_mutex);
                    ApplyConfig();
                }

                // Run in small chunks so the render thread is not blocked for the full batch.
                constexpr int kTotalFrames = 100;
                constexpr int kChunk = 10;
                int remaining = kTotalFrames;
                while (remaining > 0) {
                    const int to_run = std::min(kChunk, remaining);
                    {
                        std::lock_guard<std::mutex> lk(g.engine_mutex);
                        for (int i = 0; i < to_run; ++i) {
                            SensorFrame frame = BuildFrame();
                            DOPE_Update(&frame);
                        }
                    }
                    remaining -= to_run;
                    std::this_thread::yield();
                }

                {
                    std::lock_guard<std::mutex> lk(g.engine_mutex);
                    RefreshOutput();
                }
                g.run_batch_in_flight.store(false, std::memory_order_relaxed);
                KickUncertaintyJob();
            }).detach();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        bool auto_tick = g.auto_tick_enabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Auto Tick", &auto_tick)) {
            g.auto_tick_enabled.store(auto_tick, std::memory_order_relaxed);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Engine")) {
            std::lock_guard<std::mutex> lk(g.engine_mutex);
            ResetEngineAndState();
            KickUncertaintyJob();
        }

        ImGui::Separator();
        ImGui::InputText("Profile Library Path", g.state.profile_library_path,
                         IM_ARRAYSIZE(g.state.profile_library_path));
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
        ImGui::InputText("Preset Path", g.state.preset_path, IM_ARRAYSIZE(g.state.preset_path));
        if (ImGui::Button("Save Preset")) {
            g.state.last_action = "Save Preset";
            SavePreset();
            // ticker will refresh output at next tick
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Preset")) {
            g.state.last_action = "Load Preset";
            LoadPreset();
            std::lock_guard<std::mutex> lk(g.engine_mutex);
            ApplyConfig();
            RefreshOutput();
        }

        ImGui::End();

        ImGui::Begin("DOPE Output");
        ImGui::InputTextMultiline("##output", frame_output.data(), frame_output.size() + 1,
                                  ImVec2(-FLT_MIN, -FLT_MIN), ImGuiInputTextFlags_ReadOnly);
        ImGui::End();

        ImGui::Begin("Target View");
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        FiringSolution sol = frame_sol;

        const float hold_windage_moa = sol.hold_windage_moa;
        const float hold_elevation_moa = sol.hold_elevation_moa;
        const float impact_distance_moa = std::sqrt(hold_windage_moa * hold_windage_moa +
                                                    hold_elevation_moa * hold_elevation_moa);

        // Hoist eigendecomposition of the 2x2 uncertainty covariance matrix so the
        // auto-scale uses the true ellipse major semi-axis (axis1_moa) rather than
        // max(sigma_elev, sigma_wind), which underestimates when covariance is nonzero.
        // These variables are reused by the analytic ellipse contour section below.
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

        // Auto-scale from nominal hold only so uncertainty edits do not shift
        // the apparent reticle center by changing the MOA-to-pixel scale.
        const float ring_count = static_cast<float>(TARGET_RING_COUNT);
        const float required_span_moa = impact_distance_moa;
        float moa_per_ring = std::max(std::round(required_span_moa / ring_count), 1.0f);
        if ((moa_per_ring * ring_count) < required_span_moa) {
            moa_per_ring = std::max(moa_per_ring + 1.0f, 1.0f);
        }
        const float max_span_moa = moa_per_ring * ring_count;
        const float display_range_m = (sol.range_m > 0.0f) ? sol.range_m : g.state.lrf_range;
        const float display_range_yd = display_range_m * M_TO_YD;
        const float inches_per_moa = 1.047f * (display_range_yd / 100.0f);
        const float inches_per_ring = moa_per_ring * inches_per_moa;
        const float elev_offset_in = hold_elevation_moa * inches_per_moa;
        const float wind_offset_in = hold_windage_moa * inches_per_moa;
        const float wind_only_in = sol.wind_only_windage_moa * inches_per_moa;
        const float earth_spin_in = sol.earth_spin_windage_moa * inches_per_moa;
        const float offsets_in = sol.offsets_windage_moa * inches_per_moa;
        const float cant_added_in = sol.cant_windage_moa * inches_per_moa;
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

        ImGui::Text("Bullseye rings: %.0f MOA (%.2f in) @ %.1f yd", moa_per_ring, inches_per_ring,
                display_range_yd);
        ImGui::Text("Offset: Elev %.2f MOA (%.2f in), Wind %.2f MOA (%.2f in)", hold_elevation_moa,
                elev_offset_in, hold_windage_moa, wind_offset_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(hold_windage_moa), "[%s]",
                   direction_label(hold_windage_moa));
        ImGui::Text("Wind breakdown:");
        ImGui::Text("  Wind:       %.2f MOA (%.2f in)", sol.wind_only_windage_moa, wind_only_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.wind_only_windage_moa), "[%s]",
                           direction_label(sol.wind_only_windage_moa));
        ImGui::Text("  Earth spin: %.2f MOA (%.2f in)", sol.earth_spin_windage_moa, earth_spin_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.earth_spin_windage_moa), "[%s]",
                           direction_label(sol.earth_spin_windage_moa));
        ImGui::Text("  Offsets total:    " << sol.offsets_windage_moa << " ["
       << direction_label(sol.offsets_windage_moa) << "]\n");
        ImGui::Text("  Cant added:       " << sol.cant_windage_moa << " ["
       << direction_label(sol.cant_windage_moa) << "]\n");
        ImGui::Text("  Total windage:    " << sol.hold_windage_moa << " ["
       << direction_label(sol.hold_windage_moa) << "]\n");

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
                "MV", "Range", "WindSpd", "WindDir", "Temp", "Press", "Humid", "SightH", "Cant", "Latitude", "Twist", "ZeroRng", "MVAdj"
            };
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
        } else if (g.state.uc_config.enabled) {
            ss << "\nUncertainty: enabled but not yet computed (need valid solution).\n";
        } else {
            ss << "\nUncertainty: disabled.\n";
        }

        g.state.output_text = ss.str();

        // Publish to display snapshot so the render loop can read it without
        // ever blocking on g_engine_mutex (which may be held during a long solve).
        {
            std::lock_guard<std::mutex> dlk(g.display_mutex);
            g.display_output = g.state.output_text;
            DOPE_GetSolution(&g.display_sol);
        }
    }

    g_ticker_running = false;
    ticker_thread.join();

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    DestroyWindow(hwnd);
    UnregisterClass(wc.lpszClassName, wc.hInstance);

    return 0;
}
