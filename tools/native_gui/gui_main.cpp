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
    float reference_barrel_inches = 24.0f;
    float mv_adjustment_fps_per_in = 0.0f; // fps of MV change per inch of barrel (0 = use fallback estimator)
    std::vector<std::pair<float,float>> barrel_mv_profile; // (barrel_in, mv_fps) — multi-point table; overrides mv_adjustment_fps_per_in when non-empty
    std::vector<std::pair<float,float>> velocity_profile; // (distance_inches, velocity_ms)
    std::vector<std::pair<float,float>> trajectory_profile; // (distance_inches, trajectory_inches)
    std::vector<std::string> tags; // e.g. ["match","hpbt"]
    std::vector<std::pair<float,float>> cep_table_moa; // (range_m, cep50_moa)
    float cep_scale_floor = 1.0f;
};

struct GunPreset {
    std::string name;
    float caliber_inches = 0.308f;
    float barrel_length_in = 24.0f;
    float twist_rate_inches = 10.0f;
    float zero_range_m = 91.44f; // 100 yd
    float sight_height_mm = 38.1f;
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
    {"ST ISM330DHCX IMU (1.5 deg)", 1.5f},
    {"Level bubble (2 deg)", 2.0f},
    {"Precision digital level (0.1 deg)", 0.1f},
};
static const int kNumCantPresets = sizeof(kCantPresets) / sizeof(kCantPresets[0]);

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

struct GuiState {
    BulletProfile bullet = {};
    ZeroConfig zero = {};
    UnitSystem unit_system = UnitSystem::IMPERIAL;
    bool override_drag_coefficient = false;
    float manual_drag_coefficient = 0.0f;

    float wind_speed_ms = 0.0f;
    float wind_heading = 0.0f;
    float latitude = 0.0f;

    float baro_pressure = 101325.0f;
    float baro_temp = 18.333f; // 65 ┬░F
    float baro_humidity = 0.65f;
    float lrf_range = 457.2f; // 500 yd

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
    bool side_view_show_required_angle = false;
    bool top_down_show_required_angle = false;
    std::string output_text;
    std::string last_action;

    // Uncertainty / error-margin config (initialized from DOPE_GetDefaultUncertaintyConfig).
    UncertaintyConfig uc_config = {};
    std::vector<DOPE_CEPPoint> cartridge_cep_points; // range_m, cep50_moa
    float cartridge_cep_scale_floor = 1.0f;

    // Hardware sensor preset indices (0 = Custom/manual).
    int hw_barometer_index = kBarometerDefaultIndex;
    int hw_lrf_index = kLrfDefaultIndex;
    int hw_cant_index = 0;
    int hw_temp_sensor_index = kTempDefaultIndex; // temperature-dependent sensor
    int hw_gps_index = kGpsDefaultIndex;          // GPS/GNSS module for direct latitude feed
    int hw_mag_lat_index =
        kMagLatDefaultIndex; // magnetometer preset for dip-angle latitude estimation
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
void ComputeAdjustedMv(); // glue-layer barrel-length → MV interpolation

// Dynamic hardware preset labels / optional sigma tables loaded from JSON.
static std::vector<std::string> g_baro_labels;
static std::vector<std::string> g_temp_labels;
static std::vector<std::string> g_lrf_labels;
static std::vector<std::string> g_cant_labels;
static std::vector<float> g_cant_sigmas;
static std::vector<std::string> g_maglat_labels;
static std::vector<std::string> g_gps_labels;

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

        auto push_names = [](const json& arr, std::vector<std::string>& dst) {
            if (!arr.is_array()) return;
            for (const auto& e : arr) {
                dst.push_back(e.value("name", std::string("")));
            }
        };

        auto it = sensors->find("barometers");
        if (it != sensors->end()) push_names(*it, g_baro_labels);
        it = sensors->find("temp_sensors");
        if (it != sensors->end()) push_names(*it, g_temp_labels);
        it = sensors->find("lrfs");
        if (it != sensors->end()) push_names(*it, g_lrf_labels);
        it = sensors->find("gps");
        if (it != sensors->end()) push_names(*it, g_gps_labels);
        it = sensors->find("mag_lat");
        if (it != sensors->end()) push_names(*it, g_maglat_labels);

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

inline const DOPE_ErrorTable* GetLRFTable(int i) { return (i>=0 && i<kNumLRFPresets) ? &kLRFPresets[i].table : nullptr; }
inline const DOPE_ErrorTable* GetTempTable(int i) { return (i>=0 && i<kNumTempSensorPresets) ? &kTempSensorPresets[i].table : nullptr; }
inline const DOPE_ErrorTable* GetBarometerTable(int i) { return (i>=0 && i<kNumBarometerPresets) ? &kBarometerPresets[i].table : nullptr; }
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
        if (!j.is_array()) return false;
        std::vector<CartridgePreset> tmp;
        for (const auto& e : j) {
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
            p.reference_barrel_inches = e.value("reference_barrel_inches", 24.0f);
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
            p.cep_table_moa.clear();
            const auto cep_it = e.find("cep_table_moa");
            if (cep_it != e.end() && cep_it->is_array()) {
                for (const auto& cp : *cep_it) {
                    const float rng = cp.value("range_m", 0.0f);
                    const float cep = cp.value("cep50_moa", 0.0f);
                    p.cep_table_moa.emplace_back(rng, cep);
                }
            }
            p.cep_scale_floor = e.value("cep_scale_floor", 1.0f);
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
            p.barrel_length_in = e.value("barrel_length_in", 24.0f);
            p.twist_rate_inches = e.value("twist_rate_inches", 10.0f);
            p.zero_range_m = e.value("zero_range_m", 91.44f);
            p.sight_height_mm = e.value("sight_height_mm", 38.1f);
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

// Bullet tolerances loaded from external JSON (optional)
static json g_bullet_tolerances;

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

bool IsGunPresetCompatibleWithSelectedCaliber(const GunPreset& preset) {
    const float selected_caliber = std::fabs(GetSelectedCartridgeCaliberInches());
    const float preset_caliber = std::fabs(preset.caliber_inches);
    return std::fabs(selected_caliber - preset_caliber) <= 0.005f;
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

void EnsureProfileDefaults() {
    // Try loading external preset files; otherwise seed runtime-editable
    // preset lists from built-ins on first launch. This prevents malformed
    // JSON from poisoning GUI state by sanitizing every entry afterwards.
    // Try both the tools/native_gui path (where presets are stored in the repo)
    // and the current working directory so the GUI works when launched either
    // from the repo root or from the tools/native_gui folder.
    const std::string rel_dir = "tools/native_gui/";
    const std::string cartridge_file_repo = rel_dir + "dope_gui_cartridges.json";
    const std::string tol_file_repo = rel_dir + "dope_gui_bullet_tolerances.json";
    const std::string gun_file_repo = rel_dir + "dope_gui_guns.json";
    const std::string hw_file_repo = rel_dir + "dope_gui_hardware.json";
    // Load optional tolerances first so cartridge tag parsing can use them.
    (void)LoadBulletTolerancesFromFile(tol_file_repo);
    (void)LoadBulletTolerancesFromFile("dope_gui_bullet_tolerances.json");

    bool loaded_cartridges = LoadCartridgePresetsFromFile(cartridge_file_repo) || LoadCartridgePresetsFromFile("dope_gui_cartridges.json");
    bool loaded_guns = LoadGunPresetsFromFile(gun_file_repo) || LoadGunPresetsFromFile("dope_gui_guns.json");
    (void)LoadHardwarePresetsFromFile(hw_file_repo);
    (void)LoadHardwarePresetsFromFile("dope_gui_hardware.json");

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

float GetPresetMultiplier(const ErrorMultiplierPreset* presets, int count, int default_index,
                          int index) {
    const int safe_index = ClampPresetIndex(index, count, default_index);
    return presets[safe_index].multiplier;
}

void SyncDerivedInputUncertaintySigmas() {
    g_state.mass_error_preset_index = ClampPresetIndex(
        g_state.mass_error_preset_index, kNumMassErrorPresets, kMassErrorDefaultIndex);
    g_state.length_error_preset_index = ClampPresetIndex(
        g_state.length_error_preset_index, kNumLengthErrorPresets, kLengthErrorDefaultIndex);
    g_state.caliber_error_preset_index = ClampPresetIndex(
        g_state.caliber_error_preset_index, kNumCaliberErrorPresets, kCaliberErrorDefaultIndex);
    g_state.hw_barometer_index =
        ClampPresetIndex(g_state.hw_barometer_index, GetNumBarometerPresets(), kBarometerDefaultIndex);
    g_state.hw_lrf_index = ClampPresetIndex(g_state.hw_lrf_index, GetNumLRFPresets(), kLrfDefaultIndex);
    g_state.hw_temp_sensor_index =
        ClampPresetIndex(g_state.hw_temp_sensor_index, GetNumTempSensorPresets(), kTempDefaultIndex);
    g_state.hw_gps_index =
        ClampPresetIndex(g_state.hw_gps_index, GetNumGpsLatPresets(), kGpsDefaultIndex);
    g_state.hw_mag_lat_index =
        ClampPresetIndex(g_state.hw_mag_lat_index, GetNumMagLatPresets(), kMagLatDefaultIndex);

    // Prefer cartridge preset tags + tolerance lookup when available; fall back
    // to legacy multiplier presets if not.
    bool used_tolerances = false;
    if (!g_bullet_tolerances.is_null() && g_state.selected_cartridge_preset >= 0 &&
        g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
        const CartridgePreset& cp = g_cartridge_presets[g_state.selected_cartridge_preset];
        std::string tier_tag;
        std::string construction_tag;
        for (const auto& t : cp.tags) {
            if (g_bullet_tolerances["bullet_type_tolerances"]["tier"].contains(t)) {
                tier_tag = t;
            } else if (g_bullet_tolerances["bullet_type_tolerances"]["construction"].contains(t)) {
                construction_tag = t;
            }
        }
        const auto& tier_tbl = g_bullet_tolerances["bullet_type_tolerances"]["tier"];
        const auto& cons_tbl = g_bullet_tolerances["bullet_type_tolerances"]["construction"];

        if (!construction_tag.empty()) {
            const auto& cons = cons_tbl[construction_tag];
            // BC fractional uncertainty
            if (cons.contains("bc_fraction") && cons["bc_fraction"].is_number()) {
                g_state.uc_config.sigma_bc_fraction = cons["bc_fraction"].get<float>();
            }
            // tier_override for cast bullets: absolute tolerances
            if (cons.contains("tier_override") && !cons["tier_override"].is_null()) {
                const auto& over = cons["tier_override"];
                if (over.contains("mass_gr") && over["mass_gr"].is_number())
                    g_state.uc_config.sigma_mass_grains = over["mass_gr"].get<float>();
                if (over.contains("length_mm") && over["length_mm"].is_number())
                    g_state.uc_config.sigma_length_mm = over["length_mm"].get<float>();
                if (over.contains("caliber_in") && over["caliber_in"].is_number())
                    g_state.uc_config.sigma_caliber_inches = over["caliber_in"].get<float>();
                used_tolerances = true;
            }
        }

        // If not overridden by construction, apply tier absolute tolerances
        if (!used_tolerances && !tier_tag.empty()) {
            const auto& tier = tier_tbl[tier_tag];
            if (tier.contains("mass_gr") && tier["mass_gr"].is_number())
                g_state.uc_config.sigma_mass_grains = tier["mass_gr"].get<float>();
            if (tier.contains("length_mm") && tier["length_mm"].is_number())
                g_state.uc_config.sigma_length_mm = tier["length_mm"].get<float>();
            if (tier.contains("caliber_in") && tier["caliber_in"].is_number())
                g_state.uc_config.sigma_caliber_inches = tier["caliber_in"].get<float>();
            used_tolerances = true;
        }
    }

    if (!used_tolerances) {
        const float mass_mult =
            GetPresetMultiplier(kMassErrorPresets, kNumMassErrorPresets, kMassErrorDefaultIndex,
                                g_state.mass_error_preset_index);
        const float length_mult =
            GetPresetMultiplier(kLengthErrorPresets, kNumLengthErrorPresets, kLengthErrorDefaultIndex,
                                g_state.length_error_preset_index);
        const float caliber_mult =
            GetPresetMultiplier(kCaliberErrorPresets, kNumCaliberErrorPresets,
                                kCaliberErrorDefaultIndex, g_state.caliber_error_preset_index);

        g_state.uc_config.sigma_mass_grains = std::fabs(g_state.bullet.mass_grains) * mass_mult;
        g_state.uc_config.sigma_length_mm = std::fabs(g_state.bullet.length_mm) * length_mult;
        g_state.uc_config.sigma_caliber_inches =
            std::fabs(g_state.bullet.caliber_inches) * caliber_mult;
    }
    g_state.uc_config.sigma_twist_rate_inches = std::fabs(g_state.bullet.twist_rate_inches) * 0.01f;

    const DOPE_ErrorTable* lrf_table = GetLRFTable(g_state.hw_lrf_index);
    g_state.uc_config.use_range_error_table = (g_state.hw_lrf_index > 0 && lrf_table && lrf_table->points != nullptr);
    g_state.uc_config.range_error_table = g_state.uc_config.use_range_error_table ? *lrf_table : DOPE_ErrorTable{nullptr, 0};

    const DOPE_ErrorTable* tmp_table = GetTempTable(g_state.hw_temp_sensor_index);
    g_state.uc_config.use_temperature_error_table = (g_state.hw_temp_sensor_index > 0 && tmp_table && tmp_table->points != nullptr);
    g_state.uc_config.temperature_error_table = g_state.uc_config.use_temperature_error_table ? *tmp_table : DOPE_ErrorTable{nullptr, 0};

    const DOPE_ErrorTable* baro_table = GetBarometerTable(g_state.hw_barometer_index);
    g_state.uc_config.use_pressure_delta_temp_error_table = (g_state.hw_barometer_index >= 0 && g_state.hw_barometer_index < GetNumBarometerPresets()) && (baro_table && baro_table->points != nullptr);
    g_state.uc_config.pressure_delta_temp_error_table = g_state.uc_config.use_pressure_delta_temp_error_table ? *baro_table : DOPE_ErrorTable{nullptr, 0};
    g_state.uc_config.pressure_uncalibrated_sigma_pa = 50.0f;
    g_state.uc_config.pressure_is_calibrated = g_state.baro_is_calibrated;
    g_state.uc_config.pressure_has_calibration_temp = g_state.baro_has_calibration_temp;
    g_state.uc_config.pressure_calibration_temp_c = g_state.baro_calibration_temp_c;

    if (g_state.uc_config.use_pressure_delta_temp_error_table && !g_state.baro_is_calibrated) {
        g_state.uc_config.sigma_pressure_pa = g_state.uc_config.pressure_uncalibrated_sigma_pa;
    }

    if (g_state.hw_gps_index > 0) {
        g_state.uc_config.sigma_latitude_deg = kGpsLatPresets[g_state.hw_gps_index].sigma_lat_deg;
        g_state.hw_mag_lat_index = 0;
    } else if (g_state.hw_mag_lat_index > 0) {
        g_state.uc_config.sigma_latitude_deg =
            kMagLatPresets[g_state.hw_mag_lat_index].sigma_lat_deg;
    }

    // Sync interpolated sigmas for hardware with error tables
    if (g_state.uc_config.use_range_error_table) {
        g_state.uc_config.sigma_range_m = DOPE_InterpolateSigma(lrf_table, g_state.lrf_range);
    }
    if (g_state.uc_config.use_temperature_error_table) {
        g_state.uc_config.sigma_temperature_c = DOPE_InterpolateSigma(tmp_table, g_state.baro_temp);
    }
    if (g_state.uc_config.use_pressure_delta_temp_error_table && g_state.baro_is_calibrated && g_state.baro_has_calibration_temp) {
        const float delta_temp_c = std::fabs(g_state.baro_temp - g_state.baro_calibration_temp_c);
        g_state.uc_config.sigma_pressure_pa = DOPE_InterpolateSigma(baro_table, delta_temp_c);
    }

    g_state.cartridge_cep_points.erase(
        std::remove_if(g_state.cartridge_cep_points.begin(), g_state.cartridge_cep_points.end(),
                       [](const DOPE_CEPPoint& p) { return p.range_m <= 0.0f || p.cep50_moa <= 0.0f; }),
        g_state.cartridge_cep_points.end());
    std::sort(g_state.cartridge_cep_points.begin(), g_state.cartridge_cep_points.end(),
              [](const DOPE_CEPPoint& a, const DOPE_CEPPoint& b) { return a.range_m < b.range_m; });
    g_state.cartridge_cep_points.erase(
        std::unique(g_state.cartridge_cep_points.begin(), g_state.cartridge_cep_points.end(),
                    [](const DOPE_CEPPoint& a, const DOPE_CEPPoint& b) { return a.range_m == b.range_m; }),
        g_state.cartridge_cep_points.end());

    if (!std::isfinite(g_state.cartridge_cep_scale_floor) || g_state.cartridge_cep_scale_floor <= 0.0f)
        g_state.cartridge_cep_scale_floor = 1.0f;

    g_state.uc_config.cartridge_cep_scale_floor = g_state.cartridge_cep_scale_floor;
    const bool have_cep_table = g_state.uc_config.use_cartridge_cep_table &&
                                !g_state.cartridge_cep_points.empty();
    if (have_cep_table) {
        g_state.uc_config.cartridge_cep_table = {g_state.cartridge_cep_points.data(),
                                                 static_cast<int>(g_state.cartridge_cep_points.size())};
    } else {
        g_state.uc_config.cartridge_cep_table = {nullptr, 0};
        g_state.uc_config.use_cartridge_cep_table = false;
    }
}

void SanitizeCartridgePreset(CartridgePreset& preset) {
    if (preset.name.empty()) {
        preset.name = "Unnamed Cartridge";
    }
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
    auto& cep = preset.cep_table_moa;
    for (auto& p : cep) {
        if (!std::isfinite(p.first) || p.first < 0.0f)
            p.first = 0.0f;
        if (!std::isfinite(p.second) || p.second < 0.0f)
            p.second = 0.0f;
    }
    cep.erase(std::remove_if(cep.begin(), cep.end(), [](const std::pair<float, float>& p) {
                  return p.first <= 0.0f || p.second <= 0.0f;
              }),
              cep.end());
    std::sort(cep.begin(), cep.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    cep.erase(std::unique(cep.begin(), cep.end(),
                          [](const auto& a, const auto& b) { return a.first == b.first; }),
              cep.end());
    if (!std::isfinite(preset.cep_scale_floor) || preset.cep_scale_floor <= 0.0f)
        preset.cep_scale_floor = 1.0f;
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
    preset.caliber_inches = ClampValue(std::fabs(preset.caliber_inches), 0.10f, 1.00f);
    preset.barrel_length_in = ClampValue(preset.barrel_length_in, 2.0f, 40.0f);
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
    if (!preset.cep_table_moa.empty()) {
        item["cep_table_moa"] = json::array();
        for (const auto& p : preset.cep_table_moa) {
            json cp;
            cp["range_m"] = p.first;
            cp["cep50_moa"] = p.second;
            item["cep_table_moa"].push_back(cp);
        }
    }
    item["cep_scale_floor"] = preset.cep_scale_floor;
    item["mass_grains"] = preset.mass_grains;
    item["caliber_inches"] = preset.caliber_inches;
    item["length_mm"] = preset.length_mm;
    if (!preset.tags.empty()) {
        item["tags"] = json::array();
        for (const auto& t : preset.tags) item["tags"].push_back(t);
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
    item["twist_rate_inches"] = preset.twist_rate_inches;
    item["zero_range_m"] = preset.zero_range_m;
    item["sight_height_mm"] = preset.sight_height_mm;
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
    g_reference_mv_ms = g_state.bullet.muzzle_velocity_ms;
    g_mv_adjustment_estimated = false;
    g_active_mv_adj_fps_per_in = 0.0f;
    g_active_barrel_mv_profile.clear();

    g_state.zero.zero_range_m = 91.44f; // 100 yd
    g_state.zero.sight_height_mm = 38.1f;

    g_state.wind_speed_ms = 0.0f;
    g_state.wind_heading = 0.0f;
    g_state.latitude = 37.0f;

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
    g_state.cartridge_cep_points.clear();
    g_state.cartridge_cep_scale_floor = 1.0f;
    g_state.uc_config.use_cartridge_cep_table = false;
    g_state.uc_config.cartridge_cep_table = {nullptr, 0};
    g_state.uc_config.cartridge_cep_scale_floor = 1.0f;
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
}

void ApplyConfig() {
    // Push current GUI inputs into the DOPE engine.
    g_state.bullet.drag_model = kDragModels[g_state.drag_model_index];
    g_state.bullet.bc = g_state.override_drag_coefficient
                            ? ClampValue(g_state.manual_drag_coefficient, 0.001f, 1.20f)
                            : ComputeAutoDragCoefficient(g_state.bullet);
    DOPE_SetBulletProfile(&g_state.bullet);
    DOPE_SetZeroConfig(&g_state.zero);
    DOPE_SetWindManual(g_state.wind_speed_ms, g_state.wind_heading);
    DOPE_SetLatitude(g_state.latitude);
    SyncDerivedInputUncertaintySigmas();
    DOPE_SetUncertaintyConfig(&g_state.uc_config);
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
            "MV",     "BC",      "Range",  "WindSpd", "WindDir",  "Temp",
            "Press",  "Humid",   "SightH", "Cant",    "Latitude", "Mass",
            "Length", "Caliber", "Twist",  "ZeroRng", "MVAdj"};
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

// Tiered fps/in estimator — fallback when cartridge has no explicit value.
static float FallbackMvAdjFpsPerIn(float ref_mv_ms) {
    const float fps = ref_mv_ms * MPS_TO_FPS;
    if (fps < 550.0f)  return 8.0f;    // pistol / subsonic
    if (fps < 760.0f)  return 12.0f;   // pistol / SMG
    if (fps < 915.0f)  return 20.0f;   // intermediate / carbine
    if (fps < 1067.0f) return 28.0f;   // rifle carbine
    return 38.0f;                       // full-power rifle
}

// Resolve MV for the current barrel length.  Three-tier priority:
//   1. g_active_barrel_mv_profile non-empty  → profile table interpolation
//   2. g_active_mv_adj_fps_per_in > 0        → linear fps/in from reference barrel
//   3. fallback estimator                     → tier-based fps/in from ref MV
void ComputeAdjustedMv() {
    const float barrel_in = g_state.bullet.barrel_length_in;
    g_mv_adjustment_estimated = false;

    if (!g_active_barrel_mv_profile.empty()) {
        // Tier 1: profile lookup — MV is fully determined by barrel length.
        const float mv_fps = InterpolateBarrelMvProfile(g_active_barrel_mv_profile, barrel_in);
        g_state.bullet.muzzle_velocity_ms        = mv_fps * FPS_TO_MPS;
        g_state.bullet.reference_barrel_length_in = barrel_in; // no engine delta
        g_state.bullet.mv_adjustment_factor       = 0.0f;
    } else {
        // Tier 2/3: linear adjustment from the cartridge's reference barrel.
        float fps_per_in = g_active_mv_adj_fps_per_in;
        if (fps_per_in <= 0.0f) {
            fps_per_in = FallbackMvAdjFpsPerIn(g_reference_mv_ms);
            g_mv_adjustment_estimated = true;
        }
        // Engine computes delta: actual_mv = muzzle_velocity_ms + (barrel - ref) * factor * FPS_TO_MPS
        g_state.bullet.muzzle_velocity_ms  = g_reference_mv_ms;
        g_state.bullet.mv_adjustment_factor = fps_per_in;
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

    // Start with the reference barrel as both actual and reference length.
    if (normalized.reference_barrel_inches > 0.0f) {
        g_state.bullet.barrel_length_in           = normalized.reference_barrel_inches;
        g_state.bullet.reference_barrel_length_in = normalized.reference_barrel_inches;
    }

    g_state.cartridge_cep_points.clear();
    for (const auto& pt : normalized.cep_table_moa) {
        g_state.cartridge_cep_points.push_back({pt.first, pt.second});
    }
    g_state.cartridge_cep_scale_floor = normalized.cep_scale_floor;
    g_state.uc_config.use_cartridge_cep_table = !g_state.cartridge_cep_points.empty();
    g_state.override_drag_coefficient = true;
    g_state.manual_drag_coefficient   = normalized.bc;

    // Resolve MV for the reference barrel length.
    ComputeAdjustedMv();

    EnsureSelectedGunPresetMatchesCurrentCaliber();
}

void ApplyGunPreset(const GunPreset& preset) {
    GunPreset normalized = preset;
    SanitizeGunPreset(normalized);

    g_state.bullet.barrel_length_in = normalized.barrel_length_in;
    g_state.bullet.twist_rate_inches = normalized.twist_rate_inches;
    g_state.zero.zero_range_m = normalized.zero_range_m;
    g_state.zero.sight_height_mm = normalized.sight_height_mm;
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
    const float ref_mv = (g_reference_mv_ms > 0.0f) ? g_reference_mv_ms
                                                      : g_state.bullet.muzzle_velocity_ms;
    preset.muzzle_velocity_ms = ref_mv;
    preset.reference_barrel_inches = (g_state.bullet.reference_barrel_length_in > 0.0f)
                                         ? g_state.bullet.reference_barrel_length_in
                                         : g_state.bullet.barrel_length_in;
    preset.velocity_profile.clear();
    preset.velocity_profile.emplace_back(0.0f, ref_mv);
    preset.trajectory_profile.clear();
    preset.mass_grains = g_state.bullet.mass_grains;
    preset.caliber_inches = g_state.bullet.caliber_inches;
    preset.length_mm = g_state.bullet.length_mm;
    preset.cep_table_moa.clear();
    for (const auto& pt : g_state.cartridge_cep_points) {
        preset.cep_table_moa.emplace_back(pt.range_m, pt.cep50_moa);
    }
    preset.cep_scale_floor = g_state.cartridge_cep_scale_floor;
    // barrel-MV data from glue-layer globals
    preset.mv_adjustment_fps_per_in = g_active_mv_adj_fps_per_in;
    preset.barrel_mv_profile        = g_active_barrel_mv_profile;
    // attach tags from current UI selections where applicable
    preset.tags.clear();
    if (g_state.new_cartridge_tier_index >= 0 && g_state.new_cartridge_tier_index < kNumBulletTiers)
        preset.tags.push_back(kBulletTierLabels[g_state.new_cartridge_tier_index]);
    if (g_state.new_cartridge_construction_index >= 0 && g_state.new_cartridge_construction_index < kNumBulletConstructions)
        preset.tags.push_back(kBulletConstructionLabels[g_state.new_cartridge_construction_index]);
    if (g_state.new_cartridge_shape_index > 0 && g_state.new_cartridge_shape_index < kNumBulletShapes)
        preset.tags.push_back(kBulletShapeLabels[g_state.new_cartridge_shape_index]);

    SanitizeCartridgePreset(preset);
    return preset;
}

GunPreset CaptureCurrentGunPreset(const std::string& name) {
    GunPreset preset;
    preset.name = name;
    preset.caliber_inches = g_state.bullet.caliber_inches;
    preset.barrel_length_in = g_state.bullet.barrel_length_in;
    preset.twist_rate_inches = g_state.bullet.twist_rate_inches;
    preset.zero_range_m = g_state.zero.zero_range_m;
    preset.sight_height_mm = g_state.zero.sight_height_mm;
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

    std::ofstream out(g_state.profile_library_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        g_state.last_action = "Save Profile Library (failed)";
        g_state.output_text = "Profile library save failed: cannot open file.";
        return;
    }
    out << root.dump(2);
    g_state.last_action = "Save Profile Library";
}

// Save cartridge presets (JSON array) to given path. Returns true on success.
bool SaveCartridgePresetsToFile(const std::string& path) {
    EnsureProfileDefaults();
    json j = json::array();
    for (const auto& p : g_cartridge_presets) {
        j.push_back(SerializeCartridgePreset(p));
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << j.dump(2);
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
                preset.twist_rate_inches = item.value("twist_rate_inches", 10.0f);
                preset.zero_range_m = item.value("zero_range_m", 100.0f);
                preset.sight_height_mm = item.value("sight_height_mm", 38.1f);
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
                preset.twist_rate_inches = item.value("twist_rate_inches", 10.0f);
                preset.zero_range_m = item.value("zero_range_m", 100.0f);
                preset.sight_height_mm = item.value("sight_height_mm", 38.1f);
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
        EnsureSelectedGunPresetMatchesCurrentCaliber();
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
    j["barrel_length_in"] = g_state.bullet.barrel_length_in;
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
    j["uc_enabled"] = g_state.uc_config.enabled;
    j["uc_sigma_mv_ms"] = g_state.uc_config.sigma_muzzle_velocity_ms;
    j["uc_sigma_bc_fraction"] = g_state.uc_config.sigma_bc_fraction;
    j["uc_sigma_range_m"] = g_state.uc_config.sigma_range_m;
    j["uc_sigma_wind_speed_ms"] = g_state.uc_config.sigma_wind_speed_ms;
    j["uc_sigma_wind_heading_deg"] = g_state.uc_config.sigma_wind_heading_deg;
    j["uc_sigma_temperature_c"] = g_state.uc_config.sigma_temperature_c;
    j["uc_sigma_pressure_pa"] = g_state.uc_config.sigma_pressure_pa;
    j["uc_sigma_humidity"] = g_state.uc_config.sigma_humidity;
    j["uc_sigma_sight_height_mm"] = g_state.uc_config.sigma_sight_height_mm;
    j["uc_sigma_cant_deg"] = g_state.uc_config.sigma_cant_deg;
    j["uc_sigma_latitude_deg"] = g_state.uc_config.sigma_latitude_deg;
    j["uc_sigma_mass_grains"] = g_state.uc_config.sigma_mass_grains;
    j["uc_sigma_length_mm"] = g_state.uc_config.sigma_length_mm;
    j["uc_sigma_caliber_inches"] = g_state.uc_config.sigma_caliber_inches;
    j["uc_sigma_twist_inches"] = g_state.uc_config.sigma_twist_rate_inches;
    j["uc_sigma_zero_range_m"] = g_state.uc_config.sigma_zero_range_m;
    j["uc_sigma_mv_adj_fps_per_in"] = g_state.uc_config.sigma_mv_adjustment_fps_per_in;
    j["uc_use_cartridge_cep_table"] = g_state.uc_config.use_cartridge_cep_table;
    j["uc_cartridge_cep_scale_floor"] = g_state.cartridge_cep_scale_floor;
    if (!g_state.cartridge_cep_points.empty()) {
        j["uc_cartridge_cep_table"] = json::array();
        for (const auto& pt : g_state.cartridge_cep_points) {
            json cp;
            cp["range_m"] = pt.range_m;
            cp["cep50_moa"] = pt.cep50_moa;
            j["uc_cartridge_cep_table"].push_back(cp);
        }
    }
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
        g_state.bullet.length_mm = j.value("length_mm", 0.0f);
        g_state.bullet.caliber_inches = j.value("caliber_inches", 0.0f);
        g_state.bullet.twist_rate_inches = j.value("twist_rate_inches", 0.0f);
        g_state.bullet.barrel_length_in = j.value("barrel_length_in", 24.0f);
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

        // Uncertainty config ΓÇö load with per-field defaults matching DOPE defaults
        UncertaintyConfig uc_def = {};
        DOPE_GetDefaultUncertaintyConfig(&uc_def);
        g_state.uc_config.enabled = j.value("uc_enabled", uc_def.enabled);
        g_state.uc_config.sigma_muzzle_velocity_ms =
            j.value("uc_sigma_mv_ms", uc_def.sigma_muzzle_velocity_ms);
        g_state.uc_config.sigma_bc_fraction =
            j.value("uc_sigma_bc_fraction", uc_def.sigma_bc_fraction);
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
        g_state.uc_config.sigma_mass_grains =
            j.value("uc_sigma_mass_grains", uc_def.sigma_mass_grains);
        g_state.uc_config.sigma_length_mm = j.value("uc_sigma_length_mm", uc_def.sigma_length_mm);
        g_state.uc_config.sigma_caliber_inches =
            j.value("uc_sigma_caliber_inches", uc_def.sigma_caliber_inches);
        g_state.uc_config.sigma_twist_rate_inches =
            j.value("uc_sigma_twist_inches", uc_def.sigma_twist_rate_inches);
        g_state.uc_config.sigma_zero_range_m =
            j.value("uc_sigma_zero_range_m", uc_def.sigma_zero_range_m);
        g_state.uc_config.sigma_mv_adjustment_fps_per_in =
            j.value("uc_sigma_mv_adj_fps_per_in", uc_def.sigma_mv_adjustment_fps_per_in);

        g_state.cartridge_cep_points.clear();
        if (j.contains("uc_cartridge_cep_table") && j["uc_cartridge_cep_table"].is_array()) {
            for (const auto& cp : j["uc_cartridge_cep_table"]) {
                const float rng = cp.value("range_m", 0.0f);
                const float cep = cp.value("cep50_moa", 0.0f);
                g_state.cartridge_cep_points.push_back({rng, cep});
            }
        }
        g_state.cartridge_cep_scale_floor = j.value("uc_cartridge_cep_scale_floor", 1.0f);
        g_state.uc_config.use_cartridge_cep_table =
            j.value("uc_use_cartridge_cep_table", !g_state.cartridge_cep_points.empty());

        // Hardware sensor selections (backwards-compatible: 0 = Custom)
        g_state.hw_barometer_index = j.value("hw_barometer_index", kBarometerDefaultIndex);
        g_state.hw_lrf_index = j.value("hw_lrf_index", kLrfDefaultIndex);
        g_state.hw_cant_index = j.value("hw_cant_index", 0);
        g_state.hw_temp_sensor_index = j.value("hw_temp_sensor_index", kTempDefaultIndex);
        g_state.hw_gps_index = j.value("hw_gps_index", kGpsDefaultIndex);
        g_state.hw_mag_lat_index = j.value("hw_mag_lat_index", kMagLatDefaultIndex);
        if (g_state.hw_barometer_index < 0 || g_state.hw_barometer_index >= GetNumBarometerPresets())
            g_state.hw_barometer_index = kBarometerDefaultIndex;
        if (g_state.hw_lrf_index < 0 || g_state.hw_lrf_index >= GetNumLRFPresets())
            g_state.hw_lrf_index = kLrfDefaultIndex;
        if (g_state.hw_cant_index < 0 || g_state.hw_cant_index >= GetNumCantPresets())
            g_state.hw_cant_index = 0;
        if (g_state.hw_temp_sensor_index < 0 || g_state.hw_temp_sensor_index >= GetNumTempSensorPresets())
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
            ImGui::TextUnformatted(
                "Presets are for test-harness convenience and do not alter engine internals.");

            ImGui::InputText("Cartridge Preset Name", g_state.new_cartridge_preset_name,
                             IM_ARRAYSIZE(g_state.new_cartridge_preset_name));
            // Tag selection: Tier, Construction, optional Shape
            ImGui::TextUnformatted("Preset Tags:");
            int tier_idx = g_state.new_cartridge_tier_index;
            if (ImGui::Combo("Tier", &tier_idx, kBulletTierLabels, kNumBulletTiers)) {
                g_state.new_cartridge_tier_index = tier_idx;
            }
            int cons_idx = g_state.new_cartridge_construction_index;
            if (ImGui::Combo("Construction", &cons_idx, kBulletConstructionLabels, kNumBulletConstructions)) {
                g_state.new_cartridge_construction_index = cons_idx;
            }
            int shape_idx = g_state.new_cartridge_shape_index;
            if (ImGui::Combo("Shape", &shape_idx, kBulletShapeLabels, kNumBulletShapes)) {
                g_state.new_cartridge_shape_index = shape_idx;
            }
            if (ImGui::Button("Add/Update Cartridge Preset")) {
                const std::string preset_name = g_state.new_cartridge_preset_name;
                if (!preset_name.empty()) {
                    CartridgePreset preset = CaptureCurrentCartridgePreset(preset_name);
                    // attach tags from UI selection
                    preset.tags.clear();
                    if (g_state.new_cartridge_tier_index >= 0 && g_state.new_cartridge_tier_index < kNumBulletTiers)
                        preset.tags.push_back(kBulletTierLabels[g_state.new_cartridge_tier_index]);
                    if (g_state.new_cartridge_construction_index >= 0 && g_state.new_cartridge_construction_index < kNumBulletConstructions)
                        preset.tags.push_back(kBulletConstructionLabels[g_state.new_cartridge_construction_index]);
                    if (g_state.new_cartridge_shape_index > 0 && g_state.new_cartridge_shape_index < kNumBulletShapes)
                        preset.tags.push_back(kBulletShapeLabels[g_state.new_cartridge_shape_index]);
                    const int existing = FindCartridgePresetByName(preset_name);
                    if (existing >= 0) {
                        g_cartridge_presets[existing] = preset;
                        g_state.selected_cartridge_preset = existing;
                    } else {
                        g_cartridge_presets.push_back(preset);
                        g_state.selected_cartridge_preset =
                            static_cast<int>(g_cartridge_presets.size()) - 1;
                    }
                    g_state.last_action = "Cartridge Preset Saved";
                    // ticker will refresh output at next tick
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Cartridge") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                ApplyCartridgePreset(g_cartridge_presets[g_state.selected_cartridge_preset]);
                g_state.last_action = "Cartridge Preset Applied";
                {
                    std::lock_guard<std::mutex> lk(g_engine_mutex);
                    ApplyConfig();
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Selected Cartridge") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                g_cartridge_presets.erase(g_cartridge_presets.begin() +
                                          g_state.selected_cartridge_preset);
                if (g_cartridge_presets.empty()) {
                    g_state.selected_cartridge_preset = -1;
                } else if (g_state.selected_cartridge_preset >=
                           static_cast<int>(g_cartridge_presets.size())) {
                    g_state.selected_cartridge_preset =
                        static_cast<int>(g_cartridge_presets.size()) - 1;
                }
                g_state.last_action = "Cartridge Preset Removed";
                // ticker will refresh output at next tick
            }

            ImGui::SameLine();
            if (ImGui::Button("Save Presets to File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_cartridges.json";
                const bool ok = SaveCartridgePresetsToFile(repo_path) || SaveCartridgePresetsToFile("dope_gui_cartridges.json");
                g_state.last_action = ok ? "Saved Cartridge Presets" : "Save Cartridge Presets (failed)";
                if (!ok) g_state.output_text = "Failed to save cartridge presets.";
            }
            ImGui::SameLine();
            if (ImGui::Button("Load Presets from File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_cartridges.json";
                const bool ok = LoadCartridgePresetsFromFile(repo_path) || LoadCartridgePresetsFromFile("dope_gui_cartridges.json");
                g_state.last_action = ok ? "Loaded Cartridge Presets" : "Load Cartridge Presets (failed)";
                if (ok) {
                    EnsureProfileDefaults();
                    g_state.selected_cartridge_preset = (g_cartridge_presets.empty() ? -1 : 0);
                } else {
                    g_state.output_text = "Failed to load cartridge presets.";
                }
            }

            if (ImGui::BeginListBox("##cartridge_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g_cartridge_presets.size()); ++i) {
                    const bool is_selected = (g_state.selected_cartridge_preset == i);
                    if (ImGui::Selectable(g_cartridge_presets[i].name.c_str(), is_selected)) {
                        g_state.selected_cartridge_preset = i;
                        // populate tag UI selections from selected preset
                        const CartridgePreset& cp = g_cartridge_presets[i];
                        // reset to defaults
                        g_state.new_cartridge_tier_index = 0;
                        g_state.new_cartridge_construction_index = 0;
                        g_state.new_cartridge_shape_index = 0;
                        for (const auto& t : cp.tags) {
                            for (int ti = 0; ti < kNumBulletTiers; ++ti) {
                                if (t == kBulletTierLabels[ti]) g_state.new_cartridge_tier_index = ti;
                            }
                            for (int ci = 0; ci < kNumBulletConstructions; ++ci) {
                                if (t == kBulletConstructionLabels[ci]) g_state.new_cartridge_construction_index = ci;
                            }
                            for (int si = 1; si < kNumBulletShapes; ++si) {
                                if (t == kBulletShapeLabels[si]) g_state.new_cartridge_shape_index = si;
                            }
                        }
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
            // Editable barrel-MV fields for the selected cartridge preset.
            if (g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                CartridgePreset& selected_cp =
                    g_cartridge_presets[g_state.selected_cartridge_preset];
                ImGui::Separator();
                ImGui::TextDisabled("Cartridge Barrel-MV: %s", selected_cp.name.c_str());
                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::InputFloat("Ref Barrel (in)##cp_ref_barrel",
                                      &selected_cp.reference_barrel_inches, 0.5f, 1.0f, "%.1f")) {
                    selected_cp.reference_barrel_inches =
                        ClampValue(selected_cp.reference_barrel_inches, 1.0f, 40.0f);
                }
                if (!selected_cp.barrel_mv_profile.empty()) {
                    ImGui::TextDisabled("Profile (%d pts — edit JSON to modify):",
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
                ImGui::SetNextItemWidth(uc_on ? avail_mass * 0.45f : -1.0f);
                ImGui::InputFloat("Mass (gr)", &g_state.bullet.mass_grains, 1.0f, 10.0f, "%.2f");
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    const char* mass_labels[8];
                    for (int i = 0; i < kNumMassErrorPresets && i < 8; ++i)
                        mass_labels[i] = kMassErrorPresets[i].name;
                    ImGui::Combo("Mass Type", &g_state.mass_error_preset_index, mass_labels,
                                 kNumMassErrorPresets);
                    ImGui::Text("+/-Mass = %.3f gr", g_state.uc_config.sigma_mass_grains);
                }
            }
            float length_display =
                is_imperial ? (g_state.bullet.length_mm * MM_TO_IN) : g_state.bullet.length_mm;
            {
                const float avail_len = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_len * 0.45f : -1.0f);
                if (ImGui::InputFloat(is_imperial ? "Length (in)" : "Length (mm)", &length_display,
                                      is_imperial ? 0.01f : 0.1f, is_imperial ? 0.1f : 1.0f,
                                      is_imperial ? "%.3f" : "%.2f")) {
                    g_state.bullet.length_mm =
                        is_imperial ? (length_display * IN_TO_MM) : length_display;
                }
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    const char* length_labels[8];
                    for (int i = 0; i < kNumLengthErrorPresets && i < 8; ++i)
                        length_labels[i] = kLengthErrorPresets[i].name;
                    ImGui::Combo("Length Type", &g_state.length_error_preset_index, length_labels,
                                 kNumLengthErrorPresets);
                    const float length_sigma_display =
                        is_imperial ? (g_state.uc_config.sigma_length_mm * MM_TO_IN)
                                    : g_state.uc_config.sigma_length_mm;
                    ImGui::Text(is_imperial ? "+/-Len = %.4f in" : "+/-Len = %.3f mm",
                                length_sigma_display);
                }
            }
            float caliber_display = is_imperial ? g_state.bullet.caliber_inches
                                                : (g_state.bullet.caliber_inches * IN_TO_MM);
            {
                const float avail_cal = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_cal * 0.45f : -1.0f);
                if (ImGui::InputFloat(is_imperial ? "Caliber (in)" : "Caliber (mm)", &caliber_display,
                                      is_imperial ? 0.001f : 0.01f, is_imperial ? 0.01f : 0.1f,
                                      is_imperial ? "%.3f" : "%.2f")) {
                    g_state.bullet.caliber_inches =
                        is_imperial ? caliber_display : (caliber_display * MM_TO_IN);
                }
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    const char* caliber_labels[8];
                    for (int i = 0; i < kNumCaliberErrorPresets && i < 8; ++i)
                        caliber_labels[i] = kCaliberErrorPresets[i].name;
                    ImGui::Combo("Caliber Type", &g_state.caliber_error_preset_index, caliber_labels,
                                 kNumCaliberErrorPresets);
                    const float caliber_sigma_display =
                        is_imperial ? g_state.uc_config.sigma_caliber_inches
                                    : (g_state.uc_config.sigma_caliber_inches * IN_TO_MM);
                    ImGui::Text(is_imperial ? "+/-Cal = %.5f in" : "+/-Cal = %.4f mm",
                                caliber_sigma_display);
                }
            }
        }

        if (ImGui::CollapsingHeader("Gun Presets (GUI-only)")) {
            EnsureSelectedGunPresetMatchesCurrentCaliber();
            ImGui::InputText("Gun Preset Name", g_state.new_gun_preset_name,
                             IM_ARRAYSIZE(g_state.new_gun_preset_name));
            if (ImGui::Button("Add/Update Gun Preset")) {
                const std::string preset_name = g_state.new_gun_preset_name;
                if (!preset_name.empty()) {
                    GunPreset preset = CaptureCurrentGunPreset(preset_name);
                    const int existing = FindGunPresetByName(preset_name);
                    if (existing >= 0) {
                        g_gun_presets[existing] = preset;
                        g_state.selected_gun_preset = existing;
                    } else {
                        g_gun_presets.push_back(preset);
                        g_state.selected_gun_preset = static_cast<int>(g_gun_presets.size()) - 1;
                    }
                    g_state.last_action = "Gun Preset Saved";
                    // ticker will refresh output at next tick
                }
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
            if (ImGui::Button("Remove Selected Gun") && g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
                IsGunPresetCompatibleWithSelectedCaliber(
                    g_gun_presets[g_state.selected_gun_preset])) {
                g_gun_presets.erase(g_gun_presets.begin() + g_state.selected_gun_preset);
                if (g_gun_presets.empty()) {
                    g_state.selected_gun_preset = -1;
                } else if (g_state.selected_gun_preset >= static_cast<int>(g_gun_presets.size())) {
                    g_state.selected_gun_preset = static_cast<int>(g_gun_presets.size()) - 1;
                }
                g_state.last_action = "Gun Preset Removed";
                // ticker will refresh output at next tick
            }

            ImGui::SameLine();
            if (ImGui::Button("Save Presets to File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_guns.json";
                const bool ok = SaveGunPresetsToFile(repo_path) || SaveGunPresetsToFile("dope_gui_guns.json");
                g_state.last_action = ok ? "Saved Gun Presets" : "Save Gun Presets (failed)";
                if (!ok) g_state.output_text = "Failed to save gun presets.";
            }
            ImGui::SameLine();
            if (ImGui::Button("Load Presets from File")) {
                const std::string repo_path = "tools/native_gui/dope_gui_guns.json";
                const bool ok = LoadGunPresetsFromFile(repo_path) || LoadGunPresetsFromFile("dope_gui_guns.json");
                g_state.last_action = ok ? "Loaded Gun Presets" : "Load Gun Presets (failed)";
                if (ok) {
                    EnsureProfileDefaults();
                    EnsureSelectedGunPresetMatchesCurrentCaliber();
                } else {
                    g_state.output_text = "Failed to load gun presets.";
                }
            }

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
            {
                const float avail_tw = ImGui::GetContentRegionAvail().x;
                ImGui::SetNextItemWidth(uc_on ? avail_tw * 0.45f : -1.0f);
                if (ImGui::InputFloat(is_imperial ? "Twist (in/turn)" : "Twist (mm/turn)",
                                      &twist_display, is_imperial ? 0.1f : 1.0f,
                                      is_imperial ? 1.0f : 10.0f, is_imperial ? "%.2f" : "%.1f")) {
                    g_state.bullet.twist_rate_inches =
                        is_imperial ? twist_display : (twist_display * MM_TO_IN);
                }
                if (uc_on) {
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    const float twist_sigma_display =
                        is_imperial ? g_state.uc_config.sigma_twist_rate_inches
                                    : (g_state.uc_config.sigma_twist_rate_inches * IN_TO_MM);
                    ImGui::Text(is_imperial ? "+/-Twist (1%%) = %.2f in" : "+/-Twist (1%%) = %.1f mm",
                                twist_sigma_display);
                }
            }
            if (ImGui::InputFloat("Barrel Length (in)", &g_state.bullet.barrel_length_in, 0.1f, 1.0f,
                                  "%.1f")) {
                g_state.bullet.barrel_length_in = ClampValue(g_state.bullet.barrel_length_in, 2.0f, 40.0f);
                ComputeAdjustedMv();
            }
            {
                const float avail_mva = ImGui::GetContentRegionAvail().x;
                // MV-per-inch is automatically computed from the active cartridge preset.
                // Show read-only; append "(est.)" when the fallback tiered estimator is active.
                const float active_fps_per_in = g_state.bullet.mv_adjustment_factor;
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
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputFloat("+/-MVAdj(fps/in)##mva",
                                          &g_state.uc_config.sigma_mv_adjustment_fps_per_in, 0.0f, 0.0f,
                                          "%.2f")) {
                        if (g_state.uc_config.sigma_mv_adjustment_fps_per_in < 0.0f)
                            g_state.uc_config.sigma_mv_adjustment_fps_per_in = 0.0f;
                    }
                }
            }
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
                g_state.hw_cant_index = 0;
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
        ImGui::Checkbox("Override Drag Coefficient", &g_state.override_drag_coefficient);
        {
            const float avail_bc = ImGui::GetContentRegionAvail().x;
            if (g_state.override_drag_coefficient) {
                ImGui::SetNextItemWidth(uc_on ? avail_bc * 0.45f : -1.0f);
                ImGui::InputFloat("Drag Coefficient (G-model)", &g_state.manual_drag_coefficient,
                                  0.001f, 0.01f, "%.4f");
            } else {
                const float auto_drag = ComputeAutoDragCoefficient(g_state.bullet);
                ImGui::Text("Drag Coefficient (auto): %.4f", auto_drag);
            }
            if (uc_on) {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputFloat("+/-BC##bc_sig", &g_state.uc_config.sigma_bc_fraction, 0.0f, 0.0f,
                                  "%.3f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("1-sigma BC uncertainty as a fraction, e.g. 0.02 = 2%%");
            }
        }
        ImGui::Combo("Drag Model", &g_state.drag_model_index, kDragModelLabels,
                     IM_ARRAYSIZE(kDragModelLabels));
        float muzzle_display = is_imperial ? (g_state.bullet.muzzle_velocity_ms * MPS_TO_FPS)
                                           : g_state.bullet.muzzle_velocity_ms;
        {
            const float avail_mv = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(uc_on ? avail_mv * 0.45f : -1.0f);
            if (ImGui::InputFloat(is_imperial ? "Muzzle Velocity (fps)" : "Muzzle Velocity (m/s)",
                                  &muzzle_display, is_imperial ? 5.0f : 1.0f,
                                  is_imperial ? 50.0f : 10.0f, "%.1f")) {
                g_state.bullet.muzzle_velocity_ms =
                    is_imperial ? (muzzle_display * FPS_TO_MPS) : muzzle_display;
            }
            if (uc_on) {
                ImGui::SameLine();
                float mv_sig = is_imperial
                                   ? (g_state.uc_config.sigma_muzzle_velocity_ms * MPS_TO_FPS)
                                   : g_state.uc_config.sigma_muzzle_velocity_ms;
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputFloat(is_imperial ? "+/-MV(fps)##mv" : "+/-MV(m/s)##mv", &mv_sig,
                                      0.0f, 0.0f, "%.1f")) {
                    g_state.uc_config.sigma_muzzle_velocity_ms =
                        is_imperial ? (mv_sig * FPS_TO_MPS) : mv_sig;
                    if (g_state.uc_config.sigma_muzzle_velocity_ms < 0.0f)
                        g_state.uc_config.sigma_muzzle_velocity_ms = 0.0f;
                }
            }
        }
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

        // Cartridge accuracy: CEP50 (MOA) vs range table and scaling floor
        {
            ImGui::Separator();
            ImGui::TextUnformatted("Cartridge Accuracy (CEP50 scaling)");
            bool cep_enabled = g_state.uc_config.use_cartridge_cep_table;
            if (ImGui::Checkbox("Enable CEP scaling", &cep_enabled)) {
                g_state.uc_config.use_cartridge_cep_table = cep_enabled;
                SyncDerivedInputUncertaintySigmas();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
            if (ImGui::InputFloat("Min scale", &g_state.cartridge_cep_scale_floor, 0.0f, 0.0f,
                                  "%.2f")) {
                if (!std::isfinite(g_state.cartridge_cep_scale_floor) ||
                    g_state.cartridge_cep_scale_floor <= 0.0f) {
                    g_state.cartridge_cep_scale_floor = 1.0f;
                }
                SyncDerivedInputUncertaintySigmas();
            }
            ImGui::TextDisabled("CEP50 radius table (MOA vs range m)");
            for (size_t i = 0; i < g_state.cartridge_cep_points.size(); ++i) {
                auto& pt = g_state.cartridge_cep_points[i];
                ImGui::PushID(static_cast<int>(i));
                float range_val = pt.range_m;
                float cep_val = pt.cep50_moa;
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
                if (ImGui::InputFloat("Range (m)", &range_val, 0.0f, 0.0f, "%.1f")) {
                    pt.range_m = std::fmax(0.0f, range_val);
                    SyncDerivedInputUncertaintySigmas();
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
                if (ImGui::InputFloat("CEP50 (MOA)", &cep_val, 0.0f, 0.0f, "%.3f")) {
                    pt.cep50_moa = std::fmax(0.0f, cep_val);
                    SyncDerivedInputUncertaintySigmas();
                }
                ImGui::SameLine();
                if (ImGui::Button("Remove")) {
                    g_state.cartridge_cep_points.erase(g_state.cartridge_cep_points.begin() +
                                                       static_cast<long long>(i));
                    SyncDerivedInputUncertaintySigmas();
                    ImGui::PopID();
                    break;
                }
                ImGui::PopID();
            }
            static float new_cep_range = 100.0f;
            static float new_cep_moa = 1.0f;
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
            ImGui::InputFloat("Add Range (m)", &new_cep_range, 0.0f, 0.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
            ImGui::InputFloat("Add CEP50 (MOA)", &new_cep_moa, 0.0f, 0.0f, "%.3f");
            ImGui::SameLine();
            if (ImGui::Button("Add CEP Point")) {
                if (new_cep_range > 0.0f && new_cep_moa > 0.0f) {
                    g_state.cartridge_cep_points.push_back({new_cep_range, new_cep_moa});
                    SyncDerivedInputUncertaintySigmas();
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
                if (ImGui::Combo("GPS / GNSS Module", &g_state.hw_gps_index, gps_labels, gps_count)) {
                    if (g_state.hw_gps_index > 0) {
                        if (g_state.hw_gps_index >= 0 && g_state.hw_gps_index < kNumGpsLatPresets) {
                            g_state.uc_config.sigma_latitude_deg =
                                kGpsLatPresets[g_state.hw_gps_index].sigma_lat_deg;
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
            if (g_state.hw_gps_index == 0) {
                const int mag_count = GetNumMagLatPresets();
                const char* mag_labels[16];
                for (int i = 0; i < mag_count && i < 16; ++i)
                    mag_labels[i] = GetMagLatLabel(i);
                if (ImGui::Combo("Magnetometer (lat est.)", &g_state.hw_mag_lat_index, mag_labels,
                                 mag_count)) {
                    if (g_state.hw_mag_lat_index > 0) {
                        if (g_state.hw_mag_lat_index >= 0 && g_state.hw_mag_lat_index < kNumMagLatPresets) {
                            g_state.uc_config.sigma_latitude_deg =
                                kMagLatPresets[g_state.hw_mag_lat_index].sigma_lat_deg;
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
            const bool lat_locked =
                uc_on && (g_state.hw_gps_index > 0 || g_state.hw_mag_lat_index > 0);
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

            if (ImGui::Checkbox("Barometer Calibrated", &g_state.baro_is_calibrated)) {
                if (g_state.baro_is_calibrated && !g_state.baro_has_calibration_temp) {
                    g_state.baro_is_calibrated = false;
                }
                SyncDerivedInputUncertaintySigmas();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Calibration is only valid when an internal calibration "
                                  "temperature is provided.");
            }

            ImGui::SameLine();
            if (ImGui::Checkbox("Has Internal Cal Temp", &g_state.baro_has_calibration_temp)) {
                if (!g_state.baro_has_calibration_temp) {
                    g_state.baro_is_calibrated = false;
                }
                SyncDerivedInputUncertaintySigmas();
            }

            if (g_state.baro_has_calibration_temp) {
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
        const bool tmp_locked = uc_on && (g_state.hw_temp_sensor_index > 0 && tmp_table_lock && tmp_table_lock->points != nullptr);
        const DOPE_ErrorTable* pr_table_lock = GetBarometerTable(g_state.hw_barometer_index);
        const bool pressure_locked = uc_on && (g_state.hw_barometer_index >= 0 && g_state.hw_barometer_index < GetNumBarometerPresets()) && (pr_table_lock && pr_table_lock->points != nullptr);
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

        if (ImGui::CollapsingHeader("Sensor Fault Simulation (Dev)")) {
            ImGui::TextDisabled("Uncheck to simulate a missing/failed sensor.");
            ImGui::Checkbox("IMU Valid", &g_state.imu_valid);
            ImGui::SameLine();
            ImGui::Checkbox("Mag Valid", &g_state.mag_valid);
            ImGui::SameLine();
            ImGui::Checkbox("Baro Valid", &g_state.baro_valid);
            ImGui::Checkbox("Humidity Valid", &g_state.baro_humidity_valid);
            ImGui::SameLine();
            ImGui::Checkbox("LRF Valid", &g_state.lrf_valid);
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
                    if (ImGui::Combo("Cant Sensor", &g_state.hw_cant_index, cant_labels, cant_count)) {
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
        }
        ImGui::SameLine();
        if (ImGui::Button("Step Update")) {
            g_state.last_action = "Step Update";
            std::lock_guard<std::mutex> lk(g_engine_mutex);
            ApplyConfig();
            RunFrameUpdates(1);
            RefreshOutput();
        }
        ImGui::SameLine();
        const bool run_100_busy = g_run_batch_in_flight.load(std::memory_order_relaxed);
        ImGui::BeginDisabled(run_100_busy);
        if (ImGui::Button("Run 100")) {
            // Kick off the heavier batch on a worker so the render loop stays responsive.
            g_run_batch_in_flight.store(true, std::memory_order_relaxed);
            std::thread([] {
                std::lock_guard<std::mutex> lk(g_engine_mutex);
                g_state.last_action = "Run 100";
                ApplyConfig();
                RunFrameUpdates(100);
                RefreshOutput();
                g_run_batch_in_flight.store(false, std::memory_order_relaxed);
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
        const float display_range_m = (sol.range_m > 0.0f) ? sol.range_m : g_state.lrf_range;
        const float display_range_yd = display_range_m * M_TO_YD;
        const float inches_per_moa = 1.047f * (display_range_yd / 100.0f);
        const float inches_per_ring = moa_per_ring * inches_per_moa;
        const float impact_distance_in = impact_distance_moa * inches_per_moa;
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

        ImGui::Text("Scale: %.0f MOA/ring (%.2f in/ring @ %.1f yd)", moa_per_ring, inches_per_ring,
                    display_range_yd);
        ImGui::Text("Offset: Elev %.2f MOA (%.2f in), Wind %.2f MOA (%.2f in)", hold_elevation_moa,
                    elev_offset_in, hold_windage_moa, wind_offset_in);
        ImGui::Text("Wind breakdown:");
        ImGui::Text("  Wind:       %.2f MOA (%.2f in)", sol.wind_only_windage_moa, wind_only_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.wind_only_windage_moa), "[%s]",
                           direction_label(sol.wind_only_windage_moa));
        ImGui::Text("  Earth spin: %.2f MOA (%.2f in)", sol.earth_spin_windage_moa, earth_spin_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.earth_spin_windage_moa), "[%s]",
                           direction_label(sol.earth_spin_windage_moa));
        ImGui::Text("  Offsets:    %.2f MOA (%.2f in)", sol.offsets_windage_moa, offsets_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.offsets_windage_moa), "[%s]",
                           direction_label(sol.offsets_windage_moa));
        ImGui::Text("  Cant:       %.2f MOA (%.2f in)", sol.cant_windage_moa, cant_added_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.cant_windage_moa), "[%s]",
                           direction_label(sol.cant_windage_moa));
        ImGui::Text("Hold from center: %.2f MOA (%.2f in)", impact_distance_moa,
                    impact_distance_in);
        if (sol.uncertainty_valid &&
            (sol.sigma_elevation_moa > 0.001f || sol.sigma_windage_moa > 0.001f)) {
            ImGui::Text("Confidence: analytic 1s/2s/3s ellipse (no shot sampling)");
        }
        ImGui::Separator();

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

        const ImVec2 center(canvas_pos.x + canvas_size.x * 0.5f,
                            canvas_pos.y + canvas_size.y * 0.5f);
        const float bullseye_radius = canvas_size.x * 0.45f;
        const float moa_to_px = bullseye_radius / max_span_moa;

        for (int ring = 1; ring <= TARGET_RING_COUNT; ++ring) {
            const float ring_radius = bullseye_radius * (static_cast<float>(ring) / ring_count);
            draw_list->AddCircle(center, ring_radius, IM_COL32(210, 210, 210, 255), 0, 1.0f);

            char ring_label[64];
            const float ring_moa = moa_per_ring * static_cast<float>(ring);
            const float ring_inches = ring_moa * inches_per_moa;
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
        const ImVec2 impact_point(center.x + hold_windage_moa * moa_to_px,
                                  center.y - hold_elevation_moa * moa_to_px);

        // Analytic Gaussian uncertainty ellipse contours (no shot simulation/sampling).
        if (sol.uncertainty_valid &&
            (sol.sigma_elevation_moa > 0.001f || sol.sigma_windage_moa > 0.001f)) {
            // axis1_moa, axis2_moa, cos_r, sin_r already computed above for auto-scaling.

            constexpr int kSegments = 48;

            // Labeled contour outlines at 1╧â / 2╧â / 3╧â
            const float sigma_levels[] = {1.0f, 2.0f, 3.0f};
            const ImU32 sigma_colors[] = {IM_COL32(255, 200, 80, 230), IM_COL32(180, 120, 255, 180),
                                          IM_COL32(80, 160, 255, 140)};
            for (int si = 0; si < 3; ++si) {
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

        ImGui::End();

        ImGui::Begin("Side View Arc");
        ImDrawList* side_draw_list = ImGui::GetWindowDrawList();

        FiringSolution side_sol = frame_sol;

        float side_range_m = side_sol.horizontal_range_m;
        if (side_range_m <= 0.0f) {
            side_range_m = (side_sol.range_m > 0.0f) ? side_sol.range_m : g_state.lrf_range;
        }
        side_range_m = ClampValue(side_range_m, 1.0f, 5000.0f);

        auto nice_ceil = [](float value) -> float {
            if (value <= 0.0f) {
                return 1.0f;
            }
            const float exponent = std::floor(std::log10(value));
            const float base = std::pow(10.0f, exponent);
            const float normalized = value / base;
            float snapped = 1.0f;
            if (normalized <= 1.0f) {
                snapped = 1.0f;
            } else if (normalized <= 1.25f) {
                snapped = 1.25f;
            } else if (normalized <= 2.0f) {
                snapped = 2.0f;
            } else if (normalized <= 2.5f) {
                snapped = 2.5f;
            } else if (normalized <= 5.0f) {
                snapped = 5.0f;
            } else {
                snapped = 10.0f;
            }
            return snapped * base;
        };

        const float range_m_to_display = is_imperial ? M_TO_YD : 1.0f;
        const float range_display_to_m = is_imperial ? YD_TO_M : 1.0f;
        const float offset_m_to_display = is_imperial ? 39.3700787f : 100.0f;
        const float offset_display_to_m = is_imperial ? IN_TO_MM / 1000.0f : 0.01f;

        const float drop_m = std::fabs(side_sol.hold_elevation_moa) * MOA_TO_RAD * side_range_m;
        const float side_range_display = side_range_m * range_m_to_display;
        const float side_range_scale_display = ClampValue(nice_ceil(side_range_display * 1.15f),
                                                          10.0f, is_imperial ? 6000.0f : 5000.0f);
        const float side_range_step_display = side_range_scale_display / 5.0f;
        const float side_range_scale_m = side_range_scale_display * range_display_to_m;
        const float side_range_step_m = side_range_step_display * range_display_to_m;
        const float drop_display = drop_m * offset_m_to_display;
        const char* drop_units = is_imperial ? "in" : "cm";
        const float range_display = side_range_display;
        const char* range_units = is_imperial ? "yd" : "m";
        const float elevation_angle_rad = side_sol.hold_elevation_moa * MOA_TO_RAD;
        const float elevation_angle_deg = side_sol.hold_elevation_moa / 60.0f;
        const float wind_relative_rad =
            (g_state.wind_heading - side_sol.heading_deg_true) * dope::math::DEG_TO_RAD;
        const float headwind_ms = g_state.wind_speed_ms * std::cos(wind_relative_rad);
        const float crosswind_ms = g_state.wind_speed_ms * std::sin(wind_relative_rad);
        const float wind_ms_to_display = is_imperial ? MPS_TO_MPH : 1.0f;
        const float wind_axis_eps_ms = 0.05f;

        // NOTE: side_drop_scale_m is computed after plot_height is known (below),
        // so that the vertical chart scale can be anchored to guarantee the 6ft
        // shooter figure is always drawn at true physical scale on the chart.

        ImGui::Text("Range: %.1f %s", range_display, range_units);
        ImGui::Text("Estimated drop at target: %.2f %s", drop_display, drop_units);
        ImGui::Text("Required elevation angle: %.4f deg", elevation_angle_deg);
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
        const ImVec2 side_canvas_end(side_canvas_pos.x + side_canvas_size.x,
                                     side_canvas_pos.y + side_canvas_size.y);

        side_draw_list->AddRectFilled(side_canvas_pos, side_canvas_end, IM_COL32(24, 24, 28, 255));
        side_draw_list->AddRect(side_canvas_pos, side_canvas_end, IM_COL32(80, 80, 90, 255));

        const float left_margin = 82.0f; // wider to fit shooter silhouette
        const float right_margin = 20.0f;
        const float top_margin = 20.0f;
        const float bottom_margin = 24.0f;
        const float plot_left = side_canvas_pos.x + left_margin;
        const float plot_right = side_canvas_end.x - right_margin;
        const float plot_top = side_canvas_pos.y + top_margin;
        const float plot_bottom = side_canvas_end.y - bottom_margin;
        const float plot_width = plot_right - plot_left;
        const float plot_height = plot_bottom - plot_top;

        // Compute vertical scale NOW that we know plot_height.
        // Scale is driven by the bullet drop, not by the figure height.
        const float side_drop_min_scale_m = drop_m * 1.5f;
        const float side_drop_scale_m =
            ClampValue(side_drop_min_scale_m, offset_display_to_m, 5000.0f);
        const float side_drop_scale_display = side_drop_scale_m * offset_m_to_display;
        const float side_exaggeration =
            (side_drop_scale_m > 0.001f) ? (side_range_scale_m / side_drop_scale_m) : 1.0f;
        const float drop_scale_display = side_drop_scale_display;

        const float y_min_m = -side_drop_scale_m;
        const float y_max_m = side_drop_scale_m;
        const float y_span_m = (y_max_m - y_min_m > 0.01f) ? (y_max_m - y_min_m) : 0.01f;

        auto map_x = [&](float x_m) -> float {
            return plot_left + (x_m / side_range_scale_m) * plot_width;
        };
        auto map_y = [&](float y_m) -> float {
            const float t = (y_max_m - y_m) / y_span_m;
            return plot_top + t * plot_height;
        };

        for (int i = 0; i <= 5; ++i) {
            const float grid_x_m = side_range_step_m * static_cast<float>(i);
            const float gx = map_x(grid_x_m);
            side_draw_list->AddLine(ImVec2(gx, plot_top), ImVec2(gx, plot_bottom),
                                    IM_COL32(60, 60, 70, 150), 1.0f);

            char tick_label[32];
            const float tick_display = is_imperial ? (grid_x_m * M_TO_YD) : grid_x_m;
            std::snprintf(tick_label, sizeof(tick_label), "%.0f", tick_display);
            side_draw_list->AddText(ImVec2(gx - 8.0f, plot_bottom + 3.0f),
                                    IM_COL32(180, 180, 190, 255), tick_label);
        }

        // Small scale markers (10, 25, 50 yd/m) as requested
        {
            const float m_list[] = {10.0f, 25.0f, 50.0f};
            for (float m_val : m_list) {
                const float m_m = is_imperial ? (m_val * YD_TO_M) : m_val;
                if (m_m < side_range_scale_m * 0.98f && m_m > 0.1f) {
                    const float mx = map_x(m_m);
                    // Avoid drawing if exactly on or very close to a major grid line
                    bool redundant = false;
                    for (int i = 0; i <= 5; ++i) {
                        if (std::fabs(m_m - side_range_step_m * i) < (side_range_step_m * 0.02f)) {
                            redundant = true;
                            break;
                        }
                    }
                    if (!redundant) {
                        // Draw a shorter, dimmer tick + label
                        side_draw_list->AddLine(ImVec2(mx, plot_bottom - 5.0f),
                                                ImVec2(mx, plot_bottom),
                                                IM_COL32(130, 130, 145, 120), 1.0f);
                        char m_label[16];
                        std::snprintf(m_label, sizeof(m_label), "%.0f", m_val);
                        side_draw_list->AddText(ImVec2(mx - 6.0f, plot_bottom + 3.0f),
                                                IM_COL32(130, 130, 145, 180), m_label);
                    }
                }
            }
        }

        const float los_y = map_y(0.0f);
        side_draw_list->AddLine(ImVec2(plot_left, los_y), ImVec2(plot_right, los_y),
                                IM_COL32(170, 170, 180, 220), 1.0f);
        side_draw_list->AddLine(ImVec2(plot_left, plot_top), ImVec2(plot_left, plot_bottom),
                                IM_COL32(120, 120, 130, 180), 1.0f);

        const float half_drop_m = -side_drop_scale_m * 0.5f;
        const float half_drop_y = map_y(half_drop_m);
        side_draw_list->AddLine(ImVec2(plot_left, half_drop_y), ImVec2(plot_right, half_drop_y),
                                IM_COL32(70, 70, 80, 140), 1.0f);

        const float half_rise_m = side_drop_scale_m * 0.5f;
        const float half_rise_y = map_y(half_rise_m);
        side_draw_list->AddLine(ImVec2(plot_left, half_rise_y), ImVec2(plot_right, half_rise_y),
                                IM_COL32(70, 70, 80, 140), 1.0f);

        const int sample_count = 64;
        ImVec2 arc_points[sample_count + 1];

        // Muzzle origin (defined early so shooter silhouette and arcs share it)
        const ImVec2 muzzle_pt(map_x(0.0f), map_y(0.0f));

        // ---- Shooter silhouette ----
        // The body is drawn in the left margin (82 px) immediately to the left of muzzle_pt.
        // Physical scale: 72 in converted to meters and mapped to the chart's Y axis.
        // A tiny minimum draw height is enforced so the figure stays visible at extreme scales;
        // when the physical scale is clamped, the "72in" label is marked "(ref)".
        {
            const float shooter_height_in = 72.0f;
            const float shooter_height_m = shooter_height_in * (IN_TO_MM / 1000.0f);
            const float px_per_m_y = plot_height / (2.0f * side_drop_scale_m);
            const float h_px_true = shooter_height_m * px_per_m_y; // to-scale pixels
            // Clamp: tiny minimum only for visibility; no max cap so larger true scale is
            // preserved.
            const float h_px = std::fmax(h_px_true, 6.0f);
            const bool off_scale = (h_px > (h_px_true + 0.001f));

            // Proportions as fractions of total figure height
            // Origin: shoulder = muzzle height (bore line), everything is relative to that.
            const float shoulder_y = muzzle_pt.y;
            const float hip_down = 0.30f * h_px;                // hip below shoulder
            const float knee_down = 0.52f * h_px;               // knee below shoulder
            const float feet_down = 0.77f * h_px;               // feet (ground) below shoulder
            const float head_r = std::fmax(0.10f * h_px, 4.5f); // always visible
            const float head_cy = shoulder_y - head_r * 1.4f;   // head center above shoulder

            const float hip_y = shoulder_y + hip_down;
            const float knee_y = shoulder_y + knee_down;
            const float feet_y = shoulder_y + feet_down;

            // Body X: shoulder at muzzle_pt.x, body center 22 px left of muzzle
            const float sx = muzzle_pt.x - 22.0f;

            // Leg splay in pixels (scales with figure, min 4px so always visible)
            const float splay = std::fmax(0.08f * h_px, 4.0f);

            const ImU32 sc = IM_COL32(160, 175, 200, 220);
            const float sw = 1.6f;

            // Head
            side_draw_list->AddCircle(ImVec2(sx, head_cy), head_r, sc, 16, sw);
            // Neck + torso (single vertical line from neck to hip)
            side_draw_list->AddLine(ImVec2(sx, head_cy + head_r), ImVec2(sx, hip_y), sc, sw);
            // Upper legs (splayed outward from hip)
            side_draw_list->AddLine(ImVec2(sx, hip_y), ImVec2(sx - splay, knee_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx, hip_y), ImVec2(sx + splay, knee_y), sc, sw);
            // Lower legs (straight down from knees)
            side_draw_list->AddLine(ImVec2(sx - splay, knee_y), ImVec2(sx - splay, feet_y), sc, sw);
            side_draw_list->AddLine(ImVec2(sx + splay, knee_y), ImVec2(sx + splay, feet_y), sc, sw);
            // Feet (short horizontal ticks)
            side_draw_list->AddLine(ImVec2(sx - splay, feet_y), ImVec2(sx - splay - 5.0f, feet_y),
                                    sc, sw);
            side_draw_list->AddLine(ImVec2(sx + splay, feet_y), ImVec2(sx + splay + 5.0f, feet_y),
                                    sc, sw);

            // Arms: both arms raised forward holding the rifle (reaching toward muzzle)
            // Forward arm: from shoulder (sx, shoulder_y) to muzzle_pt
            side_draw_list->AddLine(ImVec2(sx, shoulder_y), muzzle_pt, IM_COL32(175, 188, 210, 220),
                                    sw);
            // Rear arm: from body center, angled back and slightly down (holding stock)
            side_draw_list->AddLine(ImVec2(sx, shoulder_y),
                                    ImVec2(sx - 12.0f, shoulder_y + 0.07f * h_px), sc, sw);

            // "6ft" scale brace ΓÇö vertical line left of body with ticks
            const float brace_x = sx - splay - 7.0f;
            const float top_y = head_cy - head_r;
            side_draw_list->AddLine(ImVec2(brace_x, top_y), ImVec2(brace_x, feet_y),
                                    IM_COL32(110, 125, 150, 140), 1.0f);
            side_draw_list->AddLine(ImVec2(brace_x, top_y), ImVec2(brace_x + 3.0f, top_y),
                                    IM_COL32(110, 125, 150, 140), 1.0f);
            side_draw_list->AddLine(ImVec2(brace_x, feet_y), ImVec2(brace_x + 3.0f, feet_y),
                                    IM_COL32(110, 125, 150, 140), 1.0f);
            side_draw_list->AddText(ImVec2(brace_x - 28.0f, top_y + (feet_y - top_y) * 0.5f - 5.0f),
                                    IM_COL32(130, 140, 165, 210),
                                    off_scale ? "72in\n(ref)" : "72in");
        }

        // Always draw both: aim line in bore direction (dim amber) + bullet parabola (bright cyan).
        // Aim line: required elevation direction, i.e. where the bore is pointed.
        {
            const ImVec2 aim_start(map_x(0.0f), map_y(0.0f));
            const ImVec2 aim_end(map_x(side_range_m), map_y(drop_m));
            side_draw_list->AddLine(aim_start, aim_end, IM_COL32(255, 190, 90, 160), 1.5f);
        }
        // Bullet parabolic path (bright cyan)
        {
            const float tan_theta_raw = std::tanf(elevation_angle_rad);
            const float tan_theta = std::isfinite(tan_theta_raw) ? tan_theta_raw : 0.0f;
            for (int i = 0; i <= sample_count; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(sample_count);
                const float x_m = t * side_range_m;
                // Keep the side-view arc moderately rounded so most of the downrange
                // length is visually used while preserving muzzle/impact endpoints.
                const float curvature_weight = 1.30f - (0.30f * t); // 1.30 -> 1.00
                float y_m =
                    (tan_theta * x_m) - ((tan_theta / side_range_m) * x_m * x_m * curvature_weight);
                if (!std::isfinite(y_m)) {
                    y_m = 0.0f;
                }
                arc_points[i] = ImVec2(map_x(x_m), map_y(y_m));
            }
            side_draw_list->AddPolyline(arc_points, sample_count + 1, IM_COL32(98, 203, 255, 255),
                                        ImDrawFlags_None, 2.0f);
        }

        const ImVec2 impact_pt(map_x(side_range_m), map_y(0.0f));
        const ImVec2 aim_pt(map_x(side_range_m), map_y(drop_m));
        side_draw_list->AddCircleFilled(muzzle_pt, 3.5f, IM_COL32(130, 255, 130, 255));
        side_draw_list->AddCircleFilled(impact_pt, 4.0f, IM_COL32(255, 90, 90, 255));
        side_draw_list->AddCircleFilled(aim_pt, 3.0f, IM_COL32(255, 190, 90, 200));
        side_draw_list->AddText(ImVec2(muzzle_pt.x + 6.0f, muzzle_pt.y - 16.0f),
                                IM_COL32(200, 230, 200, 255), "Muzzle");
        side_draw_list->AddText(ImVec2(impact_pt.x - 50.0f, impact_pt.y + 4.0f),
                                IM_COL32(230, 200, 200, 255), "Impact");
        side_draw_list->AddText(ImVec2(aim_pt.x - 48.0f, aim_pt.y - 16.0f),
                                IM_COL32(255, 200, 120, 200), "Aim pt");

        // ---- Elevation angle indicator: small overlay box (always readable) ----
        {
            auto draw_arrow = [&](const ImVec2& from, const ImVec2& to, ImU32 color,
                                  float thickness) {
                side_draw_list->AddLine(from, to, color, thickness);
                const float dx = to.x - from.x;
                const float dy = to.y - from.y;
                const float len = std::sqrtf((dx * dx) + (dy * dy));
                if (len <= 0.001f)
                    return;
                const float ux = dx / len;
                const float uy = dy / len;
                const float px = -uy;
                const float py = ux;
                const float head_len = 4.0f;
                const float head_w = 2.5f;
                const ImVec2 p1(to.x - (ux * head_len) + (px * head_w),
                                to.y - (uy * head_len) + (py * head_w));
                const ImVec2 p2(to.x - (ux * head_len) - (px * head_w),
                                to.y - (uy * head_len) - (py * head_w));
                side_draw_list->AddTriangleFilled(to, p1, p2, color);
            };

            // Draw a compact angle-arc indicator + text box in the bottom-left of the plot,
            // clear of the trajectory lines.
            const float ind_cx = plot_left + 36.0f;
            const float ind_cy = plot_bottom - 28.0f;
            const float arc_r = 20.0f;
            const float ref_len = 30.0f;

            // Horizontal LOS reference
            side_draw_list->AddLine(ImVec2(ind_cx, ind_cy), ImVec2(ind_cx + ref_len, ind_cy),
                                    IM_COL32(150, 150, 170, 180), 1.0f);

            // Wind direction reference (side view axis): show only active direction along bore
            // axis.
            const float wind_y = ind_cy - arc_r - 10.0f;
            const float headwind_display = std::fabs(headwind_ms) * wind_ms_to_display;
            char side_wind_axis_label[32];
            if (headwind_ms > wind_axis_eps_ms) {
                draw_arrow(ImVec2(ind_cx + 20.0f, wind_y), ImVec2(ind_cx + 6.0f, wind_y),
                           IM_COL32(255, 175, 120, 220), 1.2f);
                std::snprintf(side_wind_axis_label, sizeof(side_wind_axis_label), "%.1f",
                              headwind_display);
                side_draw_list->AddText(ImVec2(ind_cx + 22.0f, wind_y - 6.0f),
                                        IM_COL32(255, 175, 120, 220), side_wind_axis_label);
            } else if (headwind_ms < -wind_axis_eps_ms) {
                draw_arrow(ImVec2(ind_cx + 6.0f, wind_y), ImVec2(ind_cx + 20.0f, wind_y),
                           IM_COL32(120, 230, 255, 220), 1.2f);
                std::snprintf(side_wind_axis_label, sizeof(side_wind_axis_label), "%.1f",
                              headwind_display);
                side_draw_list->AddText(ImVec2(ind_cx + 22.0f, wind_y - 6.0f),
                                        IM_COL32(120, 230, 255, 220), side_wind_axis_label);
            } else {
                std::snprintf(side_wind_axis_label, sizeof(side_wind_axis_label), "%.1f",
                              headwind_display);
                side_draw_list->AddText(ImVec2(ind_cx + 22.0f, wind_y - 6.0f),
                                        IM_COL32(170, 190, 210, 220), side_wind_axis_label);
            }

            // Bore direction in screen-space
            const float step_m = side_range_scale_m * 0.001f;
            const float bore_dx = map_x(step_m) - map_x(0.0f);
            const float bore_dy = map_y(std::tanf(elevation_angle_rad) * step_m) - map_y(0.0f);
            const float bore_len = std::sqrtf(bore_dx * bore_dx + bore_dy * bore_dy);
            if (bore_len > 0.01f) {
                const float bx = bore_dx / bore_len;
                const float by = bore_dy / bore_len;
                side_draw_list->AddLine(ImVec2(ind_cx, ind_cy),
                                        ImVec2(ind_cx + bx * ref_len, ind_cy + by * ref_len),
                                        IM_COL32(255, 220, 60, 230), 1.5f);

                // Arc from horizontal to bore
                const float screen_angle = std::atan2f(by, bx);
                const int arc_segs = 14;
                for (int ai = 0; ai < arc_segs; ++ai) {
                    const float a0 = (static_cast<float>(ai) / arc_segs) * screen_angle;
                    const float a1 = (static_cast<float>(ai + 1) / arc_segs) * screen_angle;
                    side_draw_list->AddLine(
                        ImVec2(ind_cx + std::cosf(a0) * arc_r, ind_cy + std::sinf(a0) * arc_r),
                        ImVec2(ind_cx + std::cosf(a1) * arc_r, ind_cy + std::sinf(a1) * arc_r),
                        IM_COL32(255, 220, 60, 150), 1.0f);
                }
            }

            // Angle text in a dark box, below the indicator
            char elev_label[64];
            std::snprintf(elev_label, sizeof(elev_label), "Elev: %.4f\xc2\xb0",
                          elevation_angle_deg);
            const float lx = plot_left + 4.0f;
            const float ly = plot_bottom - 14.0f;
            side_draw_list->AddRectFilled(ImVec2(lx - 2.0f, ly - 2.0f),
                                          ImVec2(lx + 130.0f, ly + 12.0f),
                                          IM_COL32(10, 10, 14, 210));
            side_draw_list->AddText(ImVec2(lx, ly), IM_COL32(255, 235, 100, 255), elev_label);
        }

        side_draw_list->AddText(ImVec2(plot_right - 90.0f, plot_bottom + 4.0f),
                                IM_COL32(190, 190, 200, 255), range_units);

        ImGui::Text("Graph scale: 0..%.0f %s range, +/-%.1f %s vertical (X zoomed %.0fx vs Y)",
                    side_range_scale_display, range_units, drop_scale_display, drop_units,
                    side_exaggeration);

        char los_label[64];
        std::snprintf(los_label, sizeof(los_label), "Line of sight (0 %s)", drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, los_y - 16.0f),
                                IM_COL32(170, 190, 220, 255), los_label);

        char drop_label[64];
        std::snprintf(drop_label, sizeof(drop_label), "Drop @ target: %.2f %s", drop_display,
                      drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, plot_top + 2.0f),
                                IM_COL32(190, 190, 200, 255), drop_label);

        char half_drop_label[64];
        const float half_drop_display = (-half_drop_m) * offset_m_to_display;
        std::snprintf(half_drop_label, sizeof(half_drop_label), "-%.1f %s", half_drop_display,
                      drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, half_drop_y - 14.0f),
                                IM_COL32(150, 150, 165, 255), half_drop_label);

        char half_rise_label[64];
        const float half_rise_display = half_rise_m * offset_m_to_display;
        std::snprintf(half_rise_label, sizeof(half_rise_label), "+%.1f %s", half_rise_display,
                      drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, half_rise_y - 14.0f),
                                IM_COL32(150, 150, 165, 255), half_rise_label);

        ImGui::End();

        ImGui::Begin("Top Down Drift");
        ImDrawList* drift_draw_list = ImGui::GetWindowDrawList();

        FiringSolution drift_sol = frame_sol;

        float drift_range_m = drift_sol.horizontal_range_m;
        if (drift_range_m <= 0.0f) {
            drift_range_m = (drift_sol.range_m > 0.0f) ? drift_sol.range_m : g_state.lrf_range;
        }
        drift_range_m = ClampValue(drift_range_m, 1.0f, 5000.0f);

        const float lateral_m = drift_sol.hold_windage_moa * MOA_TO_RAD * drift_range_m;
        const float drift_range_display = drift_range_m * range_m_to_display;
        const float drift_range_target_display =
            ClampValue(std::fmax(drift_range_display * 1.03f, drift_range_display + 1.0f), 10.0f,
                       is_imperial ? 6000.0f : 5000.0f);
        const float drift_range_step_display = ClampValue(
            nice_ceil(drift_range_target_display / 5.0f), 1.0f, is_imperial ? 1200.0f : 1000.0f);
        const float drift_range_scale_display = drift_range_step_display * 5.0f;
        const float drift_range_scale_m = drift_range_scale_display * range_display_to_m;
        const float drift_range_step_m = drift_range_step_display * range_display_to_m;

        const float offset_display = lateral_m * offset_m_to_display;
        // Keep lateral scale tight enough that strong windage shots remain readable,
        // while preserving a minimum scale for tiny corrections.
        const float drift_lateral_min_scale_m = drift_range_scale_m * 0.01f;
        const float drift_lateral_scale_m =
            ClampValue(std::fmax(std::fabs(lateral_m) * 1.05f, drift_lateral_min_scale_m),
                       offset_display_to_m, 5000.0f);
        const float drift_scale_display = drift_lateral_scale_m * offset_m_to_display;
        const float drift_exaggeration =
            (drift_lateral_scale_m > 0.001f) ? (drift_range_scale_m / drift_lateral_scale_m) : 1.0f;
        const float windage_angle_deg = drift_sol.hold_windage_moa / 60.0f;
        const char* offset_units = is_imperial ? "in" : "cm";
        const char* offset_direction =
            (offset_display > 0.01f) ? "RIGHT" : ((offset_display < -0.01f) ? "LEFT" : "CENTER");

        ImGui::Text("Required aim offset: %.2f %s [%s]", std::fabs(offset_display), offset_units,
                    offset_direction);
        ImGui::Text("Graph scale: 0..%.0f %s range, +/-%.1f %s lateral (X zoomed %.0fx vs Y)",
                    drift_range_scale_display, range_units, drift_scale_display, offset_units,
                    drift_exaggeration);
        ImGui::Text("Required windage angle: %.4f deg", windage_angle_deg);
        ImGui::Separator();

        ImVec2 drift_avail = ImGui::GetContentRegionAvail();
        float drift_canvas_w = drift_avail.x;
        if (drift_canvas_w < 260.0f)
            drift_canvas_w = 260.0f;
        if (drift_canvas_w > 900.0f)
            drift_canvas_w = 900.0f;
        float drift_canvas_h = drift_avail.y;
        if (drift_canvas_h < 180.0f)
            drift_canvas_h = 180.0f;
        if (drift_canvas_h > 380.0f)
            drift_canvas_h = 380.0f;

        const ImVec2 drift_canvas_size(drift_canvas_w, drift_canvas_h);
        const ImVec2 drift_canvas_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##top_down_drift_canvas", drift_canvas_size);
        const ImVec2 drift_canvas_end(drift_canvas_pos.x + drift_canvas_size.x,
                                      drift_canvas_pos.y + drift_canvas_size.y);

        drift_draw_list->AddRectFilled(drift_canvas_pos, drift_canvas_end,
                                       IM_COL32(24, 24, 28, 255));
        drift_draw_list->AddRect(drift_canvas_pos, drift_canvas_end, IM_COL32(80, 80, 90, 255));

        const float drift_left_margin = 28.0f;
        const float drift_right_margin = 20.0f;
        const float drift_top_margin = 10.0f;
        const float drift_bottom_margin = 12.0f;
        const float drift_plot_left = drift_canvas_pos.x + drift_left_margin;
        const float drift_plot_right = drift_canvas_end.x - drift_right_margin;
        const float drift_plot_top = drift_canvas_pos.y + drift_top_margin;
        const float drift_plot_bottom = drift_canvas_end.y - drift_bottom_margin;
        const float drift_plot_width = drift_plot_right - drift_plot_left;
        const float drift_plot_height = drift_plot_bottom - drift_plot_top;
        const float drift_center_x = drift_plot_left + (drift_plot_width * 0.5f);

        auto map_drift_x = [&](float lateral_offset_m) -> float {
            return drift_center_x +
                   (lateral_offset_m / drift_lateral_scale_m) * (drift_plot_width * 0.5f);
        };
        auto map_drift_y = [&](float forward_range_m) -> float {
            return drift_plot_bottom - (forward_range_m / drift_range_scale_m) * drift_plot_height;
        };

        for (int i = 0; i <= 5; ++i) {
            const float grid_x_m = drift_range_step_m * static_cast<float>(i);
            const float gy = map_drift_y(grid_x_m);
            drift_draw_list->AddLine(ImVec2(drift_plot_left, gy), ImVec2(drift_plot_right, gy),
                                     IM_COL32(60, 60, 70, 150), 1.0f);

            char tick_label[32];
            const float tick_display = grid_x_m * range_m_to_display;
            std::snprintf(tick_label, sizeof(tick_label), "%.0f", tick_display);
            drift_draw_list->AddText(ImVec2(drift_plot_left + 3.0f, gy - 8.0f),
                                     IM_COL32(180, 180, 190, 255), tick_label);
        }

        // Small scale markers (10, 25, 50 yd/m) as requested
        {
            const float m_list[] = {10.0f, 25.0f, 50.0f};
            for (float m_val : m_list) {
                const float m_m = is_imperial ? (m_val * YD_TO_M) : m_val;
                if (m_m < drift_range_scale_m * 0.98f && m_m > 0.1f) {
                    const float gy = map_drift_y(m_m);
                    // Avoid drawing if exactly on or very close to a major grid line
                    bool redundant = false;
                    for (int i = 0; i <= 5; ++i) {
                        if (std::fabs(m_m - drift_range_step_m * i) <
                            (drift_range_step_m * 0.02f)) {
                            redundant = true;
                            break;
                        }
                    }
                    if (!redundant) {
                        // Draw a shorter, dimmer tick + label
                        drift_draw_list->AddLine(ImVec2(drift_plot_left, gy),
                                                 ImVec2(drift_plot_left + 5.0f, gy),
                                                 IM_COL32(130, 130, 145, 120), 1.0f);
                        char m_label[16];
                        std::snprintf(m_label, sizeof(m_label), "%.0f", m_val);
                        drift_draw_list->AddText(ImVec2(drift_plot_left + 3.0f, gy - 8.0f),
                                                 IM_COL32(130, 130, 145, 180), m_label);
                    }
                }
            }
        }

        const float drift_right_tick_x = map_drift_x(drift_lateral_scale_m * 0.5f);
        const float drift_left_tick_x = map_drift_x(-drift_lateral_scale_m * 0.5f);
        drift_draw_list->AddLine(ImVec2(drift_center_x, drift_plot_top),
                                 ImVec2(drift_center_x, drift_plot_bottom),
                                 IM_COL32(170, 170, 180, 220), 1.0f);
        drift_draw_list->AddLine(ImVec2(drift_right_tick_x, drift_plot_top),
                                 ImVec2(drift_right_tick_x, drift_plot_bottom),
                                 IM_COL32(70, 70, 80, 130), 1.0f);
        drift_draw_list->AddLine(ImVec2(drift_left_tick_x, drift_plot_top),
                                 ImVec2(drift_left_tick_x, drift_plot_bottom),
                                 IM_COL32(70, 70, 80, 130), 1.0f);
        drift_draw_list->AddLine(ImVec2(drift_plot_left, drift_plot_bottom),
                                 ImVec2(drift_plot_right, drift_plot_bottom),
                                 IM_COL32(120, 120, 130, 180), 1.0f);

        ImVec2 drift_points[sample_count + 1];

        // Aim direction line (dim cyan): straight line from muzzle in the barrel's aimed direction.
        // This is where you physically point the weapon to compensate for drift.
        {
            const ImVec2 aim_start(map_drift_x(0.0f), map_drift_y(0.0f));
            const ImVec2 aim_end(map_drift_x(lateral_m), map_drift_y(drift_range_m));
            drift_draw_list->AddLine(aim_start, aim_end, IM_COL32(90, 220, 255, 170), 1.5f);
        }

        // Bullet path (bright amber): starts tangent to the aim direction, drift curves it
        // back to the target center.
        //
        // Base profile t*(1-t) can look too flat on strong windage shots once projected.
        // Add a bounded shape term that increases with windage angle while preserving:
        //   t=0 => at muzzle, t=1 => at target center, and tangent at muzzle.
        // lateral(t) = lateral_m * t * (1 - t) * (1 + k*t), k in [0, 1.0].
        //   t=0 => at muzzle (0 offset), tangent matches aim direction
        //   t=1 => at target range (0 offset), i.e. bullet lands on center.
        const float drift_curve_k = ClampValue(std::fabs(windage_angle_deg) * 8.0f, 0.0f, 1.0f);
        for (int i = 0; i <= sample_count; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(sample_count);
            const float forward_range_m = t * drift_range_m;
            const float lateral_shape = t * (1.0f - t) * (1.0f + drift_curve_k * t);
            const float lateral_offset_m = lateral_m * lateral_shape;
            drift_points[i] = ImVec2(map_drift_x(lateral_offset_m), map_drift_y(forward_range_m));
        }
        drift_draw_list->AddPolyline(drift_points, sample_count + 1, IM_COL32(255, 180, 90, 255),
                                     ImDrawFlags_None, 2.0f);

        const ImVec2 drift_muzzle_pt(map_drift_x(0.0f), map_drift_y(0.0f));
        const ImVec2 drift_impact_pt(map_drift_x(0.0f),
                                     map_drift_y(drift_range_m)); // bullet lands at center
        const ImVec2 drift_aim_pt(map_drift_x(lateral_m),
                                  map_drift_y(drift_range_m)); // barrel aim point
        drift_draw_list->AddCircleFilled(drift_muzzle_pt, 3.5f, IM_COL32(130, 255, 130, 255));
        drift_draw_list->AddCircleFilled(drift_impact_pt, 4.0f, IM_COL32(255, 90, 90, 255));
        drift_draw_list->AddCircleFilled(drift_aim_pt, 3.0f, IM_COL32(90, 220, 255, 180));
        drift_draw_list->AddText(ImVec2(drift_muzzle_pt.x + 6.0f, drift_muzzle_pt.y + 2.0f),
                                 IM_COL32(200, 230, 200, 255), "Muzzle");
        drift_draw_list->AddText(ImVec2(drift_impact_pt.x + 5.0f, drift_impact_pt.y - 14.0f),
                                 IM_COL32(230, 200, 200, 255), "Impact (target)");
        drift_draw_list->AddText(ImVec2(drift_aim_pt.x - 58.0f, drift_aim_pt.y - 14.0f),
                                 IM_COL32(130, 220, 255, 200), "Aim dir");

        // ---- Windage angle indicator: compact overlay box (always readable) ----
        {
            auto draw_arrow = [&](const ImVec2& from, const ImVec2& to, ImU32 color,
                                  float thickness) {
                drift_draw_list->AddLine(from, to, color, thickness);
                const float dx = to.x - from.x;
                const float dy = to.y - from.y;
                const float len = std::sqrtf((dx * dx) + (dy * dy));
                if (len <= 0.001f)
                    return;
                const float ux = dx / len;
                const float uy = dy / len;
                const float px = -uy;
                const float py = ux;
                const float head_len = 4.0f;
                const float head_w = 2.5f;
                const ImVec2 p1(to.x - (ux * head_len) + (px * head_w),
                                to.y - (uy * head_len) + (py * head_w));
                const ImVec2 p2(to.x - (ux * head_len) - (px * head_w),
                                to.y - (uy * head_len) - (py * head_w));
                drift_draw_list->AddTriangleFilled(to, p1, p2, color);
            };

            // Placed in the bottom-right of the plot, clear of the trajectory.
            const float ind_cx = drift_plot_right - 60.0f;
            const float ind_cy = drift_plot_bottom - 28.0f;
            const float arc_r = 20.0f;
            const float ref_len = 28.0f;

            // Straight-ahead reference (upward in chart)
            drift_draw_list->AddLine(ImVec2(ind_cx, ind_cy), ImVec2(ind_cx, ind_cy - ref_len),
                                     IM_COL32(150, 150, 170, 180), 1.0f);

            // Wind component arrows (top-down): one per axis, moved to the top-right corner.
            const ImVec2 axis_origin(drift_plot_right - 72.0f, drift_plot_top + 26.0f);
            const float crosswind_display = std::fabs(crosswind_ms) * wind_ms_to_display;
            char wind_long_label[32];
            char wind_cross_label[32];

            if (headwind_ms > wind_axis_eps_ms) {
                draw_arrow(ImVec2(axis_origin.x, axis_origin.y - 8.0f),
                           ImVec2(axis_origin.x, axis_origin.y + 8.0f),
                           IM_COL32(255, 175, 120, 220), 1.2f);
                std::snprintf(wind_long_label, sizeof(wind_long_label), "%.1f",
                              std::fabs(headwind_ms) * wind_ms_to_display);
                drift_draw_list->AddText(ImVec2(axis_origin.x + 6.0f, axis_origin.y + 4.0f),
                                         IM_COL32(255, 175, 120, 220), wind_long_label);
            } else if (headwind_ms < -wind_axis_eps_ms) {
                draw_arrow(ImVec2(axis_origin.x, axis_origin.y + 8.0f),
                           ImVec2(axis_origin.x, axis_origin.y - 8.0f),
                           IM_COL32(120, 230, 255, 220), 1.2f);
                std::snprintf(wind_long_label, sizeof(wind_long_label), "%.1f",
                              std::fabs(headwind_ms) * wind_ms_to_display);
                drift_draw_list->AddText(ImVec2(axis_origin.x + 6.0f, axis_origin.y - 20.0f),
                                         IM_COL32(120, 230, 255, 220), wind_long_label);
            } else {
                std::snprintf(wind_long_label, sizeof(wind_long_label), "0.0");
                drift_draw_list->AddText(ImVec2(axis_origin.x + 6.0f, axis_origin.y - 8.0f),
                                         IM_COL32(170, 190, 210, 220), wind_long_label);
            }

            if (crosswind_ms > wind_axis_eps_ms) {
                draw_arrow(ImVec2(axis_origin.x + 10.0f, axis_origin.y + 20.0f),
                           ImVec2(axis_origin.x - 10.0f, axis_origin.y + 20.0f),
                           IM_COL32(180, 215, 255, 220), 1.2f);
                std::snprintf(wind_cross_label, sizeof(wind_cross_label), "%.1f",
                              crosswind_display);
                drift_draw_list->AddText(ImVec2(axis_origin.x - 30.0f, axis_origin.y + 24.0f),
                                         IM_COL32(180, 215, 255, 220), wind_cross_label);
            } else if (crosswind_ms < -wind_axis_eps_ms) {
                draw_arrow(ImVec2(axis_origin.x - 10.0f, axis_origin.y + 20.0f),
                           ImVec2(axis_origin.x + 10.0f, axis_origin.y + 20.0f),
                           IM_COL32(180, 215, 255, 220), 1.2f);
                std::snprintf(wind_cross_label, sizeof(wind_cross_label), "%.1f",
                              crosswind_display);
                drift_draw_list->AddText(ImVec2(axis_origin.x + 14.0f, axis_origin.y + 24.0f),
                                         IM_COL32(180, 215, 255, 220), wind_cross_label);
            } else {
                std::snprintf(wind_cross_label, sizeof(wind_cross_label), "0.0");
                drift_draw_list->AddText(ImVec2(axis_origin.x - 8.0f, axis_origin.y + 24.0f),
                                         IM_COL32(170, 190, 210, 220), wind_cross_label);
            }

            // Aim direction in screen-space
            const float aim_dx = map_drift_x(lateral_m * 0.001f) - map_drift_x(0.0f);
            const float aim_dy = map_drift_y(drift_range_scale_m * 0.001f) - map_drift_y(0.0f);
            const float aim_len2 = std::sqrtf(aim_dx * aim_dx + aim_dy * aim_dy);
            if (aim_len2 > 0.01f) {
                const float ax2 = aim_dx / aim_len2;
                const float ay2 = aim_dy / aim_len2;
                drift_draw_list->AddLine(ImVec2(ind_cx, ind_cy),
                                         ImVec2(ind_cx + ax2 * ref_len, ind_cy + ay2 * ref_len),
                                         IM_COL32(90, 220, 255, 220), 1.5f);

                const float los_angle2 = -3.14159265f * 0.5f;
                const float aim_angle2 = std::atan2f(ay2, ax2);
                const int arc_segs2 = 14;
                for (int ai = 0; ai < arc_segs2; ++ai) {
                    const float a0 = los_angle2 + (static_cast<float>(ai) / arc_segs2) *
                                                      (aim_angle2 - los_angle2);
                    const float a1 = los_angle2 + (static_cast<float>(ai + 1) / arc_segs2) *
                                                      (aim_angle2 - los_angle2);
                    drift_draw_list->AddLine(
                        ImVec2(ind_cx + std::cosf(a0) * arc_r, ind_cy + std::sinf(a0) * arc_r),
                        ImVec2(ind_cx + std::cosf(a1) * arc_r, ind_cy + std::sinf(a1) * arc_r),
                        IM_COL32(90, 220, 255, 140), 1.0f);
                }
            }

            // Angle text in a dark box, below the indicator (mirrors side-view behavior).
            char wind_label[64];
            std::snprintf(wind_label, sizeof(wind_label), "%.4f\xc2\xb0", windage_angle_deg);
            const float wx = ind_cx - 54.0f;
            const float wy = ind_cy + 8.0f;
            drift_draw_list->AddRectFilled(ImVec2(wx - 2.0f, wy - 2.0f),
                                           ImVec2(wx + 122.0f, wy + 12.0f),
                                           IM_COL32(10, 10, 14, 210));
            drift_draw_list->AddText(ImVec2(wx, wy), IM_COL32(110, 235, 255, 255), wind_label);
        }

        drift_draw_list->AddText(ImVec2(drift_plot_left + 4.0f, drift_plot_top - 16.0f),
                                 IM_COL32(190, 190, 200, 255), range_units);

        char top_tick_label[64];
        const float top_tick_display = (drift_lateral_scale_m * 0.5f) * offset_m_to_display;
        std::snprintf(top_tick_label, sizeof(top_tick_label), "+%.1f %s", top_tick_display,
                      offset_units);
        drift_draw_list->AddText(ImVec2(drift_right_tick_x + 4.0f, drift_plot_top + 2.0f),
                                 IM_COL32(150, 150, 165, 255), top_tick_label);

        char bot_tick_label[64];
        std::snprintf(bot_tick_label, sizeof(bot_tick_label), "-%.1f %s", top_tick_display,
                      offset_units);
        drift_draw_list->AddText(ImVec2(drift_left_tick_x - 56.0f, drift_plot_top + 2.0f),
                                 IM_COL32(150, 150, 165, 255), bot_tick_label);

        ImGui::End();

        ImGui::Render();
        const float clear_color_with_alpha[4] = {0.10f, 0.10f, 0.12f, 1.00f};
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        g_pSwapChain->Present(1, 0);
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
