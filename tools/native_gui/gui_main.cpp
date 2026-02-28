/**
 * @file gui_main.cpp
 * @brief Native Windows + ImGui desktop harness for BCE validation.
 *
 * This executable is a manual test console around the BCE C API.
 * It owns window/device setup, editable presets, synthetic SensorFrame input,
 * and live rendering of solution output/target hold visualization.
 */

#include <d3d11.h>
#include <tchar.h>
#include <windows.h>

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "imgui.h"
#include "backends/imgui_impl_dx11.h"
#include "backends/imgui_impl_win32.h"

#include "bce/bce_api.h"
#include "bce/bce_math_utils.h"
#include "nlohmann/json.hpp"

#ifdef _MSC_VER
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")
#endif

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {

// -----------------------------------------------------------------------------
// D3D11/ImGui host resources (process lifetime)
// -----------------------------------------------------------------------------

static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

using namespace bce::math;

constexpr uint64_t FRAME_STEP_US = 10000;
constexpr int TARGET_RING_COUNT = 4;
constexpr float TARGET_CANVAS_MIN = 160.0f;
constexpr float TARGET_CANVAS_MAX = 560.0f;

enum class UnitSystem : int {
    IMPERIAL = 0,
    METRIC = 1
};

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

// Note: these are GUI harness/demo presets only.
// They are NOT consumed by the BCE engine internals and are only used to
// pre-fill manual input fields for quick validation runs.
const CartridgeProfile kCartridgeProfiles[] = {
    { ".308 Win 175gr SMK", 0.505f, 0, 792.0f, 175.0f, 0.308f, 31.2f },
    { ".223 Rem 55gr FMJ", 0.245f, 0, 990.0f, 55.0f, 0.223f, 19.0f },
    { "6.5 Creedmoor 140gr ELD-M", 0.326f, 6, 823.0f, 140.0f, 0.264f, 35.3f },
    { ".300 Win Mag 190gr BTHP", 0.533f, 0, 884.0f, 190.0f, 0.308f, 34.0f },
    { "9mm 124gr FMJ", 0.150f, 0, 365.0f, 124.0f, 0.355f, 15.0f }
};
const int kNumCartridgeProfiles = sizeof(kCartridgeProfiles) / sizeof(kCartridgeProfiles[0]);

struct CartridgePreset {
    std::string name;
    float bc = 0.0f;
    int drag_model_index = 0;
    float muzzle_velocity_ms = 0.0f;
    float mass_grains = 0.0f;
    float caliber_inches = 0.0f;
    float length_mm = 0.0f;
};

struct GunPreset {
    std::string name;
    float caliber_inches = 0.308f;
    float barrel_length_in = 24.0f;
    float mv_adjustment_factor = 25.0f;
    float twist_rate_inches = 10.0f;
    float zero_range_m = 100.0f;
    float sight_height_mm = 38.1f;
};

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
    float baro_temp = 15.0f;
    float baro_humidity = 0.5f;
    float lrf_range = 500.0f;
    float lrf_conf = 1.0f;

    bool imu_valid = true;
    bool mag_valid = true;
    bool baro_valid = true;
    bool baro_humidity_valid = true;
    bool lrf_valid = true;

    int drag_model_index = 0;
    uint64_t now_us = 0;

    char preset_path[260] = "bce_gui_preset.json";
    char profile_library_path[260] = "bce_gui_profile_library.json";
    char new_cartridge_preset_name[64] = "";
    char new_gun_preset_name[64] = "";
    int selected_cartridge_preset = -1;
    int selected_gun_preset = -1;
    bool side_view_show_required_angle = false;
    bool top_down_show_required_angle = false;
    std::string output_text;
    std::string last_action;

    // Uncertainty / error-margin config (initialized from BCE_GetDefaultUncertaintyConfig).
    // sigma_cant_deg uses the RM3100 magnetometer default (1.5 deg).
    UncertaintyConfig uc_config = {};
};

GuiState g_state;
std::vector<CartridgePreset> g_cartridge_presets;
std::vector<GunPreset> g_gun_presets;

using json = nlohmann::json;

void SanitizeCartridgePreset(CartridgePreset& preset);
void SanitizeGunPreset(GunPreset& preset);
json SerializeCartridgePreset(const CartridgePreset& input);
json SerializeGunPreset(const GunPreset& input);
float GetSelectedCartridgeCaliberInches();
bool IsGunPresetCompatibleWithSelectedCaliber(const GunPreset& preset);
void EnsureSelectedGunPresetMatchesCurrentCaliber();

const DragModel kDragModels[8] = {
    DragModel::G1, DragModel::G2, DragModel::G3, DragModel::G4,
    DragModel::G5, DragModel::G6, DragModel::G7, DragModel::G8
};

const char* kDragModelLabels[8] = {"G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8"};

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
    // Seed runtime-editable preset lists from built-ins on first launch,
    // then sanitize every entry so malformed JSON cannot poison GUI state.
    if (g_cartridge_presets.empty()) {
        for (int i = 0; i < kNumCartridgeProfiles; ++i) {
            const auto& src = kCartridgeProfiles[i];
            CartridgePreset p;
            p.name = src.name;
            p.bc = src.bc;
            p.drag_model_index = src.drag_model_index;
            p.muzzle_velocity_ms = src.muzzle_velocity_ms;
            p.mass_grains = src.mass_grains;
            p.caliber_inches = src.caliber_inches;
            p.length_mm = src.length_mm;
            SanitizeCartridgePreset(p);
            g_cartridge_presets.push_back(p);
        }
    }

    if (g_gun_presets.empty()) {
        GunPreset gun_308;
        gun_308.name = ".308 Default";
        gun_308.caliber_inches = 0.308f;
        gun_308.barrel_length_in = 24.0f;
        gun_308.mv_adjustment_factor = 25.0f;
        gun_308.twist_rate_inches = 10.0f;
        gun_308.zero_range_m = 91.44f; // 100 yd
        gun_308.sight_height_mm = 38.1f;
        SanitizeGunPreset(gun_308);
        g_gun_presets.push_back(gun_308);

        GunPreset gun_223;
        gun_223.name = ".223 Default";
        gun_223.caliber_inches = 0.223f;
        gun_223.barrel_length_in = 16.0f;
        gun_223.mv_adjustment_factor = 25.0f;
        gun_223.twist_rate_inches = 8.0f; // 1:8 requested
        gun_223.zero_range_m = 91.44f; // 100 yd
        gun_223.sight_height_mm = 38.1f;
        SanitizeGunPreset(gun_223);
        g_gun_presets.push_back(gun_223);

        GunPreset gun_65;
        gun_65.name = "6.5 CM Default";
        gun_65.caliber_inches = 0.264f;
        gun_65.barrel_length_in = 24.0f;
        gun_65.mv_adjustment_factor = 25.0f;
        gun_65.twist_rate_inches = 8.0f;
        gun_65.zero_range_m = 91.44f; // 100 yd
        gun_65.sight_height_mm = 38.1f;
        SanitizeGunPreset(gun_65);
        g_gun_presets.push_back(gun_65);

        GunPreset gun_300;
        gun_300.name = ".300 WM Default";
        gun_300.caliber_inches = 0.308f;
        gun_300.barrel_length_in = 24.0f;
        gun_300.mv_adjustment_factor = 30.0f;
        gun_300.twist_rate_inches = 10.0f;
        gun_300.zero_range_m = 91.44f; // 100 yd
        gun_300.sight_height_mm = 38.1f;
        SanitizeGunPreset(gun_300);
        g_gun_presets.push_back(gun_300);

        GunPreset carbine_8;
        carbine_8.name = "Carbine (8\")";
        carbine_8.caliber_inches = 0.355f;
        carbine_8.barrel_length_in = 8.0f;
        carbine_8.mv_adjustment_factor = 12.0f;
        carbine_8.twist_rate_inches = 10.0f;
        carbine_8.zero_range_m = 45.72f; // 50 yd
        carbine_8.sight_height_mm = 38.1f;
        SanitizeGunPreset(carbine_8);
        g_gun_presets.push_back(carbine_8);

        GunPreset carbine_10_5;
        carbine_10_5.name = "Carbine (10.5\")";
        carbine_10_5.caliber_inches = 0.355f;
        carbine_10_5.barrel_length_in = 10.5f;
        carbine_10_5.mv_adjustment_factor = 15.0f;
        carbine_10_5.twist_rate_inches = 10.0f;
        carbine_10_5.zero_range_m = 45.72f; // 50 yd
        carbine_10_5.sight_height_mm = 38.1f;
        SanitizeGunPreset(carbine_10_5);
        g_gun_presets.push_back(carbine_10_5);

        GunPreset pdw;
        pdw.name = "PDW";
        pdw.caliber_inches = 0.355f;
        pdw.barrel_length_in = (114.0f * MM_TO_IN);
        pdw.mv_adjustment_factor = 14.0f;
        pdw.twist_rate_inches = 10.0f;
        pdw.zero_range_m = 45.72f; // 50 yd
        pdw.sight_height_mm = 38.1f;
        SanitizeGunPreset(pdw);
        g_gun_presets.push_back(pdw);

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
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

void SanitizeCartridgePreset(CartridgePreset& preset) {
    if (preset.name.empty()) {
        preset.name = "Unnamed Cartridge";
    }
    preset.bc = ClampValue(preset.bc, 0.001f, 1.20f);
    preset.drag_model_index = static_cast<int>(ClampValue(static_cast<float>(preset.drag_model_index), 0.0f, 7.0f));
    preset.muzzle_velocity_ms = ClampValue(preset.muzzle_velocity_ms, 50.0f, 1500.0f);
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
    preset.mv_adjustment_factor = ClampValue(std::fabs(preset.mv_adjustment_factor), 0.0f, 200.0f);
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
    item["mass_grains"] = preset.mass_grains;
    item["caliber_inches"] = preset.caliber_inches;
    item["length_mm"] = preset.length_mm;
    return item;
}

json SerializeGunPreset(const GunPreset& input) {
    GunPreset preset = input;
    SanitizeGunPreset(preset);

    json item;
    item["name"] = preset.name;
    item["caliber_inches"] = preset.caliber_inches;
    item["barrel_length_in"] = preset.barrel_length_in;
    item["mv_adjustment_factor"] = preset.mv_adjustment_factor;
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
        case DragModel::G1: base_form_factor = 0.58f; break;
        case DragModel::G2: base_form_factor = 0.62f; break;
        case DragModel::G3: base_form_factor = 0.70f; break;
        case DragModel::G4: base_form_factor = 0.68f; break;
        case DragModel::G5: base_form_factor = 0.63f; break;
        case DragModel::G6: base_form_factor = 0.60f; break;
        case DragModel::G7: base_form_factor = 0.52f; break;
        case DragModel::G8: base_form_factor = 0.56f; break;
        default: break;
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
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        createDeviceFlags,
        featureLevelArray,
        2,
        D3D11_SDK_VERSION,
        &sd,
        &g_pSwapChain,
        &g_pd3dDevice,
        &featureLevel,
        &g_pd3dDeviceContext
    );

    if (res == DXGI_ERROR_UNSUPPORTED) {
        res = D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            createDeviceFlags,
            featureLevelArray,
            2,
            D3D11_SDK_VERSION,
            &sd,
            &g_pSwapChain,
            &g_pd3dDevice,
            &featureLevel,
            &g_pd3dDeviceContext
        );
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
    g_state.bullet.mv_adjustment_factor = 25.0f;

    g_state.zero.zero_range_m = 100.0f;
    g_state.zero.sight_height_mm = 38.1f;

    g_state.wind_speed_ms = 0.0f;
    g_state.wind_heading = 0.0f;
    g_state.latitude = 37.0f;

    g_state.baro_pressure = 101325.0f;
    g_state.baro_temp = 15.0f;
    g_state.baro_humidity = 0.5f;
    g_state.lrf_range = 500.0f;
    g_state.lrf_conf = 1.0f;

    g_state.imu_valid = true;
    g_state.mag_valid = true;
    g_state.baro_valid = true;
    g_state.baro_humidity_valid = true;
    g_state.lrf_valid = true;

    g_state.drag_model_index = 0;
    g_state.now_us = 0;
    g_state.last_action = "Startup";
    BCE_GetDefaultUncertaintyConfig(&g_state.uc_config);
    std::snprintf(g_state.preset_path, sizeof(g_state.preset_path), "%s", "bce_gui_preset.json");
    std::snprintf(g_state.profile_library_path, sizeof(g_state.profile_library_path), "%s", "bce_gui_profile_library.json");
    std::snprintf(g_state.new_cartridge_preset_name, sizeof(g_state.new_cartridge_preset_name), "%s", "New Cartridge Preset");
    std::snprintf(g_state.new_gun_preset_name, sizeof(g_state.new_gun_preset_name), "%s", "New Gun Preset");
    g_state.selected_cartridge_preset = -1;
    g_state.selected_gun_preset = -1;

    g_cartridge_presets.clear();
    g_gun_presets.clear();
    EnsureProfileDefaults();
}

void ApplyConfig() {
    // Push current GUI inputs into the BCE engine.
    g_state.bullet.drag_model = kDragModels[g_state.drag_model_index];
    g_state.bullet.bc = g_state.override_drag_coefficient
        ? ClampValue(g_state.manual_drag_coefficient, 0.001f, 1.20f)
        : ComputeAutoDragCoefficient(g_state.bullet);
    BCE_SetBulletProfile(&g_state.bullet);
    BCE_SetZeroConfig(&g_state.zero);
    BCE_SetWindManual(g_state.wind_speed_ms, g_state.wind_heading);
    BCE_SetLatitude(g_state.latitude);
    BCE_SetUncertaintyConfig(&g_state.uc_config);
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
    frame.lrf_confidence = g_state.lrf_conf;
    frame.lrf_valid = g_state.lrf_valid;

    frame.encoder_focal_length_mm = 0.0f;
    frame.encoder_valid = false;
    return frame;
}

void RunFrameUpdates(int frame_count) {
    for (int i = 0; i < frame_count; ++i) {
        SensorFrame frame = BuildFrame();
        BCE_Update(&frame);
    }
}

void RefreshOutput() {
    // Produce the human-readable diagnostics/solution text displayed in "BCE Output".
    FiringSolution sol = {};
    BCE_GetSolution(&sol);

    BCE_Mode mode = BCE_GetMode();
    uint32_t fault = BCE_GetFaultFlags();
    uint32_t diag = BCE_GetDiagFlags();

    const char* mode_text = "UNKNOWN";
    if (mode == BCE_Mode::IDLE) mode_text = "IDLE";
    if (mode == BCE_Mode::SOLUTION_READY) mode_text = "SOLUTION_READY";
    if (mode == BCE_Mode::FAULT) mode_text = "FAULT";

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);

    auto appendFaultNames = [&](uint32_t flags) {
        bool first = true;
        auto add = [&](const char* name) {
            if (!first) ss << ", ";
            ss << name;
            first = false;
        };
        if (flags == BCE_Fault::NONE) {
            ss << "none";
            return;
        }
        if (flags & BCE_Fault::NO_RANGE) add("NO_RANGE");
        if (flags & BCE_Fault::NO_BULLET) add("NO_BULLET");
        if (flags & BCE_Fault::NO_MV) add("NO_MV");
        if (flags & BCE_Fault::NO_BC) add("NO_BC");
        if (flags & BCE_Fault::ZERO_UNSOLVABLE) add("ZERO_UNSOLVABLE");
        if (flags & BCE_Fault::AHRS_UNSTABLE) add("AHRS_UNSTABLE");
        if (flags & BCE_Fault::SENSOR_INVALID) add("SENSOR_INVALID");
    };

    auto appendDiagNames = [&](uint32_t flags) {
        bool first = true;
        auto add = [&](const char* name) {
            if (!first) ss << ", ";
            ss << name;
            first = false;
        };
        if (flags == BCE_Diag::NONE) {
            ss << "none";
            return;
        }
        if (flags & BCE_Diag::CORIOLIS_DISABLED) add("CORIOLIS_DISABLED");
        if (flags & BCE_Diag::DEFAULT_PRESSURE) add("DEFAULT_PRESSURE");
        if (flags & BCE_Diag::DEFAULT_TEMP) add("DEFAULT_TEMP");
        if (flags & BCE_Diag::DEFAULT_HUMIDITY) add("DEFAULT_HUMIDITY");
        if (flags & BCE_Diag::DEFAULT_ALTITUDE) add("DEFAULT_ALTITUDE");
        if (flags & BCE_Diag::DEFAULT_WIND) add("DEFAULT_WIND");
        if (flags & BCE_Diag::MAG_SUPPRESSED) add("MAG_SUPPRESSED");
        if (flags & BCE_Diag::LRF_STALE) add("LRF_STALE");
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

    if (mode == BCE_Mode::IDLE) {
        ss << "Hint: IDLE means no valid firing solution yet. Apply config then feed frames with Step/Run 100.\n";
    }
    if (fault & BCE_Fault::AHRS_UNSTABLE) {
        ss << "Hint: AHRS_UNSTABLE is expected in early frames. Use Run 100 or enough Step updates while IMU is static.\n";
    }
    if (fault & BCE_Fault::NO_RANGE) {
        ss << "Hint: NO_RANGE means no accepted LRF sample (check LRF valid/range/confidence).\n";
    }
    if (diag & BCE_Diag::DEFAULT_ALTITUDE) {
        ss << "Hint: DEFAULT_ALTITUDE is informational (not a fault).\n";
    }
    ss << "\n";

    const float velocity_at_target_fps = sol.velocity_at_target_ms * MPS_TO_FPS;
    const float range_yd = sol.range_m * M_TO_YD;
    const float horiz_range_yd = sol.horizontal_range_m * M_TO_YD;
    const float energy_ftlb = sol.energy_at_target_j * J_TO_FTLB;
    auto direction_label = [](float moa) -> const char* {
        if (moa > 0.01f) return "RIGHT";
        if (moa < -0.01f) return "LEFT";
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
    ss << "Coriolis elev/wind (MOA): " << sol.coriolis_elevation_moa << " / " << sol.coriolis_windage_moa << "\n";
    ss << "Spin drift (MOA): " << sol.spin_drift_moa << "\n";
    ss << "Windage Breakdown (MOA, + = right hold):\n";
     ss << "  Wind only:        " << sol.wind_only_windage_moa << " [" << direction_label(sol.wind_only_windage_moa) << "]\n";
    ss << "  Earth spin total: " << sol.earth_spin_windage_moa
       << " (Coriolis " << sol.coriolis_windage_moa
         << ", Spin " << sol.spin_drift_moa << ") [" << direction_label(sol.earth_spin_windage_moa) << "]\n";
     ss << "  Offsets total:    " << sol.offsets_windage_moa << " [" << direction_label(sol.offsets_windage_moa) << "]\n";
     ss << "  Cant added:       " << sol.cant_windage_moa << " [" << direction_label(sol.cant_windage_moa) << "]\n";
     ss << "  Total windage:    " << sol.hold_windage_moa << " [" << direction_label(sol.hold_windage_moa) << "]\n";

    if (sol.uncertainty_valid) {
        const float rms_sigma = std::sqrt((sol.sigma_elevation_moa * sol.sigma_elevation_moa +
                                           sol.sigma_windage_moa   * sol.sigma_windage_moa) * 0.5f);
        const float cep50 = 1.1774f * rms_sigma;
        const float cep95 = 2.4477f * rms_sigma;
        ss << "\nUncertainty (1-sigma propagation):\n";
        ss << "  Elevation: " << sol.sigma_elevation_moa << " MOA\n";
        ss << "  Windage:   " << sol.sigma_windage_moa   << " MOA\n";
        ss << "  50% CEP:   " << cep50 << " MOA\n";
        ss << "  95% CEP:   " << cep95 << " MOA\n";
    } else if (g_state.uc_config.enabled) {
        ss << "\nUncertainty: enabled but not yet computed (need valid solution).\n";
    } else {
        ss << "\nUncertainty: disabled.\n";
    }

    g_state.output_text = ss.str();
}

void ApplyCartridgePreset(const CartridgePreset& preset) {
    // Applying a preset intentionally enables manual drag override so the
    // exact saved BC is preserved instead of being recomputed heuristically.
    CartridgePreset normalized = preset;
    SanitizeCartridgePreset(normalized);

    g_state.bullet.bc = normalized.bc;
    g_state.drag_model_index = normalized.drag_model_index;
    g_state.bullet.muzzle_velocity_ms = normalized.muzzle_velocity_ms;
    g_state.bullet.mass_grains = normalized.mass_grains;
    g_state.bullet.caliber_inches = normalized.caliber_inches;
    g_state.bullet.length_mm = normalized.length_mm;
    g_state.override_drag_coefficient = true;
    g_state.manual_drag_coefficient = normalized.bc;
    EnsureSelectedGunPresetMatchesCurrentCaliber();
}

void ApplyGunPreset(const GunPreset& preset) {
    GunPreset normalized = preset;
    SanitizeGunPreset(normalized);

    g_state.bullet.barrel_length_in = normalized.barrel_length_in;
    g_state.bullet.mv_adjustment_factor = normalized.mv_adjustment_factor;
    g_state.bullet.twist_rate_inches = normalized.twist_rate_inches;
    g_state.zero.zero_range_m = normalized.zero_range_m;
    g_state.zero.sight_height_mm = normalized.sight_height_mm;
}

CartridgePreset CaptureCurrentCartridgePreset(const std::string& name) {
    CartridgePreset preset;
    preset.name = name;
    preset.bc = g_state.override_drag_coefficient
        ? ClampValue(g_state.manual_drag_coefficient, 0.001f, 1.20f)
        : ComputeAutoDragCoefficient(g_state.bullet);
    preset.drag_model_index = g_state.drag_model_index;
    preset.muzzle_velocity_ms = g_state.bullet.muzzle_velocity_ms;
    preset.mass_grains = g_state.bullet.mass_grains;
    preset.caliber_inches = g_state.bullet.caliber_inches;
    preset.length_mm = g_state.bullet.length_mm;
    SanitizeCartridgePreset(preset);
    return preset;
}

GunPreset CaptureCurrentGunPreset(const std::string& name) {
    GunPreset preset;
    preset.name = name;
    preset.caliber_inches = g_state.bullet.caliber_inches;
    preset.barrel_length_in = g_state.bullet.barrel_length_in;
    preset.mv_adjustment_factor = g_state.bullet.mv_adjustment_factor;
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
                preset.muzzle_velocity_ms = item.value("muzzle_velocity_ms", 792.0f);
                preset.mass_grains = item.value("mass_grains", 175.0f);
                preset.caliber_inches = item.value("caliber_inches", 0.308f);
                preset.length_mm = item.value("length_mm", 31.2f);
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
                preset.mv_adjustment_factor = std::fabs(item.value("mv_adjustment_factor", 25.0f));
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
                preset.mv_adjustment_factor = std::fabs(item.value("mv_adjustment_factor", 25.0f));
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
    j["mv_adjustment_factor"] = g_state.bullet.mv_adjustment_factor;
    j["zero_range_m"] = g_state.zero.zero_range_m;
    j["sight_height_mm"] = g_state.zero.sight_height_mm;
    j["wind_speed_ms"] = g_state.wind_speed_ms;
    j["wind_heading_deg"] = g_state.wind_heading;
    j["latitude_deg"] = g_state.latitude;
    j["baro_pressure_pa"] = g_state.baro_pressure;
    j["baro_temperature_c"] = g_state.baro_temp;
    j["baro_humidity"] = g_state.baro_humidity;
    j["lrf_range_m"] = g_state.lrf_range;
    j["lrf_confidence"] = g_state.lrf_conf;
    j["uc_enabled"]               = g_state.uc_config.enabled;
    j["uc_sigma_mv_ms"]           = g_state.uc_config.sigma_muzzle_velocity_ms;
    j["uc_sigma_bc_fraction"]     = g_state.uc_config.sigma_bc_fraction;
    j["uc_sigma_range_m"]         = g_state.uc_config.sigma_range_m;
    j["uc_sigma_wind_speed_ms"]   = g_state.uc_config.sigma_wind_speed_ms;
    j["uc_sigma_wind_heading_deg"]= g_state.uc_config.sigma_wind_heading_deg;
    j["uc_sigma_temperature_c"]   = g_state.uc_config.sigma_temperature_c;
    j["uc_sigma_pressure_pa"]     = g_state.uc_config.sigma_pressure_pa;
    j["uc_sigma_humidity"]        = g_state.uc_config.sigma_humidity;
    j["uc_sigma_sight_height_mm"] = g_state.uc_config.sigma_sight_height_mm;
    j["uc_sigma_cant_deg"]        = g_state.uc_config.sigma_cant_deg;
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
        g_state.bullet.mv_adjustment_factor = std::fabs(j.value("mv_adjustment_factor", 25.0f));
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
        g_state.baro_temp = j.value("baro_temperature_c", 15.0f);
        g_state.baro_humidity = j.value("baro_humidity", 0.5f);
        g_state.lrf_range = j.value("lrf_range_m", 500.0f);
        g_state.lrf_conf = j.value("lrf_confidence", 1.0f);

        // Uncertainty config — load with per-field defaults matching BCE defaults
        UncertaintyConfig uc_def = {};
        BCE_GetDefaultUncertaintyConfig(&uc_def);
        g_state.uc_config.enabled                  = j.value("uc_enabled",               uc_def.enabled);
        g_state.uc_config.sigma_muzzle_velocity_ms  = j.value("uc_sigma_mv_ms",           uc_def.sigma_muzzle_velocity_ms);
        g_state.uc_config.sigma_bc_fraction          = j.value("uc_sigma_bc_fraction",     uc_def.sigma_bc_fraction);
        g_state.uc_config.sigma_range_m              = j.value("uc_sigma_range_m",         uc_def.sigma_range_m);
        g_state.uc_config.sigma_wind_speed_ms        = j.value("uc_sigma_wind_speed_ms",   uc_def.sigma_wind_speed_ms);
        g_state.uc_config.sigma_wind_heading_deg     = j.value("uc_sigma_wind_heading_deg",uc_def.sigma_wind_heading_deg);
        g_state.uc_config.sigma_temperature_c        = j.value("uc_sigma_temperature_c",   uc_def.sigma_temperature_c);
        g_state.uc_config.sigma_pressure_pa          = j.value("uc_sigma_pressure_pa",     uc_def.sigma_pressure_pa);
        g_state.uc_config.sigma_humidity             = j.value("uc_sigma_humidity",        uc_def.sigma_humidity);
        g_state.uc_config.sigma_sight_height_mm      = j.value("uc_sigma_sight_height_mm", uc_def.sigma_sight_height_mm);
        g_state.uc_config.sigma_cant_deg             = j.value("uc_sigma_cant_deg",        uc_def.sigma_cant_deg);

        g_state.imu_valid = j.value("imu_valid", true);
        g_state.mag_valid = j.value("mag_valid", true);
        g_state.baro_valid = j.value("baro_valid", true);
        g_state.baro_humidity_valid = j.value("baro_humidity_valid", true);
        g_state.lrf_valid = j.value("lrf_valid", true);

        if (!has_bc && !g_state.override_drag_coefficient) {
            g_state.manual_drag_coefficient = ComputeAutoDragCoefficient(g_state.bullet);
        }
    } catch (const json::parse_error& e) {
        g_state.last_action = "Load Preset (failed)";
        std::string error_msg = "Preset load failed: JSON parse error: ";
        error_msg += e.what();
        g_state.output_text = error_msg;
    }
}

void ResetEngineAndState() {
    BCE_Init();
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
                g_pSwapChain->ResizeBuffers(0, static_cast<UINT>(LOWORD(lParam)), static_cast<UINT>(HIWORD(lParam)), DXGI_FORMAT_UNKNOWN, 0);
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

}  // namespace

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    // Desktop harness startup sequence: defaults -> window/device -> ImGui -> BCE init.
    ResetStateDefaults();

    WNDCLASSEX wc = {
        sizeof(WNDCLASSEX),
        CS_CLASSDC,
        WndProc,
        0L,
        0L,
        GetModuleHandle(nullptr),
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        _T("BCE_ImGui_Test"),
        nullptr
    };
    RegisterClassEx(&wc);

    HWND hwnd = CreateWindow(
        wc.lpszClassName,
        _T("BCE Basic Test GUI (ImGui)"),
        WS_OVERLAPPEDWINDOW,
        100,
        100,
        1280,
        800,
        nullptr,
        nullptr,
        wc.hInstance,
        nullptr
    );

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

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("BCE Inputs");
        int unit_mode = static_cast<int>(g_state.unit_system);
        if (ImGui::Combo("Unit System", &unit_mode, kUnitSystemLabels, IM_ARRAYSIZE(kUnitSystemLabels))) {
            g_state.unit_system = static_cast<UnitSystem>(unit_mode);
            g_state.last_action = "Unit Mode Changed";
            RefreshOutput();
        }

        if (ImGui::CollapsingHeader("Cartridge Presets (GUI-only)")) {
            ImGui::TextUnformatted("Presets are for test-harness convenience and do not alter engine internals.");

            ImGui::InputText("Cartridge Preset Name", g_state.new_cartridge_preset_name, IM_ARRAYSIZE(g_state.new_cartridge_preset_name));
            if (ImGui::Button("Add/Update Cartridge Preset")) {
                const std::string preset_name = g_state.new_cartridge_preset_name;
                if (!preset_name.empty()) {
                    CartridgePreset preset = CaptureCurrentCartridgePreset(preset_name);
                    const int existing = FindCartridgePresetByName(preset_name);
                    if (existing >= 0) {
                        g_cartridge_presets[existing] = preset;
                        g_state.selected_cartridge_preset = existing;
                    } else {
                        g_cartridge_presets.push_back(preset);
                        g_state.selected_cartridge_preset = static_cast<int>(g_cartridge_presets.size()) - 1;
                    }
                    g_state.last_action = "Cartridge Preset Saved";
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Cartridge") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                ApplyCartridgePreset(g_cartridge_presets[g_state.selected_cartridge_preset]);
                g_state.last_action = "Cartridge Preset Applied";
                ApplyConfig();
                RefreshOutput();
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Selected Cartridge") &&
                g_state.selected_cartridge_preset >= 0 &&
                g_state.selected_cartridge_preset < static_cast<int>(g_cartridge_presets.size())) {
                g_cartridge_presets.erase(g_cartridge_presets.begin() + g_state.selected_cartridge_preset);
                if (g_cartridge_presets.empty()) {
                    g_state.selected_cartridge_preset = -1;
                } else if (g_state.selected_cartridge_preset >= static_cast<int>(g_cartridge_presets.size())) {
                    g_state.selected_cartridge_preset = static_cast<int>(g_cartridge_presets.size()) - 1;
                }
                g_state.last_action = "Cartridge Preset Removed";
                RefreshOutput();
            }

            if (ImGui::BeginListBox("##cartridge_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g_cartridge_presets.size()); ++i) {
                    const bool is_selected = (g_state.selected_cartridge_preset == i);
                    if (ImGui::Selectable(g_cartridge_presets[i].name.c_str(), is_selected)) {
                        g_state.selected_cartridge_preset = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
        }

        if (ImGui::CollapsingHeader("Gun Presets (GUI-only)")) {
            EnsureSelectedGunPresetMatchesCurrentCaliber();
            ImGui::InputText("Gun Preset Name", g_state.new_gun_preset_name, IM_ARRAYSIZE(g_state.new_gun_preset_name));
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
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Gun") &&
                g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
                IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[g_state.selected_gun_preset])) {
                ApplyGunPreset(g_gun_presets[g_state.selected_gun_preset]);
                g_state.last_action = "Gun Preset Applied";
                ApplyConfig();
                RefreshOutput();
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Selected Gun") &&
                g_state.selected_gun_preset >= 0 &&
                g_state.selected_gun_preset < static_cast<int>(g_gun_presets.size()) &&
                IsGunPresetCompatibleWithSelectedCaliber(g_gun_presets[g_state.selected_gun_preset])) {
                g_gun_presets.erase(g_gun_presets.begin() + g_state.selected_gun_preset);
                if (g_gun_presets.empty()) {
                    g_state.selected_gun_preset = -1;
                } else if (g_state.selected_gun_preset >= static_cast<int>(g_gun_presets.size())) {
                    g_state.selected_gun_preset = static_cast<int>(g_gun_presets.size()) - 1;
                }
                g_state.last_action = "Gun Preset Removed";
                RefreshOutput();
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
        }

        const bool is_imperial = (g_state.unit_system == UnitSystem::IMPERIAL);

        ImGui::TextUnformatted("Manual Inputs");
        ImGui::Checkbox("Override Drag Coefficient", &g_state.override_drag_coefficient);
        if (g_state.override_drag_coefficient) {
            ImGui::InputFloat("Drag Coefficient (G-model)", &g_state.manual_drag_coefficient, 0.001f, 0.01f, "%.4f");
        } else {
            const float auto_drag = ComputeAutoDragCoefficient(g_state.bullet);
            ImGui::Text("Drag Coefficient (auto): %.4f", auto_drag);
        }
        ImGui::Combo("Drag Model", &g_state.drag_model_index, kDragModelLabels, IM_ARRAYSIZE(kDragModelLabels));
        float muzzle_display = is_imperial ? (g_state.bullet.muzzle_velocity_ms * MPS_TO_FPS) : g_state.bullet.muzzle_velocity_ms;
        if (ImGui::InputFloat(is_imperial ? "Muzzle Velocity (fps)" : "Muzzle Velocity (m/s)", &muzzle_display, is_imperial ? 5.0f : 1.0f, is_imperial ? 50.0f : 10.0f, "%.1f")) {
            g_state.bullet.muzzle_velocity_ms = is_imperial ? (muzzle_display * FPS_TO_MPS) : muzzle_display;
        }
        ImGui::InputFloat("Mass (gr)", &g_state.bullet.mass_grains, 1.0f, 10.0f, "%.2f");
        float length_display = is_imperial ? (g_state.bullet.length_mm * MM_TO_IN) : g_state.bullet.length_mm;
        if (ImGui::InputFloat(is_imperial ? "Length (in)" : "Length (mm)", &length_display, is_imperial ? 0.01f : 0.1f, is_imperial ? 0.1f : 1.0f, is_imperial ? "%.3f" : "%.2f")) {
            g_state.bullet.length_mm = is_imperial ? (length_display * IN_TO_MM) : length_display;
        }
        float caliber_display = is_imperial ? g_state.bullet.caliber_inches : (g_state.bullet.caliber_inches * IN_TO_MM);
        if (ImGui::InputFloat(is_imperial ? "Caliber (in)" : "Caliber (mm)", &caliber_display, is_imperial ? 0.001f : 0.01f, is_imperial ? 0.01f : 0.1f, is_imperial ? "%.3f" : "%.2f")) {
            g_state.bullet.caliber_inches = is_imperial ? caliber_display : (caliber_display * MM_TO_IN);
        }
        float twist_display = is_imperial ? g_state.bullet.twist_rate_inches : (g_state.bullet.twist_rate_inches * IN_TO_MM);
        if (ImGui::InputFloat(is_imperial ? "Twist (in/turn)" : "Twist (mm/turn)", &twist_display, is_imperial ? 0.1f : 1.0f, is_imperial ? 1.0f : 10.0f, is_imperial ? "%.2f" : "%.1f")) {
            g_state.bullet.twist_rate_inches = is_imperial ? twist_display : (twist_display * MM_TO_IN);
        }
        ImGui::InputFloat("Barrel Length (in)", &g_state.bullet.barrel_length_in, 0.1f, 1.0f, "%.1f");
        if (ImGui::InputFloat("MV Adjustment (fps/in)", &g_state.bullet.mv_adjustment_factor, 1.0f, 10.0f, "%.1f")) {
            g_state.bullet.mv_adjustment_factor = std::fabs(g_state.bullet.mv_adjustment_factor);
        }

        ImGui::Separator();
        float zero_display = is_imperial ? (g_state.zero.zero_range_m * M_TO_YD) : g_state.zero.zero_range_m;
        if (ImGui::InputFloat(is_imperial ? "Zero Range (yd)" : "Zero Range (m)", &zero_display, is_imperial ? 1.0f : 1.0f, is_imperial ? 10.0f : 10.0f, "%.2f")) {
            g_state.zero.zero_range_m = is_imperial ? (zero_display * YD_TO_M) : zero_display;
        }
        float sight_display = is_imperial ? (g_state.zero.sight_height_mm * MM_TO_IN) : g_state.zero.sight_height_mm;
        if (ImGui::InputFloat(is_imperial ? "Sight Height (in)" : "Sight Height (mm)", &sight_display, is_imperial ? 0.01f : 0.1f, is_imperial ? 0.1f : 1.0f, is_imperial ? "%.3f" : "%.2f")) {
            g_state.zero.sight_height_mm = is_imperial ? (sight_display * IN_TO_MM) : sight_display;
        }
        float wind_display = is_imperial ? (g_state.wind_speed_ms * MPS_TO_MPH) : g_state.wind_speed_ms;
        if (ImGui::InputFloat(is_imperial ? "Wind Speed (mph)" : "Wind Speed (m/s)", &wind_display, is_imperial ? 0.5f : 0.1f, is_imperial ? 5.0f : 1.0f, "%.2f")) {
            g_state.wind_speed_ms = is_imperial ? (wind_display * MPH_TO_MPS) : wind_display;
        }
        ImGui::InputFloat("Wind Heading (deg)", &g_state.wind_heading, 1.0f, 5.0f, "%.1f");
        ImGui::InputFloat("Latitude (deg)", &g_state.latitude, 0.1f, 1.0f, "%.3f");

        ImGui::Separator();
        ImGui::TextUnformatted("Sensor Frame");
        float pressure_display = g_state.baro_pressure;
        if (ImGui::InputFloat("Baro Pressure (Pa)", &pressure_display, 10.0f, 100.0f, "%.1f")) {
            g_state.baro_pressure = pressure_display;
        }
        float temp_display = is_imperial ? ((g_state.baro_temp * 9.0f / 5.0f) + 32.0f) : g_state.baro_temp;
        if (ImGui::InputFloat(is_imperial ? "Baro Temp (F)" : "Baro Temp (C)", &temp_display, is_imperial ? 0.5f : 0.1f, is_imperial ? 2.0f : 1.0f, "%.2f")) {
            g_state.baro_temp = is_imperial ? ((temp_display - 32.0f) * 5.0f / 9.0f) : temp_display;
        }
        ImGui::InputFloat("Humidity (0..1)", &g_state.baro_humidity, 0.01f, 0.1f, "%.2f");
        float lrf_display = is_imperial ? (g_state.lrf_range * M_TO_YD) : g_state.lrf_range;
        if (ImGui::InputFloat(is_imperial ? "LRF Range (yd)" : "LRF Range (m)", &lrf_display, 1.0f, 10.0f, "%.2f")) {
            g_state.lrf_range = is_imperial ? (lrf_display * YD_TO_M) : lrf_display;
        }
        ImGui::InputFloat("LRF Confidence", &g_state.lrf_conf, 0.01f, 0.1f, "%.2f");

        ImGui::Checkbox("IMU Valid", &g_state.imu_valid);
        ImGui::SameLine();
        ImGui::Checkbox("Mag Valid", &g_state.mag_valid);
        ImGui::SameLine();
        ImGui::Checkbox("Baro Valid", &g_state.baro_valid);
        ImGui::Checkbox("Humidity Valid", &g_state.baro_humidity_valid);
        ImGui::SameLine();
        ImGui::Checkbox("LRF Valid", &g_state.lrf_valid);

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Error Margins (1-sigma)")) {
            ImGui::Checkbox("Enable Uncertainty Propagation", &g_state.uc_config.enabled);
            if (g_state.uc_config.enabled) {
                ImGui::Indent();
                // Muzzle velocity — fps in imperial
                float mv_sig = is_imperial ? (g_state.uc_config.sigma_muzzle_velocity_ms * MPS_TO_FPS) : g_state.uc_config.sigma_muzzle_velocity_ms;
                if (ImGui::InputFloat(is_imperial ? "MV sigma (fps)" : "MV sigma (m/s)", &mv_sig, 0.5f, 5.0f, "%.1f"))
                    g_state.uc_config.sigma_muzzle_velocity_ms = is_imperial ? (mv_sig * FPS_TO_MPS) : mv_sig;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma muzzle velocity spread (chrono SD)");

                ImGui::InputFloat("BC sigma (fraction)", &g_state.uc_config.sigma_bc_fraction, 0.001f, 0.01f, "%.3f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma BC uncertainty as a fraction, e.g. 0.02 = 2%%");

                // Range — yd in imperial
                float rng_sig = is_imperial ? (g_state.uc_config.sigma_range_m * M_TO_YD) : g_state.uc_config.sigma_range_m;
                if (ImGui::InputFloat(is_imperial ? "Range sigma (yd)" : "Range sigma (m)", &rng_sig, 0.1f, 1.0f, "%.2f"))
                    g_state.uc_config.sigma_range_m = is_imperial ? (rng_sig * YD_TO_M) : rng_sig;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma LRF ranging uncertainty");

                // Wind speed — mph in imperial
                float wsp_sig = is_imperial ? (g_state.uc_config.sigma_wind_speed_ms * MPS_TO_MPH) : g_state.uc_config.sigma_wind_speed_ms;
                if (ImGui::InputFloat(is_imperial ? "Wind speed sigma (mph)" : "Wind speed sigma (m/s)", &wsp_sig, 0.1f, 1.0f, "%.2f"))
                    g_state.uc_config.sigma_wind_speed_ms = is_imperial ? (wsp_sig * MPH_TO_MPS) : wsp_sig;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma wind speed estimation error");

                ImGui::InputFloat("Wind dir sigma (deg)", &g_state.uc_config.sigma_wind_heading_deg, 0.5f, 5.0f, "%.1f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma wind direction estimation error");

                // Temperature — F in imperial (sigma scales by 9/5 just like absolute temp)
                float tmp_sig = is_imperial ? (g_state.uc_config.sigma_temperature_c * 9.0f / 5.0f) : g_state.uc_config.sigma_temperature_c;
                if (ImGui::InputFloat(is_imperial ? "Temp sigma (F)" : "Temp sigma (C)", &tmp_sig, 0.1f, 1.0f, "%.2f"))
                    g_state.uc_config.sigma_temperature_c = is_imperial ? (tmp_sig * 5.0f / 9.0f) : tmp_sig;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma baro temperature sensor uncertainty");

                ImGui::InputFloat("Pressure sigma (Pa)", &g_state.uc_config.sigma_pressure_pa, 10.0f, 100.0f, "%.0f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma baro pressure sensor uncertainty");

                ImGui::InputFloat("Humidity sigma (frac)", &g_state.uc_config.sigma_humidity, 0.005f, 0.05f, "%.3f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma relative humidity uncertainty (0..1)");

                // Sight height — inches in imperial
                float sht_sig = is_imperial ? (g_state.uc_config.sigma_sight_height_mm * MM_TO_IN) : g_state.uc_config.sigma_sight_height_mm;
                if (ImGui::InputFloat(is_imperial ? "Sight height sigma (in)" : "Sight height sigma (mm)", &sht_sig, 0.01f, 0.1f, "%.3f"))
                    g_state.uc_config.sigma_sight_height_mm = is_imperial ? (sht_sig * IN_TO_MM) : sht_sig;
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma sight height mounting uncertainty");

                ImGui::InputFloat("Cant sigma (deg)", &g_state.uc_config.sigma_cant_deg, 0.1f, 1.0f, "%.2f");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("1-sigma cant/roll uncertainty from the RM3100 magnetometer");
                ImGui::Unindent();
            }
        }

        if (ImGui::Button("Apply Config")) {
            g_state.last_action = "Apply Config";
            ApplyConfig();
            RefreshOutput();
        }
        ImGui::SameLine();
        if (ImGui::Button("Step Update")) {
            g_state.last_action = "Step Update";
            ApplyConfig();
            RunFrameUpdates(1);
            RefreshOutput();
        }
        ImGui::SameLine();
        if (ImGui::Button("Run 100")) {
            g_state.last_action = "Run 100";
            ApplyConfig();
            RunFrameUpdates(100);
            RefreshOutput();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Engine")) {
            ResetEngineAndState();
        }

        ImGui::Separator();
        ImGui::InputText("Profile Library Path", g_state.profile_library_path, IM_ARRAYSIZE(g_state.profile_library_path));
        if (ImGui::Button("Save Profile Library")) {
            SaveProfileLibrary();
            RefreshOutput();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Profile Library")) {
            LoadProfileLibrary();
            RefreshOutput();
        }

        ImGui::Separator();
        ImGui::InputText("Preset Path", g_state.preset_path, IM_ARRAYSIZE(g_state.preset_path));
        if (ImGui::Button("Save Preset")) {
            g_state.last_action = "Save Preset";
            SavePreset();
            RefreshOutput();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Preset")) {
            g_state.last_action = "Load Preset";
            LoadPreset();
            ApplyConfig();
            RefreshOutput();
        }

        ImGui::End();

        ImGui::Begin("BCE Output");
        ImGui::InputTextMultiline(
            "##output",
            g_state.output_text.data(),
            g_state.output_text.size() + 1,
            ImVec2(-FLT_MIN, -FLT_MIN),
            ImGuiInputTextFlags_ReadOnly
        );
        ImGui::End();

        ImGui::Begin("Target View");
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        FiringSolution sol = {};
        BCE_GetSolution(&sol);

        const float hold_windage_moa = sol.hold_windage_moa;
        const float hold_elevation_moa = sol.hold_elevation_moa;
        const float impact_distance_moa = std::sqrt(
            hold_windage_moa * hold_windage_moa + hold_elevation_moa * hold_elevation_moa);
        // Auto-scale: include 3-sigma uncertainty ellipse footprint when available.
        const float sigma_span_moa = (sol.uncertainty_valid)
            ? 3.0f * std::max(sol.sigma_elevation_moa, sol.sigma_windage_moa)
            : 0.0f;

        const float ring_count = static_cast<float>(TARGET_RING_COUNT);
        const float required_span_moa = impact_distance_moa + sigma_span_moa;
        float moa_per_ring = ClampValue(std::round(required_span_moa / ring_count), 1.0f, 30.0f);
        if ((moa_per_ring * ring_count) < required_span_moa) {
            moa_per_ring = ClampValue(moa_per_ring + 1.0f, 1.0f, 30.0f);
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
            if (moa > 0.01f) return "RIGHT";
            if (moa < -0.01f) return "LEFT";
            return "CENTER";
        };
        auto direction_color = [](float moa) -> ImVec4 {
            if (moa > 0.01f) return ImGui::GetStyleColorVec4(ImGuiCol_PlotHistogram);
            if (moa < -0.01f) return ImGui::GetStyleColorVec4(ImGuiCol_PlotLines);
            return ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
        };

        ImGui::Text("Scale: %.0f MOA/ring (%.2f in/ring @ %.1f yd)", moa_per_ring, inches_per_ring, display_range_yd);
        ImGui::Text("Offset: Elev %.2f MOA (%.2f in), Wind %.2f MOA (%.2f in)", hold_elevation_moa, elev_offset_in, hold_windage_moa, wind_offset_in);
        ImGui::Text("Wind breakdown:");
        ImGui::Text("  Wind:       %.2f MOA (%.2f in)", sol.wind_only_windage_moa, wind_only_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.wind_only_windage_moa), "[%s]", direction_label(sol.wind_only_windage_moa));
        ImGui::Text("  Earth spin: %.2f MOA (%.2f in)", sol.earth_spin_windage_moa, earth_spin_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.earth_spin_windage_moa), "[%s]", direction_label(sol.earth_spin_windage_moa));
        ImGui::Text("  Offsets:    %.2f MOA (%.2f in)", sol.offsets_windage_moa, offsets_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.offsets_windage_moa), "[%s]", direction_label(sol.offsets_windage_moa));
        ImGui::Text("  Cant:       %.2f MOA (%.2f in)", sol.cant_windage_moa, cant_added_in);
        ImGui::SameLine();
        ImGui::TextColored(direction_color(sol.cant_windage_moa), "[%s]", direction_label(sol.cant_windage_moa));
        ImGui::Text("Center->Impact: %.2f MOA (%.2f in)", impact_distance_moa, impact_distance_in);
        if (sol.uncertainty_valid) {
            const float cep_radius_moa = 1.1774f * std::sqrt((sol.sigma_elevation_moa * sol.sigma_elevation_moa +
                                                               sol.sigma_windage_moa  * sol.sigma_windage_moa) * 0.5f);
            ImGui::Text("1-sigma: Elev %.2f MOA, Wind %.2f MOA  |  50%% CEP %.2f MOA",
                        sol.sigma_elevation_moa, sol.sigma_windage_moa, cep_radius_moa);
        } else {
            ImGui::TextDisabled("Uncertainty propagation disabled or not yet computed.");
        }
        ImGui::Separator();

        ImVec2 avail = ImGui::GetContentRegionAvail();
        float canvas_side = (avail.x < avail.y) ? avail.x : avail.y;
        if (canvas_side > TARGET_CANVAS_MAX) canvas_side = TARGET_CANVAS_MAX;
        if (canvas_side < TARGET_CANVAS_MIN) canvas_side = TARGET_CANVAS_MIN;
        const ImVec2 canvas_size(canvas_side, canvas_side);

        float pad_x = (avail.x - canvas_size.x) * 0.5f;
        if (pad_x < 0.0f) pad_x = 0.0f;
        float pad_y = (avail.y - canvas_size.y) * 0.5f;
        if (pad_y < 0.0f) pad_y = 0.0f;
        const ImVec2 draw_origin = ImGui::GetCursorScreenPos();
        ImGui::SetCursorScreenPos(ImVec2(draw_origin.x + pad_x, draw_origin.y + pad_y));

        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##target_canvas", canvas_size);

        ImVec2 canvas_end(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
        draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(24, 24, 28, 255));
        draw_list->AddRect(canvas_pos, canvas_end, IM_COL32(80, 80, 90, 255));

        const ImVec2 center(canvas_pos.x + canvas_size.x * 0.5f, canvas_pos.y + canvas_size.y * 0.5f);
        const float bullseye_radius = canvas_size.x * 0.45f;
        const float moa_to_px = bullseye_radius / max_span_moa;

        for (int ring = 1; ring <= TARGET_RING_COUNT; ++ring) {
            const float ring_radius = bullseye_radius * (static_cast<float>(ring) / ring_count);
            draw_list->AddCircle(center, ring_radius, IM_COL32(210, 210, 210, 255), 0, 1.0f);

            char ring_label[64];
            const float ring_moa = moa_per_ring * static_cast<float>(ring);
            const float ring_inches = ring_moa * inches_per_moa;
            std::snprintf(ring_label, sizeof(ring_label), "%.0f MOA (%.1f in)", ring_moa, ring_inches);
            draw_list->AddText(
                ImVec2(center.x + 6.0f, center.y - ring_radius - 14.0f),
                IM_COL32(200, 200, 210, 255),
                ring_label
            );
        }

        draw_list->AddLine(ImVec2(center.x - bullseye_radius, center.y), ImVec2(center.x + bullseye_radius, center.y), IM_COL32(180, 180, 180, 180), 1.0f);
        draw_list->AddLine(ImVec2(center.x, center.y - bullseye_radius), ImVec2(center.x, center.y + bullseye_radius), IM_COL32(180, 180, 180, 180), 1.0f);

        // Positive elevation hold draws up on screen (negative Y in ImGui).
        const ImVec2 impact_point(
            center.x + hold_windage_moa * moa_to_px,
            center.y - hold_elevation_moa * moa_to_px
        );

        // Gaussian error ellipse — multi-shell gradient (center=red/opaque, outer=blue/transparent)
        if (sol.uncertainty_valid && (sol.sigma_elevation_moa > 0.001f || sol.sigma_windage_moa > 0.001f)) {
            const float se  = std::max(sol.sigma_elevation_moa, 0.001f);
            const float sw  = std::max(sol.sigma_windage_moa,   0.001f);
            const float cov = sol.covariance_elev_wind;

            // Eigendecomposition of 2x2 covariance matrix
            const float tr_half    = (se*se + sw*sw) * 0.5f;
            const float disc       = std::sqrt(std::max(((se*se - sw*sw) * 0.5f) * ((se*se - sw*sw) * 0.5f) + cov*cov, 0.0f));
            const float lambda1    = tr_half + disc;   // larger eigenvalue
            const float lambda2    = std::max(tr_half - disc, 0.0f);
            const float axis1_moa  = std::sqrt(lambda1); // major semi-axis (MOA)
            const float axis2_moa  = std::sqrt(lambda2);
            // Rotation: angle of eigenvector-1 from the elevation axis
            const float rot_rad    = std::atan2(lambda1 - se*se, cov);
            const float cos_r      = std::cos(rot_rad);
            const float sin_r      = std::sin(rot_rad);

            constexpr int kShells   = 24;
            constexpr int kSegments = 48;
            constexpr float kMaxSig = 3.0f;

            // Draw filled shells from outermost -> innermost so center is brightest
            for (int shell = kShells - 1; shell >= 0; --shell) {
                const float k   = kMaxSig * (shell + 1.0f) / static_cast<float>(kShells);
                const float t   = static_cast<float>(shell) / static_cast<float>(kShells - 1); // 0=inner, 1=outer
                const uint8_t r = static_cast<uint8_t>(255.0f * (1.0f - t));
                const uint8_t g = static_cast<uint8_t>(120.0f * (1.0f - t * 0.85f));
                const uint8_t b = static_cast<uint8_t>(30.0f  + 200.0f * t);
                const uint8_t a = static_cast<uint8_t>(160.0f * (1.0f - t * 0.80f) + 10.0f);
                const ImU32 col = IM_COL32(r, g, b, a);

                ImVec2 pts[kSegments];
                for (int s = 0; s < kSegments; ++s) {
                    const float ang      = 2.0f * 3.14159265f * s / static_cast<float>(kSegments);
                    const float u        = k * axis1_moa * std::cos(ang);
                    const float v        = k * axis2_moa * std::sin(ang);
                    const float elev_moa = u * cos_r - v * sin_r;
                    const float wind_moa = u * sin_r + v * cos_r;
                    pts[s] = ImVec2(impact_point.x + wind_moa * moa_to_px,
                                    impact_point.y - elev_moa * moa_to_px);
                }
                draw_list->AddConvexPolyFilled(pts, kSegments, col);
            }

            // Labeled contour outlines at 1σ / 2σ / 3σ
            const float sigma_levels[]  = { 1.0f, 2.0f, 3.0f };
            const ImU32 sigma_colors[]  = { IM_COL32(255, 200, 80, 230),
                                             IM_COL32(180, 120, 255, 180),
                                             IM_COL32(80,  160, 255, 140) };
            const char* sigma_labels[]  = { "1\xcf\x83 39%", "2\xcf\x83 86%", "3\xcf\x83 99%" };
            for (int si = 0; si < 3; ++si) {
                const float k = sigma_levels[si];
                ImVec2 pts[kSegments];
                for (int s = 0; s < kSegments; ++s) {
                    const float ang      = 2.0f * 3.14159265f * s / static_cast<float>(kSegments);
                    const float u        = k * axis1_moa * std::cos(ang);
                    const float v        = k * axis2_moa * std::sin(ang);
                    const float elev_moa = u * cos_r - v * sin_r;
                    const float wind_moa = u * sin_r + v * cos_r;
                    pts[s] = ImVec2(impact_point.x + wind_moa * moa_to_px,
                                    impact_point.y - elev_moa * moa_to_px);
                }
                draw_list->AddPolyline(pts, kSegments, sigma_colors[si], ImDrawFlags_Closed, 1.2f);
                // Label near the topmost point of the contour (s=kSegments/4 → top)
                const int label_seg = kSegments / 4;
                draw_list->AddText(ImVec2(pts[label_seg].x + 2.0f, pts[label_seg].y - 12.0f),
                                   sigma_colors[si], sigma_labels[si]);
            }
        }

        draw_list->AddCircleFilled(impact_point, 4.0f, IM_COL32(255, 70, 70, 255));

        ImGui::End();

        ImGui::Begin("Side View Arc");
        ImDrawList* side_draw_list = ImGui::GetWindowDrawList();

        FiringSolution side_sol = {};
        BCE_GetSolution(&side_sol);

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
            } else if (normalized <= 2.0f) {
                snapped = 2.0f;
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
        const float side_range_scale_display = ClampValue(nice_ceil(side_range_display * 1.15f), 10.0f, is_imperial ? 6000.0f : 5000.0f);
        const float side_range_step_display = side_range_scale_display / 5.0f;
        const float side_range_scale_m = side_range_scale_display * range_display_to_m;
        const float side_range_step_m = side_range_step_display * range_display_to_m;
        const float drop_display = drop_m * offset_m_to_display;
        const float side_drop_scale_display = ClampValue(nice_ceil((drop_display * 1.20f) + 0.1f), 1.0f, is_imperial ? 8000.0f : 20000.0f);
        const float side_drop_scale_m = side_drop_scale_display * offset_display_to_m;
        const char* drop_units = is_imperial ? "in" : "cm";
        const float range_display = side_range_display;
        const char* range_units = is_imperial ? "yd" : "m";
        const float range_scale_display = side_range_scale_display;
        const float range_step_display = side_range_step_display;
        const float drop_scale_display = side_drop_scale_display;
        const float elevation_angle_rad = side_sol.hold_elevation_moa * MOA_TO_RAD;
        const float elevation_angle_deg = side_sol.hold_elevation_moa / 60.0f;

        if (ImGui::Button(g_state.side_view_show_required_angle ? "Show Bullet Path" : "Show Required Elevation Angle")) {
            g_state.side_view_show_required_angle = !g_state.side_view_show_required_angle;
        }

        ImGui::Text("Range: %.1f %s", range_display, range_units);
        ImGui::Text("Estimated drop at target: %.2f %s", drop_display, drop_units);
        ImGui::Text("Graph scale: 0..%.0f %s (step %.0f), vertical +/-%.1f %s", range_scale_display, range_units, range_step_display, drop_scale_display, drop_units);
        ImGui::Text("Required elevation angle: %.3f deg", elevation_angle_deg);
        ImGui::Separator();

        ImVec2 side_avail = ImGui::GetContentRegionAvail();
        float side_canvas_w = side_avail.x;
        if (side_canvas_w < 260.0f) side_canvas_w = 260.0f;
        if (side_canvas_w > 900.0f) side_canvas_w = 900.0f;
        float side_canvas_h = side_avail.y;
        if (side_canvas_h < 180.0f) side_canvas_h = 180.0f;
        if (side_canvas_h > 320.0f) side_canvas_h = 320.0f;

        const ImVec2 side_canvas_size(side_canvas_w, side_canvas_h);
        const ImVec2 side_canvas_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##side_view_arc_canvas", side_canvas_size);
        const ImVec2 side_canvas_end(side_canvas_pos.x + side_canvas_size.x, side_canvas_pos.y + side_canvas_size.y);

        side_draw_list->AddRectFilled(side_canvas_pos, side_canvas_end, IM_COL32(24, 24, 28, 255));
        side_draw_list->AddRect(side_canvas_pos, side_canvas_end, IM_COL32(80, 80, 90, 255));

        const float left_margin = 28.0f;
        const float right_margin = 20.0f;
        const float top_margin = 20.0f;
        const float bottom_margin = 24.0f;
        const float plot_left = side_canvas_pos.x + left_margin;
        const float plot_right = side_canvas_end.x - right_margin;
        const float plot_top = side_canvas_pos.y + top_margin;
        const float plot_bottom = side_canvas_end.y - bottom_margin;
        const float plot_width = plot_right - plot_left;
        const float plot_height = plot_bottom - plot_top;

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
            side_draw_list->AddLine(ImVec2(gx, plot_top), ImVec2(gx, plot_bottom), IM_COL32(60, 60, 70, 150), 1.0f);

            char tick_label[32];
            const float tick_display = is_imperial ? (grid_x_m * M_TO_YD) : grid_x_m;
            std::snprintf(tick_label, sizeof(tick_label), "%.0f", tick_display);
            side_draw_list->AddText(ImVec2(gx - 8.0f, plot_bottom + 3.0f), IM_COL32(180, 180, 190, 255), tick_label);
        }

        const float los_y = map_y(0.0f);
        side_draw_list->AddLine(ImVec2(plot_left, los_y), ImVec2(plot_right, los_y), IM_COL32(170, 170, 180, 220), 1.0f);
        side_draw_list->AddLine(ImVec2(plot_left, plot_top), ImVec2(plot_left, plot_bottom), IM_COL32(120, 120, 130, 180), 1.0f);

        const float half_drop_m = -side_drop_scale_m * 0.5f;
        const float half_drop_y = map_y(half_drop_m);
        side_draw_list->AddLine(ImVec2(plot_left, half_drop_y), ImVec2(plot_right, half_drop_y), IM_COL32(70, 70, 80, 140), 1.0f);

        const float half_rise_m = side_drop_scale_m * 0.5f;
        const float half_rise_y = map_y(half_rise_m);
        side_draw_list->AddLine(ImVec2(plot_left, half_rise_y), ImVec2(plot_right, half_rise_y), IM_COL32(70, 70, 80, 140), 1.0f);

        const int sample_count = 64;
        ImVec2 arc_points[sample_count + 1];
        if (!g_state.side_view_show_required_angle) {
            const float tan_theta = std::tanf(elevation_angle_rad);
            for (int i = 0; i <= sample_count; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(sample_count);
                const float x_m = t * side_range_m;
                const float y_m = (tan_theta * x_m) - ((tan_theta / side_range_m) * x_m * x_m);
                arc_points[i] = ImVec2(map_x(x_m), map_y(y_m));
            }
            side_draw_list->AddPolyline(arc_points, sample_count + 1, IM_COL32(98, 203, 255, 255), ImDrawFlags_None, 2.0f);
        } else {
            const ImVec2 aim_start(map_x(0.0f), map_y(0.0f));
            const ImVec2 aim_end(map_x(side_range_m), map_y(drop_m));
            side_draw_list->AddLine(aim_start, aim_end, IM_COL32(255, 190, 90, 255), 2.0f);
        }

        const ImVec2 muzzle_pt(map_x(0.0f), map_y(0.0f));
        const ImVec2 impact_pt = g_state.side_view_show_required_angle
            ? ImVec2(map_x(side_range_m), map_y(drop_m))
            : ImVec2(map_x(side_range_m), map_y(0.0f));
        side_draw_list->AddCircleFilled(muzzle_pt, 3.5f, IM_COL32(130, 255, 130, 255));
        side_draw_list->AddCircleFilled(impact_pt, 4.0f, IM_COL32(255, 90, 90, 255));
        side_draw_list->AddText(ImVec2(muzzle_pt.x + 6.0f, muzzle_pt.y - 16.0f), IM_COL32(200, 230, 200, 255), "Muzzle");
        side_draw_list->AddText(ImVec2(impact_pt.x - 56.0f, impact_pt.y - 16.0f), IM_COL32(230, 200, 200, 255), g_state.side_view_show_required_angle ? "Aim Point" : "Impact");

        side_draw_list->AddText(ImVec2(plot_right - 90.0f, plot_bottom + 4.0f), IM_COL32(190, 190, 200, 255), range_units);

        char los_label[64];
        std::snprintf(los_label, sizeof(los_label), "Line of sight (0 %s)", drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, los_y - 16.0f), IM_COL32(170, 190, 220, 255), los_label);

        char drop_label[64];
        std::snprintf(drop_label, sizeof(drop_label), "Drop @ target: %.2f %s", drop_display, drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, plot_top + 2.0f), IM_COL32(190, 190, 200, 255), drop_label);

        char half_drop_label[64];
        const float half_drop_display = (-half_drop_m) * offset_m_to_display;
        std::snprintf(half_drop_label, sizeof(half_drop_label), "-%.1f %s", half_drop_display, drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, half_drop_y - 14.0f), IM_COL32(150, 150, 165, 255), half_drop_label);

        char half_rise_label[64];
        const float half_rise_display = half_rise_m * offset_m_to_display;
        std::snprintf(half_rise_label, sizeof(half_rise_label), "+%.1f %s", half_rise_display, drop_units);
        side_draw_list->AddText(ImVec2(plot_left + 4.0f, half_rise_y - 14.0f), IM_COL32(150, 150, 165, 255), half_rise_label);

        ImGui::End();

        ImGui::Begin("Top Down Drift");
        ImDrawList* drift_draw_list = ImGui::GetWindowDrawList();

        FiringSolution drift_sol = {};
        BCE_GetSolution(&drift_sol);

        float drift_range_m = drift_sol.horizontal_range_m;
        if (drift_range_m <= 0.0f) {
            drift_range_m = (drift_sol.range_m > 0.0f) ? drift_sol.range_m : g_state.lrf_range;
        }
        drift_range_m = ClampValue(drift_range_m, 1.0f, 5000.0f);

        const float lateral_m = drift_sol.hold_windage_moa * MOA_TO_RAD * drift_range_m;
        const float drift_range_display = drift_range_m * range_m_to_display;
        const float drift_range_scale_display = ClampValue(nice_ceil(drift_range_display * 1.15f), 10.0f, is_imperial ? 6000.0f : 5000.0f);
        const float drift_range_step_display = drift_range_scale_display / 5.0f;
        const float drift_range_scale_m = drift_range_scale_display * range_display_to_m;
        const float drift_range_step_m = drift_range_step_display * range_display_to_m;

        const float drift_display = lateral_m * offset_m_to_display;
        const float drift_scale_display = ClampValue(nice_ceil((std::fabs(drift_display) * 1.25f) + 0.1f), 1.0f, is_imperial ? 6000.0f : 20000.0f);
        const float drift_lateral_scale_m = drift_scale_display * offset_display_to_m;
        const float windage_angle_deg = drift_sol.hold_windage_moa / 60.0f;
        const char* drift_units = is_imperial ? "in" : "cm";
        const char* drift_direction = (drift_display > 0.01f) ? "RIGHT" : ((drift_display < -0.01f) ? "LEFT" : "CENTER");

        if (ImGui::Button(g_state.top_down_show_required_angle ? "Show Bullet Path" : "Show Required Windage Angle")) {
            g_state.top_down_show_required_angle = !g_state.top_down_show_required_angle;
        }

        ImGui::Text("Total side drift: %.2f %s [%s]", std::fabs(drift_display), drift_units, drift_direction);
        ImGui::Text("Graph scale: 0..%.0f %s range, +/-%.1f %s lateral", drift_range_scale_display, range_units, drift_scale_display, drift_units);
        ImGui::Text("Required windage angle: %.3f deg", windage_angle_deg);
        ImGui::Separator();

        ImVec2 drift_avail = ImGui::GetContentRegionAvail();
        float drift_canvas_w = drift_avail.x;
        if (drift_canvas_w < 260.0f) drift_canvas_w = 260.0f;
        if (drift_canvas_w > 900.0f) drift_canvas_w = 900.0f;
        float drift_canvas_h = drift_avail.y;
        if (drift_canvas_h < 180.0f) drift_canvas_h = 180.0f;
        if (drift_canvas_h > 320.0f) drift_canvas_h = 320.0f;

        const ImVec2 drift_canvas_size(drift_canvas_w, drift_canvas_h);
        const ImVec2 drift_canvas_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##top_down_drift_canvas", drift_canvas_size);
        const ImVec2 drift_canvas_end(drift_canvas_pos.x + drift_canvas_size.x, drift_canvas_pos.y + drift_canvas_size.y);

        drift_draw_list->AddRectFilled(drift_canvas_pos, drift_canvas_end, IM_COL32(24, 24, 28, 255));
        drift_draw_list->AddRect(drift_canvas_pos, drift_canvas_end, IM_COL32(80, 80, 90, 255));

        const float drift_left_margin = 28.0f;
        const float drift_right_margin = 20.0f;
        const float drift_top_margin = 20.0f;
        const float drift_bottom_margin = 24.0f;
        const float drift_plot_left = drift_canvas_pos.x + drift_left_margin;
        const float drift_plot_right = drift_canvas_end.x - drift_right_margin;
        const float drift_plot_top = drift_canvas_pos.y + drift_top_margin;
        const float drift_plot_bottom = drift_canvas_end.y - drift_bottom_margin;
        const float drift_plot_width = drift_plot_right - drift_plot_left;
        const float drift_plot_height = drift_plot_bottom - drift_plot_top;
        const float drift_center_x = drift_plot_left + (drift_plot_width * 0.5f);

        auto map_drift_x = [&](float lateral_offset_m) -> float {
            return drift_center_x + (lateral_offset_m / drift_lateral_scale_m) * (drift_plot_width * 0.5f);
        };
        auto map_drift_y = [&](float forward_range_m) -> float {
            return drift_plot_bottom - (forward_range_m / drift_range_scale_m) * drift_plot_height;
        };

        for (int i = 0; i <= 5; ++i) {
            const float grid_x_m = drift_range_step_m * static_cast<float>(i);
            const float gy = map_drift_y(grid_x_m);
            drift_draw_list->AddLine(ImVec2(drift_plot_left, gy), ImVec2(drift_plot_right, gy), IM_COL32(60, 60, 70, 150), 1.0f);

            char tick_label[32];
            const float tick_display = grid_x_m * range_m_to_display;
            std::snprintf(tick_label, sizeof(tick_label), "%.0f", tick_display);
            drift_draw_list->AddText(ImVec2(drift_plot_left + 3.0f, gy - 8.0f), IM_COL32(180, 180, 190, 255), tick_label);
        }

        const float drift_right_tick_x = map_drift_x(drift_lateral_scale_m * 0.5f);
        const float drift_left_tick_x = map_drift_x(-drift_lateral_scale_m * 0.5f);
        drift_draw_list->AddLine(ImVec2(drift_center_x, drift_plot_top), ImVec2(drift_center_x, drift_plot_bottom), IM_COL32(170, 170, 180, 220), 1.0f);
        drift_draw_list->AddLine(ImVec2(drift_right_tick_x, drift_plot_top), ImVec2(drift_right_tick_x, drift_plot_bottom), IM_COL32(70, 70, 80, 130), 1.0f);
        drift_draw_list->AddLine(ImVec2(drift_left_tick_x, drift_plot_top), ImVec2(drift_left_tick_x, drift_plot_bottom), IM_COL32(70, 70, 80, 130), 1.0f);
        drift_draw_list->AddLine(ImVec2(drift_plot_left, drift_plot_bottom), ImVec2(drift_plot_right, drift_plot_bottom), IM_COL32(120, 120, 130, 180), 1.0f);

        ImVec2 drift_points[sample_count + 1];
        if (!g_state.top_down_show_required_angle) {
            for (int i = 0; i <= sample_count; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(sample_count);
                const float forward_range_m = t * drift_range_m;
                const float lateral_offset_m = lateral_m * t * t;
                drift_points[i] = ImVec2(map_drift_x(lateral_offset_m), map_drift_y(forward_range_m));
            }
            drift_draw_list->AddPolyline(drift_points, sample_count + 1, IM_COL32(255, 180, 90, 255), ImDrawFlags_None, 2.0f);
        } else {
            const ImVec2 aim_start(map_drift_x(0.0f), map_drift_y(0.0f));
            const ImVec2 aim_end(map_drift_x(lateral_m), map_drift_y(drift_range_m));
            drift_draw_list->AddLine(aim_start, aim_end, IM_COL32(90, 220, 255, 255), 2.0f);
        }

        const ImVec2 drift_muzzle_pt(map_drift_x(0.0f), map_drift_y(0.0f));
        const ImVec2 drift_impact_pt(map_drift_x(lateral_m), map_drift_y(drift_range_m));
        drift_draw_list->AddCircleFilled(drift_muzzle_pt, 3.5f, IM_COL32(130, 255, 130, 255));
        drift_draw_list->AddCircleFilled(drift_impact_pt, 4.0f, IM_COL32(255, 90, 90, 255));
        drift_draw_list->AddText(ImVec2(drift_muzzle_pt.x + 6.0f, drift_muzzle_pt.y - 16.0f), IM_COL32(200, 230, 200, 255), "Muzzle");
        drift_draw_list->AddText(ImVec2(drift_impact_pt.x - 56.0f, drift_impact_pt.y - 16.0f), IM_COL32(230, 200, 200, 255), g_state.top_down_show_required_angle ? "Aim Point" : "Impact");

        drift_draw_list->AddText(ImVec2(drift_plot_left + 4.0f, drift_plot_top - 16.0f), IM_COL32(190, 190, 200, 255), range_units);

        char center_label[64];
        std::snprintf(center_label, sizeof(center_label), "Centerline (0 %s)", drift_units);
        drift_draw_list->AddText(ImVec2(drift_center_x + 4.0f, drift_plot_top + 2.0f), IM_COL32(170, 190, 220, 255), center_label);

        char top_tick_label[64];
        const float top_tick_display = (drift_lateral_scale_m * 0.5f) * offset_m_to_display;
        std::snprintf(top_tick_label, sizeof(top_tick_label), "+%.1f %s", top_tick_display, drift_units);
        drift_draw_list->AddText(ImVec2(drift_right_tick_x + 4.0f, drift_plot_top + 2.0f), IM_COL32(150, 150, 165, 255), top_tick_label);

        char bot_tick_label[64];
        std::snprintf(bot_tick_label, sizeof(bot_tick_label), "-%.1f %s", top_tick_display, drift_units);
        drift_draw_list->AddText(ImVec2(drift_left_tick_x - 56.0f, drift_plot_top + 2.0f), IM_COL32(150, 150, 165, 255), bot_tick_label);

        ImGui::End();

        ImGui::Render();
        const float clear_color_with_alpha[4] = {0.10f, 0.10f, 0.12f, 1.00f};
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        g_pSwapChain->Present(1, 0);
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    DestroyWindow(hwnd);
    UnregisterClass(wc.lpszClassName, wc.hInstance);

    return 0;
}
