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

constexpr float MPS_TO_FPS = 3.280839895f;
constexpr float FPS_TO_MPS = 0.3048f;
constexpr float MPS_TO_MPH = 2.23693629f;
constexpr float MPH_TO_MPS = 0.44704f;
constexpr float M_TO_YD = 1.093613298f;
constexpr float YD_TO_M = 0.9144f;
constexpr float MM_TO_IN = 0.0393700787f;
constexpr float IN_TO_MM = 25.4f;
constexpr float J_TO_FTLB = 0.737562149f;
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
    float barrel_length_in;
    float mv_adjustment_factor;
    float twist_rate_inches;
    float length_mm;
};

// Note: these are GUI harness/demo presets only.
// They are NOT consumed by the BCE engine internals and are only used to
// pre-fill manual input fields for quick validation runs.
const CartridgeProfile kCartridgeProfiles[] = {
    { ".308 Win 175gr SMK", 0.505f, 0, 792.0f, 175.0f, 0.308f, 24.0f, 25.0f, 10.0f, 31.2f },
    { ".223 Rem 55gr FMJ", 0.245f, 0, 990.0f, 55.0f, 0.223f, 20.0f, 25.0f, 12.0f, 19.0f },
    { "6.5 Creedmoor 140gr ELD-M", 0.326f, 6, 823.0f, 140.0f, 0.264f, 26.0f, 25.0f, 8.0f, 35.3f },
    { ".300 Win Mag 190gr BTHP", 0.533f, 0, 884.0f, 190.0f, 0.308f, 24.0f, 30.0f, 10.0f, 34.0f }
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
    float barrel_length_in = 24.0f;
    float mv_adjustment_factor = 25.0f;
    float twist_rate_inches = 10.0f;
};

struct RiflePreset {
    std::string name;
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
    char new_rifle_preset_name[64] = "";
    int selected_cartridge_preset = -1;
    int selected_rifle_preset = -1;
    std::string output_text;
    std::string last_action;
};

GuiState g_state;
std::vector<CartridgePreset> g_cartridge_presets;
std::vector<RiflePreset> g_rifle_presets;

using json = nlohmann::json;

void SanitizeCartridgePreset(CartridgePreset& preset);
void SanitizeRiflePreset(RiflePreset& preset);
json SerializeCartridgePreset(const CartridgePreset& input);
json SerializeRiflePreset(const RiflePreset& input);

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

int FindRiflePresetByName(const std::string& name) {
    for (size_t i = 0; i < g_rifle_presets.size(); ++i) {
        if (g_rifle_presets[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

const CartridgeProfile* FindBuiltInCartridgeProfileByName(const std::string& name) {
    for (int i = 0; i < kNumCartridgeProfiles; ++i) {
        if (name == kCartridgeProfiles[i].name) {
            return &kCartridgeProfiles[i];
        }
    }
    return nullptr;
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
            p.barrel_length_in = src.barrel_length_in;
            p.mv_adjustment_factor = src.mv_adjustment_factor;
            p.twist_rate_inches = src.twist_rate_inches;
            SanitizeCartridgePreset(p);
            g_cartridge_presets.push_back(p);
        }
    }

    if (g_rifle_presets.empty()) {
        RiflePreset base;
        base.name = "Default Rifle";
        base.barrel_length_in = 24.0f;
        base.mv_adjustment_factor = 25.0f;
        base.twist_rate_inches = 10.0f;
        base.zero_range_m = 100.0f;
        base.sight_height_mm = 38.1f;
        SanitizeRiflePreset(base);
        g_rifle_presets.push_back(base);
    }

    for (auto& preset : g_cartridge_presets) {
        SanitizeCartridgePreset(preset);
    }
    for (auto& preset : g_rifle_presets) {
        SanitizeRiflePreset(preset);
    }

    if (g_state.selected_cartridge_preset < 0 && !g_cartridge_presets.empty()) {
        g_state.selected_cartridge_preset = 0;
    }
    if (g_state.selected_rifle_preset < 0 && !g_rifle_presets.empty()) {
        g_state.selected_rifle_preset = 0;
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
    preset.barrel_length_in = ClampValue(preset.barrel_length_in, 8.0f, 40.0f);
    preset.mv_adjustment_factor = ClampValue(std::fabs(preset.mv_adjustment_factor), 0.0f, 200.0f);
    preset.twist_rate_inches = ClampValue(std::fabs(preset.twist_rate_inches), 1.0f, 30.0f);
}

void SanitizeRiflePreset(RiflePreset& preset) {
    if (preset.name.empty()) {
        preset.name = "Unnamed Rifle";
    }
    preset.barrel_length_in = ClampValue(preset.barrel_length_in, 8.0f, 40.0f);
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
    item["barrel_length_in"] = preset.barrel_length_in;
    item["mv_adjustment_factor"] = preset.mv_adjustment_factor;
    item["twist_rate_inches"] = preset.twist_rate_inches;
    return item;
}

json SerializeRiflePreset(const RiflePreset& input) {
    RiflePreset preset = input;
    SanitizeRiflePreset(preset);

    json item;
    item["name"] = preset.name;
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
    std::snprintf(g_state.preset_path, sizeof(g_state.preset_path), "%s", "bce_gui_preset.json");
    std::snprintf(g_state.profile_library_path, sizeof(g_state.profile_library_path), "%s", "bce_gui_profile_library.json");
    std::snprintf(g_state.new_cartridge_preset_name, sizeof(g_state.new_cartridge_preset_name), "%s", "New Cartridge Preset");
    std::snprintf(g_state.new_rifle_preset_name, sizeof(g_state.new_rifle_preset_name), "%s", "New Rifle Preset");
    g_state.selected_cartridge_preset = -1;
    g_state.selected_rifle_preset = -1;

    g_cartridge_presets.clear();
    g_rifle_presets.clear();
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
    g_state.bullet.barrel_length_in = normalized.barrel_length_in;
    g_state.bullet.mv_adjustment_factor = normalized.mv_adjustment_factor;
    g_state.bullet.twist_rate_inches = normalized.twist_rate_inches;
    g_state.override_drag_coefficient = true;
    g_state.manual_drag_coefficient = normalized.bc;
}

void ApplyRiflePreset(const RiflePreset& preset) {
    RiflePreset normalized = preset;
    SanitizeRiflePreset(normalized);

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
    preset.barrel_length_in = g_state.bullet.barrel_length_in;
    preset.mv_adjustment_factor = g_state.bullet.mv_adjustment_factor;
    preset.twist_rate_inches = g_state.bullet.twist_rate_inches;
    SanitizeCartridgePreset(preset);
    return preset;
}

RiflePreset CaptureCurrentRiflePreset(const std::string& name) {
    RiflePreset preset;
    preset.name = name;
    preset.barrel_length_in = g_state.bullet.barrel_length_in;
    preset.mv_adjustment_factor = g_state.bullet.mv_adjustment_factor;
    preset.twist_rate_inches = g_state.bullet.twist_rate_inches;
    preset.zero_range_m = g_state.zero.zero_range_m;
    preset.sight_height_mm = g_state.zero.sight_height_mm;
    SanitizeRiflePreset(preset);
    return preset;
}

void SaveProfileLibrary() {
    EnsureProfileDefaults();

    json root;
    root["cartridge_presets"] = json::array();
    root["rifle_presets"] = json::array();

    for (const auto& preset : g_cartridge_presets) {
        root["cartridge_presets"].push_back(SerializeCartridgePreset(preset));
    }

    for (const auto& preset : g_rifle_presets) {
        root["rifle_presets"].push_back(SerializeRiflePreset(preset));
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
        std::vector<RiflePreset> loaded_rifle;
        bool used_legacy_defaults = false;

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
                const CartridgeProfile* built_in = FindBuiltInCartridgeProfileByName(preset.name);
                const float fallback_barrel = built_in ? built_in->barrel_length_in : 24.0f;
                const float fallback_mv_adjust = built_in ? built_in->mv_adjustment_factor : 25.0f;
                const float fallback_twist = built_in ? built_in->twist_rate_inches : 10.0f;
                // Legacy profile files predate barrel/twist/MV-adjust fields.
                // Fall back to matching built-in cartridge characteristics.
                if (!item.contains("barrel_length_in") || !item.contains("mv_adjustment_factor") || !item.contains("twist_rate_inches")) {
                    used_legacy_defaults = true;
                }
                preset.barrel_length_in = item.value("barrel_length_in", fallback_barrel);
                preset.mv_adjustment_factor = std::fabs(item.value("mv_adjustment_factor", fallback_mv_adjust));
                preset.twist_rate_inches = item.value("twist_rate_inches", fallback_twist);
                SanitizeCartridgePreset(preset);
                loaded_cartridge.push_back(preset);
            }
        }

        if (root.contains("rifle_presets") && root["rifle_presets"].is_array()) {
            for (const auto& item : root["rifle_presets"]) {
                if (!item.contains("name") || !item["name"].is_string()) {
                    continue;
                }
                RiflePreset preset;
                preset.name = item["name"].get<std::string>();
                preset.barrel_length_in = item.value("barrel_length_in", 24.0f);
                preset.mv_adjustment_factor = std::fabs(item.value("mv_adjustment_factor", 25.0f));
                preset.twist_rate_inches = item.value("twist_rate_inches", 10.0f);
                preset.zero_range_m = item.value("zero_range_m", 100.0f);
                preset.sight_height_mm = item.value("sight_height_mm", 38.1f);
                SanitizeRiflePreset(preset);
                loaded_rifle.push_back(preset);
            }
        }

        if (!loaded_cartridge.empty()) {
            g_cartridge_presets = loaded_cartridge;
            g_state.selected_cartridge_preset = 0;
        }
        if (!loaded_rifle.empty()) {
            g_rifle_presets = loaded_rifle;
            g_state.selected_rifle_preset = 0;
        }

        EnsureProfileDefaults();
        g_state.last_action = used_legacy_defaults
            ? "Load Profile Library (legacy fields upgraded)"
            : "Load Profile Library";
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

        if (ImGui::CollapsingHeader("Rifle Presets (GUI-only)")) {
            ImGui::InputText("Rifle Preset Name", g_state.new_rifle_preset_name, IM_ARRAYSIZE(g_state.new_rifle_preset_name));
            if (ImGui::Button("Add/Update Rifle Preset")) {
                const std::string preset_name = g_state.new_rifle_preset_name;
                if (!preset_name.empty()) {
                    RiflePreset preset = CaptureCurrentRiflePreset(preset_name);
                    const int existing = FindRiflePresetByName(preset_name);
                    if (existing >= 0) {
                        g_rifle_presets[existing] = preset;
                        g_state.selected_rifle_preset = existing;
                    } else {
                        g_rifle_presets.push_back(preset);
                        g_state.selected_rifle_preset = static_cast<int>(g_rifle_presets.size()) - 1;
                    }
                    g_state.last_action = "Rifle Preset Saved";
                    RefreshOutput();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Apply Selected Rifle") &&
                g_state.selected_rifle_preset >= 0 &&
                g_state.selected_rifle_preset < static_cast<int>(g_rifle_presets.size())) {
                ApplyRiflePreset(g_rifle_presets[g_state.selected_rifle_preset]);
                g_state.last_action = "Rifle Preset Applied";
                ApplyConfig();
                RefreshOutput();
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Selected Rifle") &&
                g_state.selected_rifle_preset >= 0 &&
                g_state.selected_rifle_preset < static_cast<int>(g_rifle_presets.size())) {
                g_rifle_presets.erase(g_rifle_presets.begin() + g_state.selected_rifle_preset);
                if (g_rifle_presets.empty()) {
                    g_state.selected_rifle_preset = -1;
                } else if (g_state.selected_rifle_preset >= static_cast<int>(g_rifle_presets.size())) {
                    g_state.selected_rifle_preset = static_cast<int>(g_rifle_presets.size()) - 1;
                }
                g_state.last_action = "Rifle Preset Removed";
                RefreshOutput();
            }

            if (ImGui::BeginListBox("##rifle_presets", ImVec2(-FLT_MIN, 110.0f))) {
                for (int i = 0; i < static_cast<int>(g_rifle_presets.size()); ++i) {
                    const bool is_selected = (g_state.selected_rifle_preset == i);
                    if (ImGui::Selectable(g_rifle_presets[i].name.c_str(), is_selected)) {
                        g_state.selected_rifle_preset = i;
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
        const float confidence_radius_moa = ClampValue((1.0f - g_state.lrf_conf) * 5.0f, 0.0f, 5.0f);

        // Auto-scale the target so both hold offset and confidence circle fit.
        const float ring_count = static_cast<float>(TARGET_RING_COUNT);
        const float required_span_moa = impact_distance_moa + confidence_radius_moa;
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
        const float confidence_radius_in = confidence_radius_moa * inches_per_moa;

        ImGui::Text("Scale: %.0f MOA/ring (%.2f in/ring @ %.1f yd)", moa_per_ring, inches_per_ring, display_range_yd);
        ImGui::Text("Offset: Elev %.2f MOA (%.2f in), Wind %.2f MOA (%.2f in)", hold_elevation_moa, elev_offset_in, hold_windage_moa, wind_offset_in);
        ImGui::Text("Center->Impact: %.2f MOA (%.2f in)", impact_distance_moa, impact_distance_in);
        ImGui::Text("Confidence Radius: %.2f MOA (%.2f in)", confidence_radius_moa, confidence_radius_in);
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
        const float confidence_radius_px = confidence_radius_moa * moa_to_px;

        draw_list->AddCircle(impact_point, confidence_radius_px, IM_COL32(255, 220, 64, 220), 0, 1.5f);
        draw_list->AddCircleFilled(impact_point, 4.0f, IM_COL32(255, 70, 70, 255));

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
