# Key Differences: Current vs. Legacy gui_main.cpp

This document summarizes the major shifts and differences between the current `gui_main.cpp` and the legacy files (`gui_main_old.cpp`, `gui_main_merged.cpp`, `gui_main_utf8.cpp`).

## 1. Project Rebranding (BCE to DOPE)
- **Legacy (`gui_main_old.cpp`):** Used the `bce` namespace and `BCE_` prefix for API calls (e.g., `BCE_Update`, `BCE_ErrorTable`).
- **Current:** Uses the `dope` namespace and `DOPE_` prefix (e.g., `DOPE_Update`, `DOPE_ErrorTable`).

## 2. Advanced Physics & Preset Expansion
- **Presets:** The current `CartridgePreset` and `GunPreset` structures have been significantly expanded.
  - **New Cartridge Fields:** `sd_mv_fps`, `mv_adjustment_fps_per_in`, `barrel_mv_profile`, `muzzle_diameter_in`, `cold_bore_velocity_bias`, `angular_sigma_moa`, and MOA-based uncertainty fields (`cep50`, `manufacturer_spec`).
  - **New Gun Fields:** `barrel_material`, `free_floated`, `suppressor_attached`, `barrel_tuner_attached`.
- **Dynamic Logic:** The current version includes complex interpolation for barrel length vs. muzzle velocity (`ComputeAdjustedMv`) and a refined barrel stiffness model (`ComputeStiffnessMoa`).

## 3. GUI State & Functionality
- **State Management:** `GuiState` now tracks much more information, including target elevation, barrel finish, and specific hardware sensor calibration states.
- **Uncertainty Propagation:** The current version has a much more robust uncertainty propagation system, including background jobs (`g_uncertainty_job`) and batch processing flags to prevent UI blocking.
- **Hardware Presets:** The hardware sensor resolution logic is now more flexible, supporting dynamic label mapping and resolution between static presets and user-loaded JSON data.

## 4. Cross-Platform Support
- **Legacy Variants (`gui_main_merged.cpp` / `gui_main_utf8.cpp`):** Contained `#ifdef _WIN32` / `#else` blocks to support both DirectX 11 (Windows) and GLFW/OpenGL (Linux) in a single file.
- **Current (`gui_main.cpp`):** Appears to have specialized back into a Windows-focused DirectX 11 implementation, likely for performance or stability during active development on Windows.

## 5. File Encoding (UTF-8)
- Many legacy files had `_utf8` variants. The current `gui_main.cpp` is properly encoded and handles Unicode/UTF-8 symbols (like degree signs and atmospheric units) directly, making the separate `_utf8.cpp` files redundant.
