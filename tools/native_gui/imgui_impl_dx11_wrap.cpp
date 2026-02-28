/**
 * @file imgui_impl_dx11_wrap.cpp
 * @brief Wrapper TU that compiles Dear ImGui's DX11 backend into this target.
 *
 * Keeping this include in a local source file avoids patching third_party
 * project files while still linking backend implementation code.
 */

#include "../third_party/imgui/backends/imgui_impl_dx11.cpp"