/**
 * @file imgui_all.cpp
 * @brief Single translation unit that compiles core Dear ImGui sources.
 *
 * This keeps native GUI build wiring simple while leaving vendor code layout
 * unchanged under third_party.
 */

#include "../third_party/imgui/imgui.cpp"
#include "../third_party/imgui/imgui_draw.cpp"
#include "../third_party/imgui/imgui_tables.cpp"
#include "../third_party/imgui/imgui_widgets.cpp"