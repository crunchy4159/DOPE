#pragma once

namespace dope {
namespace math {

constexpr float PI = 3.14159265358979323846f;

constexpr float MPS_TO_FPS = 3.280839895f;
constexpr float FPS_TO_MPS = 0.3048f;
constexpr float MPS_TO_MPH = 2.23693629f;
constexpr float MPH_TO_MPS = 0.44704f;

constexpr float M_TO_YD = 1.093613298f;
constexpr float YD_TO_M = 0.9144f;
constexpr float MM_TO_IN = 0.0393700787f;
constexpr float IN_TO_MM = 25.4f;
constexpr float INCHES_TO_M = 0.0254f;
constexpr float MM_TO_M = 0.001f;

constexpr float J_TO_FTLB = 0.737562149f;
constexpr float FTLB_TO_J = 1.355817948f;

constexpr float MOA_TO_RAD = PI / (180.0f * 60.0f);
constexpr float RAD_TO_MOA = (180.0f * 60.0f) / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float MRAD_TO_RAD = 0.001f;
constexpr float RAD_TO_MRAD = 1000.0f;

constexpr float GRAINS_TO_KG = 6.479891e-5f;

// Legacy ballistic drag conversion constant used by the point-mass model.
// Kept here to centralize model-specific scalars alongside other math constants.
constexpr float BALLISTIC_DRAG_CONSTANT = 900.0f;

} // namespace math
} // namespace dope
