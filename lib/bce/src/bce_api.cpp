/**
 * @file bce_api.cpp
 * @brief C-linkage API implementation — wraps the BCE_Engine singleton.
 *
 * All functions operate on a single static BCE_Engine instance.
 * Zero heap allocation after init.
 */

#include "bce/bce_api.h"
#include "engine/bce_engine.h"

// Single static engine instance — no heap
static BCE_Engine s_engine;

extern "C" {

void BCE_Init(void) {
    s_engine.init();
}

void BCE_Update(const SensorFrame* frame) {
    s_engine.update(frame);
}

void BCE_SetBulletProfile(const BulletProfile* profile) {
    s_engine.setBulletProfile(profile);
}

void BCE_SetZeroConfig(const ZeroConfig* config) {
    s_engine.setZeroConfig(config);
}

void BCE_SetWindManual(float speed_ms, float heading_deg) {
    s_engine.setWindManual(speed_ms, heading_deg);
}

void BCE_SetLatitude(float latitude_deg) {
    s_engine.setLatitude(latitude_deg);
}

void BCE_SetDefaultOverrides(const BCE_DefaultOverrides* defaults) {
    s_engine.setDefaultOverrides(defaults);
}

void BCE_SetIMUBias(const float accel_bias[3], const float gyro_bias[3]) {
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    s_engine.setIMUBias(accel_bias ? accel_bias : zero,
                        gyro_bias ? gyro_bias : zero);
}

void BCE_SetMagCalibration(const float hard_iron[3], const float soft_iron[9]) {
    const float zero_hi[3] = {0.0f, 0.0f, 0.0f};
    const float identity_si[9] = {1.0f, 0.0f, 0.0f,
                                  0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f};
    s_engine.setMagCalibration(hard_iron ? hard_iron : zero_hi,
                               soft_iron ? soft_iron : identity_si);
}

void DOPE_CalibrateGyro(void) {
    s_engine.calibrateGyro();
}

void BCE_SetBoresightOffset(const BoresightOffset* offset) {
    if (!offset) return;
    s_engine.setBoresightOffset(offset->vertical_moa, offset->horizontal_moa);
}

void BCE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa) {
    s_engine.setReticleOffset(vertical_moa, horizontal_moa);
}

void BCE_CalibrateBaro(void) {
    s_engine.calibrateBaro();
}

void BCE_CalibrateGyro(void) {
    // Compatibility alias for legacy integrations.
    s_engine.calibrateGyro();
}

void BCE_SetAHRSAlgorithm(AHRS_Algorithm algo) {
    s_engine.setAHRSAlgorithm(algo);
}

void BCE_SetMagDeclination(float declination_deg) {
    s_engine.setMagDeclination(declination_deg);
}

void BCE_SetExternalReferenceMode(bool enabled) {
    s_engine.setExternalReferenceMode(enabled);
}

void BCE_GetSolution(FiringSolution* out) {
    s_engine.getSolution(out);
}

BCE_Mode BCE_GetMode(void) {
    return s_engine.getMode();
}

uint32_t BCE_GetFaultFlags(void) {
    return s_engine.getFaultFlags();
}

uint32_t BCE_GetDiagFlags(void) {
    return s_engine.getDiagFlags();
}

} // extern "C"
