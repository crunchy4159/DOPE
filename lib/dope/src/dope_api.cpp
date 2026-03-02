/**
 * @file dope_api.cpp
 * @brief C-linkage API implementation — wraps the BCE_Engine singleton.
 *
 * All functions operate on a single static BCE_Engine instance.
 * Zero heap allocation after init.
 */

#include "dope/dope_api.h"
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

void BCE_CalibrateGyro(void) {
    s_engine.calibrateGyro();
}

void BCE_SetBoresightOffset(const BoresightOffset* offset) {
    if (offset) {
        s_engine.setBoresightOffset(offset->vertical_moa, offset->horizontal_moa);
    }
}

void BCE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa) {
    s_engine.setReticleOffset(vertical_moa, horizontal_moa);
}

void BCE_CalibrateBaro(void) {
    s_engine.calibrateBaro();
}

void BCE_SetAHRSAlgorithm(AHRS_Algorithm algo) {
    s_engine.setAHRSAlgorithm(algo);
}

void BCE_SetAHRSConfig(const BCE_AHRSConfig* config) {
    s_engine.setAHRSConfig(config);
}

void BCE_SetLRFConfig(const BCE_LRFConfig* config) {
    s_engine.setLRFConfig(config);
}

void BCE_SetMagDeclination(float declination_deg) {
    s_engine.setMagDeclination(declination_deg);
}

void BCE_SetExternalReferenceMode(bool enabled) {
    s_engine.setExternalReferenceMode(enabled);
}

float BCE_InterpolateSigma(const BCE_ErrorTable* table, float x) {
    if (!table || table->count <= 0) return 0.0f;
    if (table->count == 1)  return table->points[0].sigma;
    if (x <= table->points[0].x) return table->points[0].sigma;
    if (x >= table->points[table->count - 1].x)
        return table->points[table->count - 1].sigma;
    for (int i = 0; i < table->count - 1; ++i) {
        if (x <= table->points[i + 1].x) {
            float t = (x - table->points[i].x)
                    / (table->points[i + 1].x - table->points[i].x);
            return table->points[i].sigma
                 + t * (table->points[i + 1].sigma - table->points[i].sigma);
        }
    }
    return table->points[table->count - 1].sigma;
}

void BCE_SetUncertaintyConfig(const UncertaintyConfig* config) {
    s_engine.setUncertaintyConfig(config);
}

void BCE_GetDefaultUncertaintyConfig(UncertaintyConfig* out) {
    BCE_Engine::getDefaultUncertaintyConfig(out);
}

void BCE_GetSolution(FiringSolution* out) {
    s_engine.getSolution(out);
}

void BCE_GetRealtimeSolution(RealtimeSolution* out) {
    s_engine.getRealtimeSolution(out);
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

float BCE_GetHFOV(void) {
    return s_engine.getHFOV();
}

float BCE_GetVFOV(void) {
    return s_engine.getVFOV();
}

} // extern "C"
