/**
 * @file dope_api.cpp
 * @brief C-linkage API implementation — wraps the DOPE_Engine singleton.
 *
 * All functions operate on a single static DOPE_Engine instance.
 * Zero heap allocation after init.
 */

#include "dope/dope_api.h"
#include "engine/dope_engine.h"

// Single static engine instance — no heap
static DOPE_Engine s_engine;

extern "C" {

void DOPE_Init(void) {
    s_engine.init();
}

void DOPE_Update(const SensorFrame* frame) {
    s_engine.update(frame);
}

void DOPE_SetBulletProfile(const BulletProfile* profile) {
    s_engine.setBulletProfile(profile);
}

void DOPE_SetZeroConfig(const ZeroConfig* config) {
    s_engine.setZeroConfig(config);
}

void DOPE_SetWindManual(float speed_ms, float heading_deg) {
    s_engine.setWindManual(speed_ms, heading_deg);
}

void DOPE_SetLatitude(float latitude_deg) {
    s_engine.setLatitude(latitude_deg);
}

void DOPE_NotifyShotFired(uint64_t timestamp_us, float ambient_temp_c) {
    s_engine.notifyShotFired(timestamp_us, ambient_temp_c);
}

void DOPE_SetDefaultOverrides(const DOPE_DefaultOverrides* defaults) {
    s_engine.setDefaultOverrides(defaults);
}

void DOPE_SetIMUBias(const float accel_bias[3], const float gyro_bias[3]) {
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    s_engine.setIMUBias(accel_bias ? accel_bias : zero,
                        gyro_bias ? gyro_bias : zero);
}

void DOPE_SetMagCalibration(const float hard_iron[3], const float soft_iron[9]) {
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

void DOPE_SetBoresightOffset(const BoresightOffset* offset) {
    if (offset) {
        s_engine.setBoresightOffset(offset->vertical_moa, offset->horizontal_moa);
    }
}

void DOPE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa) {
    s_engine.setReticleOffset(vertical_moa, horizontal_moa);
}

void DOPE_CalibrateBaro(void) {
    s_engine.calibrateBaro();
}

void DOPE_SetAHRSAlgorithm(AHRS_Algorithm algo) {
    s_engine.setAHRSAlgorithm(algo);
}

void DOPE_SetAHRSConfig(const DOPE_AHRSConfig* config) {
    s_engine.setAHRSConfig(config);
}

void DOPE_SetLRFConfig(const DOPE_LRFConfig* config) {
    s_engine.setLRFConfig(config);
}

void DOPE_SetMagDeclination(float declination_deg) {
    s_engine.setMagDeclination(declination_deg);
}

void DOPE_SetExternalReferenceMode(bool enabled) {
    s_engine.setExternalReferenceMode(enabled);
}

float DOPE_InterpolateSigma(const DOPE_ErrorTable* table, float x) {
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

void DOPE_SetUncertaintyConfig(const UncertaintyConfig* config) {
    s_engine.setUncertaintyConfig(config);
}

void DOPE_GetDefaultUncertaintyConfig(UncertaintyConfig* out) {
    DOPE_Engine::getDefaultUncertaintyConfig(out);
}

void DOPE_GetSolution(FiringSolution* out) {
    s_engine.getSolution(out);
}

void DOPE_GetRealtimeSolution(RealtimeSolution* out) {
    s_engine.getRealtimeSolution(out);
}

DOPE_Mode DOPE_GetMode(void) {
    return s_engine.getMode();
}

uint32_t DOPE_GetFaultFlags(void) {
    return s_engine.getFaultFlags();
}

uint32_t DOPE_GetDiagFlags(void) {
    return s_engine.getDiagFlags();
}

float DOPE_GetHFOV(void) {
    return s_engine.getHFOV();
}

float DOPE_GetVFOV(void) {
    return s_engine.getVFOV();
}

} // extern "C"
