/**
 * @file main.cpp
 * @brief Minimal embedded entry point that boots the BCE engine.
 *
 * Platform/application-specific sensor plumbing is expected to be added by
 * firmware code around this skeleton.
 */

// Placeholder for ESP32 application main
// The BCE library lives in lib/bce/ and is platform-agnostic.
// This file is only compiled for the esp32p4 environment.

#include "bce/bce_api.h"

extern "C" void app_main(void) {
    BCE_Init();
    // Application layer would feed SensorFrames here
}
