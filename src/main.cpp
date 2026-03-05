/**
 * @file main.cpp
 * @brief Minimal embedded entry point that boots the DOPE engine.
 *
 * Platform/application-specific sensor plumbing is expected to be added by
 * firmware code around this skeleton.
 */

// Placeholder for ESP32 application main
// The DOPE library lives in lib/dope/ and is platform-agnostic.
// This file is only compiled for the esp32sim environment.

#include "dope/dope_api.h"

#if defined(DOPE_PLATFORM_NATIVE)
int main() {
    DOPE_Init();
    return 0;
}
#else
extern "C" void app_main(void) {
    DOPE_Init();
    // Application layer would feed SensorFrames here
}
#endif
