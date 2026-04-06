#pragma once
#include <stdint.h>

struct DecoResult {
    bool inDeco;
    float nextStopDepth;  // meter to nearest 3 m
    int stopTime;         // minute at next stop
    int timeToSurface;    // minute time to surface
};

// Initialise tissue compartments
void decoInit();

// Update tissue
void decoUpdate(float pressureAtm, float dtMin);

// Calculate decompression
DecoResult decoCompute(float currentPressureAtm);

// Enable/disable raw model (no gradient factors)
void mad_man_mode(bool enabled);