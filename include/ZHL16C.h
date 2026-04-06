#pragma once
#include <stdint.h>

struct DecoResult {
    bool inDeco;
    float nextStopDepth;  // meter to nearest 3 m
    int stopTime;         // minute at next stop
    int timeToSurface;    // minute time to surface
    float surfGF;         // percentage surface GF
};

// Initialise tissue compartments
void decoInit();

// Update tissue
void decoUpdate(float pressureAtm, float dtMin);

// Calculate decompression
DecoResult decoCompute(float currentPressureAtm);

// Disable/enable gradient factors
void mad_man_mode(bool enabled);