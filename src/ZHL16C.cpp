// Buhlmann ZHL-16C model
// Fixed GF, CCR mode single set point
// No helium penalty
// Assume no prior dives

#include "ZHL16C.h"
#include <math.h>
#include <string.h>

// ZHL-16C Constants
static const float kHT[16] = {  // N2 half-times (minutes)
    5.0f, 8.0f, 12.5f, 18.5f, 27.0f, 38.3f, 54.3f, 77.0f,
    109.0f, 146.0f, 187.0f, 239.0f, 305.0f, 390.0f, 498.0f, 635.0f
};
static const float kA[16] = {  // N2 a-values
    1.1696f, 1.0000f, 0.8618f, 0.7562f, 0.6491f, 0.5316f, 0.4681f, 0.4301f,
    0.4049f, 0.3719f, 0.3447f, 0.3176f, 0.2828f, 0.2716f, 0.2523f, 0.2327f
};
static const float kB[16] = {  // N2 b-values
    0.5578f, 0.6514f, 0.7222f, 0.7825f, 0.8126f, 0.8434f, 0.8693f, 0.8910f,
    0.9092f, 0.9222f, 0.9319f, 0.9403f, 0.9477f, 0.9544f, 0.9602f, 0.9653f
};

// Model Constants
static constexpr float GF_LOW = 0.60f;       // GF Low 60
static constexpr float GF_HIGH = 0.85f;      // GF High 85
static constexpr float PO2_SETPOINT = 1.2f;  // Set point
static constexpr float ASCENT_RATE = 9.0f;   // Ascent rate m/min

// Physics Constants
static constexpr float LN2 = 0.693147f;              // Natural log of 2
static constexpr float GRAVITY = 9.81f;              // Gravity m/s²
static constexpr float ATM_PRESSURE_PA = 101325.0f;  // Pa per atm
static constexpr float N2_FRAC_AIR = 0.7902f;        // FiN2 in air
static constexpr float WATER_VAPOR = 0.0627f;        // Alveolar water vapour pressure
static constexpr float SURFACE_ATM = 1.0f;           // 1 atm at surface
static constexpr float SEAWATER_DENSITY = 1020.0f;   // EN13319 density kg/m³
static constexpr float METERS_PER_ATM = ATM_PRESSURE_PA / (SEAWATER_DENSITY * GRAVITY);


// Current PPN2 in compartments
static float N2[16];

// Gradient factor enabled/disabled
static bool sanity = true;

// Convert atm to depth in meters
static inline float depthFromPressureAtm(float pressureAtm) {
    const float depthM = (pressureAtm - SURFACE_ATM) * METERS_PER_ATM;
    return (depthM < 0.0f) ? 0.0f : depthM;
}

// Convert depth in meters to atm
static inline float pressureAtmFromDepth(float depthM) {
    return SURFACE_ATM + depthM / METERS_PER_ATM;
}

// Alveolar PPN2
static inline float ppN2AlvFromAmb(float ambientAtm) {
    const float p = ambientAtm - PO2_SETPOINT - WATER_VAPOR;
    return (p < 0.0f) ? 0.0f : p;
}

// Haldane tissue update from ambient pressure
static void tickTissuesAmbient(float n2[], float ambientAtm, float dtMin) {
    const float pAlv = ppN2AlvFromAmb(ambientAtm);
    for (int i = 0; i < 16; i++) {
        const float k = LN2 / kHT[i];
        n2[i] += (pAlv - n2[i]) * (1.0f - expf(-k * dtMin));
    }
}

// Haldane tissue update from depth for ascent simulation
static void tickTissuesDepth(float n2[], float depthM, float dtMin) {
    tickTissuesAmbient(n2, pressureAtmFromDepth(depthM), dtMin);
}

// Ceiling
static float ceilDepth(const float n2[], float gf) {
    float maxAtm = 0.0f;
    for (int i = 0; i < 16; i++) {
        const float p = n2[i];
        if (p <= 0.0f) continue;
        const float denom = gf / kB[i] + 1.0f - gf;
        if (denom <= 0.0f) continue;
        const float c = (p - kA[i] * gf) / denom;
        if (c > maxAtm) maxAtm = c;
    }
    const float d = (maxAtm - SURFACE_ATM) * METERS_PER_ATM;
    return (d < 0.0f) ? 0.0f : d;
}

// Surface GF
static float surfaceGfPercent(const float n2[]) {
    float maxGf = 0.0f;
    for (int i = 0; i < 16; i++) {
        const float p = n2[i];
        if (p <= 0.0f) continue;
        const float denom = kA[i] + SURFACE_ATM / kB[i] - SURFACE_ATM;
        if (denom <= 0.0f) continue;
        const float gf = (p - SURFACE_ATM) / denom;
        if (gf > maxGf) maxGf = gf;
    }
    if (maxGf < 0.0f) maxGf = 0.0f;
    return maxGf * 100.0f;
}

// GF Interpolation
static float gfAt(float depth, float dFirst) {
    if (dFirst <= 0.0f) return GF_HIGH;
    const float gf = GF_HIGH + (GF_LOW - GF_HIGH) * depth / dFirst;
    if (gf < GF_LOW) return GF_LOW;
    if (gf > GF_HIGH) return GF_HIGH;
    return gf;
}

// Simulate ascent for TTS calculation
static void simAscent(float n2[], float from, float to, int *totalMin) {
    if (from <= to) return;
    const float avg      = (from + to) * 0.5f;
    const float travelMin = (from - to) / ASCENT_RATE;
    tickTissuesDepth(n2, avg, travelMin);
    *totalMin += (int)ceilf(travelMin);
}


// Initialise tissue compartments
void decoInit() {
    const float ppN2Surf = (SURFACE_ATM - WATER_VAPOR) * N2_FRAC_AIR;
    for (int i = 0; i < 16; i++) {
        N2[i] = ppN2Surf;
    }
}

// Enable/disable raw model (no gradient factors)
void mad_man_mode(bool enabled) {
    sanity = !enabled;
}

// Update tissue
void decoUpdate(float pressureAtm, float dtMin) {
    if (dtMin > 0.0f) {
        tickTissuesAmbient(N2, pressureAtm, dtMin);
    }
}

// Calculate decompression
DecoResult decoCompute(float currentPressureAtm) {
    const float currentDepthM = depthFromPressureAtm(currentPressureAtm);
    float n2[16];
    memcpy(n2, N2, sizeof(n2));

    // Calculate surface GF
    const float surfGF = surfaceGfPercent(n2);

    // Find the deepest stop
    const float firstCeilGf = sanity ? GF_LOW : 1.0f;
    const float rawCeil = ceilDepth(n2, firstCeilGf);
    float dFirst = 0.0f;
    if (rawCeil > 0.1f) {
        dFirst = ceilf(rawCeil / 3.0f) * 3.0f;  // round up to nearest 3m
    }

    // NDL
    if (dFirst < 0.01f) {
        return {false, 0.0f, 0, (int)ceilf(currentDepthM / ASCENT_RATE), surfGF};
    }
    int totalMin = 0;

    // Simulate ascent from current depth to first stop
    if (currentDepthM > dFirst + 0.1f) {
        simAscent(n2, currentDepthM, dFirst, &totalMin);
    }

    // 3 meter stops
    bool firstRecorded = false;
    int  firstStopMin  = 0;
    float dStop = dFirst;

    while (dStop > 0.0f) {
        const float dNext  = (dStop <= 3.0f) ? 0.0f : dStop - 3.0f;
        const float gfNext = sanity ? gfAt(dNext, dFirst) : 1.0f;

        // Wait at stop until ceiling clear to next stop
        int stopMin = 0;
        while (ceilDepth(n2, gfNext) > dNext && stopMin < 99) {
            tickTissuesDepth(n2, dStop, 1.0f); // 1-minute tick
            stopMin++;
            totalMin++;
        }
        if (!firstRecorded) {
            firstStopMin  = stopMin;
            firstRecorded = true;
        }

        // Final stop to surface
        if (dNext <= 0.0f) {
            totalMin += (int)ceilf(dStop / ASCENT_RATE);
            break;
        }
        
        simAscent(n2, dStop, dNext, &totalMin);
        dStop = dNext;
    }
    return {true, dFirst, firstStopMin, totalMin, surfGF};
}