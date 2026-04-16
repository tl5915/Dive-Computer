#pragma once
#include <stdint.h>

static inline float simulatedDepthMeters(uint64_t elapsedMS) {
  const float tSec = static_cast<float>(elapsedMS) / 1000.0f;
  if (tSec < 120.0f) return 0.0f + 30.0f * (tSec / 60.0f);               // 0 -> 60 in 2 min
  if (tSec < 1800.0f) return 60.0f;                                      // 60m 28 min
  if (tSec < 2040.0f) return 60.0f - 9.0f * ((tSec - 1800.0f) / 60.0f);  // 60 -> 24
  if (tSec < 2160.0f) return 24.0f;                                      // 24m 2 min
  if (tSec < 2180.0f) return 24.0f - 9.0f * ((tSec - 2160.0f) / 60.0f);  // 24 -> 21
  if (tSec < 2300.0f) return 21.0f;                                      // 21m 2 min
  if (tSec < 2320.0f) return 21.0f - 9.0f * ((tSec - 2300.0f) / 60.0f);  // 21 -> 18
  if (tSec < 2500.0f) return 18.0f;                                      // 18m 3 min
  if (tSec < 2520.0f) return 18.0f - 9.0f * ((tSec - 2500.0f) / 60.0f);  // 18 -> 15
  if (tSec < 2820.0f) return 15.0f;                                      // 15m 5 min
  if (tSec < 2840.0f) return 15.0f - 9.0f * ((tSec - 2820.0f) / 60.0f);  // 15 -> 12
  if (tSec < 3200.0f) return 12.0f;                                      // 12m 6 min
  if (tSec < 3220.0f) return 12.0f - 9.0f * ((tSec - 3200.0f) / 60.0f);  // 12 -> 9
  if (tSec < 3700.0f) return 9.0f;                                       // 9m 8 min
  if (tSec < 3720.0f) return 9.0f - 9.0f * ((tSec - 3700.0f) / 60.0f);   // 9 -> 6
  if (tSec < 5820.0f) return 6.0f;                                       // Hold 6 for 35 min
  if (tSec < 5860.0f) return 6.0f - 9.0f * ((tSec - 5820.0f) / 60.0f);   // 6 -> 0
  return 0.0f;
}
