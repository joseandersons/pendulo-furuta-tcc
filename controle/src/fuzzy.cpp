#include "fuzzy.h"

#include <math.h>

#include "ang.h"
#include "config.h"

#ifndef FUZZY_ALPHA_SMALL_FULL_DEG
#define FUZZY_ALPHA_SMALL_FULL_DEG 2.0f
#endif

#ifndef FUZZY_ALPHA_MEDIUM_FULL_START_DEG
#define FUZZY_ALPHA_MEDIUM_FULL_START_DEG 3.0f
#endif

#ifndef FUZZY_ALPHA_MEDIUM_FULL_END_DEG
#define FUZZY_ALPHA_MEDIUM_FULL_END_DEG 4.0f
#endif

#ifndef FUZZY_ALPHA_LARGE_FULL_DEG
#define FUZZY_ALPHA_LARGE_FULL_DEG 5.0f
#endif

#ifndef FUZZY_ALPHA_MAX_DEG
#define FUZZY_ALPHA_MAX_DEG 6.0f
#endif

#ifndef FUZZY_ALPHA_DOT_LOW_FULL_DEG_S
#define FUZZY_ALPHA_DOT_LOW_FULL_DEG_S 15.0f
#endif

#ifndef FUZZY_ALPHA_DOT_LOW_END_DEG_S
#define FUZZY_ALPHA_DOT_LOW_END_DEG_S 40.0f
#endif

#ifndef FUZZY_ALPHA_DOT_MED_START_DEG_S
#define FUZZY_ALPHA_DOT_MED_START_DEG_S 25.0f
#endif

#ifndef FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S
#define FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S 50.0f
#endif

#ifndef FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S
#define FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S 70.0f
#endif

#ifndef FUZZY_ALPHA_DOT_MED_END_DEG_S
#define FUZZY_ALPHA_DOT_MED_END_DEG_S 95.0f
#endif

#ifndef FUZZY_ALPHA_DOT_HIGH_START_DEG_S
#define FUZZY_ALPHA_DOT_HIGH_START_DEG_S 80.0f
#endif

#ifndef FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S
#define FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S 110.0f
#endif

#ifndef FUZZY_ALPHA_DOT_MAX_DEG_S
#define FUZZY_ALPHA_DOT_MAX_DEG_S 150.0f
#endif

static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static float trapmf(float x, float a, float b, float c, float d) {
  if (x <= a) {
    return (a == b) ? 1.0f : 0.0f;
  }

  if (x < b) {
    return clamp01((x - a) / (b - a));
  }

  if (x <= c) {
    return 1.0f;
  }

  if (x < d) {
    return clamp01((d - x) / (d - c));
  }

  return (c == d) ? 1.0f : 0.0f;
}

static float fuzzy_and(float a, float b) {
  return a < b ? a : b;
}

static uint16_t quantize_microsteps(float crisp_microsteps) {
  if (crisp_microsteps >= 192.0f) return 256;
  if (crisp_microsteps >= 96.0f) return 128;
  if (crisp_microsteps >= 48.0f) return 64;
  if (crisp_microsteps >= 24.0f) return 32;
  return 16;
}

FuzzySupervisorOutput fuzzy_evaluate_microsteps(
  float alpha_rad,
  float alpha_dot_rad_s
) {
  FuzzySupervisorOutput out{};

  const float alpha_deg = fabsf(rad2deg(wrapPi(alpha_rad)));
  const float alpha_dot_deg_s = fabsf(rad2deg(alpha_dot_rad_s));

  out.alpha_small = trapmf(
    alpha_deg,
    0.0f,
    0.0f,
    FUZZY_ALPHA_SMALL_FULL_DEG,
    FUZZY_ALPHA_MEDIUM_FULL_START_DEG
  );

  out.alpha_medium = trapmf(
    alpha_deg,
    FUZZY_ALPHA_SMALL_FULL_DEG,
    FUZZY_ALPHA_MEDIUM_FULL_START_DEG,
    FUZZY_ALPHA_MEDIUM_FULL_END_DEG,
    FUZZY_ALPHA_LARGE_FULL_DEG
  );

  out.alpha_large = trapmf(
    alpha_deg,
    FUZZY_ALPHA_MEDIUM_FULL_END_DEG,
    FUZZY_ALPHA_LARGE_FULL_DEG,
    FUZZY_ALPHA_MAX_DEG,
    FUZZY_ALPHA_MAX_DEG
  );

  out.alpha_dot_low = trapmf(
    alpha_dot_deg_s,
    0.0f,
    0.0f,
    FUZZY_ALPHA_DOT_LOW_FULL_DEG_S,
    FUZZY_ALPHA_DOT_LOW_END_DEG_S
  );

  out.alpha_dot_medium = trapmf(
    alpha_dot_deg_s,
    FUZZY_ALPHA_DOT_MED_START_DEG_S,
    FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S,
    FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S,
    FUZZY_ALPHA_DOT_MED_END_DEG_S
  );

  out.alpha_dot_high = trapmf(
    alpha_dot_deg_s,
    FUZZY_ALPHA_DOT_HIGH_START_DEG_S,
    FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S,
    FUZZY_ALPHA_DOT_MAX_DEG_S,
    FUZZY_ALPHA_DOT_MAX_DEG_S
  );

  const float a[3] = {
    out.alpha_small,
    out.alpha_medium,
    out.alpha_large
  };

  const float v[3] = {
    out.alpha_dot_low,
    out.alpha_dot_medium,
    out.alpha_dot_high
  };

  const uint16_t rules[3][3] = {
    {256, 128,  64},
    {128,  64,  32},
    { 16,  16,  16}
  };

  float numerator = 0.0f;
  float denominator = 0.0f;

  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      const float firing = fuzzy_and(a[i], v[j]);
      numerator += firing * (float)rules[i][j];
      denominator += firing;
    }
  }

  if (denominator <= 1e-6f) {
    out.crisp_microsteps = 256.0f;
    out.microsteps = 256;
    return out;
  }

  out.crisp_microsteps = numerator / denominator;
  out.microsteps = quantize_microsteps(out.crisp_microsteps);

  return out;
}

uint16_t fuzzy_select_microsteps(
  float alpha_rad,
  float alpha_dot_rad_s
) {
  return fuzzy_evaluate_microsteps(alpha_rad, alpha_dot_rad_s).microsteps;
}
