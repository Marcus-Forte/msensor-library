#pragma once
#include <algorithm>
#include <cmath>

#include "stdint.h"

static inline void getRainbowColor(float value, float &r, float &g, float &b) {
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  float n = 1 - f;

  if (i <= 1)
    r = n, g = 0, b = 1;
  else if (i == 2)
    r = 0, g = n, b = 1;
  else if (i == 3)
    r = 0, g = 1, b = n;
  else if (i == 4)
    r = n, g = 1, b = 0;
  else if (i >= 5)
    r = 1, g = n, b = 0;
}

inline void Int2RGB(float intensity, float &r, float &g, float &b) {
  const float min_i = 10;
  const float max_i = 150;

  // Normalize
  float norm_i = 1.0 - ((intensity - min_i) / (max_i - min_i));

  // Convert
  // to
  // HSV
  // (rainbow)
  float r_f, g_f, b_f;
  getRainbowColor(norm_i, r_f, g_f, b_f);

  r = r_f * 255.0f;
  g = g_f * 255.0f;
  b = b_f * 255.0f;
}