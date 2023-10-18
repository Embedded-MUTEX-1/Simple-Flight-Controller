#include "data.h"
#include <cstdio>
#include <string>

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std::string format(float f, int num) {
    std::string out = std::to_string((int16_t)f);
    return out;
}