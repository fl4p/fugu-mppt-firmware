#ifndef INCLUDE_ASCII_CONSTANTS_H_
#define INCLUDE_ASCII_CONSTANTS_H_

#include <limits>

namespace ascii {
constexpr float kPI = 3.14159265358979323846;
constexpr float kDoubleNotANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float kDoubleInfinity = (std::numeric_limits<float>::infinity)();
constexpr float kDoubleNegInfinity =
    (-(std::numeric_limits<float>::infinity)());
} // namespace ascii
#endif