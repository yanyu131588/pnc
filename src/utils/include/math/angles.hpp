#ifndef ANGLES_HPP_
#define ANGLES_HPP_

#include <cmath>

double normalize_angle(double angle) {
    double normalized_angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (normalized_angle < 0.0)
        normalized_angle += 2.0 * M_PI; 

    return normalized_angle - M_PI;
}
#endif
