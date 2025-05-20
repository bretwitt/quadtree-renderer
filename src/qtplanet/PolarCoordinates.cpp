#include "qtplanet/PolarCoordinates.h"
#include <cmath>

std::array<Polar::Boundary,4>
CoordinateTraits<Polar>::getChildBounds(const Polar::Boundary& b) {
    float halfDR = b.dr * 0.5f, halfDT = b.dtheta * 0.5f;
    return {{
        { b.r + halfDR, b.theta + halfDT, halfDR, halfDT },
        { b.r + halfDR, b.theta - halfDT, halfDR, halfDT },
        { b.r - halfDR, b.theta + halfDT, halfDR, halfDT },
        { b.r - halfDR, b.theta - halfDT, halfDR, halfDT }
    }};
}

std::pair<float,float>
CoordinateTraits<Polar>::cartesianAt(const Polar::Boundary& b, int i, int j, int divisions) {
    float dr = b.dr/divisions, dTheta = b.dtheta/divisions;
    float r     = b.r - b.dr*0.5f + j*dr;
    float theta = b.theta - b.dtheta*0.5f + i*dTheta;
    return { r*std::cos(theta), r*std::sin(theta) };
}
