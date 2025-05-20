#include "qtplanet/CartesianCoordinates.h"
#include <cmath>
#include <algorithm>

std::array<Cartesian::Boundary,4>
CoordinateTraits<Cartesian>::getChildBounds(const Cartesian::Boundary& b) {
    float hw = b.width * 0.5f, hh = b.height * 0.5f;
    return {{
        { b.x + hw, b.y - hh, hw, hh }, // NE
        { b.x - hw, b.y - hh, hw, hh }, // NW
        { b.x + hw, b.y + hh, hw, hh }, // SE
        { b.x - hw, b.y + hh, hw, hh }  // SW
    }};
}

bool CoordinateTraits<Cartesian>::contains(const Cartesian::Boundary& b, float x, float y) {
    float left = b.x - b.width, right = b.x + b.width;
    float top  = b.y - b.height, bottom = b.y + b.height;
    return x >= left && x <= right && y >= top && y <= bottom;
}

float CoordinateTraits<Cartesian>::distanceTo(const Cartesian::Boundary& b, float x, float y, float z, float elevation) {
    float dx = std::max(std::abs(x - b.x) - b.width, 0.0f);
    float dy = std::max(std::abs(y - b.y) - b.height, 0.0f);
    float dz = elevation - z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::array<Cartesian::Boundary,4>
CoordinateTraits<Cartesian>::subdivide(const Cartesian::Boundary& b) {
    return getChildBounds(b);
}

float CoordinateTraits<Cartesian>::distance(Cartesian::Position p1, Cartesian::Position p2) {
    float dx = p2.x - p1.x, dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

std::pair<float,float>
CoordinateTraits<Cartesian>::cartesianAt(const Cartesian::Boundary& b, int i, int j, int divisions) {
    float stepX = (2*b.width)/divisions, stepY = (2*b.height)/divisions;
    float startX = b.x - b.width, startY = b.y - b.height;
    return { startX + i*stepX, startY + j*stepY };
}
