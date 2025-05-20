#ifndef QTPLANET_POLARCOORDINATES_H
#define QTPLANET_POLARCOORDINATES_H

#include "CoordinateSystems.h"
#include <array>

template<>
struct CoordinateTraits<Polar> {
    static std::array<Polar::Boundary,4> getChildBounds(const Polar::Boundary&);
    static std::pair<float,float> cartesianAt(const Polar::Boundary&, int i, int j, int divisions);
};

#endif // QTPLANET_POLARCOORDINATES_H